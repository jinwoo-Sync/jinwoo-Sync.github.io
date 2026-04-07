---
layout: post
title: "CMake setup.cmake 매크로 기반 계층적 빌드 체계"
tags: [CMake, C++, Build System, Cross Compilation, ARM, x86]
---

MMS(Mobile Mapping System) 프로젝트에서 x86·ARM 동시 배포, 6개 앱 독립 빌드를 위해 `setup.cmake` 매크로 기반 계층적 빌드 체계를 운영했다.

---

## 먼저 — CMake 핵심 함수 정리

이 글에서 자주 나오는 cmake 명령어들이다.

| 함수 | 역할 |
|------|------|
| `add_subdirectory(src/lidar bin/lidar)` | `src/lidar/CMakeLists.txt`를 빌드 대상에 포함. 실제 컴파일이 여기서 등록됨 |
| `include_directories(경로)` | 컴파일러가 `#include`를 찾을 헤더 경로 등록 |
| `target_link_libraries(타겟 라이브러리)` | 빌드된 타겟에 링크할 라이브러리 지정 |
| `add_dependencies(A B)` | A를 빌드하기 전에 B가 반드시 완료돼야 한다고 cmake에 선언 |
| `add_library(이름 STATIC 소스)` | 정적 라이브러리(.a) 생성 |
| `add_library(이름 SHARED 소스)` | 동적 라이브러리(.so) 생성 |
| `add_library(이름 MODULE 소스)` | dlopen 전용 플러그인(.so) 생성, 링크 시점에 참조하지 않음 |
| `file(COPY 파일 DESTINATION 경로)` | 빌드 시 파일을 지정 경로로 복사 |
| `include_guard(GLOBAL)` | 이 cmake 파일을 전체 빌드에서 딱 한 번만 로드 |
| `macro(이름) ... endmacro()` | 반복 작업을 묶어두는 cmake 매크로 (함수처럼 호출 가능) |
| `if(NOT TARGET 이름)` | 해당 타겟이 아직 등록되지 않은 경우에만 실행 |
| `set_target_properties(타겟 PROPERTIES ...)` | 타겟의 출력 경로·이름 등 속성 설정 |

---

## 왜 이 구조가 필요했나

**문제 1 — 카메라 SDK가 아키텍처마다 달랐다**

Flycapture, Spinnaker, Vimba는 x86 전용이고, Aravis·Pylon은 `lib_x64`/`lib_arm` 바이너리가 따로 존재했다. MMS 장비가 x86 PC와 Jetson(ARM) 양쪽에 배포되므로, 빌드 시스템이 **실행 환경을 자동 감지해서 올바른 SDK를 선택**해야 했다.

**문제 2 — 앱이 6개, 개별/전체 빌드가 모두 가능해야 했다**

LoggingTool, ExtractTool, tool_intrinsic, tool_extrinsic, visualizer_cpu — 각 앱을 개별로도 전체로도 빌드할 수 있어야 문제 모듈만 수정·재배포하거나 앱마다 버전을 독립 관리할 수 있다.

---

## 전체 계층 구조

```
[Layer 0]  3rd/cmake/setup.cmake
           Eigen3, yaml-cpp, jsoncpp, ceres 등 서드파티 소스 빌드

[Layer 1]  loggingcoremodule/cmake/setup.cmake  ← 핵심 매크로 파일
           MMS_UTIL → MMS_IO → MMS_LIDAR / MMS_GPS / MMS_RADAR ...  (STATIC)
           CameraAravis / CameraPylon / CameraFlycapture ...          (MODULE)

[Layer 2]  ModuleLoder.so  (SHARED)
           센서 STATIC 라이브러리를 전부 흡수해 하나로 묶음

[Layer 3]  각 앱  →  setup.cmake include 후 필요한 매크로만 호출
```

---

## 아키텍처 자동 감지 — 3단계

```cmake
# 1단계: cmake 실행 머신의 CPU 감지 → ARCH_FLAGS 변수 하나로 이후 전체를 제어
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    set(ARCH_FLAGS "-march=armv8-a")   # ARM이면 이 값
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH_FLAGS "-m64")             # x86이면 이 값
endif()

# 2단계: ARCH_FLAGS 기준으로 빌드 대상 SDK 자체를 포함/제외
#        ARM에선 add_subdirectory 자체가 없으니 컴파일 시도조차 안 함
if(ARCH_FLAGS MATCHES "-m64")          # x86이면
    add_subdirectory(flycapture)       # x86 전용 SDK 포함
    add_subdirectory(spinnaker)
    add_subdirectory(aravis)           # ARM/x86 모두 지원
    add_subdirectory(vimba)
    add_subdirectory(pylon)
elseif(ARCH_FLAGS MATCHES "arm|aarch64")  # ARM이면
    # add_subdirectory(flycapture)     # ← 아예 제외 (ARM 바이너리 없음)
    # add_subdirectory(spinnaker)
    add_subdirectory(aravis)           # ARM 바이너리 있는 것만 포함
    add_subdirectory(pylon)
    add_subdirectory(econ_mipi)        # ARM 전용
endif()

# 3단계: 포함된 SDK 안에서 올바른 바이너리 경로 선택 + 배포 디렉터리에 자동 복사
if(ARCH_FLAGS MATCHES "-m64")
    target_link_libraries(CameraAravis .../lib_x64/libaravis-0.8.so ...)
    file(COPY .../lib_x64/libaravis-0.8.so   # x86 바이너리를 bin/lib/에 복사
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
elseif(ARCH_FLAGS MATCHES "arm|aarch64")
    target_link_libraries(CameraAravis .../lib_arm/libaravis-0.8.so ...)
    file(COPY .../lib_arm/libaravis-0.8.so   # ARM 바이너리를 bin/lib/에 복사
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
endif()
```

`ARCH_FLAGS` 변수 하나가 2·3단계를 모두 제어한다. cmake 실행 머신 환경만으로 자동 결정되므로 별도 옵션이 없다.

---

## setup.cmake 구성

### 전역 설정 (파일 상단)

```cmake
include_guard(GLOBAL)                           # 이 파일을 전체 빌드에서 한 번만 로드
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-fPIC -std=c++14 -fopenmp")
set(OUTPUT_PATH ${BASE_DIRECTORY}/bin_${ARCH})  # bin_x64 or bin_x86

# 모든 앱이 이 매크로 한 줄로 출력 경로·install 설정 완료
macro(_finalize_target _PROJNAME)
  set_target_properties(${_PROJNAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_PATH}/"  # .a 파일
    LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_PATH}/"  # .so 파일
    RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_PATH}/") # 실행파일
  install(TARGETS ${_PROJNAME} DESTINATION bin_${ARCH})
endmacro()

# setup.cmake를 include하는 순간 CoreModule 헤더가 전 프로젝트에 등록됨
# 앱마다 include_directories를 반복 선언할 필요 없음
include_directories(
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule/include
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src
)
```

### 센서 모듈 매크로 — 5줄 패턴

```cmake
macro(_add_package_MMS_LIDAR)
  include_directories(${BASE_DIRECTORY}/.../lidar)  # 이 모듈 헤더 경로 등록
  include_directories(${BASE_DIRECTORY}/.../io)     # 의존 모듈 헤더도 함께 등록
  if(NOT TARGET MMS_LIDAR)                          # 이미 등록됐으면 전부 skip (중복 방지)
    _add_package_MMS_UTIL()                         # 의존 모듈을 먼저 재귀 호출 → cmake 그래프 등록 순서 보장
    _add_package_MMS_IO()                           # LIDAR보다 IO가 먼저 add_subdirectory됨
    add_subdirectory(src/lidar bin/lidar)           # 이 모듈을 실제 빌드 대상에 등록
    add_dependencies(MMS_LIDAR MMS_IO MMS_UTIL)    # make -j 병렬 빌드 시 링크 순서 보장
  endif()
endmacro()
```

앱 CMakeLists.txt에서 `_add_package_MMS_LIDAR()` 한 줄이면 UTIL·IO까지 자동으로 딸려온다.

### 모듈별 의존 관계

| 매크로 | 내부에서 먼저 호출 | add_dependencies |
|--------|-------------------|-----------------|
| `_add_package_MMS_UTIL` | — | — |
| `_add_package_MMS_IO` | UTIL, CAMERA_UTIL | — |
| `_add_package_MMS_LIDAR` | UTIL, IO | MMS_IO, MMS_UTIL |
| `_add_package_MMS_RADAR` | UTIL | MMS_IO |
| `_add_package_MMS_GPS` | UTIL | MMS_IO |
| `_add_package_MMS_TRIGGER` | UTIL | MMS_IO |
| `_add_package_MMS_COMMUNICATION` | UTIL | MMS_UTIL |
| `_add_package_MMS_MODULELODER` | (전부 포함) | — |

`_add_package_MMS_MODULELODER`는 CoreModule 전체를 add_subdirectory하므로, 이걸 하나 호출하면 위의 모든 센서 모듈이 한 번에 빌드된다.

### 3가지 핵심 메커니즘 정리

| 메커니즘 | 역할 |
|----------|------|
| 매크로 안 재귀 호출 순서 | cmake 그래프 등록 순서 보장 (UTIL → IO → LIDAR) |
| `add_dependencies()` | `make -j` 병렬 빌드 시에도 링크 순서 보장 |
| `if(NOT TARGET ...)` | 여러 앱이 같은 모듈을 요청해도 한 번만 빌드 |

두 메커니즘이 같이 있어야 안전하다. 재귀 호출만으로는 `make -j`에서 타이밍 문제가 생길 수 있고, `add_dependencies`만으로는 include 경로가 안 잡힐 수 있다.

---

## STATIC / MODULE / SHARED 라이브러리 역할 분리

### 라이브러리 타입별 개념

**정적 라이브러리(.a)**

```
컴파일:   소스.cpp  →  오브젝트.o
아카이브: .o + .o + .o  →  라이브러리.a
링크:     실행파일 = 내 코드.o + 라이브러리.a (코드를 통째로 복사해 넣음)
```

링크 시점에 `.a` 안의 코드가 실행파일 안으로 직접 복사된다. 빌드 후에는 `.a` 파일이 없어도 실행파일이 혼자 작동한다.

**동적 라이브러리(.so)**

```
빌드:  실행파일에는 "libfoo.so를 써라"는 참조만 기록
실행:  OS가 /usr/lib/ 등에서 libfoo.so를 찾아 메모리에 올림
      → 실행파일과 libfoo.so가 메모리에서 연결됨
```

실행 시 `.so` 파일이 반드시 존재해야 한다. 대신 여러 프로세스가 같은 `.so`를 공유하므로 메모리를 아낀다.

**MODULE 타입(.so, dlopen 전용)**

`add_library(이름 MODULE ...)`로 만들면 일반 SHARED와 거의 같은 `.so`가 생성되지만, **링크 시점에 참조하지 않고 런타임에 `dlopen()`으로 직접 열어 쓰는 플러그인 전용** 타입이다.

```cpp
// ModuleLoder.cpp 내부 동작 (개념)
void* handle = dlopen("lib/CameraAravis.so", RTLD_LAZY);  // 런타임에 .so 파일 열기
auto createFn = dlsym(handle, "create");                  // 함수 포인터 획득
ICamera* cam = ((ICamera*(*)())createFn)();               // 카메라 객체 생성
```

`.so` 파일이 빌드 디렉터리에 없어도 빌드 자체는 성공한다. 런타임에만 필요하다.

---

### 각 컴포넌트에 타입을 선택한 이유

**3rd party (yaml-cpp, jsoncpp, eigen 등) → STATIC**

```cmake
set(BUILD_SHARED_LIBS off)    # 서드파티 전체 → 정적 빌드 강제
set(BUILD_STATIC_LIBS ON)
```

- **버전 충돌 방지**: 배포 환경마다 시스템에 설치된 버전이 다르다. 동적 링크하면 현장 PC의 버전과 개발 버전이 달라 런타임 오류가 생긴다. 정적 링크하면 테스트한 버전이 실행파일에 고정된다.
- **배포 단순화**: ModuleLoder.so 안에 흡수되므로 별도 `.so` 파일을 챙길 필요가 없다.
- **변경 없음**: Eigen, yaml-cpp 같은 라이브러리는 프로젝트 기간 동안 버전을 교체할 일이 없어서 동적 링크의 장점(런타임 교체)이 필요 없다.

**MMS_LIDAR, MMS_GPS 등 센서 모듈 → STATIC**

```cmake
add_library(MMS_LIDAR STATIC ${MODULE_CPP})
```

이 STATIC 라이브러리들은 최종적으로 `ModuleLoder.so` 안으로 링크 인(link-in)된다. 배포물에 `.a` 파일은 따로 존재하지 않는다.

```
MMS_UTIL.a   ─┐
MMS_IO.a     ─┤
MMS_LIDAR.a  ─┼──→  ModuleLoder.so  (이 안에 전부 흡수됨)
MMS_GPS.a    ─┤
MMS_RADAR.a  ─┘
```

센서 드라이버 각각을 `.so`로 만들면 배포물이 늘어나고, `ModuleLoder.so` 로드 시 의존 체인이 줄줄이 따라온다. `.a`로 묶어 하나 안에 넣으면 배포·관리가 단순해진다.

**카메라 SDK 래퍼 (CameraAravis 등) → MODULE**

```cmake
add_library(CameraAravis MODULE CameraAravis.cpp ../CameraModule.cpp)
```

- **현장 구성이 장비마다 다름**: 어떤 카메라가 붙을지 빌드 시점에 모른다. `dlopen()`으로 런타임에 필요한 카메라 모듈만 로드하면 배포물은 같아도 현장에서 필요한 `.so`만 챙겨가면 된다.
- **SDK 의존성 격리**: 카메라 SDK 래퍼를 STATIC으로 `ModuleLoder.so`에 넣으면, SDK가 없는 환경에서 `ModuleLoder.so` 로드 자체가 실패한다. MODULE로 분리하면 해당 SDK가 없어도 `ModuleLoder.so`는 정상 로드된다. 없는 카메라 모듈은 그냥 로드하지 않으면 된다.
- **카메라 드라이버만 교체 가능**: SDK 업데이트나 새 벤더 지원 시 `CameraAravis.so`만 다시 빌드해서 교체하면 된다. 앱도, `ModuleLoder.so`도 재빌드 불필요.

**ModuleLoder → SHARED**

```cmake
add_library(ModuleLoder SHARED ${MODULE_CPP})
```

LoggingTool, ExtractTool, tool_intrinsic, tool_extrinsic, visualizer_cpu — 5개 앱이 전부 센서 드라이버를 사용한다.

```
# STATIC이었다면 → 같은 드라이버 코드가 5개 실행파일에 각각 복사됨
LoggingTool    = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 1)
ExtractTool    = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 2)
tool_intrinsic = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 3)

# SHARED로 만들면 → 메모리에 한 번만 올라가 5개 앱이 공유
ModuleLoder.so ← 메모리에 한 번만 올라감
    ↑               ↑               ↑
LoggingTool     ExtractTool     tool_intrinsic
```

드라이버 버그를 수정할 때 `ModuleLoder.so`만 교체하면 5개 앱 전체에 반영된다. 재빌드 없이 업데이트 가능하다.

---

## 최종 배포물 구조

```
bin_x64/
├── LoggingTool           ← 실행파일
├── ExtractTool           ← 실행파일
├── tool_intrinsic        ← 실행파일
├── ModuleLoder.so        ← 센서 드라이버 통합 (MMS_LIDAR.a 등 흡수됨)
└── lib/
    ├── CameraAravis.so   ← MODULE: dlopen으로 런타임 로드
    ├── CameraPylon.so
    └── libaravis-0.8.so  ← 카메라 벤더 SDK 원본 .so
```

| 컴포넌트 | 타입 | 배포 파일 | 핵심 이유 |
|----------|------|-----------|-----------|
| yaml-cpp, jsoncpp, eigen | STATIC (.a) | 없음 (ModuleLoder.so 안에 흡수) | 버전 고정, 배포 단순화 |
| MMS_LIDAR, MMS_GPS 등 | STATIC (.a) | 없음 (ModuleLoder.so 안에 흡수) | ModuleLoder.so 하나로 배포 |
| CameraAravis 등 | MODULE (.so) | lib/Camera*.so | 런타임 플러그인, SDK 격리 |
| ModuleLoder | SHARED (.so) | ModuleLoder.so | 여러 앱 공유, 단독 교체 가능 |
