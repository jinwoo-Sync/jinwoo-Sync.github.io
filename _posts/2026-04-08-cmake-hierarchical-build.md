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

**SHARED vs MODULE — 둘 다 `.so`인데 차이가 있나?**

리눅스에서 파일 자체는 둘 다 `.so`로 동일하다. 차이는 CMake가 어떻게 다루느냐다.

| | SHARED | MODULE |
|--|--|--|
| `target_link_libraries(앱 foo)` | 가능 — 빌드 시 링크 | CMake 에러 — 불가 |
| 런타임 로드 | OS가 앱 실행 시 자동으로 | 앱이 `dlopen()`으로 직접 |
| ELF NEEDED 기록 | 됨 | 안 됨 |

카메라 모듈을 MODULE로 만든 이유가 바로 이것이다. SHARED로 만들면 `ModuleLoder.so`가 빌드 시 `CameraAravis.so`를 링크해야 하고, Aravis SDK가 없는 장비에서 `ModuleLoder.so` 로드 자체가 실패한다. MODULE로 만들면 빌드 시 참조가 없으니, 런타임에 `dlopen("CameraAravis.so")`가 실패해도 `ModuleLoder.so`는 정상적으로 뜬다.

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
  include_directories(${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/lidar)  # 이 모듈 헤더 경로 등록
  include_directories(${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/io)     # 의존 모듈 헤더도 함께 등록
  include_directories(${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/camera)
  include_directories(${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/util)
  if(NOT TARGET MMS_LIDAR)                                                        # 이미 등록됐으면 전부 skip (중복 방지)
    _add_package_MMS_UTIL()                                                       # 의존 모듈을 먼저 재귀 호출 → cmake 그래프 등록 순서 보장
    _add_package_MMS_IO()                                                         # LIDAR보다 IO가 먼저 add_subdirectory됨
    add_subdirectory(${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/lidar     # 이 모듈을 실제 빌드 대상에 등록
                     ${CMAKE_BINARY_DIR}/loggingcoremodule/CoreModule/src/lidar)
    add_dependencies(MMS_LIDAR MMS_IO MMS_UTIL)                                  # make -j 병렬 빌드 시 링크 순서 보장
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

### 3가지 빌드 순서 제어 방법

| 방법 | 역할 |
|------|------|
| 매크로 안 재귀 호출 순서 | cmake 그래프 등록 순서 보장 (UTIL → IO → LIDAR) |
| `add_dependencies()` | `make -j` 병렬 빌드 시에도 링크 순서 보장 |
| `if(NOT TARGET ...)` | 여러 앱이 같은 모듈을 요청해도 한 번만 빌드 |

재귀 호출만으로는 `make -j`에서 타이밍 문제가 생길 수 있고, `add_dependencies`만으로는 include 경로가 안 잡힐 수 있다. 둘 다 있어야 안전하다.

---

## STATIC / MODULE / SHARED 라이브러리 역할 분리

### 타입별 개념 요약

| 타입 | 링크 시점 | 특징 |
|------|-----------|------|
| **STATIC (.a)** | 빌드 시 실행파일 안으로 코드 복사 | 배포 후 .a 파일 불필요. 실행파일 자급자족 |
| **SHARED (.so)** | 실행 시 OS가 메모리에 로드해 연결 | 여러 프로세스가 공유. .so 교체만으로 업데이트 가능 |
| **MODULE (.so)** | 링크 시 참조 안 함 — `dlopen()`으로 런타임에 직접 열어 씀 | 플러그인 전용. .so 없어도 빌드 성공 |

MODULE 타입 동작 예시:

```cpp
void* handle = dlopen("lib/CameraAravis.so", RTLD_LAZY);  // 런타임에 .so 파일 열기
auto createFn = dlsym(handle, "create");                  // 함수 포인터 획득
ICamera* cam = ((ICamera*(*)())createFn)();               // 카메라 객체 생성
```

### 각 컴포넌트 선택 근거

**3rd party (yaml-cpp, eigen 등) → STATIC**: 배포 환경마다 시스템 버전이 다르면 동적 링크 시 런타임 오류. 정적 링크로 버전을 실행파일에 고정. 프로젝트 기간 동안 버전 교체도 없으니 동적 링크의 장점이 없다.

**MMS_LIDAR, MMS_GPS 등 센서 모듈 → STATIC**: `ModuleLoder.so` 하나 안으로 흡수시키기 위해. 각각을 `.so`로 만들면 배포물이 늘어나고 의존 체인이 복잡해진다.

```
MMS_UTIL.a + MMS_IO.a + MMS_LIDAR.a + MMS_GPS.a  →  ModuleLoder.so (전부 흡수)
```

**카메라 SDK 래퍼 (CameraAravis 등) → MODULE**: 장비마다 붙는 카메라가 달라 빌드 시점에 구성을 모른다. `dlopen()`으로 필요한 카메라 모듈만 런타임 로드. SDK가 없는 장비에서도 `ModuleLoder.so`는 정상 로드된다.

**ModuleLoder → SHARED**: 5개 앱(LoggingTool, ExtractTool, tool_intrinsic 등)이 전부 센서 드라이버를 공유. STATIC이었다면 같은 드라이버 코드가 5개 실행파일에 각각 복사됨. SHARED면 메모리에 한 번만 올라가고, 버그 수정 시 `ModuleLoder.so` 하나만 교체하면 전체 앱에 반영된다.

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
