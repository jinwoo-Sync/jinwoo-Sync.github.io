---
layout: post
title: "CMake setup.cmake 매크로 기반 계층적 빌드 체계"
tags: [CMake, C++, Build System, Cross Compilation, ARM, x86]
---

하나의 코드베이스에서 여러 애플리케이션을 빌드하고, x86과 ARM 양쪽에 동시에 배포해야 할 때 빌드 시스템 설계가 중요해진다. MMS(Mobile Mapping System) 프로젝트에서 CMake `setup.cmake` 매크로 기반으로 계층적 빌드 체계를 운영한 경험을 정리한다.

---

## 배경 — 두 가지 문제

### 문제 1. 카메라 SDK가 아키텍처마다 달랐다

카메라 벤더마다 지원하는 CPU 아키텍처가 제각각이었다.

| SDK | x86_64 | ARM(aarch64) | 비고 |
|-----|--------|--------------|------|
| Flycapture (Point Grey) | O | X | ARM 바이너리 없음 |
| Spinnaker (FLIR) | O | X | ARM 미지원 |
| Vimba (Allied Vision) | O | X | ARM 미지원 |
| **Aravis** (GigE Vision) | O | O | `lib_x64` / `lib_arm` 분리 |
| **Pylon** (Basler) | O | O | `lib_x64` / `lib_arm` 분리 |
| econ_mipi | X | O | ARM 전용 (임베디드 카메라) |

MMS 장비가 x86 PC뿐 아니라 **Jetson Orin, Jetson Nano 같은 ARM 임베디드 보드**에서도 동작했기 때문에, 빌드 시스템이 **실행 환경의 아키텍처를 자동 감지해서 올바른 SDK 바이너리를 선택**해야 했다.

### 문제 2. 애플리케이션이 여러 개, 개별/전체 빌드가 모두 가능해야 했다

하나의 코드베이스에서 6개 타겟을 지원했다.

| 타겟 | 역할 |
|------|------|
| CoreModule (ModuleLoder) | 센서 드라이버 집합 공유 라이브러리 |
| LoggingTool | 데이터 수집 CLI 툴 |
| ExtractTool | 데이터 추출 CLI 툴 |
| tool_intrinsic | 내부 캘리브레이션 Qt 앱 |
| tool_extrinsic | 외부 캘리브레이션 Qt 앱 |
| visualizer_cpu | CPU 기반 포인트클라우드 렌더러 Qt 앱 |

각 앱을 **개별로도, 전체를 한 번에도** 빌드할 수 있어야 했다. 이게 가능하면 문제 있는 모듈만 수정해서 재빌드하거나, 앱마다 버전을 독립적으로 관리할 수 있다.

---

## 전체 계층 구조

```
[Layer 0]  3rd/cmake/setup.cmake
           소스 포함 서드파티 라이브러리 (소스 빌드)
           └─ Eigen3, yaml-cpp, jsoncpp, proj4, ndt_omp, ceres, sophus ...

[Layer 1]  loggingcoremodule/cmake/setup.cmake  ← 핵심 매크로 파일
           센서별 STATIC 라이브러리
           ├─ MMS_UTIL        (공통 유틸)
           ├─ MMS_IO          (파일 입출력)  → UTIL 의존
           ├─ MMS_LIDAR       (라이다)       → IO + UTIL 의존
           ├─ MMS_GPS         (GPS/IMU)      → IO + UTIL 의존
           ├─ MMS_RADAR       (레이더)       → IO 의존
           ├─ MMS_TRIGGER     (트리거)       → IO 의존
           └─ MMS_COMMUNICATION (통신)       → UTIL 의존

           카메라 SDK (MODULE 라이브러리, 런타임 동적 로드)
           └─ x86_64: Flycapture, Spinnaker, Aravis, Vimba, HIK, Pylon
              ARM:    Aravis, Ozray, Pylon, econ_mipi

[Layer 2]  CoreModule → ModuleLoder (SHARED 라이브러리)
           센서 STATIC 라이브러리 + 카메라 MODULE들을 하나로 묶음

[Layer 3]  애플리케이션 (각자 setup.cmake include 후 필요한 매크로만 호출)
           ├─ replicalogging  (LoggingTool, ExtractTool)
           ├─ tool_intrinsic
           ├─ tool_extrinsic
           └─ visualizer_cpu
```

---

## 아키텍처 자동 감지 — 3단계 흐름

```
cmake 실행
  → CMAKE_SYSTEM_PROCESSOR 읽기           (1단계: 어떤 CPU?)
  → ARCH_FLAGS 설정
  → add_subdirectory 포함/제외 결정       (2단계: 어떤 SDK를 빌드?)
  → lib_x64 or lib_arm 경로 선택 + COPY  (3단계: 어떤 바이너리를 링크?)
```

1단계에서 세운 `ARCH_FLAGS` 하나가 2, 3단계 전체를 제어한다. 별도 cmake 옵션 없이 **cmake를 실행하는 머신 환경만으로 자동 결정**된다.

### 1단계 — 아키텍처 감지

```cmake
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    set(ARCH_FLAGS "-march=armv8-a")   # ARM이면 이 값
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH_FLAGS "-m64")             # x86이면 이 값
endif()
```

cmake가 실행되는 머신의 CPU를 `CMAKE_SYSTEM_PROCESSOR`로 읽어 `ARCH_FLAGS` 변수에 저장한다. 이후 모든 분기의 기준값이 된다.

### 2단계 — 빌드할 SDK 목록 선택

```cmake
if(ARCH_FLAGS MATCHES "-m64")               # x86이면
    add_subdirectory(flycapture)            # 빌드 포함
    add_subdirectory(spinnaker)
    add_subdirectory(aravis)
    add_subdirectory(vimba)
    add_subdirectory(pylon)
elseif(ARCH_FLAGS MATCHES "arm|aarch64")    # ARM이면
    # add_subdirectory(flycapture)          # 아예 빠짐
    # add_subdirectory(spinnaker)
    add_subdirectory(aravis)                # ARM 바이너리 있으니 포함
    add_subdirectory(ozray)
    add_subdirectory(pylon)
    add_subdirectory(econ_mipi)             # ARM 전용
endif()
```

x86 전용 SDK(flycapture 등)는 ARM 빌드에서 `add_subdirectory` 자체가 없으므로 **컴파일 시도조차 안 한다**.

### 3단계 — 올바른 바이너리 경로 선택

```cmake
if(ARCH_FLAGS MATCHES "-m64")
    set(ARAVIS_LIB_PATH .../aravis-0.8/lib_x64)
    target_link_libraries(CameraAravis .../libaravis-0.8.so.0.8.19 ...)
    file(COPY .../lib_x64/libaravis-0.8.so.0.8.19
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)

elseif(ARCH_FLAGS MATCHES "arm|aarch64")
    set(ARAVIS_LIB_PATH .../aravis-0.8/lib_arm)
    target_link_libraries(CameraAravis .../libaravis-0.8.so.0.8.20 ...)
    file(COPY .../lib_arm/libaravis-0.8.so.0.8.20
         DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
endif()
```

2단계에서 포함된 SDK라면, 어느 폴더의 `.so`를 링크하고 복사할지 결정한다. `file(COPY ... DESTINATION bin/)` 까지 처리되므로 빌드 후 배포 디렉터리가 자동으로 완성된다.

---

## setup.cmake 전체 구성

`loggingcoremodule/cmake/setup.cmake` 파일은 크게 4구역으로 나뉜다.

### 구역 1 — 전역 컴파일 환경 설정

```cmake
include_guard(GLOBAL)            # 이 파일 자체를 한 번만 로드

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-fPIC -std=c++14 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS -no-pie)
set(LIB_TYPE STATIC)

# 포인터 크기로 아키텍처 판별 → 출력 디렉터리 이름에 사용
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "x64")
else()
  set(ARCH "x86")
endif()

set(OUTPUT_PATH ${BASE_DIRECTORY}/bin_${ARCH})  # bin_x64 or bin_x86
```

이 파일을 include한 순간 **C++14, fPIC, OpenMP, 출력 경로**가 전 프로젝트에 일괄 적용된다.

### 구역 2 — 출력 경로 통일 매크로

```cmake
macro(_set_target_output _PROJNAME)
  set_target_properties(${_PROJNAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_PATH}/"
    LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_PATH}/"
    RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_PATH}/"
  )
endmacro()

macro(_finalize_target _PROJNAME)
  _set_target_output(${_PROJNAME})
  install(TARGETS ${_PROJNAME} DESTINATION bin_${ARCH})
endmacro()
```

모든 바이너리가 `bin_x64/` 한 곳에 모인다. 앱을 추가할 때 `_finalize_target(MyApp)` 한 줄로 경로·설치 설정이 끝난다.

### 구역 3 — CoreModule 전역 include 경로 등록

```cmake
include_directories(
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule/include
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src
  ${BASE_DIRECTORY}/loggingcoremodule/CoreModule/src/camera
)
```

setup.cmake를 include하는 순간 CoreModule 헤더 경로가 전체 프로젝트에 등록된다. 개별 앱에서 경로를 다시 잡을 필요가 없다.

### 구역 4 — 센서 모듈 매크로

모든 매크로가 완전히 동일한 구조다.

```cmake
macro(_add_package_MMS_XXX)
  include_directories(해당_모듈_src_경로)       # 헤더 경로 등록
  include_directories(의존_모듈_src_경로)       # 의존 모듈 헤더도 등록
  if(NOT TARGET MMS_XXX)                        # 중복 방지 가드
    _add_package_MMS_의존모듈()                 # 의존 모듈 먼저 등록
    add_subdirectory(src/XXX  bin/XXX)          # 이 모듈 빌드 등록
    add_dependencies(MMS_XXX MMS_의존모듈)      # make -j 순서 보장
  endif()
endmacro()
```

각 모듈별 의존 관계:

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

---

## 매크로 핵심 패턴 5가지

### 패턴 1 — 빌드 순서 보장: 재귀 호출 + `add_dependencies`

```cmake
macro(_add_package_MMS_LIDAR)
  if(NOT TARGET MMS_LIDAR)
    _add_package_MMS_UTIL()                          # (A) UTIL 먼저 등록
    _add_package_MMS_IO()                            # (A) IO 먼저 등록
    add_subdirectory(... MMS_LIDAR)
    add_dependencies(MMS_LIDAR MMS_IO MMS_UTIL)      # (B) 병렬 빌드 순서 명시
  endif()
endmacro()
```

| 메커니즘 | 역할 |
|----------|------|
| 매크로 안 재귀 호출 순서 | cmake 그래프 등록 순서 보장 |
| `add_dependencies()` | 병렬 빌드(`make -j`) 시 링크 순서 보장 |

두 개가 같이 있어야 안전하다. 재귀 호출만으로는 `make -j`에서 타이밍 문제가 생길 수 있고, `add_dependencies`만으로는 include 경로가 안 잡힐 수 있다.

### 패턴 2 — `if(NOT TARGET ...)` 가드: 중복 빌드 방지

```cmake
macro(_add_package_MMS_LIDAR)
  if(NOT TARGET MMS_LIDAR)   # 이미 추가됐으면 skip
    ...
  endif()
endmacro()
```

여러 앱이 동시에 같은 모듈을 요청해도 한 번만 빌드된다. `include_guard(GLOBAL)`로 setup.cmake 파일 자체도 중복 로드를 막는다.

### 패턴 3 — 자동 의존성 해소

```cmake
macro(_add_package_MMS_LIDAR)
  _add_package_MMS_UTIL()   # LIDAR → UTIL 필요, 알아서 포함
  _add_package_MMS_IO()     # LIDAR → IO 필요, 알아서 포함
  ...
endmacro()
```

앱 CMakeLists.txt에서 `_add_package_MMS_LIDAR()` 한 줄이면 하위 의존성이 자동 해소된다. UTIL, IO까지 자동으로 딸려 온다.

### 패턴 4 — `BASE_DIRECTORY` 단일 루트

```cmake
find_path(BASE_DIRECTORY
  NAMES loggingcoremodule/cmake/setup.cmake
  PATHS ${CMAKE_CURRENT_SOURCE_DIR} ...
)
```

어느 위치에서 cmake를 실행하든 루트를 자동 탐지한다. 하위 모듈 경로를 전부 `${BASE_DIRECTORY}/...`로 표현해 이동·재배치에 유연하다.

### 패턴 5 — 피처 플래그로 선택적 빌드

```cmake
if(NOT CAMERA_MODUEL)  set(CAMERA_MODUEL ON)   endif()
if(NOT XSENS_MODULE)   set(XSENS_MODULE ON)    endif()
if(NOT LICENSE_VIEW)   set(LICENSE_VIEW OFF)   endif()
```

`cmake -DCAMERA_MODUEL=OFF ..` 식으로 모듈 온오프가 가능하다.

---

## STATIC / MODULE / SHARED 라이브러리 역할 분리

### 라이브러리 타입별 개념

**정적 라이브러리(.a)**

```
컴파일:  소스.cpp  →  오브젝트.o
아카이브: .o + .o + .o  →  라이브러리.a
링크:    실행파일 = 내 코드.o + 라이브러리.a (통째로 복사해 넣음)
```

링크 시점에 `.a` 안의 코드가 실행파일 안으로 직접 복사된다. 빌드 후에는 `.a` 파일이 없어도 실행파일이 혼자 작동한다.

**동적 라이브러리(.so)**

```
빌드:  실행파일에는 "libfoo.so를 써라"는 참조만 기록
실행:  OS가 /usr/lib/ 등에서 libfoo.so를 찾아 메모리에 올림
      → 실행파일과 libfoo.so가 메모리에서 연결됨
```

실행 시 `.so` 파일이 반드시 존재해야 한다. 대신 여러 프로세스가 공유하므로 메모리를 아낀다.

**MODULE 타입(.so, dlopen 전용)**

`add_library(이름 MODULE ...)`로 만들면 일반 SHARED와 거의 같은 `.so`가 생성되지만, **링크 시점에 참조하지 않고 런타임에 `dlopen()`으로 직접 열어 쓰는 플러그인 전용** 타입이다.

```cpp
// ModuleLoder.cpp 내부 동작 (개념)
void* handle = dlopen("lib/CameraAravis.so", RTLD_LAZY);
auto createFn = dlsym(handle, "create");
ICamera* cam = ((ICamera*(*)())createFn)();
```

`.so` 파일이 빌드 디렉터리에 없어도 빌드 자체는 성공한다. 런타임에만 필요하다.

---

### 각 컴포넌트에 타입을 선택한 이유

**3rd party(yaml-cpp, jsoncpp, eigen 등) → STATIC**

```cmake
set(BUILD_SHARED_LIBS off)    # 서드파티 전체 → 정적 빌드 강제
set(BUILD_STATIC_LIBS ON)
```

- **버전 충돌 방지**: 배포 환경마다 시스템에 설치된 버전이 다르다. 정적 링크하면 테스트한 버전이 실행파일에 고정된다.
- **배포 단순화**: ModuleLoder.so 안에 흡수되므로 별도 `.so` 파일 없이 배포된다.
- **변경 안 됨**: Eigen, yaml-cpp 같은 라이브러리는 프로젝트 기간 동안 버전을 교체할 일이 없어 동적 링크의 장점이 필요 없다.

**MMS_LIDAR, MMS_GPS 등 센서 모듈 → STATIC**

```cmake
add_library(MMS_LIDAR STATIC ${MODULE_CPP})
```

이 STATIC 라이브러리들은 최종적으로 `ModuleLoder.so` 안으로 링크 인(link-in)된다.

```
MMS_UTIL.a   ─┐
MMS_IO.a     ─┤
MMS_LIDAR.a  ─┼──→  ModuleLoder.so  (이 안에 전부 흡수됨)
MMS_GPS.a    ─┤
MMS_RADAR.a  ─┘
```

센서 드라이버 각각을 `.so`로 만들면 배포물이 늘어나고, `ModuleLoder.so`가 로드될 때 의존 체인이 줄줄이 따라온다. `.a`로 묶어 하나 안에 넣으면 배포·관리가 단순해진다.

**카메라 SDK 래퍼(CameraAravis 등) → MODULE**

```cmake
add_library(CameraAravis MODULE CameraAravis.cpp ../CameraModule.cpp)
```

- **현장 구성이 장비마다 다름**: 어떤 카메라가 붙을지 빌드 시점에 모른다. `dlopen()`으로 런타임에 필요한 카메라 모듈만 로드하면 배포물은 같아도 현장에서 필요한 카메라 `.so`만 챙겨가면 된다.
- **SDK 의존성 격리**: 카메라 SDK 래퍼를 STATIC으로 `ModuleLoder.so`에 넣으면 SDK가 없는 환경에서 `ModuleLoder.so` 로드 자체가 실패한다. MODULE로 분리하면 해당 SDK가 없어도 `ModuleLoder.so`는 정상 로드된다.
- **카메라 드라이버만 교체 가능**: SDK 업데이트나 새 벤더 지원 시 `CameraAravis.so`만 다시 빌드해서 교체하면 된다.

**ModuleLoder → SHARED**

```cmake
add_library(ModuleLoder SHARED ${MODULE_CPP})
```

LoggingTool, ExtractTool, tool_intrinsic, tool_extrinsic, visualizer_cpu — 5개 앱이 전부 센서 드라이버를 사용한다.

```
# STATIC이었다면
LoggingTool 실행파일    = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 1)
ExtractTool 실행파일    = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 2)
tool_intrinsic 실행파일 = 내 코드 + MMS_LIDAR + MMS_GPS + ... (복사본 3)

# SHARED로 만들면
ModuleLoder.so ← 메모리에 한 번만 올라감
    ↑               ↑               ↑
LoggingTool     ExtractTool     tool_intrinsic
```

드라이버 버그를 수정할 때 `ModuleLoder.so`만 교체하면 5개 앱 전체에 반영된다.

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
    ├── CameraFlycapture.so
    ├── CameraPylon.so
    └── libaravis-0.8.so  ← 카메라 벤더 SDK 원본 .so
```

| 컴포넌트 | 타입 | 배포 파일 | 핵심 이유 |
|----------|------|-----------|-----------|
| yaml-cpp, jsoncpp, eigen | STATIC (.a) | 없음 (ModuleLoder.so 안에 흡수) | 버전 고정, 배포 단순화 |
| MMS_LIDAR, MMS_GPS 등 | STATIC (.a) | 없음 (ModuleLoder.so 안에 흡수) | ModuleLoder.so 하나로 배포 |
| CameraAravis 등 | MODULE (.so) | lib/Camera*.so | 런타임 플러그인, SDK 격리 |
| ModuleLoder | SHARED (.so) | ModuleLoder.so | 여러 앱 공유, 단독 교체 가능 |

---

## 핵심 키워드 요약

| 키워드 | 한 줄 설명 |
|--------|-----------|
| `CMAKE_SYSTEM_PROCESSOR` | 아키텍처 자동 감지 (x86_64 / aarch64) |
| `ARCH_FLAGS` | `-m64` or `-march=armv8-a`, 전역 분기 기준 |
| `lib_x64` / `lib_arm` | 벤더 SDK 사전 빌드 바이너리, 아키텍처별 분리 보관 |
| `if(NOT TARGET ...)` | 모듈 중복 빌드 방지 |
| `include_guard(GLOBAL)` | setup.cmake 파일 자체 중복 로드 방지 |
| `add_dependencies()` | 병렬 빌드(`make -j`) 시 링크 순서 보장 |
| `_finalize_target()` | 출력 경로 통일 + install 한 줄 처리 |
| `BASE_DIRECTORY` | 어디서 실행해도 루트 자동 탐지 |
| MODULE 타입 | dlopen 전용 플러그인, 링크 시점 참조 없음 |
