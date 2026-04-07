---
layout: post
title: "CMake setup.cmake 매크로 기반 계층적 빌드 체계"
tags: [CMake, C++, Build System, Cross Compilation, ARM, x86]
---

MMS(Mobile Mapping System) 프로젝트에서 x86·ARM 동시 배포, 6개 앱 독립 빌드를 위해 `setup.cmake` 매크로 기반 계층적 빌드 체계를 운영했다.

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
# 1단계: CPU 감지
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
    set(ARCH_FLAGS "-march=armv8-a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH_FLAGS "-m64")
endif()

# 2단계: 빌드할 SDK 선택 (ARM이면 Flycapture·Spinnaker·Vimba 아예 제외)
if(ARCH_FLAGS MATCHES "-m64")
    add_subdirectory(flycapture)
    add_subdirectory(spinnaker)
    add_subdirectory(aravis)
    ...
elseif(ARCH_FLAGS MATCHES "arm|aarch64")
    add_subdirectory(aravis)       # ARM 바이너리 있는 것만
    add_subdirectory(pylon)
    add_subdirectory(econ_mipi)
endif()

# 3단계: 올바른 바이너리 경로 선택 + 자동 복사
if(ARCH_FLAGS MATCHES "-m64")
    target_link_libraries(CameraAravis .../lib_x64/libaravis-0.8.so ...)
    file(COPY .../lib_x64/libaravis-0.8.so DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
elseif(ARCH_FLAGS MATCHES "arm|aarch64")
    target_link_libraries(CameraAravis .../lib_arm/libaravis-0.8.so ...)
    file(COPY .../lib_arm/libaravis-0.8.so DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib)
endif()
```

`ARCH_FLAGS` 변수 하나가 2·3단계를 모두 제어한다. cmake 실행 머신 환경만으로 자동 결정되므로 별도 옵션이 없다.

---

## setup.cmake 핵심 패턴

### 센서 모듈 매크로 구조 (5줄 패턴)

```cmake
macro(_add_package_MMS_LIDAR)
  include_directories(...)              # 헤더 경로 등록
  if(NOT TARGET MMS_LIDAR)             # 중복 방지 가드
    _add_package_MMS_UTIL()            # 의존 모듈 먼저 재귀 호출
    _add_package_MMS_IO()
    add_subdirectory(src/lidar bin/lidar)
    add_dependencies(MMS_LIDAR MMS_IO MMS_UTIL)  # make -j 순서 보장
  endif()
endmacro()
```

앱 CMakeLists.txt에서 `_add_package_MMS_LIDAR()` 한 줄이면 UTIL·IO까지 자동 해소된다.

| 메커니즘 | 역할 |
|----------|------|
| 매크로 안 재귀 호출 | cmake 그래프 등록 순서 보장 |
| `add_dependencies()` | `make -j` 병렬 빌드 시 링크 순서 보장 |
| `if(NOT TARGET ...)` | 여러 앱이 같은 모듈을 요청해도 한 번만 빌드 |

### 전역 설정 (setup.cmake 상단)

```cmake
include_guard(GLOBAL)               # 파일 자체 중복 로드 방지
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-fPIC -std=c++14 -fopenmp")
set(OUTPUT_PATH ${BASE_DIRECTORY}/bin_${ARCH})   # bin_x64 or bin_x86

macro(_finalize_target _PROJNAME)
  set_target_properties(${_PROJNAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_PATH}/")
  install(TARGETS ${_PROJNAME} DESTINATION bin_${ARCH})
endmacro()
```

`_finalize_target(앱이름)` 한 줄로 경로·설치가 끝난다. 배포할 때 `bin_x64/` 하나만 가져가면 된다.

---

## STATIC / MODULE / SHARED 선택 근거

| 컴포넌트 | 타입 | 이유 |
|----------|------|------|
| yaml-cpp, eigen 등 서드파티 | STATIC | 버전 고정, ModuleLoder.so 안에 흡수해 배포 단순화 |
| MMS_LIDAR, MMS_GPS 등 센서 모듈 | STATIC | ModuleLoder.so 하나로 묶어 배포, 수정 빈도 낮음 |
| CameraAravis 등 카메라 래퍼 | MODULE | dlopen 플러그인: 장비마다 카메라 구성이 달라 런타임 선택, 벤더 SDK 격리 |
| ModuleLoder | SHARED | 5개 앱이 드라이버 공유, 버그 수정 시 .so 하나만 교체 |

```
# 최종 배포물
bin_x64/
├── LoggingTool / ExtractTool / tool_intrinsic ...  (실행파일)
├── ModuleLoder.so   ← MMS_LIDAR.a, MMS_GPS.a ... 전부 흡수
└── lib/
    ├── CameraAravis.so    ← dlopen으로 런타임 로드
    ├── CameraPylon.so
    └── libaravis-0.8.so   ← 벤더 SDK 원본
```

카메라 SDK를 MODULE로 분리했기 때문에, 해당 SDK가 없는 장비에서도 `ModuleLoder.so` 로드 자체는 정상 동작한다. 없는 카메라 모듈은 그냥 로드하지 않으면 된다.
