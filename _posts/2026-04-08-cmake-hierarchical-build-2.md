---
layout: post
title: "CMake 계층적 빌드 체계 — 실제 CMakeLists.txt 연결 구조"
tags: [CMake, C++, Build System]
---

## Layer 0 — 루트 CMakeLists.txt (전체 빌드 진입점)

```cmake
# BASE_DIRECTORY 자동 탐지 (어디서 cmake 실행해도 루트 찾아옴)
find_path(BASE_DIRECTORY NAMES loggingcoremodule/cmake/setup.cmake ...)

# 서드파티 로드
include(${BASE_DIRECTORY}/3rd/cmake/setup.cmake)
_add_package_eigen3()
_add_package_yaml()

# 센서 매크로 로드 + CoreModule 전체 빌드 등록 (이 한 줄로 Layer 1~2 전부 끌려옴)
include(${BASE_DIRECTORY}/loggingcoremodule/cmake/setup.cmake)
_add_package_MMS_MODULELODER()

# 앱 등록
add_subdirectory(${BASE_DIRECTORY}/replicalogging/src)
include(${BASE_DIRECTORY}/tool_intrinsic/cmake/setup.cmake)
include(${BASE_DIRECTORY}/tool_extrinsic/cmake/setup.cmake)
```

---

## Layer 2 — CoreModule/CMakeLists.txt (ModuleLoder.so 생성)

`_add_package_MMS_MODULELODER()`가 실제로 부르는 파일. 센서 `.a`들을 전부 흡수해 `ModuleLoder.so`를 만든다.

```cmake
project(ModuleLoder)

include(${BASE_DIRECTORY}/loggingcoremodule/cmake/setup.cmake)

# 센서 매크로 호출 → 각 STATIC .a 빌드 등록
_add_package_MMS_UTIL()
_add_package_MMS_IO()
_add_package_MMS_LIDAR()
_add_package_MMS_GPS()
_add_package_MMS_RADAR()
_add_package_MMS_TRIGGER()
_add_package_MMS_COMMUNICATION()

if(${CAMERA_MODUEL})
  add_subdirectory(src/camera)   # 아키텍처별 카메라 MODULE 분기
endif()

# STATIC .a들을 전부 링크 → ModuleLoder.so 안으로 흡수
add_library(ModuleLoder SHARED src/ModuleLoder.cpp src/camera/CameraModule.cpp)
target_link_libraries(ModuleLoder
    dl pthread ${OpenCV_LIBS}
    MMS_UTIL MMS_IO MMS_LIDAR MMS_GPS MMS_RADAR MMS_COMMUNICATION MMS_TRIGGER
)
```

---

## Layer 1 — 센서 모듈 CMakeLists.txt (MMS_LIDAR 예시)

매크로가 등록하는 말단 파일. 단순하다.

```cmake
project(MMS_LIDAR)

add_library(MMS_LIDAR STATIC   # STATIC → 나중에 ModuleLoder.so 안으로 흡수됨
    LidarModule.cpp LidarOuster.cpp LidarVelodyne.cpp LidarHesai.cpp ...
)
target_link_libraries(MMS_LIDAR pthread yaml-cpp MMS_UTIL MMS_TRIGGER)
```

---

## Layer 3 — 앱 CMakeLists.txt (전체/단독 빌드 분기)

앱은 `ModuleLoder` 하나만 링크하면 끝이다. 내부 `.a`들을 알 필요 없다.

```cmake
# replicalogging/src/CMakeLists.txt

# REPLICA_ALL 미정의 = 단독 빌드 → 자체적으로 setup.cmake 로드
# REPLICA_ALL 정의됨 = 전체 빌드 → 루트에서 이미 로드됐으니 skip
if(NOT DEFINED REPLICA_ALL)
  find_path(BASE_DIRECTORY ...)
  include(${BASE_DIRECTORY}/loggingcoremodule/cmake/setup.cmake)
  _add_package_MMS_MODULELODER()
endif()

set(ALL_LIBS ${OpenCV_LIBS} ${QT_LIBS} ModuleLoder ImageView GUI_common)

add_subdirectory(LoggingTool)
add_subdirectory(ExtractTool)
add_dependencies(LoggingTool ModuleLoder)   # ModuleLoder 완성 후 링크
add_dependencies(ExtractTool ModuleLoder)
```

```cmake
# LoggingTool/CMakeLists.txt — 실행파일. ModuleLoder 하나만 링크
add_executable(LoggingTool main.cpp mainwindow.cpp ...)
target_link_libraries(LoggingTool ${ALL_LIBS})
```

---

## 파일별 역할 한 줄 요약

| 파일 | 역할 |
|------|------|
| 루트 `CMakeLists.txt` | 전체 빌드 진입점. setup.cmake 로드 + 앱 등록 |
| `CoreModule/CMakeLists.txt` | 센서 매크로 호출 + `ModuleLoder.so` 생성 |
| `src/lidar/CMakeLists.txt` 등 | `MMS_LIDAR.a` STATIC 생성 |
| `src/camera/CMakeLists.txt` | 아키텍처별 카메라 MODULE 분기 |
| `replicalogging/src/CMakeLists.txt` | 전체/단독 빌드 분기 + `ModuleLoder` 링크 |
