---
layout: post
title: "x86 → ARM(Jetson Orin) 이식 — 플랫폼 의존성 이슈 분석 및 해결"
tags: [C++, CMake, ARM, Jetson, Cross Compilation, GPIO, ELF]
---

MMS 소프트웨어를 x86에서 Jetson Orin Nano(aarch64)로 이식하며 마주친 플랫폼 의존성 이슈 두 가지를 정리한다.

---

## 케이스 1 — GPIO 라이브러리 플랫폼 의존성

기존 트리거는 SC400 보드(USB 시리얼)만 지원했는데, Jetson에서는 GPIO 핀을 직접 써야 했다. 문제는 `libJetsonGPIO.so`가 Jetson에만 존재해서, 그냥 링크하면 x86 빌드가 깨진다는 것이었다.

**해결**: cmake에서 라이브러리 존재 여부로 플래그를 자동 결정하고, 소스에서 `#ifdef`로 분리.

```cmake
if(EXISTS "/usr/local/lib/libJetsonGPIO.so")
    target_compile_definitions(${PROJECT_NAME} PUBLIC EN_JECSONS=1)
    target_link_libraries(${PROJECT_NAME} PUBLIC /usr/local/lib/libJetsonGPIO.so)
endif()
```

```cpp
#ifdef EN_JECSONS
#include <JetsonGPIO.h>
void OrinNano::start() { GPIO::setmode(GPIO::BCM); ... }
#endif
```

런타임 구현체 선택은 팩토리 패턴으로 처리해 상위 레이어는 플랫폼을 몰라도 된다.

---

## 케이스 2 — GPS Xsens SDK ELF Format Error

Xsens SDK가 `lib_x64` / `lib_arm` 바이너리를 별도 제공하는데, 기존 CMake가 `lib_x64`를 하드코딩했다. Jetson 빌드 시 오류:

```
wrong ELF class: ELFCLASS64 (x86)   # x86 바이너리를 aarch64에서 로드 시도
```

**해결**: `ARCH_FLAGS`(= `CMAKE_SYSTEM_PROCESSOR` 감지 변수)로 경로 분기.

```cmake
if(ARCH_FLAGS MATCHES "-m64")
    find_library(XSENS_CONT xscontroller HINTS xsens/lib_x64)
    file(COPY "xsens/lib_x64/libxscontroller.so" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
elseif(ARCH_FLAGS MATCHES "arm|aarch64")
    find_library(XSENS_CONT xscontroller HINTS xsens/lib_arm)
    file(COPY "xsens/lib_arm/libxscontroller.so" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()
```

