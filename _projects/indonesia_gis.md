---
layout: project
title: "대규모 도시급 항공 GIS 상용 툴 개발"
period: "2025.07 ~ Present"
category: "GIS & Data Pipeline"
tech: "C#(.NET), Python, PyTorch, C++ Interop(Marshalling), WPF, PDAL, GDAL, SAM3, PyInstaller, Cython"
role: "Team Member"
company: "Mobiltech (Indonesia Project)"
order: 1
---

## 프로젝트 개요 (Overview)
- **프로젝트명**: 대규모 도시급 항공 GIS 상용 툴 개발 (Indonesia Project)
- **기간**: 2025.07 ~ Present
- **역할**: Team Member (C# 파이프라인 설계 및 최적화, 딥러닝 모듈 통합)
- **기술 (Tech Stack)**: C#(.NET), Python, PyTorch, C++ Interop(Marshalling), WPF, PDAL, GDAL, SAM3, PyInstaller, Cython

##  주요 성과 (Key Achievements)
- 연구팀의 Python 기반 고복잡도 로직을 상용 배포용 C# 네이티브 코드로 독자 재구현하여 서비스 가용성 및 배포 안정성 확보
- **C++ COPC 라이브러리를 C# 마샬링으로 직접 통합**하여 단일 파이프라인 내 대용량 점군 실시간 로드 환경 구축

##  상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 보안 고도화 및 핵심 알고리즘 상용 패키징
- **문제 상황/목표**: 15개 연구 모듈의 소스코드가 배포 시 역컴파일러에 의해 노출될 위험(IP 유출)을 식별, 이를 방지할 상용 수준의 보안 아키텍처 및 통합 배포 체계 구축이 필요함.
- **해결 방안 (Action)**:
  - **자산 선별적 보안(Selective Hardening)**: 알고리즘 로직 13종을 **Cython 바이너리($\.pyd$)**로 변환하여 소스 레벨 유출을 원천 차단하고, PyInstaller를 통해 독립 실행형($\.exe$) 환경 구축.
  - **프레임워크 가용성 확보**: PyTorch 등 외부 라이브러리 의존성으로 인해 컴파일이 제한되는 모듈은 안정성을 우선하여 패키징 구조를 최적화하는 이원화 전략 채택.
- **결과 (Result)**: 컴파일 적용 모듈(13종) 기준 소스코드 복구율 0% 및 전체 패키지 무단 변조 리스크 최소화. 15개 이기종 모듈을 단일 빌드 파이프라인으로 통합하여 별도 환경 구축 없이 실행 가능한 상용 수준의 Standalone 패키지 배포 성공.

### 2. C++ 기반 COPC 라이브러리 C# 마샬링 통합
- **문제 상황/목표**: C++로 작성된 COPC 라이브러리를 C# 상용 파이프라인에서 직접 호출해야 하는 상황.
- **해결 방안 (Action)**: `[DllImport]` 정적 바인딩 대신 `NativeLibrary.Load()`로 런타임에 DLL을 동적 로드하고, `NativeLibrary.GetExport()`로 함수 포인터를 획득한 뒤 `Marshal.GetDelegateForFunctionPointer<T>()`로 C# 델리게이트로 변환하는 방식 적용. C++ 측이 할당한 포인터 배열(`IntPtr`)은 `Marshal.Copy()`로 C# 관리 배열에 복사 후 `FreePointArrays()`로 명시적 해제하여 메모리 누수 방지.
- **결과 (Result)**: 별도 프로세스 없이 C# 단일 파이프라인 내에서 COPC 기능 직접 호출 가능.

> **[참고] 마샬링이란?**
> C#(관리 코드)과 C++(비관리 코드) 사이에서 데이터 타입과 메모리 레이아웃을 서로 이해할 수 있는 형태로 변환하는 작업. C++은 원시 포인터와 수동 메모리 관리를 사용하고 C#은 GC 기반 관리 메모리를 사용하기 때문에, 두 언어 간 함수 호출 시 이 변환 과정이 필요함.

---

## 관련 기술 블로그
- **[Nuitka를 이용한 Python 프로젝트 보안 배포](https://jinwoo-sync.github.io/2026/03/01/nuitka-python-deployment.html)**: 상용 소프트웨어 배포를 위한 소스코드 보안 및 실행 환경 최적화 과정 정리

---

### 부록: 시스템 구동 시연

<div style="display: flex; gap: 20px; justify-content: center; margin: 20px 0; flex-wrap: wrap;">
  <div style="width: 48%; text-align: center;">
    <video width="100%" height="auto" autoplay loop muted playsinline style="border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);">
      <source src="/assets/videos/projects/indonesia_gis/gis_tool_demo.webm" type="video/webm">
      Your browser does not support the video tag.
    </video>
    <p style="color: #666; font-size: 0.9em; margin-top: 8px;"><i>도시급 항공 데이터 처리 상용 GIS 툴 구동 시연</i></p>
  </div>
  <div style="width: 48%; text-align: center;">
    <video width="100%" height="auto" autoplay loop muted playsinline style="border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);">
      <source src="/assets/videos/projects/indonesia_gis/COPC_마살량.webm" type="video/webm">
      <source src="/assets/videos/projects/indonesia_gis/COPC_마살량.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
    <p style="color: #666; font-size: 0.9em; margin-top: 8px;"><i>COPC 마샬링 기반 실시간 점군 스트리밍 시연 — Bounding Box 입력 시 해당 영역 즉시 로드</i></p>
  </div>
</div>
