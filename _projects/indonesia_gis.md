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
- 데이터 처리 시간 36시간 → 약 17시간으로 단축 (70MB/min 처리 속도 기준 약 2.1배 성능 개선)
- 연구팀의 Python 기반 고복잡도 로직을 상용 배포용 C# 네이티브 코드로 독자 재구현하여 서비스 가용성 및 배포 안정성 확보
- 양 팀(연구/개발) 간의 성능 및 속도 Trade-off 정밀 검증과 기술적 간극 해소를 통해 최적화된 엔진을 상용 파이프라인에 최종 반영
- **COPC 마샬링 및 실시간 가시화** 구현으로 대용량 점군 데이터 기반 모델링 정밀 검증 환경 구축

##  상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 보안 고도화 및 핵심 알고리즘 상용 패키징
- **문제 상황/목표**: 15개 연구 모듈의 소스코드가 배포 시 역컴파일러에 의해 노출될 위험(IP 유출)을 식별, 이를 방지할 상용 수준의 보안 아키텍처 및 통합 배포 체계 구축이 필요함.
- **해결 방안 (Action)**:
  - **자산 선별적 보안(Selective Hardening)**: 알고리즘 로직 13종을 **Cython 바이너리($\.pyd$)**로 변환하여 소스 레벨 유출을 원천 차단하고, PyInstaller를 통해 독립 실행형($\.exe$) 환경 구축.
  - **프레임워크 가용성 확보**: PyTorch 등 외부 라이브러리 의존성으로 인해 컴파일이 제한되는 모듈은 안정성을 우선하여 패키징 구조를 최적화하는 이원화 전략 채택.
- **결과 (Result)**: 컴파일 적용 모듈(13종) 기준 소스코드 복구율 0% 및 전체 패키지 무단 변조 리스크 최소화. 15개 이기종 모듈을 단일 빌드 파이프라인으로 통합하여 별도 환경 구축 없이 실행 가능한 상용 수준의 Standalone 패키지 배포 성공.

### 2. 고속 DTM 생성 엔진 및 COPC 기반 실시간 점군 가시화·검증 통합
- **문제 상황/목표**: 기존 연구팀 Python 기반 로직(PDAL CSF)의 성능 한계(36시간 소요)와 연구 환경 편중 코드 구조로 인한 상용 배포 어려움. 자동 생성 모델링의 실시간 검증 환경 부재 및 대용량 .las 파일에서 특정 공간 영역만 즉시 로드하는 기능 필요.
- **해결 방안 (Action)**: 핵심 DTM 로직을 C# 네이티브로 독자 재설계하고, C++ 기반 [COPC 라이브러리](https://github.com/RockRobotic/copc-lib)를 **직접 C# 마샬링**으로 통합. 원본 .las를 공간 정렬된 .laz로 변환 후 Bounding Box 입력 시 해당 영역만 실시간 스트리밍 로드. 연구팀↔개발팀 간 **정밀 성능 비교 및 Trade-off 분석**을 주도하여 최적화된 엔진 확정.
- **결과 (Result)**: 69GB 처리 시간 36시간 → 약 17시간 단축(70MB/min). 기가바이트 단위 데이터셋에서 Bounding Box 기반 실시간 점군 로드 및 정밀 검증 가능한 상용 통합 엔진 완성.

<div style="display: flex; gap: 10px; justify-content: space-between; margin-top: 20px;">
  <div style="flex: 1; text-align: center;">
    <img src="/assets/images/projects/indonesia_gis/ground_extraction_algorithm.png" style="width: 50%; border-radius: 4px; box-shadow: 0 2px 5px rgba(0,0,0,0.1);">
    <p style="font-size: 0.75em; color: #666; margin-top: 5px;"><i>Voxel 필터링 과정</i></p>
  </div>
  <div style="flex: 1; text-align: center;">
    <img src="/assets/images/projects/indonesia_gis/dtm_csharp.png" style="width: 50%; border-radius: 4px; box-shadow: 0 2px 5px rgba(0,0,0,0.1);">
    <p style="font-size: 0.75em; color: #666; margin-top: 5px;"><i>C# 엔진 생성 결과</i></p>
  </div>
  <div style="flex: 1; text-align: center;">
    <img src="/assets/images/projects/indonesia_gis/ground_surface_detail.png" style="width: 50%; border-radius: 4px; box-shadow: 0 2px 5px rgba(0,0,0,0.1);">
    <p style="font-size: 0.75em; color: #666; margin-top: 5px;"><i>최종 지면 추출 디테일</i></p>
  </div>
</div>

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
