---
layout: project
title: "Replica Program (Mobile Mapping System) Tool Development"
period: "2022.03 ~ Present"
category: "Mobile Mapping System"
tech: "C++, LiDAR/Camera SDK, NVIDIA Orin"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 5
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Replica Program - MMS Core Tool Development (사내 핵심 MMS 센서 인터페이스 및 로깅 툴)
- **기간**: 2022.03 ~ Present
- **역할**: Team Member (머신비전 카메라 인터페이스 개발, 로깅/추출 툴 유지보수, 하드웨어 제어 모듈 구현)
- **기술 (Tech Stack)**: C++, LiDAR/Camera SDK (Hikvision, Vimba, Pylon, Ozray, Flycapture), NVIDIA Orin

사내 핵심 프로그램인 모바일 매핑 시스템(MMS)의 센서 데이터 수집, 로깅, 추출을 담당하는 기반 시스템의 개발 및 유지보수.

## 주요 성과 (Key Achievements)
- 다양한 머신비전 카메라 SDK(Hikvision, Vimba, Pylon, Ozray, Flycapture) 기반 데이터 연동 모듈 통합
- 단일 카메라/라이다 IPC 공유 구조에서 **다중 카메라 및 다중 라이다 IPC 확장** 달성
- Mobiltech Logging Suite(MLS) **공인 GS(Good Software) 인증 획득**

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 머신비전 카메라 센서 인터페이스 개발
- **문제 상황/목표**: MMS 시스템에 다양한 카메라(Hik, E-con MIPI, E-con GMSL)를 안정적으로 연동해야 하며, 고해상도 이미지 데이터의 캡처/전송/저장 파이프라인이 필요.
- **해결 방안 (Action)**: Hikvision, Vimba, Pylon, Ozray, Flycapture 등 다양한 머신비전 카메라 SDK 기반의 데이터 연동 모듈을 개발. 고해상도 이미지 데이터의 안정적인 캡처, 전송 및 저장 시스템 구축에 기여.
- **결과 (Result)**: 카메라 종류에 관계없이 통합된 인터페이스로 센서 데이터를 수집할 수 있는 환경 구축.

### 2. LiDAR 센서 인터페이스 업데이트
- **문제 상황/목표**: 기존 LiDAR SDK의 노후화로 신규 Hesai LiDAR SDK로의 업데이트가 필요.
- **해결 방안 (Action)**: 신규 Hesai LiDAR SDK로 업데이트하여 기존 데이터 로깅 파이프라인에 연동.
- **결과 (Result)**: 기존 파이프라인에 큰 에러 없이 안정적으로 통합 완료.

### 3. MMS 로깅 및 추출 툴 개발/유지보수
- **문제 상황/목표**: 대량의 센서 데이터를 효율적으로 로깅(Logging)하고, 조건별로 필요한 데이터를 추출(Extraction)하는 안정적인 툴이 필요.
- **해결 방안 (Action)**: 로깅/추출 툴의 성능 최적화 및 안정성 확보를 위한 지속적인 코드 리팩토링과 버그 수정. 데이터 추출 툴과 Intrinsic Tool / MMS Mapping Tool 간의 IPC 동기화 안정화 후 프로그램 파이프라인 구축.
- **결과 (Result)**: 단일 카메라/라이다 IPC 공유에서 다중 카메라 및 다중 라이다 IPC로 확장 달성.

### 4. 하드웨어 트리거 제어 모듈 구현
- **문제 상황/목표**: 카메라 및 LiDAR 등 여러 센서 간의 정확한 시각 동기화를 위한 하드웨어 제어 시스템이 필요.
- **해결 방안 (Action)**: NVIDIA Orin Nano의 GPIO를 활용한 트리거 보드 인터페이스를 개발하여 센서 간 동기화 및 제어 시스템 구축.
- **결과 (Result)**: 다중 센서 환경에서 정확한 시각 동기화 기반의 데이터 수집 체계 확립.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/replica/system_diagram.png)

![](/assets/images/projects/replica/sensor_interface.png)

![](/assets/images/projects/replica/mapping_result.png)
*저가 센서 xsens 기반 MMS 매핑 결과물*

**공인 인증**

- **Mobiltech Logging Suite(MLS)**: 공인 GS(Good Software) 인증 획득

![](/assets/images/projects/replica/공인_GS인증.png)