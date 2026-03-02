---
layout: project
title: "Replica Program (Mobile Mapping System) Tool Development"
period: "2022.03 ~ Present"
category: "Mobile Mapping System"
tech: "C++, Camera SDK (Hikvision, MIPI, GMSL), NVIDIA Orin, IPC"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 5
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Replica Program - MMS Core Tool Development (사내 핵심 MMS 센서 인터페이스 및 로깅 툴)
- **기간**: 2022.03 ~ Present
- **역할**: Team Member (신규 카메라 인터페이스 개발, ARM 시스템 이식, DMI 데이터 파싱 및 로깅 로직 구현)
- **기술 (Tech Stack)**: C++, Hikvision SDK, E-con MIPI/GMSL SDK, LV125 Protocol, NVIDIA Orin

사내 핵심 프로그램인 모바일 매핑 시스템(MMS)의 센서 데이터 수집, 로깅, 추출을 담당하는 기반 시스템의 고도화 및 하드웨어 확장.

## 주요 성과 (Key Achievements)
- 신규 머신비전 카메라(Hikvision, MIPI, GMSL) 인터페이스 추가 및 시스템 안정화 성공
- x86 기반 시스템에서 ARM(NVIDIA Orin) 보드로의 시스템 확장 및 이식 주도
- **확장형 IPC 구조 설계 기여**를 통한 전사 툴 체인(Calibration, Mapping) 연동 실현
- Mobiltech Logging Suite(MLS) **공인 GS(Good Software) 인증 획득** 기여

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 신규 카메라 인터페이스 개발 및 MMS 특화 데이터(DMI) 확보
- **문제 상황/목표**: 고정밀 매핑 수요에 따른 신규 센서(Hik, E-con MIPI, E-con GMSL) 연동과 함께, 기존 시스템에서 부재했던 정밀 거리/시간 정보(DMI)의 실시간 확보가 필요.
- **해결 방안 (Action)**: Hikvision 및 MIPI/GMSL 카메라 인터페이스를 신규 구현. 특히 **LV125 프로토콜을 정밀 분석하여 DMI(Distance Measurement Instrument) 데이터를 직접 파싱**하고, 이를 전체 시스템 파이프라인에 통합. 또한 ARM(NVIDIA Orin) 환경 최적화 수행.
- **결과 (Result)**: 기존에 없던 정밀 거리 데이터를 백엔드 시스템에 공급함으로써 **'10m 단위 Local SLAM 맵 제작'** 및 **'DMI 기반 정시 상황 판단 로깅'** 기능을 실현. 저전력 ARM 환경에서도 고해상도 이미지와 MMS 특화 데이터를 동시에 안정적으로 수집하는 구조 확립.

### 2. LiDAR 센서 인터페이스 업데이트
- **문제 상황/목표**: 기존 LiDAR SDK의 노후화로 신규 Hesai LiDAR SDK로의 업데이트가 필요.
- **해결 방안 (Action)**: 신규 Hesai LiDAR SDK로 업데이트하여 기존 데이터 로깅 파이프라인에 연동.
- **결과 (Result)**: 기존 파이프라인에 큰 에러 없이 안정적으로 통합 완료.

### 3. MMS 추출 툴 고도화 및 SLAM 엔진 모듈화
- **문제 상황/목표**: 대규모 센서 데이터 추출 과정에서 10m 단위의 고정밀 Local Map 생성이 요구됨. 그러나 기존 SLAM 로직은 ROS(Robot Operating System) 환경 및 특정 외부 라이브러리에 강하게 종속되어 있어 상용 추출 툴에 직접 통합하기 어려운 구조적 한계 존재.
- **해결 방안 (Action)**: ROS 기반의 SLAM 코드를 분석하여 의존성을 제거하고, 핵심 알고리즘을 독립적인 C++ 라이브러리로 **모듈화(Modularization)**. 이를 사내 데이터 추출 툴의 신규 플러그인 기능으로 이식하여 하이브리드 형태의 처리 파이프라인 구축.
- **결과 (Result)**: 별도의 외부 툴 없이도 추출 과정에서 즉시 '10m 단위 Local SLAM 맵' 제작이 가능해짐에 따라, 제품의 기능적 완성도를 높이고 데이터 후처리 워크플로우를 획기적으로 단축.

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
