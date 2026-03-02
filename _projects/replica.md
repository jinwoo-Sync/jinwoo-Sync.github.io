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

### 3. MMS 상용 소프트웨어 판매를 위한 SLAM 엔진 통합 시도
- **문제 상황/목표**: MMS 소프트웨어의 상용 판매 및 고객사 납품 과정에서 '10m 단위 고정밀 Local Map' 생성 기능이 핵심 제품 요구사항으로 제기됨. 그러나 기존 SLAM 로직은 ROS(Robot Operating System) 환경과 방대한 외부 라이브러리에 파편화되어 있어, 단일 실행 파일 형태의 상용 툴로 통합하기 어려운 구조적 한계 존재.
- **해결 방안 (Action)**: 고객사 납품 일정을 준수하기 위해, ROS 기반 SLAM 코드의 핵심 시퀀스를 분석하여 상용 추출 프로그램 내부에서 직접 호출 가능한 플러그인 형태로 통합을 시도. 촉박한 일정상 불필요한 라이브러리 의존성을 완벽히 제거하는 리팩토링에는 한계가 있었으나, 기능적 요구사항을 충족하기 위한 실행 환경 단일화에 집중.
- **결과 (Result)**: 기술적 의존성이 일부 잔류하는 제약 속에서도 제품 판매에 필수적인 '10m 단위 Local SLAM 맵' 제작 파이프라인을 최초로 구축하여 성공적인 제품 납품에 기여함.

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
