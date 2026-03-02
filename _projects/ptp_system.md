---
layout: project
title: "Data Synchronization (Mapping System)"
period: "2023.02 ~ Present"
category: "Time Synchronization"
tech: "C++, PTP, GPRMC, GPIO"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 6
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Data Synchronization - MMS 시간 동기화 시스템 고도화
- **기간**: 2023.02 ~ Present
- **역할**: Team Member (PTP 시스템 구축, GPRMC 파싱 및 데이터 검증, 동기화 보드 업그레이드 기여)
- **기술 (Tech Stack)**: C++, PTP (SW/HW), GPRMC, GPIO

사내 MMS 시스템의 핵심인 시간 동기화 체계를 구형 동기화 보드에서 신형 보드로 업그레이드하고, LiDAR에서 직접 RMC 신호를 받는 방식에서 SW/HW PTP 기반의 확장된 시스템으로 전환하는 프로젝트.

## 주요 성과 (Key Achievements)
- SW PTP를 통한 **us(마이크로초) 단위** 시간 동기화 시스템 구축
- **HW PTP 아키텍처 구축 기여 (ns 단위 동기화)**: Raspberry Pi CM4와 HW Timestamp 기능을 지원하는 MicroTik 스위치 허브를 연동. RMC와 PPS 신호를 CM4에 연결하여 GPS Master 역할을 수행하게 함으로써, 실제 센서들이 PTP로 정밀한 시간 동기화를 받을 수 있도록 중간 교두보 역할 확립에 참여
- SC-400 보드의 가상 GPS 신호 생성 기능 구현으로 GPS 음영 구간 데이터 손실 문제 해결
- 2025년부터 해당 동기화 제품(SC-400) **판매 진행 중**

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. PTP 시간 동기화 시스템 구축
- **문제 상황/목표**: 기존 시스템은 LiDAR 하드웨어에서 직접 RMC 신호를 받을 수 있는 특정 센서에서만 시간 동기화가 가능한 제약이 존재.
- **해결 방안 (Action)**: SW PTP를 통해 초기 소프트웨어 타임스탬프 기반 PTP 시스템을 구축(us 단위 동기화). 고도화를 위해 팀원으로서 **Raspberry Pi CM4와 MicroTik 스위치 허브** 기반의 HW PTP 시스템 설계 및 교두보 환경 구축에 참여. CM4가 RMC/PPS를 받아 GPS Master로서 기능하며 이기종 센서들에게 PTP를 분배하는 세팅을 지원하고, GPS time이 분기마다 변경되는 이슈를 확인 및 수정.
- **결과 (Result)**: 센서 종류에 제약 없이 us/ns 수준의 범용적인 시간 동기화 체계 확립.

### 2. GPS 음영 구간 대응을 위한 가상 GPS 신호 생성 (SC-400)
- **문제 상황/목표**: 터널이나 지하 등 GPS 신호가 단절되는 구간에서 센서들의 시간 동기화가 풀리고 데이터가 유실되는 문제 해결 필요.
- **해결 방안 (Action)**: NVIDIA Orin 환경에서 선행 구현했던 **가상 GPS 신호(PPS + NMEA) 생성 로직을 SC-400 보드용으로 이식 및 최적화**. 신호 생성의 핵심인 GPRMC 문장 파싱 및 시스템 연동 로직을 담당.
- **결과 (Result)**: GPS 신호가 끊기더라도(Unlock) 시스템의 **Local Time을 마스터로 하여 가상의 GPS 신호(PPS + NMEA)를 생성하는 기능 구현에 참여(STM32 기반)**. 다만, GPS 복구 시 시간이 동기화되는 찰나의 시점에서는 미세한 데이터 손실이 발생할 수 있는 기술적 특성을 확인 및 관리함. 해당 모듈은 2025년부터 상용화되어 SC-400 제품으로 판매 중.

### 3. 트리거 타임 성능 평가
- **문제 상황/목표**: 동기화 시스템의 정확도를 정량적으로 검증할 필요.
- **해결 방안 (Action)**: 트리거 타임의 정확도 및 안정성을 측정하는 성능 평가 체계 수립 및 테스트 수행.
- **결과 (Result)**: 동기화 시스템의 성능을 정량적으로 검증하여 제품 신뢰성 확보.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/ptp_system/sc300_board.png)

![](/assets/images/projects/ptp_system/ptp_architecture.png)

![](/assets/images/projects/ptp_system/sc400_gprmc.png)

![](/assets/images/projects/ptp_system/sc400_system.png)

**트리거 타임 성능 평가**

![](/assets/images/projects/ptp_system/trigger_performance.png)