---
layout: project
title: "Data Synchronization (Mapping System)"
period: "2024.09 ~ 2025.01"
category: "Time Synchronization"
tech: "C++, PTP, GPRMC, GPIO"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 6
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Data Synchronization - MMS 시간 동기화 시스템 고도화
- **기간**: 2024.09 ~ 2025.01
- **역할**: Team Member (PTP 시스템 구축, GPRMC 파싱 및 데이터 검증, 동기화 보드 업그레이드 기여)
- **기술 (Tech Stack)**: C++, PTP (SW/HW), GPRMC, GPIO

사내 MMS 시스템의 핵심인 시간 동기화 체계를 구형 동기화 보드에서 신형 보드로 업그레이드하고, LiDAR에서 직접 RMC 신호를 받는 방식에서 SW/HW PTP 기반의 확장된 시스템으로 전환하는 프로젝트.

## 주요 성과 (Key Achievements)
- SW PTP를 통한 **us(마이크로초) 단위** 시간 동기화 시스템 구축
- **HW PTP 아키텍처 구축 기여 (ns 단위 동기화)**: Raspberry Pi CM4 + `ts2phc` / `phc2sys` / `ptp4l` 파이프라인으로 GPS 기준 시각을 CM4 내부까지 동기화한 뒤, MicroTik 스위치(BC)를 통해 차량 전체 센서에 PTP를 분배하는 시스템 초기 구축에 참여
- **Jetson Orin(ARM) 환경에서 GPIO를 활용한 트리거 타이밍 시스템 구축**

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. PTP 시간 동기화 시스템 구축
- **문제 상황/목표**: 기존 시스템은 LiDAR 하드웨어에서 직접 RMC 신호를 받을 수 있는 특정 센서에서만 시간 동기화가 가능한 제약이 존재.
- **해결 방안 (Action)**: SW PTP를 통해 초기 소프트웨어 타임스탬프 기반 PTP 시스템을 구축(us 단위 동기화). 고도화를 위해 팀원으로서 **Raspberry Pi CM4 + MicroTik 스위치(BC)** 기반의 HW PTP 시스템 초기 구축에 참여.
  - **CM4 GPS 동기화 파이프라인**: 외부 GNSS 수신기의 PPS 및 RMC 신호를 CM4에 직접 연결 → `ts2phc`로 CM4 NIC PHC(Physical Hardware Clock)를 GPS 기준 시각으로 동기화 → `phc2sys`로 CM4 시스템 클럭을 PHC에 종속 → `ptp4l`로 CM4를 **GM(Grandmaster)**으로 동작시켜 차량 내 네트워크로 PTP 배포.
  - **MicroTik 스위치(BC)**: GM인 CM4로부터 PTP를 수신하여 자체 PHC에 동기화 후, 연결된 LiDAR·카메라·IMU 등 이기종 센서들에게 PTP를 재배포하는 **Boundary Clock** 역할 수행.
  - GPS 리프 초(leap second) 적용으로 분기마다 GPS time이 오프셋되는 이슈를 확인 및 수정.
- **결과 (Result)**: 센서 종류에 제약 없이 ns 수준의 범용적인 시간 동기화 체계 확립.

> **BC(Boundary Clock) vs TC(Transparent Clock)**
> - **BC**: 스위치 자체가 PTP slave가 되어 upstream GM과 동기화하고, 자신이 새로운 GM으로서 downstream 장치들에 시간을 재배포. 각 hop에서 딜레이를 완전히 보정하므로 정확도가 높음.
> - **TC**: 스위치는 자체 클럭 없이 PTP 패킷이 통과할 때 내부 체류 시간(residence time)을 측정해 패킷에 보정값(correction field)만 추가. End-to-end GM–slave 경로를 유지하면서 스위치 지연만 보상하는 방식.

### 2. Jetson Orin Nano GPIO 트리거 시스템 구축
- **문제 상황/목표**: SC-400 전용 트리거 보드 도입 이전, Jetson Orin Nano(ARM) 환경에서 카메라·LiDAR 센서에 동기화된 트리거 신호를 직접 생성할 수 있는 시스템 필요.
- **해결 방안 (Action)**:
  - **GPS 시각 동기화 레이어**: Garmin GPS의 PPS 신호를 Orin Nano GPIO 핀으로 입력(`pps-gpio` 커널 드라이버)하고, NMEA(GPRMC)를 UART(`/dev/ttyTHS1`)로 수신하여 `gpsd`에 연결. `chrony`의 `refclock SHM`(NMEA) + `refclock PPS /dev/pps0`(Hardpps) 구성으로 시스템 클럭(`CLOCK_REALTIME`)을 GPS 기준으로 동기화.
  - **GPIO 트리거 출력**: GPS 동기화된 `CLOCK_REALTIME`을 기준으로 1초를 100ms 단위로 **10분할(10Hz)** 하여 GPIO BCM 18·19번 핀에 트리거 펄스를 출력(`JetsonGPIO` 라이브러리, `m_PPS_DIV_Mode`). 카메라·LiDAR 트리거 라인에 직접 연결해 다중 센서 동기 트리거 구현.
- **결과 (Result)**: Garmin PPS/GPRMC 기반 GPS 시각 동기화 → 10Hz GPIO 하드웨어 트리거 출력 파이프라인 구축.
- **한계 및 개발 중단**: 트리거 생성의 시간 기준이 `CLOCK_REALTIME`이고, 이 클럭 자체가 GPS(PPS/NMEA)에 종속되는 구조였기 때문에, **GPS 신호가 불안정하거나 단절될 경우 트리거 간격도 그대로 흔들리는 근본적인 문제**가 존재. `chrony`가 slewing으로 일부 완충하지만, GPS가 트리거 안정성의 단일 장애점(SPOF)이 되는 구조적 한계를 해결할 수 없어 **개발 중단**. 이후 독립적인 하드웨어 클럭 및 GPS freerun 기능을 갖춘 전용 SC-400 보드로 전환.

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