---
layout: project
title: "PTP Time Synchronization System"
period: "2023.02 ~ Present"
category: "Time Synchronization"
tech_stack: "C++, Hardware Integration"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 6
---

## 프로젝트 개요
구형 동기화 보드를 신형 PTP(Precision Time Protocol) 기반 시스템으로 업그레이드하여 센서 간 정밀 시간 동기화 시스템 구축

## 주요 활동

### [시스템 업그레이드]
- **구형 시스템**: LiDAR 하드웨어 트리거 타임 기반
- **신형 시스템**: Hardware/Software PTP 기반 시간 동기화
- **확장성**: RMC 신호 지원 LiDAR뿐만 아니라 다양한 센서로 확장

### [PTP 시스템 구축]

#### SW PTP (초기 버전)
- **초기 구현**: Software timestamp 기반 PTP 시스템
- **동기화 정확도**: 마이크로초(μs) 단위 동기화
- **안정성**: 소프트웨어 기반 타임스탬프 처리

#### HW PTP (고정밀 버전)
- **하드웨어 기반**: Raspberry Pi 기반 하드웨어 PTP
- **동기화 정확도**: 나노초(ns) 단위 동기화
- **이슈 해결**: GPS 시간 분기별 변경 이슈 확인 및 수정
- **제품 안정화**: 상용 제품 수준의 안정성 확보

### [GPRMC Local Time 시스템]

#### 기존 시스템 문제점
- **GPS 신호 단절 시**: LiDAR-GPS-Trigger-Local Time 동기화 손실
- **데이터 손실**: GPS 신호 복구 시까지 모든 데이터 폐기
- **운영 효율성 저하**: 터널, 지하 등 GPS 신호 불안정 환경에서 문제

#### SC-400 보드 개발
- **가상 GPS 신호 생성**: GPS 신호 없는 환경에서도 동작
- **PPS 및 NMEA 생성**: GPS 초당 펄스 및 NMEA 신호 생성 가능
- **GPRMC 파싱**: GPS 데이터 파싱 및 검증 로직 구현
- **데이터 무결성**: GPS 신호 단절 시에도 연속적인 데이터 수집
- **상용화**: 2025년부터 제품 판매 시작

### [Trigger Time 성능 평가]
- **정량적 평가**: 시간 동기화 정확도 측정 시스템 구축
- **성능 검증**: 다양한 시나리오에서의 동기화 성능 테스트
- **품질 보증**: 상용 제품 수준의 성능 검증

## 기술적 성과
- 마이크로초에서 나노초 단위로 동기화 정확도 향상
- GPS 신호 단절 환경에서도 안정적인 데이터 수집 시스템 구축
- 하드웨어-소프트웨어 통합 시간 동기화 시스템 개발 경험
- 상용 제품 개발 및 판매 경험

## 프로젝트 결과물

### SC-300 보드
![SC-300 Board](/assets/images/projects/ptp_system/sc300_board.png)
*초기 PTP 시스템 하드웨어*

### PTP 아키텍처
![PTP Architecture](/assets/images/projects/ptp_system/ptp_architecture.png)
*Hardware/Software PTP 기반 시간 동기화 시스템 구조*

### SC-400 GPRMC 시스템
![SC-400 GPRMC](/assets/images/projects/ptp_system/sc400_gprmc.png)
*가상 GPS 신호 생성 시스템*

### SC-400 전체 시스템
![SC-400 System](/assets/images/projects/ptp_system/sc400_system.png)
*SC-400 보드 기반 통합 시스템*

### Trigger Time 성능 평가
![Trigger Performance](/assets/images/projects/ptp_system/trigger_performance.png)
*시간 동기화 성능 측정 및 검증 결과*
