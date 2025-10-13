---
layout: project
title: "Replica Program (MMS Tool Development)"
period: "2022.03 ~ Present"
category: "Mobile Mapping System"
tech_stack: "C++"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 5
---

## 프로젝트 개요
사내 핵심 프로그램인 Mobile Mapping System(MMS)의 핵심 기능을 지원하는 로깅, 추출 및 센서 인터페이스 툴 개발 및 유지보수

## 주요 활동

### [정밀 센서 인터페이스 개발]

#### 카메라 인터페이스
- **머신비전 카메라 SDK 연동**:
  - Hikvision, Vimba, Pylon, Ozray, Flycapture SDK 기반 모듈 개발
  - Hik, E-con MIPI, E-con GMSL 등 다양한 머신비전 카메라 인터페이스 담당
- **고해상도 이미지 처리**: 안정적인 캡처, 전송, 저장 시스템 구축

#### LiDAR 인터페이스
- **Hesai LiDAR SDK 업그레이드**: 신규 SDK로 업데이트
- **기존 파이프라인 연동**: 에러 없이 기존 로깅 파이프라인과 통합
- **데이터 안정성**: 대량 포인트 클라우드 데이터의 안정적 수집

### [MMS 로깅 및 추출 툴]
- **대량 센서 데이터 로깅**: 효율적인 데이터 수집 시스템
- **조건별 데이터 추출**: 특정 조건에 따른 데이터 필터링 및 추출
- **성능 최적화**: 지속적인 리팩토링 및 버그 수정
- **코드 안정성**: 장시간 운영 환경에서의 안정성 확보

### [하드웨어 제어 시스템]
- **트리거 보드 인터페이스**: NVIDIA Orin Nano GPIO 활용
- **센서 동기화**: Camera, LiDAR 등 다중 센서 간 시각 동기화
- **하드웨어 제어**: 정확한 타이밍 제어 시스템 구축

### [IPC 동기화 시스템]
- **단일 센서 공유**: 초기 단일 카메라/LiDAR IPC 공유 구현
- **다중 센서 확장**: 다중 카메라 및 다중 LiDAR로 IPC 확장
- **프로그램 파이프라인**: 데이터 추출 툴과 intrinsic tool, MMS mapping tool 간 안정적 연동

## 기술적 성과
- 다양한 머신비전 카메라 SDK 통합 경험
- 고성능 센서 데이터 처리 시스템 구축
- 장기간 안정적으로 운영되는 핵심 시스템 유지보수
- 하드웨어-소프트웨어 통합 제어 시스템 개발

## 프로젝트 결과물

### 시스템 다이어그램
![System Diagram](/assets/images/projects/replica/system_diagram.png)
*MMS 로깅 및 추출 시스템 전체 구조*

### 센서 인터페이스
![Sensor Interface](/assets/images/projects/replica/sensor_interface.png)
*다양한 센서 인터페이스 연동 구조*

### 저가 센서 MMS 매핑 결과
![Mapping Result](/assets/images/projects/replica/mapping_result.png)
*Xsens 센서 기반 MMS 매핑 결과물*
