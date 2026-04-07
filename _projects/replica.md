---
layout: project
title: "Replica Program (Mobile Mapping System) Tool Development"
period: "2022.03 ~ Present"
category: "Mobile Mapping System"
tech: "C++, Camera SDK (Hikvision, MIPI, GMSL), NVIDIA Orin"
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
- 핵심 메인 개발자로서 프로그램 전반의 신규 센서 인터페이스 개발, 시스템 이식 및 데이터 로깅 로직 등 전방위적 기능 개발 주도
- Mobiltech Logging Suite(MLS) **공인 GS(Good Software) 인증 획득** 기여

## 상세 업무 및 기여 (Responsibilities & Contributions)
프로그램의 핵심 메인 개발자로서 데이터 수집부터 제어, 시스템 통합까지 전 과정의 기능 고도화를 주도하며 다음과 같은 업무를 수행했습니다.

### 1. 신규 카메라 인터페이스 개발 및 신규 엔코더 장비 추가
- **문제 상황/목표**: 고정밀 매핑을 위한 신규 센서(Hik, E-con MIPI, E-con GMSL) 연동 및 기존 시스템에서 부재했던 **정밀 거리/시간 정보(DMI)의 로깅 필요**.
- **해결 방안 (Action)**: Hikvision 및 MIPI/GMSL 카메라 인터페이스를 신규 구현. 특히 **LV125 프로토콜을 정밀 분석하여 DMI(Distance Measurement Instrument) 데이터를 직접 파싱**하고, 이를 전체 시스템 파이프라인에 통합.
- **결과 (Result)**: 기존 추출 시스템에 DMI 데이터를 활용한 신규 기능을 추가하여, **'10m 단위 Local SLAM 맵 제작'** 및 **'DMI 기반 정시 상황 판단 로깅'** 기능을 실현.

### 2. LiDAR 센서 인터페이스 업데이트 및 MIPI/GMSL 카메라 V4L2 연동
- 신규 Hesai LiDAR SDK로 업데이트하여 기존 로깅 파이프라인에 연동.
- E-con MIPI/GMSL 카메라는 별도 SDK 없이 **Linux V4L2 커널 인터페이스를 직접 구현** (Jetson Orin 전용). MIPI와 GMSL은 `/dev/videoN` 번지만 다르고 동일한 코드로 동작.
- HIK, MIPI, GMSL 모두 동일한 `CameraModule` 인터페이스로 상위 코드 변경 없이 교체 가능.
- 상세 구현: [V4L2로 MIPI/GMSL 카메라 직접 연결하기](/posts/2026/04/08/v4l2-mipi-gmsl-camera)

### 3. Replica Lite SW 트리거 기능 개발
- **문제 상황/목표**: 단일 카메라·단일 LiDAR·Garmin GPS + Jetson Orin Nano로 구성된 도로 POI 데이터 취득 특화 경량 제품군(Replica Lite)에서, 별도 하드웨어 트리거 보드 없이 로깅 툴 내에서 SW적으로 트리거 신호를 생성하는 기능이 필요.
- **해결 방안 (Action)**: Jetson Orin Nano의 GPIO를 활용하여 로깅 툴 내부에서 SW적으로 트리거 타이밍을 생성하는 기능을 구현하고, 기존 로깅 파이프라인에 통합.
- **결과 (Result)**: Jetson Orin 플랫폼에서 별도 트리거 보드 없이 SW만으로 트리거 기능을 구현하여 Replica Lite 제품군의 로깅 시스템에 정상 적용.

---

## 시스템 운용 구성 및 대역폭

- **주요 구성**:
  - **표준 MMS 구성**: 4K 카메라 5대(Hikvision MV-CH120-10GC) + LiDAR 1대(Pandar128) + GNSS + DMI + SC400 트리거 동시 로깅.
  - **LiDAR 단독 대규모 구성**: LiDAR 24대 동시 취득 (싱가폴 현대자동차 공장 프로젝트).
  - **독립 센서 로깅**: 카메라·LiDAR·GNSS 등 각 센서를 독립적으로 선택하여 로깅 가능한 구조.

- **대역폭 계산 (표준 MMS 구성 기준)**:

  | 센서 | 계산 | 대역폭 |
  |------|------|--------|
  | 카메라 × 5 (MV-CH120-10GC, 4K, 10fps, 8-bit) | 3840×2160×1B×10fps × 5 | ~415 MB/s |
  | LiDAR × 1 (Pandar128, Single Return, 10Hz) | 128채널 UDP 패킷 기준 | ~12.5 MB/s |
  | GNSS / DMI / SC400 | 직렬·소형 패킷 | 무시 가능 |
  | **합계** | | **~428 MB/s** |

  카메라는 각각 1GigE로 PC에 직결하며, 다중 LiDAR 구성 시에는 스위치를 통해 PTP 동기화 후 단일 이더넷 포트로 데이터를 수신하는 구조. 실제 로깅 스토리지는 PCIe-to-SATA 연결 장치에 SATA SSD를 사용하여 현장 데이터를 저장.

---

### 부록: 프로젝트 결과물

**실제 로깅 세션 로그 / 카메라 5대·LiDAR·GPS·SC400 트리거 동시 로깅 세션 초기화 로그**

![](/assets/images/projects/replica/logging_log.png)


![](/assets/images/projects/replica/system_diagram.png)

**sw 트리거 제어 모듈 뷰**

![](/assets/images/projects/replica/trigger_view.png)



**DMI 기반 로컬 맵 생성**

![](/assets/images/projects/mms/module_architecture.png)

![](/assets/images/projects/mms/mms.gif)

**공인 인증**

- **Mobiltech Logging Suite(MLS)**: 공인 GS(Good Software) 인증 획득

![](/assets/images/projects/replica/공인_GS인증.png)
