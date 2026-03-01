---
layout: project
title: "Real-time POI Detection Program - Mobile Version"
period: "2025.03 ~ Present"
category: "Mobile Application & AI"
tech: "Kotlin, Coroutines, YOLOv8(TFLite), Camera2 API, GNSS Raw, FFmpeg"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
thumbnail: "/assets/images/projects/real-time_poi/system_overview.png"
order: 4
---

## 프로젝트 개요
산업용 카메라, LiDAR, Jetson AGX 기반의 고가 하드웨어 구성을 **범용 스마트폰 단일 기기(Samsung Galaxy S20 Ultra & S21 Ultra)**로 대체하여 운영 비용을 90% 이상 절감하는 모바일 POI 탐지 솔루션이다. 본 프로젝트는 단순히 기술적 구현을 넘어 두 가지 전략적 목표를 위해 수행되었다.

1.  **사업적 확장**: 탑차, 버스 등 물류·운송 서비스와의 **디지털 트윈(Digital Twin) 사업 협업**을 위한 기술적 교두보 마련
2.  **공정 효율화**: 내부 'Lite 버전' 장비의 판매부터 유지보수까지 발생하는 **고질적인 공수(Man-Month)를 소프트웨어 기반으로 전환**하여 비즈니스 운영 비용을 극단적으로 절감하는 선행 개발 프로젝트다.

## 주요 성과

- **비즈니스 공수(Man-Month) 및 비용 혁신**: 고가 하드웨어 의존성을 제거하여 판매부터 설치, 유지보수까지 발생하는 전체 리소스를 극단적으로 절감했다.
- **신규 플랫폼 확장의 시발점**: 모바일 환경에서의 성공적인 AI 모델 탑재 및 시스템 안정화를 통해, 사내 기술들을 리눅스 환경이 아닌 다양한 플랫폼으로 확장하는 첫 시발점이 된 프로젝트다.
- **실무 배포 가능성 입증**: 초기 30분 이내 종료되던 불안정한 시스템을 3시간 이상 연속 가동 가능하도록 최적화하여, 실제 현장 배포가 가능한 수준의 기술적 신뢰성을 확보했다.

## 활동 내용

### [시스템 안정화 및 성능 최적화 (Technical Highlight)]
- **통합 데이터 파이프라인**: **센서 데이터 수집 → 실시간 AI 추론 → UI 렌더링 → 실시간 전송 및 .mp4 영상 저장**으로 이어지는 고부하 파이프라인 구축
- **메모리 관리 및 GC 부하 해결**: 초기 개발 단계에서 빈번한 비트맵 생성/소멸로 인한 GC(Garbage Collection) 오버헤드로 앱이 30분 이내에 강제 종료되던 치명적인 결함 해결
- **가동 시간 혁신 (30분 → 3시간+)**: `AdvancedTaggedBitmapPool`(60 Slots) 및 제로카피(Zero-copy) 기반의 `HighSpeedZeroCopyProcessor`를 직접 설계·구현하여 GC 부하를 원천 차단, 차량 부착 테스트 시 **3시간 이상 연속 가동**되는 안정성을 확보하여 시험 배포 단계 달성

### [개발 프로세스 및 아키텍처]
- **Clean Architecture 적용**: Data, UI, Domain 레이어 분리를 통해 모듈 간 의존성을 낮추고 확장성 확보
- **비동기 파이프라인**: Kotlin Coroutines Actor 모델 및 Channel/Flow를 활용하여 센서 데이터의 논블로킹(Non-blocking) 병렬 처리 구현

### [안드로이드 앱 개발 및 최적화]
- **Target Device 최적화**: 고부하 연산 처리를 위해 **Samsung Galaxy S20 Ultra 및 S21 Ultra** 환경을 기준으로 시스템 개발 및 하드웨어 가속 최적화 수행
- **고성능 이미지 파이프라인**: Camera2 API 연동 및 하드웨어 가속을 통한 15fps 고정 프레임 스트리밍 제어
- **고속 이미지 처리**: YUV_420_888 to RGB 변환 최적화 및 FFmpeg 기반의 영상 인코딩 로직 구축으로 CPU 점유율 30% 이상 절감

### [AI 및 추론 시스템]
- **적응형 추론 제어**: `DeepLearningAdaptiveManager` 구현을 통해 하드웨어 부하(Latency)에 따라 추론 주기(Skip Interval)를 동적으로 조절하여 시스템 안정성 확보
- **YOLOv8 TFLite**: 딥러닝 연구팀의 경량화 모델을 활용해 모바일 환경 실시간 추론 성능 달성 및 TFLite GPU Delegate 적용

### [센서 퓨전 및 위치 추정 안정화]
- **오픈소스 기반 위치 추정 최적화**: 짧은 개발 기간 내에 신뢰성 있는 위치 데이터를 확보하기 위해 **[mad-location-manager](https://github.com/maddevsio/mad-location-manager)** (Mad Kalman Filter)를 시스템에 안정적으로 통합 및 적용했다.
- **데이터 정합성 보장**: Kalman Filter를 통해 GPS 노이즈를 억제하고, GNSS Raw Measurements(4Hz) 및 GPS(1Hz) 데이터를 효율적으로 핸들링하여 **서비스 환경 수준**에서 수용 가능한 수준의 위치 추정 성능을 보장하는 시스템을 구축했다.
- **데이터 정합성 유지**: GPS 음영 지역 이탈 시, 신호 단절 구간 동안 로컬 타임으로 수집된 데이터들에 최신 GPS 시간 오프셋을 **소급 적용(Backdating)**하여 전체 데이터셋의 시간축을 정렬하는 로직 구현

## 개발 현황 및 최종 결과
- **현재 상태**: **BM(Business Model) 모델 설계 이슈로 인해 서버 연동 및 실시간 전송 기능 개발 잠정 중단**
- **공학적 성과**: 기존 Lite 장비의 생산·운용 파이프라인(3D 외형 제작, 센서 조립/보정, PC 환경 및 원격 세팅 등)을 앱 설치 및 업데이트 방식으로 일원화하여, **제품 생애 주기 전반의 Man-Month를 혁신적으로 절감할 수 있는 양산형 모바일 솔루션의 기술적 토대**를 마련했다.

## 프로젝트 결과물
![](/assets/images/projects/real-time_poi/system_overview.png)
![](/assets/images/projects/real-time_poi/mobile_app.png)

---

#### 부록: 안정성 및 결함 분석 테스트
| GC 오버헤드로 인한 앱 크래시 사례 (안정화 전) | 야외 주행 및 장기 가동 테스트 (안정화 진행 중) |
| :---: | :---: |
| ![](/assets/images/projects/real-time_poi/1000007357.gif) | ![](/assets/images/projects/real-time_poi/1000007319.gif) <br> ![](/assets/images/projects/real-time_poi/1000007347.gif) |
| **비정상적 메모리 점유 및 시스템 강제 종료** | **안정적인 15fps 유지 및 3시간+ 연속 가동** |
