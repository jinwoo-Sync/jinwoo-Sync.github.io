---
layout: project
title: "Real-time POI Detection Program - Mobile Version"
period: "2025.01 ~ 2025.09"
category: "Mobile Application & AI"
tech: "Kotlin, YOLOv8(TFLite), FFmpeg"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
thumbnail: "/assets/images/projects/real-time_poi/system_overview.png"
order: 4
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Real-time POI Detection Program - Mobile Version (모바일 POI 실시간 탐지 솔루션)
- **기간**: 2025.01 ~ 2025.09
- **역할**: Project Manager & Lead Developer (시스템 아키텍처 설계, 전체 파이프라인 구현, 성능 최적화 총괄)
- **기술 (Tech Stack)**: Kotlin, YOLOv8(TFLite), FFmpeg

산업용 카메라, LiDAR, Jetson AGX 기반의 고가 하드웨어 구성을 범용 스마트폰 단일 기기(Samsung Galaxy S20 Ultra & S21 Ultra)로 대체하여 운영 비용을 절감하는 모바일 POI 탐지 솔루션. 두 가지 전략적 목표를 위해 수행되었다.

1. **사업적 확장**: 탑차, 버스 등 물류·운송 서비스와의 디지털 트윈(Digital Twin) 사업 협업을 위한 기술적 교두보 마련
2. **공정 효율화**: 내부 Lite 버전 장비의 판매부터 유지보수까지 발생하는 고질적인 공수(Man-Month)를 소프트웨어 기반으로 전환하는 선행 개발 프로젝트

## 주요 성과 (Key Achievements)
- 초기 30분 이내 강제 종료되던 시스템을 **3시간 이상 연속 가동** 가능하도록 안정화하여 시험 배포 단계 달성
- 고가 하드웨어(산업용 카메라, LiDAR, Jetson AGX) 의존성을 제거하여 장비 비용 및 유지보수 공수 절감의 기술적 토대 마련
- 사내 기술을 리눅스 환경 외 다양한 플랫폼으로 확장하는 첫 시발점이 된 프로젝트
- 현재 BM(Business Model) 설계 이슈로 서버 연동 및 실시간 전송 기능 개발 잠정 중단 상태

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 시스템 안정화 및 메모리 최적화
- **문제 상황/목표**: 고부하 파이프라인(센서 수집 → AI 추론 → UI 렌더링 → 영상 저장)에서 빈번한 비트맵 생성/소멸로 GC(Garbage Collection) 오버헤드가 발생하여 앱이 30분 이내에 강제 종료되는 치명적 결함이 존재. 또한 카메라 프레임 처리 지연 현상이 발생.
- **해결 방안 (Action)**: 자체 구현한 `PerfettoManager.kt` 모듈을 통해 안드로이드 프로파일링 도구(**Perfetto 및 Method Tracing**)를 연동. `Trace.beginSection`과 `Debug.startMethodTracing`을 활용해 시스템 병목 구간을 정밀 분석.
  1. **GC 부하 원천 차단**: 분석 결과 매 프레임 객체 생성/소멸이 원인임을 파악하여, 60개의 슬롯을 가진 원형 큐(Circular Queue) 기반의 `AdvancedTaggedBitmapPool`을 직접 설계해 메모리 재사용 구조 확립.
- **결과 (Result)**: 차량 부착 테스트 시 3시간 이상 연속 가동 안정성 확보. 15fps 고정 프레임 스트리밍 유지.

### 2. AI 추론 시스템 및 적응형 제어
- **문제 상황/목표**: 모바일 환경의 제한된 연산 자원에서 실시간 객체 탐지 추론을 안정적으로 수행해야 함.
- **해결 방안 (Action)**: 딥러닝 연구팀의 YOLOv8 경량화 모델을 TFLite GPU Delegate로 탑재. DeepLearningAdaptiveManager를 구현하여 하드웨어 부하(Latency)에 따라 추론 주기(Skip Interval)를 동적으로 조절.
- **결과 (Result)**: 모바일 환경에서 실시간 추론 성능 달성 및 시스템 부하에 따른 자동 조절로 안정성 확보.

### 3. 센서 퓨전 및 위치 추정 안정화
- **문제 상황/목표**: 짧은 개발 기간 내 신뢰성 있는 위치 데이터를 확보해야 하며, GPS 음영 지역에서의 데이터 정합성도 보장해야 함.
- **해결 방안 (Action)**: 오픈소스 mad-location-manager(Mad Kalman Filter)를 시스템에 안정적으로 통합. GNSS Raw Measurements(4Hz) 및 GPS(1Hz) 데이터를 효율적으로 핸들링. GPS 음영 지역 이탈 시 로컬 타임으로 수집된 데이터에 최신 GPS 시간 오프셋을 소급 적용(Backdating)하여 시간축을 정렬하는 로직 구현.
- **결과 (Result)**: 서비스 환경 수준에서 수용 가능한 위치 추정 성능 보장 및 GPS 단절 구간의 데이터 정합성 유지.

### 4. 아키텍처 설계 및 실시간 데이터 파이프라인 통합
- **문제 상황/목표**: 카메라 센서부터 딥러닝 추론, UI 렌더링, 서버 통신, 그리고 .mp4 영상 저장까지 이어지는 **고부하 실시간 파이프라인**을 데이터 손실 없이 안정적으로 병렬 처리해야 함.
- **해결 방안 (Action)**: Clean Architecture(Data/UI/Domain 레이어 분리)를 적용하여 각 모듈 간의 의존성을 분리. 특히 Kotlin Coroutines Actor 모델 및 Channel/Flow를 활용하여 각 파이프라인(추론, 저장, 전송)이 서로의 성능에 영향을 주지 않는 논블로킹(Non-blocking) 병렬 처리 구조 구현.
- **결과 (Result)**: 멀티태스킹 환경에서도 병목 없는 실시간 처리를 달성함은 물론, **사내 웹 개발팀과 협의된 JSON 포맷을 기반으로 서버 측 타임스탬프 동기화 및 데이터 매핑 로직을 최종 검증하여 데이터 오차 없는 실시간 탐지 결과 표출을 확인.**

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/real-time_poi/system_overview.png)
![](/assets/images/projects/real-time_poi/mobile_app.png)

**서버 연동 검증**

![](/assets/images/projects/real-time_poi/timestamp_verification.png)
*카메라 프레임과 딥러닝 추론 결과의 서버 측 타임스탬프 동기화 검증*

![](/assets/images/projects/real-time_poi/web_service_integration.png)
*실제 웹 인터페이스에서의 실시간 탐지 결과 표출 확인*

**안정성 및 결함 분석 테스트**

1. **[결함 사례] 메모리 관리 최적화 전 GC 오버헤드로 인한 앱 크래시 현상**
   - 비정상적인 메모리 점유율 상승으로 인해 시스템이 30분 이내에 강제 종료되는 상태 (안정화 전)
   ![](/assets/images/projects/real-time_poi/1000007357.gif)

2. **[안정화 테스트] 야외 주행 시 고부하 파이프라인 성능 검증**
   - 실시간 AI 추론 및 센서 데이터 수집 중에도 안정적인 15fps를 유지하는 모습 (안정화 진행 중)
   ![](/assets/images/projects/real-time_poi/1000007319.gif)

3. **[안정화 테스트] 차량 부착 환경에서의 3시간 이상 장기 가동 안정성 확보**
   - BitmapPool 및 Zero-copy 기술 적용 후, 연속 가동 시에도 프레임 드랍 없이 정상 동작 확인
   ![](/assets/images/projects/real-time_poi/1000007347.gif)