---
layout: project
title: "Real-time POI Detection Program - Mobile Version"
period: "2025.03 ~ Present"
category: "Mobile Application & AI"
tech: "Kotlin, Coroutines, YOLOv8(TFLite), Camera2 API, GNSS Raw, FFmpeg"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

## 프로젝트 개요
산업용 카메라, LiDAR, Jetson AGX 등 고가의 하드웨어 구성을 스마트폰 단일 기기로 대체하여 운영 비용을 90% 이상 절감하는 모바일 기반 POI 탐지 솔루션이다. 단순한 기능 구현을 넘어, 공학적으로 판매부터 유지보수 단계까지 발생하는 실제 Man-Month 비용을 극단적으로 절감할 수 있는 선행 개발을 수행했다.

## 활동 내용

### [시스템 안정화 및 성능 최적화 (Technical Highlight)]
- **통합 데이터 파이프라인**: 센서 데이터 수집 → 실시간 AI 추론 → UI 렌더링 → 실시간 전송 및 **.mp4 영상 저장**으로 이어지는 고부하 파이프라인 구축
- **메모리 관리 및 GC 부하 해결**: 초기 개발 단계에서 빈번한 비트맵 생성/소멸로 인한 GC(Garbage Collection) 오버헤드로 앱이 30분 이내에 강제 종료되던 치명적인 결함 해결
- **가동 시간 혁신 (30분 → 3시간+)**: `AdvancedTaggedBitmapPool`(60 Slots) 및 제로카피(Zero-copy) 기반의 `HighSpeedZeroCopyProcessor`를 직접 설계·구현하여 GC 부하를 원천 차단, 차량 부착 테스트 시 **3시간 이상 연속 가동**되는 안정성을 확보하여 시험 배포 단계 달성

### [개발 프로세스 및 아키텍처]
- **Clean Architecture 적용**: Data, UI, Domain 레이어 분리를 통해 모듈 간 의존성을 낮추고 확장성 확보
- **비동기 파이프라인**: Kotlin Coroutines Actor 모델 및 Channel/Flow를 활용하여 센서 데이터의 논블로킹(Non-blocking) 병렬 처리 구현

### [안드로이드 앱 개발 및 최적화]
- **고성능 이미지 파이프라인**: Camera2 API 연동 및 하드웨어 가속을 통한 15fps 고정 프레임 스트리밍 제어
- **고속 이미지 처리**: YUV_420_888 to RGB 변환 최적화 및 FFmpeg 기반의 영상 인코딩 로직 구축으로 CPU 점유율 30% 이상 절감

### [AI 및 추론 시스템]
- **적응형 추론 제어**: `DeepLearningAdaptiveManager` 구현을 통해 하드웨어 부하(Latency)에 따라 추론 주기(Skip Interval)를 동적으로 조절하여 시스템 안정성 확보
- **YOLOv8 TFLite**: YOLOv8 모델 경량화 및 TFLite GPU Delegate 적용으로 모바일 환경 실시간 추론 성능 달성

### [센서 퓨전 및 위치 추정 안정화]
- **오픈소스 기반 위치 추정 최적화**: 짧은 개발 기간 내에 신뢰성 있는 위치 데이터를 확보하기 위해 **[mad-location-manager](https://github.com/maddevsio/mad-location-manager)** (Mad Kalman Filter)를 시스템에 안정적으로 통합 및 적용했다.
- **데이터 정합성 보장**: Kalman Filter를 통해 GPS 노이즈를 억제하고, GNSS Raw Measurements(4Hz) 및 GPS(1Hz) 데이터를 효율적으로 핸들링하여 실무 환경에서 수용 가능한 수준의 위치 추정 성능을 보장하는 시스템을 구축했다.
- **데이터 정합성 유지**: GPS 음영 지역 이탈 시, 신호 단절 구간 동안 로컬 타임으로 수집된 데이터들에 최신 GPS 시간 오프셋을 **소급 적용(Backdating)**하여 전체 데이터셋의 시간축을 정렬하는 로직 구현

### [데이터 스트리밍 및 서버 연동]
- **실시간 데이터 전송**: 실시간 데이터 스트리밍을 위한 `SensorDataTransmitter` 모듈 설계 및 데이터 Batch 전송 로직 구현
- **유실 방지**: 내부 Circular Queue 구조의 버퍼링 시스템을 활용하여 네트워크 불안정 시 데이터 보존성 확보
- **개발 현황**: **BM(Business Model) 모델 설계 이슈로 인해 서버 연동 및 실시간 전송 기능 개발 잠정 중단**

## 프로젝트 결과물
![](/assets/images/projects/real-time_poi/system_overview.png)
![](/assets/images/projects/real-time_poi/mobile_app.png)
