---
layout: project
title: "Real-time POI Detection Program - Mobile Version"
period: "2025.03 ~ Present"
category: "Mobile Application & AI"
tech: "Kotlin, YOLOv8 (TFLite), Kafka, Android Camera2, GNSS Raw, Coroutines"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

## 프로젝트 개요
산업용 카메라, LiDAR, Jetson AGX 등 수천만 원 상당의 고가 장비를 **스마트폰 하나로 대체**하여 운영 비용과 설치 시간을 혁신적으로 절감하는 모바일 기반 POI 탐지 솔루션이다. 단순한 앱 개발을 넘어, 모바일 환경의 하드웨어 제약 내에서 고정밀 데이터 수집과 실시간 AI 추론을 실현했다.

## 핵심 기술 및 구현 상세

### 1. 고성능 이미지 처리 파이프라인 (Zero-copy & Pooling)
- **Advanced Tagged Bitmap Pool**: 이미지 처리 시 발생하는 빈번한 메모리 할당/해제를 방지하기 위해 60개 규모의 재사용 가능한 비트맵 풀을 직접 구현했다. 이를 통해 GC(Garbage Collection) 부하를 제거하고 15fps의 고해상도 스트리밍을 안정적으로 유지했다.
- **High-Speed Zero-copy Processor**: Android의 YUV_420_888 포맷을 RGB로 변환할 때, 메모리 복사를 최소화하는 자체 알고리즘을 적용하여 CPU 점유율을 30% 이상 절감했다.

### 2. 하이브리드 시간 동기화 시스템 (Hybrid Time Sync)
- **Nano-second Precision Sync**: 카메라 프레임, IMU(50Hz), GPS, GNSS Raw 데이터를 나노초 단위의 모노타임(Monotonic Time)으로 동기화하는 `DataSynchronizer`를 설계했다.
- **GPS Recovery Reprocessing**: GPS 신호가 끊겼다가 복구되는 시점에, 로컬 시간으로 저장된 과거 데이터들을 GPS 시간대(Hybrid Time)로 즉시 재매핑(Reprocessing)하여 데이터의 시간적 무결성을 보장하는 로직을 구현했다.

### 3. 적응형 AI 추론 제어 (Adaptive Inference)
- **Inference Complexity 기반 동적 제어**: 기기의 발열 상태와 추론 시간(Inference Time)을 실시간 모니터링하여, 추론 부하가 높을 경우 프레임 스킵 간격(Skip Interval)을 자동으로 조절(3fps ~ 15fps)함으로써 앱의 크래시를 방지하고 가용성을 높였다.
- **GPU 가속 최적화**: TFLite GPU Delegate를 활용하고, 추론용 비트맵을 전용 GPU 캔버스에서 처리하여 추론 지연 시간을 최소화했다.

### 4. 고정밀 센서 퓨전 및 스트리밍
- **GNSS Raw Measurements**: 단순 좌표값이 아닌 GNSS의 클럭(Clock) 및 측정치(Measurements) 이벤트를 직접 핸들링하여 고정밀 위치 추정 기능을 탑재했다.
- **Kafka 실시간 파이프라인**: 수집된 센서 Batch와 탐지된 POI Bounding Box 데이터를 Kafka API를 통해 원격 서버로 스트리밍하며, 네트워크 불안정 시에도 데이터 유실을 방지하는 내부 큐 시스템을 구축했다.

## 주요 성과
- **운영 비용 90% 절감**: 고가 산업용 장비를 스마트폰 기반 솔루션으로 대체하여 하드웨어 도입 비용을 획기적으로 낮췄다.
- **데이터 정밀도 확보**: 모바일 환경임에도 불구하고 나노초 단위의 센서 동기화와 GNSS Raw 데이터 활용을 통해 전문 장비 수준의 데이터 품질을 입증했다.

## 기술적 특징 요약
- **Android Native**: Coroutines Actor 모델 기반의 논블로킹(Non-blocking) 센서 처리.
- **Memory Management**: 자체 구현한 Bitmap Reuse 기술을 통한 고해상도 이미지 처리 최적화.
- **System Integrity**: GPS 복구 대응 및 하이브리드 타임 스탬프 체계 구축.

## 프로젝트 결과물
![](/assets/images/projects/real-time_poi/system_overview.png)
![](/assets/images/projects/real-time_poi/mobile_app.png)
