---
layout: project
title: "모바일 기반 실시간 POI 탐지 프로그램"
period: "2025.01 ~ 2025.09"
category: "Mobile Application & AI"
tech: "Kotlin, YOLOv8(TFLite), MediaCodec, MediaMuxer"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
thumbnail: "/assets/images/projects/real-time_poi/system_overview.png"
order: 4
---

## 프로젝트 개요 (Overview)
- **프로젝트명**: 모바일 POI 실시간 탐지 솔루션
- **기간**: 2025.01 ~ 2025.09
- **역할**: Project Manager & Lead Developer (시스템 설계 및 앱 개발 총괄)
- **기술 (Tech Stack)**: Kotlin, YOLOv8(TFLite), MediaCodec, MediaMuxer

산업용 카메라, LiDAR, Jetson AGX 기반의 고가 하드웨어 구성을 스마트폰 단일 기기(Galaxy S20/S21 Ultra)로 대체하여 운영 비용을 절감하는 솔루션입니다. 물류·운송 차량과의 협업을 위한 기술적 교두보 마련 및 장비 유지보수 효율화를 목표로 수행되었습니다.

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 메모리 최적화 및 시스템 안정성 개선
- **배경**: 센서 데이터 수집부터 AI 추론, 저장까지 이어지는 고부하 파이프라인에서 무분별한 비트맵 생성/소멸로 인한 GC 오버헤드 발생. 이로 인해 앱이 30분 내에 강제 종료되거나 프레임 드랍이 생기는 치명적인 결함이 있었음.
- **주요 작업**:
  - `FileLogger`를 통해 각 파이프라인 구간의 타임스탬프를 파일로 기록하여 병목 구간을 파악.
  - 매 프레임 객체 생성을 원천 차단하기 위해 60개 슬롯의 원형 큐 기반 **비트맵 풀링(Bitmap Pool)** 시스템을 설계하여 메모리 재사용 구조로 전환.
  - **영상 로깅 시스템**: `LoggerManager`가 `SimpleVideoEncoder`를 관리하는 구조. 카메라 프레임이 들어오면 `CircularQueue`(최대 3,600프레임)에 적재 → `MediaCodec`으로 H.264 인코딩(YUV420SemiPlanar 변환 포함) → `MediaMuxer`로 `.mp4` 컨테이너에 기록. 세션 단위로 파일을 분리하며, 출력 경로는 `LoggerManager`가 주입하는 방식으로 관리.
- **성과**: 차량 실행 테스트 결과 3시간 이상 연속 가동 시에도 크래시 없이 안정적으로 동작함을 확인했으며, 15fps의 일정한 스트리밍 성능을 확보함.

### 2. 센서 데이터 동기화 및 위치 정보 안정화
- **배경**: 터널이나 도심 협곡 같은 GPS 음영 지역에서 신호가 끊기면, 로컬에 저장 중인 영상/IMU 데이터와 GPS 데이터 간의 시간 축이 어긋나는 문제 해결이 필요했음.
- **주요 작업**:
  - GPS 수신이 재개되는 시점에 GPS Time과 시스템 시간(Local Time)의 오차를 계산.
  - GPS가 없던 구간에 로컬 시간으로 임시 기록된 데이터들을 실제 GPS 시간 축에 맞춰 재정렬하는 동기화 로직 구현.
  - 위치 추정의 연속성을 위해 오픈소스 **Mad Kalman Filter**를 이식하여 안정적인 위치 정보를 유지함.
- **성과**: 실시간 전송 및 로컬 로깅 전 구간에서 영상-위치 정보의 정합성을 확보함. 특히, 시간 축이 정밀하게 정렬된 데이터를 제공함으로써 추후 Mono Camera 및 IMU 기반의 VPS/Visual SLAM 연동을 위한 기술적 토대 마련

### 3. 실시간 데이터 파이프라인 아키텍처 설계
- **배경**: 센서 수집부터 서버 전송, 영상 저장까지 동시에 수행되는 환경에서 데이터 손실 없이 각 프로세스가 독립적으로 돌아갈 수 있는 병렬 처리 구조가 필요했음.
- **주요 작업**:
  - **Actor 모델 및 Channel**을 활용해 각 파이프라인이 서로를 간섭하지 않는 논블로킹(Non-blocking) 구조 설계. 카메라 Raw 데이터 → YOLOv8 딥러닝 추론 → UI에 이미지·BBox 동시 출력으로 이어지는 파이프라인의 안정성을 확보하고, 로깅 데이터는 큐에 적재하여 `.mp4` 및 `.jpg` 포맷으로 저장.
  - 사내 웹 개발팀과 협의하여 서버 측 타임스탬프 동기화 및 JSON 매핑 로직 검증.
- **성과**: 멀티태스킹 환경에서 병목 없는 실시간 처리 파이프라인을 구축했으며, 서버-앱 간의 데이터 오차 없는 탐지 결과 표출을 성공적으로 마무리함.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/real-time_poi/system_overview.png)
![](/assets/images/projects/real-time_poi/mobile_app.png)

**서버 연동 및 가시화 검증**

![](/assets/images/projects/real-time_poi/timestamp_verification.png)
*카메라 프레임과 AI 추론 결과의 서버 측 타임스탬프 동기화 검증*

![](/assets/images/projects/real-time_poi/web_service_integration.png)
*실제 웹 인터페이스에서의 실시간 탐지 데이터 표출*

**안정성 테스트 결과**

1. **[최적화 전] 메모리 점유율 상승으로 인한 앱 크래시**
   ![](/assets/images/projects/real-time_poi/1000007357.gif)

2. **[안정화 진행] 야외 주행 시 고부하 파이프라인 성능 검증 (15fps 유지)**
   ![](/assets/images/projects/real-time_poi/1000007319.gif)

3. **[최종 결과] 차량 부착 환경에서의 3시간 이상 장기 주행 안정성 확보**
   ![](/assets/images/projects/real-time_poi/1000007347.gif)