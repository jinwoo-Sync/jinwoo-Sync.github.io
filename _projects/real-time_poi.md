---
layout: project
title: "Real-time POI Detection System"
period: "2024.03 ~ Present"
category: "Mobile Application & AI"
tech_stack: "Kotlin, YOLOv8, Kafka"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

## 프로젝트 개요
고가의 산업용 하드웨어를 스마트폰 기반 솔루션으로 대체하여 실시간 POI(Point of Interest) 탐지 시스템 개발

## 주요 활동

### [모바일 솔루션 개발]
- **스마트폰 센서 활용**: Camera, GPS, GNSS, IMU 데이터 통합
- **백엔드 연동**: 실시간 센서 로깅 및 데이터 전송 애플리케이션
- **센서 데이터 동기화**: 다중 센서 데이터의 정확한 타임스탬프 동기화

### [비용 효율성]
- **하드웨어 비용 절감**: 기존 시스템 (산업용 카메라, LiDAR, Jetson AGX) → 스마트폰
- **운영 효율화**: 현장 설치 시간 및 유지보수 비용 획기적 절감
- **확장성**: 저비용으로 다수의 데이터 수집 포인트 운영 가능

### [데이터 처리 시스템]
- **Kafka API 서버**: 실시간 이미지 및 센서 데이터 스트리밍
- **원격 관제 시스템**: 실시간 모니터링 및 데이터 처리
- **v2 개발 진행**: 개선된 성능과 기능으로 업그레이드 중

### [AI 기반 POI 탐지]
- **YOLOv8 Mobile**: 스마트폰 환경 최적화된 실시간 객체 탐지
- **실시간 처리**: 모바일 디바이스에서 즉각적인 POI 인식
- **데이터 저장**: 탐지된 POI 정보와 센서 데이터 연계 저장

## 기술적 성과
- 산업용 하드웨어 대비 90% 이상 비용 절감 가능성 검증
- 실시간 데이터 처리 파이프라인 구축
- 모바일 환경에서의 AI 모델 최적화 경험
- 대규모 센서 데이터 스트리밍 시스템 구축

## 프로젝트 결과물

### 시스템 전체 구조
![System Overview](/assets/images/projects/real-time_poi/system_overview.png)
*Kafka 기반 실시간 데이터 처리 아키텍처*

### 모바일 애플리케이션
![Mobile Application](/assets/images/projects/real-time_poi/mobile_app.png)
*센서 로깅 및 실시간 POI 탐지 화면*
