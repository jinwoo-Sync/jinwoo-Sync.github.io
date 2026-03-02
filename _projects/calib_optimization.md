---
layout: project
title: "Calibration Program Optimization"
period: "2022.06 ~ 2023.06"
category: "Calibration"
tech: "C++, OpenCV, PnP Algorithm"
role: "Research Engineer"
company: "Mobiltech (Internal R&D)"
order: 1
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Calibration Program Optimization (사내 캘리브레이션 프로그램 자동화 고도화)
- **기간**: 2022.06 ~ 2023.06
- **역할**: Research Engineer (PnP 기반 자동 캘리브레이션 기능 설계 및 구현, 정량적/정성적 평가 체계 구축)
- **기술 (Tech Stack)**: C++, OpenCV, PnP Algorithm

기존 사내 캘리브레이션 프로그램이 수동/시각적 보정 방식에 의존하던 구조에서, PnP(Perspective-n-Point) 알고리즘을 활용한 자동화된 캘리브레이션 기능을 추가로 개발한 프로젝트.

## 주요 성과 (Key Achievements)
- 정면/후면 카메라 기준 RMS **1.xxx pixel 미만** 달성 (오차가 적은 공간)
- Map 바닥면 기준 RMS **6 pixel 미만** 달성 (오차가 최대인 공간)
- LiDAR-Camera, LiDAR-LiDAR, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 **이종 센서 간 캘리브레이션 경험** 보유
- 사내 정식 채택은 되지 않았으나, 컴퓨터 비전 및 캘리브레이션 기술에 대한 깊은 이해를 확보한 선행 연구 프로젝트

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. PnP 기반 자동 캘리브레이션 기능 개발
- **문제 상황/목표**: 기존 사내 프로그램은 사용자가 직접 파라미터를 조절하며 결과물의 시각적 정합성을 확인하는 수동 보정 방식으로, 작업자 숙련도에 따라 결과 품질이 크게 좌우됨.
- **해결 방안 (Action)**: PnP(Perspective-n-Point) 알고리즘을 활용해 2D-3D 포인트 매칭 기반의 자동화된 캘리브레이션 기능을 추가. 사용자가 점과 직선을 생성하여 저장만 하면 자동으로 최적화가 동작하는 프로그램으로 구현.
- **결과 (Result)**: 정면/후면 카메라 기준 RMS 1.xxx pixel 미만, Map 바닥면 기준 RMS 6 pixel 미만 달성.

### 2. 이종 센서 간 캘리브레이션 경험
- **문제 상황/목표**: MMS 시스템에서 다양한 센서 조합의 캘리브레이션이 요구됨.
- **해결 방안 (Action)**: LiDAR-Camera, LiDAR-LiDAR, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 다양한 이종 센서 조합에 대해 캘리브레이션을 직접 수행.
- **결과 (Result)**: 다양한 센서 조합에 대한 캘리브레이션 실무 경험 축적.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/calib_optimization/workflow.png)

![](/assets/images/projects/calib_optimization/result2.png)