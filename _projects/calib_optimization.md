---
layout: project
title: "Calibration Program Optimization"
period: "2022.06 ~ 2023.06"
category: "Calibration"
tech_stack: "C++"
role: "Research Engineer"
company: "Mobiltech (Internal R&D)"
order: 1
---

## 프로젝트 개요
이전 사내 버전의 수동/시각적 보정 기반 캘리브레이션 프로그램을 **PnP(Perspective-n-Point) 알고리즘**을 활용한 자동화된 캘리브레이션 시스템으로 개선

## 주요 활동

### [기존 시스템의 문제점]
- 수동/시각적 보정 기반
- 사용자가 직접 파라미터를 조절하며 결과물의 시각적 정합성을 확인하는 방식
- 시간 소모적이고 정확도가 사용자 숙련도에 의존

### [신규 개발 시스템]
- **PnP 알고리즘**: 2D-3D 포인트 매칭 기반의 자동화된 캘리브레이션
- **최적화 프로세스**: 사용자는 점과 직선을 생성하여 저장만 하면 자동으로 동작
- **다중 센서 캘리브레이션**: LiDAR-Camera, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 이종 센서 간 캘리브레이션 경험

### [정량적/정성적 평가 시스템]
- **정량적 평가**: RMS(Root Mean Square) 기반 정확도 측정
  - 정면/후면 카메라: RMS 1.xxx pixel 미만
  - Map 바닥면: RMS 6 pixel 미만
- **정성적 평가**:
  - 2D 이미지 projection
  - 3D 시각적 평가 지표 구현 (정지 상태 LiDAR-Camera 캘리브레이션)

## 기술적 성과
- 컴퓨터 비전 및 캘리브레이션 기술에 대한 깊은 이해 확보
- 자동화를 통한 작업 시간 단축 및 정확도 향상
- 다양한 센서 조합에 대한 캘리브레이션 노하우 축적

## 프로젝트 결과물

### 워크플로우
![Calibration Workflow](/assets/images/projects/calib_optimization/workflow.png)

### 캘리브레이션 결과 1
![Calibration Result 1](/assets/images/projects/calib_optimization/result1.png)

### 캘리브레이션 결과 2
![Calibration Result 2](/assets/images/projects/calib_optimization/result2.png)
