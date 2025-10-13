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

## 주요 활동

- 이전 사내 버전: **수동/시각적 보정** 기반의 캘리브레이션 프로그램을 개발. 이는 사용자가 직접 파라미터를 조절하며 결과물의 시각적 정합성을 확인하는 방식이었음.

- 신규 개발한 업그레이드 버전: **PnP(Perspective-n-Point)** 알고리즘을 활용해 2D-3D 포인트 매칭 기반의 **자동화된 캘리브레이션** 기능을 추가. 최적화를 도입하여 사용자는 점과 직선을 잘만 만들어서 저장만 하면 동작되는 프로그램 구현.

- **성과:** 비록 사내 정식 채택은 되지 않았지만, 이 프로젝트를 통해 **컴퓨터 비전 및 캘리브레이션 기술에 대한 깊은 이해**를 가지게 됨.

  - [ 오차가 적은 공간 ] 정면 카메라, 후면 카메라 기준 rms 1.xxx pixel 미만 도달.

  - [ 오차가 최대인 공간 ] map의 바닥면 rms 6pixel 미만 도달.

- **정지 상태에서 라이다-카메라 캘리브레이션 정확도 평가 : 정량적 평가 ( rms ) 와 정성적 평가 ( 이미지 projection )이외의 3차원 시각적 정성적 평가 지표 구현.**

- **lidar - lidar / radar - lidar / camera - lidar / camera - lidar -radar / map - camera - lidar 등 이종간 캘립 진행 경험 유**

## 프로젝트 결과물

![](/assets/images/projects/calib_optimization/workflow.png)

![](/assets/images/projects/calib_optimization/result1.png)

[ 결과물 ]

![](/assets/images/projects/calib_optimization/result2.png)
