---
layout: project
title: "Calibration Program Optimization"
period: "2022.06 ~ 2023.01"
category: "Calibration"
tech: "C++, OpenCV, Ceres Solver"
role: "Research Engineer"
company: "Mobiltech (Internal R&D)"
order: 1
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Calibration Program Optimization (사내 캘리브레이션 프로그램 자동화 고도화)
- **기간**: 2022.06 ~ 2023.01
- **역할**: Research Engineer (PnP 기반 자동 캘리브레이션 기능 설계 및 구현, 정량적/정성적 평가 체계 구축)
- **기술 (Tech Stack)**: C++, OpenCV, Ceres Solver

기존 사내 캘리브레이션 프로그램이 수동/시각적 보정 방식에 의존하던 구조에서, OpenCV 및 Ceres Solver 라이브러리를 활용한 자동화된 캘리브레이션 기능을 추가로 개발한 프로젝트.

## 주요 성과 (Key Achievements)
- 정면/후면 카메라 기준 RMS **1.xxx pixel 미만** 달성 (오차가 적은 공간)
- Map 바닥면 기준 RMS **6 pixel 미만** 달성 (오차가 최대인 공간)
- LiDAR-Camera, LiDAR-LiDAR, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 **이종 센서 간 캘리브레이션 경험** 보유
- 사내 정식 채택은 되지 않았으나, 컴퓨터 비전 및 캘리브레이션 기술에 대한 깊은 이해를 확보한 선행 연구 프로젝트

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. OpenCV + Ceres 기반 Intrinsic 캘리브레이션 파이프라인 구성
- **내용**: OpenCV와 Ceres Solver 라이브러리를 활용하여 체커보드 기반 카메라 Intrinsic 캘리브레이션 파이프라인을 구성. `cv::findChessboardCorners()` → `cv::cornerSubPix()` 로 체커보드 코너를 검출하고, 내부적으로 3D 체커보드 좌표와 2D 이미지 좌표 간의 호모그래피를 통해 초기 K와 per-image RT를 추정(`cv::calibrateCamera()` / `cv::fisheye::calibrate()`). 이후 `cv::solvePnP()` 로 각 이미지의 초기 자세를 산출하고, Ceres Solver로 전체 재투영 오차를 최소화하는 비선형 최적화를 수행. Pinhole / Fisheye 렌즈 모델 모두 지원.
  - **3단계 순차 최적화**: 초기값이 나쁜 상태에서 한 번에 전체를 풀면 발산할 수 있어 `calibrator.run(flag)`을 3회 순차 호출. ① `FIX_K|FIX_D5` — K 고정, k1~k4+Extrinsic 최적화 → ② `FIX_K|FIX_D` — K·D 모두 고정, Extrinsic만 최적화 → ③ `FIX_D5` — K·D·Extrinsic 전체 최적화. 단계적으로 자유도를 늘려가며 수렴.
  - **파라미터 고정 처리 과정**: intrinsic 파라미터를 `intr[]` 배열 하나에 순서대로 관리(`[fx=0, fy=1, cx=2, cy=3, skew=4, k1=5, k2=6, …]`). 고정할 파라미터의 배열 인덱스를 `const_intr` 벡터에 수집하고, `ceres::SubsetParameterization(n_intr, const_intr)`에 전달하면 Ceres가 해당 인덱스는 최적화에서 제외하고 나머지만 업데이트. 이 인덱스 지정 방식으로 단계마다 고정할 파라미터를 바꿔가며 3회 순차 최적화를 진행.

> **[참고] 호모그래피(Homography)란?**
> 두 평면 사이의 점 대응 관계를 표현하는 3×3 변환 행렬. 체커보드는 Z=0인 평면이므로, 체커보드 위의 3D 좌표 (X, Y, 0)와 카메라 이미지의 2D 좌표 (u, v) 사이의 관계를 하나의 행렬(H)로 표현할 수 있다. `cv::calibrateCamera()`는 이 H를 이미지별로 추정한 뒤, 그 안에서 카메라 내부 파라미터(K)와 각 이미지의 자세(R, t)를 분리 추출하는 방식으로 초기값을 산출한다. 즉, 호모그래피는 캘리브레이션의 초기 추정 단계에서 K와 RT를 뽑아내기 위한 연산 과정.

> **[한계]**
> - **FOV 180° 이상 불가**: 호모그래피는 평면-평면 대응이 전제이므로, 투영 자체가 성립하지 않는 FOV 180° 이상의 초광각 렌즈에는 적용 불가.
> - **아웃라이어에 취약**: Robust loss(Huber 등)를 별도로 적용하지 않아, 체커보드 코너 검출이 잘못된 이미지가 섞이면 최적화가 발산할 수 있음.
>   - *[참고] Robust loss(Huber)란?* 최적화에서 오차가 작을 때는 일반 제곱 오차(L2)처럼 동작하고, 오차가 일정 임계값을 넘으면 선형(L1)으로 전환되는 손실 함수. 아웃라이어(잘못 검출된 코너 등)가 섞여도 제곱 오차처럼 폭발적으로 커지지 않아 최적화가 덜 망가지는 효과가 있음. 쉽게 말해 "이상한 데이터가 들어왔을 때 그게 결과를 너무 많이 망치지 않도록 패널티를 줄여주는 장치".
> - **체커보드 검출 노이즈 민감**: `cv::findChessboardCorners()`는 주변에 체커보드와 유사한 격자 패턴이 존재하면 오검출 가능성이 있으며, 자동으로 불량 이미지를 거르는 기능이 없어 입력 데이터 품질에 크게 의존함.

### 2. 이종 센서 간 캘리브레이션 경험
- **문제 상황/목표**: MMS 시스템에서 다양한 센서 조합의 캘리브레이션이 요구됨.
- **해결 방안 (Action)**: LiDAR-Camera, LiDAR-LiDAR, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 다양한 이종 센서 조합에 대해 캘리브레이션을 직접 수행.
- **결과 (Result)**: 다양한 센서 조합에 대한 캘리브레이션 실무 경험 축적.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/calib_optimization/workflow.png)

![](/assets/images/projects/calib_optimization/img_00024.jpg)

![](/assets/images/projects/calib_optimization/최적화.png)

[ 이종 센서 간 캘리브레이션 - Radar-LiDAR ]

![](/assets/images/projects/calib_optimization/lidar_radar_calib.png)