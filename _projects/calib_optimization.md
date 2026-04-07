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
  - **파라미터 고정 메커니즘**: intrinsic 파라미터를 `intr[]` 배열 하나에 순서대로 관리(`[fx=0, fy=1, cx=2, cy=3, skew=4, k1=5, k2=6, …]`). 고정할 파라미터의 배열 인덱스를 `const_intr` 벡터에 수집하고, `ceres::SubsetParameterization(n_intr, const_intr)`에 전달하면 Ceres가 해당 인덱스는 최적화에서 제외하고 나머지만 업데이트. 이 인덱스 지정 방식으로 단계마다 고정할 파라미터를 바꿔가며 3회 순차 최적화를 진행.

> **[참고] 호모그래피(Homography)란?**
> 두 평면 사이의 점 대응 관계를 표현하는 3×3 변환 행렬. 체커보드는 Z=0인 평면이므로, 체커보드 위의 3D 좌표 (X, Y, 0)와 카메라 이미지의 2D 좌표 (u, v) 사이의 관계를 하나의 행렬(H)로 표현할 수 있다. `cv::calibrateCamera()`는 이 H를 이미지별로 추정한 뒤, 그 안에서 카메라 내부 파라미터(K)와 각 이미지의 자세(R, t)를 분리 추출하는 방식으로 초기값을 산출한다. 즉, 호모그래피는 캘리브레이션의 초기 추정 단계에서 K와 RT를 뽑아내기 위한 연산 과정.

### 2. PnP 기반 자동 캘리브레이션 및 범위 제약 조건 기능 개발
- **문제 상황/목표**: 기존 사내 프로그램은 사용자가 직접 파라미터를 조절하며 결과물의 시각적 정합성을 확인하는 수동 보정 방식으로, 작업자 숙련도에 따라 결과 품질이 크게 좌우됨.
- **해결 방안 (Action)**: PnP(Perspective-n-Point) + LM(Levenberg-Marquardt) 최적화 기반의 자동화된 캘리브레이션 기능을 순수 OpenCV만으로 구현. 사용자가 마우스·키보드 조작(Shift+클릭으로 3D 투영점 선택 → 우클릭 드래그로 목표 2D 위치 지정 → 중클릭으로 확정)으로 2D-3D 대응쌍을 생성하면 최적화가 자동으로 동작하도록 설계.
  - **방향 제약 (Angle Constraint)**: 직선 경계(가로/세로 엣지)처럼 한 축 방향으로만 오차를 제약해야 하는 경우, 오차 벡터를 지정한 각도로 회전하여 수직 성분만 추출(`diff_rot_y = sin(−θ)·Δx + cos(−θ)·Δy`). 0° = 수평 경계, 90° = 수직 경계.
  - **범위 제약 (Margin Constraint)**: 정확한 픽셀 위치가 아닌 **"이 범위 안에 있으면 된다"** 는 허용 구간을 사용자가 지정. 범위 내에서는 이차 패널티(`diff²/margin`, 부드럽게 수렴), 범위 초과 시 선형 패널티(`diff + (diff ∓ margin)`, 강하게 복원)를 적용하는 dead-zone loss function으로 구현. 3D 포인트가 2D 이미지상의 특정 구조물 경계 안쪽에 반드시 위치해야 하는 제약을 표현하는 데 활용.
- **결과 (Result)**: 정면/후면 카메라 기준 RMS 1.xxx pixel 미만, Map 바닥면 기준 RMS 6 pixel 미만 달성.

### 2. 이종 센서 간 캘리브레이션 경험
- **문제 상황/목표**: MMS 시스템에서 다양한 센서 조합의 캘리브레이션이 요구됨.
- **해결 방안 (Action)**: LiDAR-Camera, LiDAR-LiDAR, Radar-LiDAR, Camera-LiDAR-Radar, Map-Camera-LiDAR 등 다양한 이종 센서 조합에 대해 캘리브레이션을 직접 수행.
- **결과 (Result)**: 다양한 센서 조합에 대한 캘리브레이션 실무 경험 축적.

---

### 부록: 프로젝트 결과물

![](/assets/images/projects/calib_optimization/workflow.png)

![](/assets/images/projects/calib_optimization/img_00024.jpg)

![](/assets/images/projects/calib_optimization/최적화.png)