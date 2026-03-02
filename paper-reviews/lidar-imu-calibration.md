---
layout: default
title: "LiDAR-IMU Calibration 검증 방법론 (논문 서베이)"
tags: [Calibration, LiDAR, IMU, SLAM, Validation]
---

<div class="container py-5">
  <nav aria-label="breadcrumb">
    <ol class="breadcrumb">
      <li class="breadcrumb-item"><a href="{{ '/' | relative_url }}">홈</a></li>
      <li class="breadcrumb-item"><a href="{{ '/papers.html' | relative_url }}">논문 리뷰</a></li>
      <li class="breadcrumb-item active">LiDAR-IMU Calibration 검증</li>
    </ol>
  </nav>

  <h1 class="mb-3">LiDAR-IMU Calibration 검증 방법론 (논문 서베이)</h1>
  <p class="text-muted">주요 캘리브레이션 논문 서베이 및 실전 검증 시나리오 분석</p>

  <div class="card mt-4 mb-4">
    <div class="card-header"><strong>서베이 대상 논문</strong></div>
    <div class="card-body">
      <ol class="mb-0">
        <li class="mb-2"><strong>LI_Calib</strong> — J. Lv et al., <em>"Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation"</em>, <strong>IROS 2020</strong></li>
        <li class="mb-2">J. Li et al., <em>"3D LiDAR/IMU Calibration Based on Continuous-Time Trajectory Estimation in Structured Environments"</em>, <strong>IEEE Access 2021</strong></li>
        <li class="mb-2"><strong>OA-LICalib</strong> — J. Lv et al., <em>"Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems"</em>, <strong>IEEE T-RO 2022</strong></li>
        <li class="mb-2"><strong>LiDAR2INS</strong> — G. Yan et al., <em>"An Extrinsic Calibration Method of a 3D-LiDAR and a Pose Sensor for Autonomous Driving"</em>, <strong>arXiv 2022</strong></li>
        <li><strong>GRIL-Calib</strong> — T. Shan et al., <em>"GRIL-Calib: Targetless Ground Robot IMU-LiDAR Extrinsic Calibration Method using Ground Plane Motion Constraints"</em>, <strong>IEEE RA-L 2024</strong></li>
      </ol>
    </div>
  </div>

  <hr>

<div class="content" markdown="1">

## 배경

LiDAR-IMU 캘리브레이션의 **검증 방법**을 체계화하기 위해, 최신 논문(2020~2024)의 검증 전략을 서베이하고 실제 산업 현장에 적용 가능한 시나리오를 분석한 문서이다. 단순 연구 목적이 아닌 **시스템 신뢰성 확보**를 위해 보수적인 검증이 필요하다는 관점에서 작성되었다.

---

## 1. LiDAR-IMU Calibration 방법 분류

### 1.1 타겟 기반 (Target-based)
특정 타겟의 위치 및 자세를 활용하여 LiDAR와 IMU 간의 상대 변환 행렬 $R \| t$를 계산한다.
- **다중 특징 보드 및 평면 타겟**: 타공판(Circular holes) 또는 체커보드를 활용하여 LiDAR와 카메라/열화상 센서 등을 동시 보정
- **원통형 기둥(Poles)**: 고반사율 테이프 등을 부착하여 엣지와 곡률 중심을 추출하는 방식

### 1.2 모션 기반 (Motion-based)
두 센서에 동일한 움직임을 부여하고, 각각의 데이터를 결합해 변환 관계를 추정한다.
- **연속 시간 궤적 최적화**: B-Spline 기반 궤적 추정 및 EKF/LM 기반 비선형 최적화
- **평면 운동 제약 (Planar Constraints)**: 지상 로봇 등 평면 이동 모델에서 발생하는 Z축 및 Roll/Pitch의 비관측성(Unobservability)을 수식적으로 해결

---

## 2. 주요 오픈소스 도구

| 구분 | 도구 | 특징 |
| :--- | :--- | :--- |
| 타겟 기반 | **Kalibr** (ETH-ASL) | 카메라/IMU/LiDAR 통합 캘리브레이션 |
| 타겟리스 (초기화) | **LI_init** (HKU-Mars) | LiDAR-IMU 초기화 및 캘리브레이션 |
| 타겟리스 (모션) | **GRIL_Calib** | 지면 평면 제약 기반 주행 캘리브레이션 (ROS1/2 지원) |
| 종합 패키지 | **OpenCalib** (PJLab) | LiDAR2INS, LiDAR2Cam 등 다양한 센서 보정 |

---

## 3. 캘리브레이션 검증 방법론 (시뮬레이션 기반)

*(참고: CARLA, Gazebo, Isaac Sim 등의 시뮬레이터가 LiDAR-IMU 캘리브레이션 검증에 널리 사용된다)*

**10 Monte-Carlo Simulations**: 동일한 시뮬레이션을 노이즈나 초기 조건을 달리하여 10번 반복 실행하여, 알고리즘의 성능/정확도/일관성을 통계적으로 평가하는 방법이다.

---

## 4. 논문별 검증 방법 분석

### 4.1 LI_Calib (IROS 2020)
Targetless, 모션 기반 연속 시간 배치 추정 알고리즘이다.
- **데이터셋/시뮬레이션**: 3개의 직교 평면 환경과 사인파 형태의 IMU 경로. 10 Monte-Carlo 시뮬레이션 적용.
- **결과**: 변환 오차 0.0043±0.0006m, 방향 오차 0.0224±0.0026도.

### 4.2 IEEE Access 2021
연속시간 궤적 추정 기반, 구조화된 환경(수직 벽면, 평면, 코너)에서의 캘리브레이션.
- **평가 지표**: 포인트클라우드 정합 오차, RMSE, 모션 모델 일관성.

### 4.3 OA-LICalib (IEEE T-RO 2022)
LI_Calib의 확장 버전으로, **관측 가능성(Observability-Aware)** 분석 및 데이터 선택 기법이 추가된 알고리즘이다.
- **관측 가능성의 의미**: 센서의 움직임(Motion Excitation)이 부족한 구간(예: 단순 직진 주행)에서는 특정 축의 캘리브레이션 파라미터를 수학적으로 추정(관측)할 수 없다. 이를 무시하고 최적화를 돌리면 값이 발산하게 된다.
- **해결 방식 (TSVD 적용)**: 최적화 과정에서 TSVD(Truncated Singular Value Decomposition)를 적용해, 현재 모션으로 관측 가능한 파라미터 방향만 업데이트하고, 불확실한 방향은 업데이트를 차단하여 오류를 방지한다.
- **성능 향상**:
  - 발산 방지: 직진 주행과 같은 퇴화 모션(Degenerate motion)에서도 알고리즘이 발산하지 않고 강건하게 수렴한다.
  - 연산 최적화: 정보 이론(Information-Theoretic)을 기반으로 캘리브레이션에 유의미한 데이터 구간만 자동으로 선별하여 연산량을 크게 단축한다.
  - 정확도: 초기 추정값(Initial guess)이 부정확해도 0.2도 이내의 높은 회전 오차 정밀도로 수렴함을 증명하였다.

### 4.4 LiDAR2INS (arXiv 2022)
실제 자율주행 환경(교차로 8자 주행)에서의 모션 기반 캘리브레이션.
- **GT**: 하드웨어 설계 CAD 모델 기반.

### 4.5 GRIL-Calib (IEEE RA-L 2024) - 신규 분석 추가
지상 로봇(Ground Robot)과 같이 Z축 및 특정 회전의 자극이 부족한 **평면 운동(Planar Motion)**의 제약을 수학적으로 극복한 타겟리스 알고리즘이다.
- **데이터셋**: 평면 위 8자 주행 및 직선 주행 (M2DGR, HILTI 등 오픈 데이터셋 및 실측)
- **제안 방법**: 지면 평면 제약(Ground Plane Motion Constraints)을 통해 부족한 DOF(자유도)의 관측성을 강제 확보하며 Rotation과 Translation을 한 번에 최적화(Single Optimization) 수행.
- **평가 지표**: 평면 주행만으로도 전체 6-DOF 캘리브레이션의 RMSE 및 맵 매칭 오차가 크게 감소함을 증명.

---

## 5. 사내 검증 시나리오 제안

### 5.1 시뮬레이션 기반 Ground Truth 확보 및 관측성 한계 테스트
- Isaac Sim과 같은 물리 기반 시뮬레이터를 활용하여 LiDAR 프레임($\mathcal{L}$)에서 IMU 프레임($\mathcal{I}$)으로의 공간적 외부 파라미터(Spatial Extrinsics)인 $\mathbf{T}_L^I \in SE(3)$ 설계값을 완벽한 Ground Truth(GT)로 확보한다.
- 8자 주행, 사인파, 등속 직진 등 모션 궤적(Trajectory)을 다양하게 인가하며, 특정 축에 대한 Motion Excitation이 부족할 때 B-Spline 연속 시간 궤적 최적화 과정에서 매니폴드(Manifold) 상의 $\delta \boldsymbol{\xi} \in \mathbb{R}^6$ 증분 업데이트가 어느 시점부터 발산하는지(Edge case) 수치적으로 파악한다.
- 실제 데이터 취득 시나리오와 동일한 모션 제약을 시뮬레이션 상에 구현하여, 알고리즘의 최적화 수렴성이 실측 환경의 노이즈를 어느 정도까지 허용하는지 정합성을 교차 검증한다.

### 5.2 LiDAR-GNSS/IMU 오프셋 교차 검증 (8자 주행 맵 정밀도 측정)
지상 차량 환경에서 특정 축(Z축 등)의 모션 자극이 제한적인 상황을 극복하기 위해, 8자 주행을 통한 주변 맵 정밀도 측정 방식을 활용한다.

![8자 주행 맵 매칭 결과]({{ '/assets/images/papers/lidar-imu-calibration/gril_calib_trajectory.gif' | relative_url }})
- **방법**: 차량에 탑재된 센서(LiDAR, GNSS/IMU) 마운트에서 특정 축(예: y축) 오프셋을 의도적으로 5cm 물리적 이동시킨 후 8자 형태의 주행 데이터를 취득한다.
- **검증**: 캘리브레이션 알고리즘이 도출한 변환 행렬에서 해당 축의 변화량(5cm)만을 정확히 계산해내는지 확인하고, 이를 통해 생성된 주변 맵의 매칭 정밀도를 정량적으로 평가한다.

### 5.3 LiDAR-Camera 재투영 (Project to Image) 오차 시각화
LiDAR-Camera 간의 복합 정밀도를 직관적으로 확인하기 위해 원형 타공 패턴 등이 포함된 캘리브레이션 전용 타겟을 활용한다.

![멀티 피처 타겟 예시]({{ '/assets/images/papers/lidar-imu-calibration/multi_feature_board.png' | relative_url }})
- 계산된 LiDAR-Camera 캘리브레이션 행렬을 바탕으로, LiDAR 포인트 클라우드를 카메라 2D 영상 평면에 재투영한다.
- 타공 패턴의 구멍 테두리(Edge)에 재투영된 LiDAR 포인트가 얼마나 일치하는지 육안으로 확인하고, 픽셀 단위의 허용 오차 기준을 수립한다.

### 5.4 캘리브레이션 타겟 배치 및 수학적 모델링 교차 평가
대형 보드 설치가 어려운 실내외 환경을 고려하여 기둥형 타겟(수직/수평 봉) 기반의 검증 환경을 구축한다.

![기둥 타겟 환경]({{ '/assets/images/papers/lidar-imu-calibration/retro_reflective_poles.png' | relative_url }})
- 시뮬레이터 상에 동일한 규격의 기둥 타겟을 배치해 이상적인 캘리브레이션 결과를 얻은 뒤, 실측 데이터로 계산한 결과와 대조한다.
- 시뮬레이션에서는 정상이나 실측에서 포인트 정합 오차가 발생할 경우, 소프트웨어 알고리즘 문제가 아닌 **하드웨어 설치 공차(Mounting error)**로 원인을 분리하여 진단한다.

</div>
</div>
