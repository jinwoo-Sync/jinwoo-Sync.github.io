---
layout: default
title: "LiDAR-IMU Calibration 검증 방법론"
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

  <h1 class="mb-3">LiDAR-IMU Calibration 검증 방법론</h1>
  <p class="text-muted">사내 R&D팀 캘리브레이션 연구 — 논문 서베이 및 실전 검증 시나리오 제안</p>

  <hr>

<div class="content" markdown="1">

## 배경

회사 내부에서 LiDAR-IMU 캘리브레이션의 **검증 방법**을 체계화하기 위해, 주요 논문의 검증 전략을 서베이하고 사내 적용 가능한 시나리오를 제안한 연구다. 단순 연구 목적이 아닌 **제품 신뢰성 확보**를 위해 보수적인 검증이 필요하다는 판단 하에 작성했다.

---

## 1. LiDAR-IMU Calibration 방법 분류

### 1.1 타겟 기반 (Target-based)
특정 타겟의 위치 및 자세를 활용하여 LiDAR와 IMU 간의 상대 변환 행렬 $R \| t$를 계산한다.
- **체커보드 및 평면 타겟**: LiDAR로 타겟을 감지하고, IMU의 중력 벡터 기반 자세 추정치를 결합하여 상대적인 $R \| t$를 도출
- **레이저 트래커**: 고정밀 레이저 장치를 사용하여 LiDAR와 IMU의 상대 위치를 직접 측정

### 1.2 모션 기반 (Motion-based)
두 센서에 동일한 움직임을 부여하고, 각각의 데이터를 결합해 변환 관계를 추정한다.
- **비선형 최적화**: LiDAR 오도메트리와 IMU 데이터를 EKF 또는 Levenberg-Marquardt로 최적화
- **지면 평면 제약 (Ground Plane Constraints)**: 지면이 고정 평면임을 가정, LiDAR 지면 감지와 IMU 중력 벡터 비교
- **NDT**: LiDAR 정합 움직임과 IMU 움직임을 비교
- **Kalman Filter**: IMU 각속도/가속도 적분 → LiDAR 오도메트리와 ESIKF 결합

---

## 2. 주요 오픈소스 도구

| 구분 | 도구 | 특징 |
| :--- | :--- | :--- |
| 타겟 기반 | **Kalibr** (ETH-ASL) | 카메라/IMU/LiDAR 통합 캘리브레이션 |
| 타겟리스 (초기화) | **LI_init** (HKU-Mars) | LiDAR-IMU 초기화 및 캘리브레이션 |
| 타겟리스 (모션) | **GRIL_Calib** | 지면 평면 제약 기반 주행 캘리브레이션 |
| 종합 패키지 | **OpenCalib** (PJLab) | LiDAR2INS, LiDAR2Cam 등 다양한 센서 보정 |

---

## 3. 캘리브레이션 시뮬레이터

![캘리브레이션 시뮬레이터 비교]({{ '/assets/images/papers/lidar-imu-calibration/isaac_sim.png' | relative_url }})
*주요 시뮬레이터 비교 — CARLA와 Gazebo가 LiDAR-IMU 캘리브레이션 검증에 널리 사용된다*

**10 Monte-Carlo Simulations**: 동일한 시뮬레이션을 노이즈나 초기 조건을 달리하여 10번 반복 실행하여, 알고리즘의 성능/정확도/일관성을 통계적으로 평가하는 방법이다.

---

## 4. 논문별 검증 방법 분석

### 4.1 LI_Calib (IROS 2020)

Targetless, 모션 기반 연속 시간 배치 추정(Continuous-time Batch Estimation) 알고리즘이다.

![LI_Calib 실험 환경]({{ '/assets/images/papers/lidar-imu-calibration/li_calib_env.png' | relative_url }})
*LI_Calib 실험 센서 구성 — Velodyne LiDAR와 3개의 IMU 위치*

- **데이터셋**: Custom 데이터 (indoor/outdoor) + 시뮬레이션
- **GT**: CAD 어셈블리 도면에서 추정된 3개의 IMU 위치를 참조값으로 사용
- **시뮬레이션**: 3개의 직교 평면 환경, IMU는 사인파 경로로 이동
  - IMU 400Hz, LiDAR 10Hz, 수평 360도 / 수직 ±15도 FOV
  - 10 Monte-Carlo simulations (각 10초 시퀀스)
  - 가우시안 노이즈 추가
- **결과**: 변환 오차 0.0043±0.0006m, 방향 오차 0.0224±0.0026도
- **평가 지표**: 포인트클라우드 정합 오차, 모션 모델 일관성

![LI_Calib IMU 위치 변화]({{ '/assets/images/papers/lidar-imu-calibration/li_calib_imu_positions.png' | relative_url }})
*시뮬레이션 환경 — 3개의 직교 평면과 사인파 형태 IMU 경로*

### 4.2 3D LiDAR/IMU Calibration (IEEE Access 2021)

연속시간 궤적 추정 기반, 구조화된 환경(수직 벽면, 수평 평면, 코너)에서의 캘리브레이션이다.

![IEEE Access 실험 환경]({{ '/assets/images/papers/lidar-imu-calibration/ieee_access_setup.png' | relative_url }})
*구조화된 환경에서의 캘리브레이션 시뮬레이션 설정*

![연속시간 궤적 시뮬레이션]({{ '/assets/images/papers/lidar-imu-calibration/continuous_time_sim.png' | relative_url }})
*연속시간 모델 기반 센서 궤적 시뮬레이션*

- **데이터셋**: 시뮬레이션 + Custom 데이터 (구조화된 환경)
- **시뮬레이션**: 실제 센서 특성과 일치하도록 설정, 10 Monte-Carlo
- **평가 지표**: 포인트클라우드 정합 오차, RMSE, 모션 모델 일관성

### 4.3 OA_LICalib (TRO 2022)

LI_Calib의 확장 버전으로, Observability-Aware 분석이 추가되었다. 평가 방법은 LI_Calib과 동일하다.

### 4.4 LiDAR2INS (Arxiv 2022)

실제 자율주행 환경에서의 모션 기반 캘리브레이션이다.

![LiDAR2INS 실험 플랫폼]({{ '/assets/images/papers/lidar-imu-calibration/lidar2ins_env.png' | relative_url }})
*실차 기반 실험 플랫폼 — Top LiDAR와 Novatel GNSS/INS*

![LiDAR2INS 데이터 수집]({{ '/assets/images/papers/lidar-imu-calibration/lidar2ins_results.png' | relative_url }})
*교차로에서 8자 주행으로 캘리브레이션 데이터 수집*

- **데이터셋**: 실제 주행 환경 (복잡한 도로, 다양한 지형)
- **GT**: CAD 모델 (하드웨어 설계값)
- **평가 지표**: RMSE, GT 대비 회전/변환 오차

---

## 5. 사내 적용 검증 시나리오 제안

단순 연구를 넘어 실제 제품의 신뢰성을 확보하기 위해 제안한 4가지 검증 시나리오다.

### 5.1 시뮬레이션 및 설계값 비교 (GT 확보)
- 정확한 설계상의 $R \| t$ 값을 얻을 수 있는 시뮬레이션 환경(Isaac Sim 등) 구축
- 환경 변화와 노이즈를 추가하며 알고리즘의 한계를 테스트
- 실측 데이터 시나리오와 동일하게 시뮬레이션하면 실측과 수학적 데이터 비교가 가능

### 5.2 실측 데이터 $R \| t$ 비교 검증

IMU 위치만을 의도적으로 변경하여 데이터를 취득하고, 캘리브레이션 결과가 해당 변화를 정확히 찾아내는지 확인한다.

![IMU 위치 변경 검증]({{ '/assets/images/papers/lidar-imu-calibration/imu_location_test.png' | relative_url }})
*Location 1과 Location 2 사이에 y축 5cm 차이를 두고 데이터를 취득한다. 캘리브레이션 결과에서 y값 외 모든 값이 유사하고, y값만 5cm에 근사하게 나와야 한다.*

### 5.3 재투영 오차 시각화 (Project to Image)
- LiDAR-IMU 캘리브레이션 결과가 적용된 포인트를 카메라 이미지에 재투영
- LiDAR-Cam 캘리브레이션이 완벽하다는 가정 하에 수행
- 사내 오차 허용 범위 기준 수립 필요

### 5.4 캘립샵 타겟 배치 및 CAD 모델링 시뮬레이션

캘리브레이션 룸 구성 시 기준이 될 수 있는 타겟(가로/세로 봉, 평면 판)을 배치하고, 시뮬레이션과 실측 데이터에서 비교 평가한다.

![캘리브레이션 타겟 예시 1]({{ '/assets/images/papers/lidar-imu-calibration/target_panels.png' | relative_url }})
*원형 홀이 있는 캘리브레이션 보드 — 거리별(1.5m, 3.0m, 4.5m) 배치*

![캘리브레이션 타겟 예시 2]({{ '/assets/images/papers/lidar-imu-calibration/target_rods.png' | relative_url }})
*봉 형태의 캘리브레이션 타겟*

- 시뮬레이션에서 캘립 good / 실측에서 bad → **하드웨어 설치 문제**로 판단 가능
- 실측 데이터의 정성적 평가에 유용

</div>
</div>
