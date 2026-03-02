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
        <li><strong>LiDAR2INS</strong> — G. Yan et al., <em>"An Extrinsic Calibration Method of a 3D-LiDAR and a Pose Sensor for Autonomous Driving"</em>, <strong>arXiv 2022</strong></li>
      </ol>
    </div>
  </div>

  <hr>

<div class="content" markdown="1">

## 배경

LiDAR-IMU 캘리브레이션의 **검증 방법**을 체계화하기 위해, 주요 논문의 검증 전략을 서베이하고 실제 산업 현장에 적용 가능한 시나리오를 분석한 문서이다. 단순 연구 목적이 아닌 **시스템 신뢰성 확보**를 위해 보수적인 검증이 필요하다는 관점에서 작성되었다.

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

## 3. 캘리브레이션 검증 방법론 (시뮬레이션 기반)

*(참고: CARLA, Gazebo, Isaac Sim 등의 시뮬레이터가 LiDAR-IMU 캘리브레이션 검증에 널리 사용된다)*

**10 Monte-Carlo Simulations**: 동일한 시뮬레이션을 노이즈나 초기 조건을 달리하여 10번 반복 실행하여, 알고리즘의 성능/정확도/일관성을 통계적으로 평가하는 방법이다.

---

## 4. 논문별 검증 방법 분석

### 4.1 "Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation" (LI_Calib, IROS 2020)

Targetless, 모션 기반 연속 시간 배치 추정 알고리즘이다.

*(논문 참고: Velodyne LiDAR와 3개의 IMU 위치 구성을 통한 실험 환경)*

- **데이터셋**: Custom 데이터 (indoor/outdoor) + 시뮬레이션
- **GT**: 도면(CAD) 등에서 추정된 3개의 IMU 위치를 참조값으로 사용
- **시뮬레이션**: 3개의 직교 평면 환경, IMU는 사인파 경로로 이동
  - IMU 400Hz, LiDAR 10Hz, 수평 360도 / 수직 ±15도 FOV
  - 10 Monte-Carlo simulations (각 10초 시퀀스)
  - 가우시안 노이즈 추가
- **결과**: 변환 오차 0.0043±0.0006m, 방향 오차 0.0224±0.0026도
- **평가 지표**: 포인트클라우드 정합 오차, 모션 모델 일관성

*(논문 참고: 3개의 직교 평면과 사인파 형태 IMU 경로로 구성된 시뮬레이션 환경)*

### 4.2 "3D LiDAR/IMU Calibration Based on Continuous-Time Trajectory Estimation in Structured Environments" (IEEE Access 2021)

연속시간 궤적 추정 기반, 구조화된 환경(수직 벽면, 수평 평면, 코너)에서의 캘리브레이션이다.

*(논문 참고: 구조화된 환경에서의 캘리브레이션 시뮬레이션 및 연속시간 궤적 모델링)*

- **데이터셋**: 시뮬레이션 + Custom 데이터 (구조화된 환경)
- **시뮬레이션**: 실제 센서 특성과 일치하도록 설정, 10 Monte-Carlo
- **평가 지표**: 포인트클라우드 정합 오차, RMSE, 모션 모델 일관성

### 4.3 "Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems" (OA-LICalib, IEEE T-RO 2022)

LI_Calib의 확장 버전으로, Observability-Aware 분석이 추가되었다. 평가 방법은 LI_Calib과 동일하다.

### 4.4 "An Extrinsic Calibration Method of a 3D-LiDAR and a Pose Sensor for Autonomous Driving" (LiDAR2INS, arXiv 2022)

실제 자율주행 환경에서의 모션 기반 캘리브레이션이다.

*(논문 참고: Top LiDAR와 GNSS/INS 실차 기반 실험 플랫폼 및 교차로 8자 주행 데이터 수집)*

- **데이터셋**: 실제 주행 환경 (복잡한 도로, 다양한 지형)
- **GT**: CAD 모델 (하드웨어 설계값)
- **평가 지표**: RMSE, GT 대비 회전/변환 오차

---

## 5. 실무 환경 맞춤형 검증 시나리오 제안

연구 수준의 성과를 넘어, 실제 프로덕트 레벨에서 요구되는 신뢰성 검증을 위해 4단계의 시나리오를 구성하였다.

### 5.1 시뮬레이션 환경 기반 GT(Ground Truth) 확보 및 한계 테스트
- Isaac Sim과 같은 물리 기반 시뮬레이터를 활용하여, 정확한 수학적 $R \| t$ (설계값)를 GT로 확보한다.
- 시뮬레이터 상에서 다양한 노이즈와 환경 변화를 인가하며 알고리즘이 어느 수준까지 버티는지(Edge case) 한계를 파악한다.
- 실제 데이터 수집 시나리오와 동일한 모션을 시뮬레이션 상에 구현함으로써, 실측 결과와 이상적인 수학적 모델 간의 직접적인 비교 분석이 가능해진다.

### 5.2 의도적 물리 오프셋 변경을 통한 정합성 검증

IMU 등 특정 센서의 위치만 제한적으로 변경한 상태에서 데이터를 취득한 후, 도출된 캘리브레이션 값이 물리적인 변화량을 정확히 반영하는지 교차 검증하는 방식이다.

![IMU 위치 변경 검증]({{ '/assets/images/papers/lidar-imu-calibration/imu_location_test.png' | relative_url }})
*위치 1(Location 1)과 위치 2(Location 2) 간에 y축 방향으로 정확히 5cm의 오프셋을 부여하여 수집한 데이터. 최종 캘리브레이션 결과에서 다른 파라미터는 동일하게 유지되면서 오직 y축 변환(Translation) 값만 5cm의 차이를 보여야 정상적인 추정으로 판단한다.*

### 5.3 이미지 재투영(Project to Image) 기반 직관적 오차 분석
- 계산된 LiDAR-IMU 캘리브레이션 행렬을 바탕으로, LiDAR 포인트 클라우드 데이터를 카메라 2D 이미지 평면 위에 투영(Projection)하여 시각적 일치도를 검증한다.
- 이 방식은 LiDAR-Camera 간의 캘리브레이션이 매우 정밀하게 맞춰져 있다는 전제 하에 유효하다.
- 허용 가능한 최대 오차 범위 등 명확한 자체 품질 기준 가이드라인 수립이 동반되어야 한다.

### 5.4 캘리브레이션 전용 타겟을 활용한 시뮬레이션 교차 평가

실제 캘리브레이션 공간(Calibration Room)을 구축할 때, 특징점 추출이 용이한 타겟 구조물(가로/세로 기둥, 특수 패턴 보드 등)을 배치하여 시뮬레이션과 실측을 대조하는 방법이다.

![캘리브레이션 타겟 예시 1]({{ '/assets/images/papers/lidar-imu-calibration/target_panels.png' | relative_url }})
*원형 타공 패턴이 포함된 캘리브레이션 보드를 다양한 깊이(1.5m, 3.0m, 4.5m 등)에 배치한 환경 구성안*

![캘리브레이션 타겟 예시 2]({{ '/assets/images/papers/lidar-imu-calibration/target_rods.png' | relative_url }})
*수직/수평 엣지 특징을 제공하는 원통형 기둥 타겟 배치*

- **교차 검증 효과**: 시뮬레이션 환경에서는 캘리브레이션이 정상이나 실측 데이터에서 불량으로 나타난다면, 알고리즘 결함이 아닌 **센서 하드웨어 장착 불량(설치 공차)** 문제로 원인을 좁힐 수 있다.
- 실측 데이터의 퀄리티를 정성적으로 판단하고 하드웨어/소프트웨어 문제를 분리하는 데 매우 효과적이다.

</div>
</div>
