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
        <li><strong>GRIL-Calib</strong> — T. Shan et al., <em>"GRIL-Calib: Targetless Ground Robot IMU-LiDAR Extrinsic Calibration Method using Ground Plane Motion Constraints"</em>, <strong>IEEE RA-L 2024 (신규 추가)</strong></li>
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
LI_Calib의 확장 버전으로, 관측 가능성(Observability-Aware) 분석이 추가됨.

### 4.4 LiDAR2INS (arXiv 2022)
실제 자율주행 환경(교차로 8자 주행)에서의 모션 기반 캘리브레이션.
- **GT**: 하드웨어 설계 CAD 모델 기반.

### 4.5 GRIL-Calib (IEEE RA-L 2024) - 신규 분석 추가
지상 로봇(Ground Robot)과 같이 Z축 및 특정 회전의 자극이 부족한 **평면 운동(Planar Motion)**의 제약을 수학적으로 극복한 타겟리스 알고리즘이다.
- **데이터셋**: 평면 위 8자 주행 및 직선 주행 (M2DGR, HILTI 등 오픈 데이터셋 및 실측)
- **제안 방법**: 지면 평면 제약(Ground Plane Motion Constraints)을 통해 부족한 DOF(자유도)의 관측성을 강제 확보하며 Rotation과 Translation을 한 번에 최적화(Single Optimization) 수행.
- **평가 지표**: 평면 주행만으로도 전체 6-DOF 캘리브레이션의 RMSE 및 맵 매칭 오차가 크게 감소함을 증명.

---

## 5. 실무 환경 맞춤형 검증 시나리오 제안 (최신 논문 기반)

단순 알고리즘 테스트를 넘어, 실제 프로덕트 레벨의 하드웨어 장착 및 시스템 신뢰성 검증을 위해 최신 연구 동향(2023-2024)을 반영하여 재구성한 4단계 시나리오이다.

### 5.1 시뮬레이션 기반 GT 확보 및 관측성(Observability) 한계 테스트
- Isaac Sim 등을 활용하여 수학적 $R \| t$ GT를 확보한다.
- 궤적의 형태(8자, 사인파 등)를 달리하며, **OA-LICalib** 및 **GRIL-Calib**에서 언급된 특정 축의 모션 자극(Excitation)이 부족할 때 알고리즘이 얼마나 발산하는지 Edge Case를 파악한다.

### 5.2 평면 주행(Planar Motion) 제약 기반의 오프셋 교차 검증 (GRIL-Calib 방법론)
실제 지상 차량이나 로봇의 경우 Z축 자극이 극히 제한적이다. GRIL-Calib(2024)에서 증명한 바와 같이, Z/Roll/Pitch 제약이 강하게 걸린 환경에서 물리적 센서 위치만을 임의 변경한 뒤 추정 성능을 평가한다.

![GRIL-Calib의 8자 궤적 주행 평가]({{ '/assets/images/papers/lidar-imu-calibration/gril_calib_trajectory.gif' | relative_url }})
*(그림 참조: 평면 주행 로봇의 모션 궤적 및 제약 모델 - GRIL-Calib 논문 Figure 발췌)*
- **방법**: 평지에 위치한 센서 장착 마운트에서 y축 오프셋을 5cm만 물리적으로 옮긴 후, 평면 주행 데이터를 취득. 알고리즘이 해당 Translation의 변화량(5cm)만을 독립적으로 계산해내는지 확인하여 과적합 여부를 판단한다.

### 5.3 멀티 피처 보드(Multi-feature Board)를 활용한 재투영 검증 (Dalirani et al., 2023 참조)
LiDAR-IMU-Camera의 복합 정밀도를 확인하기 위해 원형 타공 패턴(Circular holes)이 포함된 최신 캘리브레이션 타겟 설계 방식을 응용한다.

![멀티 피처 캘리브레이션 보드 예시]({{ '/assets/images/papers/lidar-imu-calibration/multi_feature_board.png' | relative_url }})
*(그림 참조: 원형 타공과 평면, 체커보드가 결합되어 이종 센서 간 엣지 정합성을 검증하는 타겟 디자인)*
- 계산된 LiDAR-IMU 캘리브레이션 행렬을 바탕으로, LiDAR 포인트 클라우드를 카메라 평면에 재투영(Projection)한다.
- 타공 패턴의 구멍 테두리(Edge)에 LiDAR 포인트가 얼마나 일치하는지를 통해 픽셀 단위의 허용 오차 기준을 수립한다.

### 5.4 반사율(Intensity) 특성 기둥(Pole) 타겟을 활용한 교차 평가 (Xue et al., 2024 참조)
최근 연구들에서 대형 타겟 보드의 대안으로 활용하는 수직형 기둥 구조물 및 고반사 테이프 환경을 구축한다.

![기둥 형태의 캘리브레이션 타겟]({{ '/assets/images/papers/lidar-imu-calibration/retro_reflective_poles.png' | relative_url }})
*(그림 참조: 반사 테이프가 부착된 다수의 원통형 기둥을 서로 다른 거리에 배치한 환경 - Xue et al., 2024 방법론 응용)*
- 시뮬레이션(노이즈 없음)과 실측(센서 장착 공차 존재) 데이터를 동일 타겟 환경에서 수집하여 대조한다.
- 시뮬레이션에서는 이상적이나 실측에서 오차가 두드러진다면, 소프트웨어가 아닌 **하드웨어 설치(Mounting) 틀어짐** 문제로 원인을 명확히 분리할 수 있다.

</div>
</div>
