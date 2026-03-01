---
layout: default
title: "LiDAR-IMU Calibration 검증 방법론"
paper: "LiDAR-IMU Calibration 검증 방법 서베이"
authors: "사내 연구"
venue: "Mobiltech Internal Research"
tags: [Calibration, LiDAR-IMU, Verification, Simulation]
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
  <p class="text-muted">Mobiltech 연구개발팀 내부 서베이</p>

  <hr>

## 배경

LiDAR-IMU 캘리브레이션의 정확도를 어떻게 검증할 것인가에 대한 서베이입니다. 주요 논문들의 검증 방법을 분석하고, 회사 환경에 적합한 검증 시나리오를 제안합니다.

## LiDAR-IMU 캘리브레이션 분류

### 타겟 기반 (Target-based)
- 체커보드/평면 타겟으로 LiDAR-IMU 간 상대 변환 행렬 계산
- 레이저 트래커로 직접 측정

### 모션 기반 (Motion-based)
- 동일한 움직임을 부여하고 두 센서 데이터를 결합
- 방법: 비선형 최적화(EKF, LM), Ground Plane Constraints, NDT, Kalman Filter

## 주요 오픈소스 도구

| 도구 | 방식 | 링크 |
|------|------|------|
| Kalibr | Target-based | ethz-asl/kalibr |
| LI_init | Targetless | hku-mars/LiDAR_IMU_Init |
| GRIL-Calib | Targetless | Taeyoung96/GRIL-Calib |
| OpenCalib/LiDAR2INS | Targetless | PJLab-ADG/SensorsCalibration |

## 논문별 검증 방법 분석

### 1. LI_Calib (IROS 2020)

- **데이터**: Custom 취득 (indoor/outdoor) + 시뮬레이션
- **GT**: CAD 어셈블리 도면의 IMU 위치 (3개)
- **시뮬레이션**: 10 Monte-Carlo, 3개 직교 평면 + 사인파 궤적
- **성과**: 변환 오차 0.0043±0.0006m, 방향 오차 0.0224±0.0026도
- **평가 지표**: 포인트클라우드 정합 오차, 모션 모델 일관성

### 2. 3D LiDAR/IMU Calibration (IEEE Access 2021)

- **데이터**: 시뮬레이션 + Custom 취득
- **시뮬레이션**: 10 Monte-Carlo, 구조화된 환경 (수직 벽면, 수평 평면, 코너)
- **평가 지표**: 포인트클라우드 정합 오차, RMSE, 모션 모델 일관성

### 3. OA-LICalib (TRO 2022)

- LI_Calib의 확장 버전으로 **동일한 평가 방법** 사용

### 4. LiDAR2INS (Arxiv 2022)

- **데이터**: 실제 주행 환경 (다양한 도로, 지형)
- **GT**: CAD 모델 (하드웨어 설정값)
- **평가 지표**: RMSE (포인트클라우드 + 궤적), 회전/변환 오차

## 제안하는 검증 시나리오

### 시나리오 1: 시뮬레이션 또는 하드웨어 설계값 → GT

**시뮬레이션**:
- 정확한 설계상의 R|t 값을 GT로 사용
- 환경 변화와 노이즈를 추가하여 다양한 조건에서 테스트
- 실측 데이터 시나리오와 동일하게 시뮬레이션하여 비교

**하드웨어 설계값**:
- 시뮬레이션이 어려운 경우 CAD/설계값 사용
- 정확한 물리적 측정이 전제

### 시나리오 2: 실측 데이터 R|t 비교 검증

IMU 위치만을 의도적으로 변경하여 데이터를 취득합니다.

예: Location 1과 Location 2에서 y방향으로 5cm 이동
- y를 제외한 모든 값이 유사하게 추정되어야 함
- y 값은 5cm에 근사하게 나와야 함

### 시나리오 3: 재투영 오차 비교

LiDAR-IMU 캘리브레이션 값이 적용된 포인트를 이미지에 재투영하여 오차를 비교합니다.

전제: LiDAR-Camera 캘리브레이션이 정확하다는 가정

### 시나리오 4: 캘리브레이션 샵 구성

- Target 배치 및 CAD 모델링 제작 후 시뮬레이션에 적용
- Target 예시: 가로/세로 봉, 평평한 판
- 시뮬레이션에서 캘리브레이션 OK / 실측에서 캘리브레이션 NG → 하드웨어 설치 문제 판별 가능

## 핵심 시사점

1. 대부분의 논문이 **시뮬레이션 + Monte-Carlo**를 검증의 기본으로 사용
2. CAD/하드웨어 설계값을 GT로 활용하는 것이 현실적인 접근
3. 회사 환경에서는 **보수적인 다중 검증** (시뮬 + 실측 + 재투영)이 필요

</div>
