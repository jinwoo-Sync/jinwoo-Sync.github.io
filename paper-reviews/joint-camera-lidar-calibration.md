---
layout: default
title: "Joint Camera Intrinsic and LiDAR-Camera Extrinsic Calibration"
paper: "Joint Camera Intrinsic and LiDAR-Camera Extrinsic Calibration"
authors: "Yan et al."
venue: "arXiv:2202.13708"
tags: [Calibration, LiDAR-Camera, Joint Optimization, Levenberg-Marquardt]
---

<div class="container py-5">
  <nav aria-label="breadcrumb">
    <ol class="breadcrumb">
      <li class="breadcrumb-item"><a href="{{ '/' | relative_url }}">홈</a></li>
      <li class="breadcrumb-item"><a href="{{ '/papers.html' | relative_url }}">논문 리뷰</a></li>
      <li class="breadcrumb-item active">Joint Camera-LiDAR Calibration</li>
    </ol>
  </nav>

  <h1 class="mb-3">Joint Camera Intrinsic & LiDAR-Camera Extrinsic Calibration</h1>
  <p class="text-muted">Yan et al. — arXiv:2202.13708</p>

  <hr>

## 핵심 문제

기존 LiDAR-카메라 캘리브레이션은 **2단계(Two-stage)** 방식을 사용합니다:
1. 카메라 내부파라미터(Intrinsics) 먼저 추정
2. 그 결과로 LiDAR-카메라 외부파라미터(Extrinsics) 추정

문제는 1단계의 미세한 오류가 2단계 정확도에 악영향을 준다는 것입니다. 이 논문은 **내부+외부 파라미터를 한 번에 공동 최적화**하는 방법을 제안합니다.

## 캘리브레이션 타겟 설계

**하이브리드 보드**: 중앙에 체커보드 패턴 + 주변에 4개의 원형 구멍

- **체커보드**: 카메라에서 코너 점을 정밀하게 검출
- **원형 홀**: LiDAR 포인트클라우드에서 깊이 불연속(edge)으로 뚜렷하게 검출

원형 타겟은 사각 패턴보다 회전에 무관하게 연속적인 에지를 제공하므로 검출 안정성이 높습니다.

## 파이프라인

### 1. 센서별 타겟 검출

**카메라**: OpenCV `findChessboardCorners`로 코너 픽셀 좌표 추출

**LiDAR**:
1. ROI 필터링으로 배경 제거
2. RANSAC 평면 피팅으로 보드 평면 탐지
3. 원형 홀 배치 패턴을 마스크로 생성
4. 평면 내 회전(1DOF) + 이동(2DOF)만 Grid Search하여 홀 중심 좌표 추출

### 2. 초기 파라미터 추정 (Zhang 방법)

체커보드 코너로 카메라 내부파라미터 K, 왜곡 계수 D, 보드-카메라 변환을 초기 추정합니다.

### 3. 공동 최적화 (Joint Optimization)

**파라미터 벡터**:

```
Θ = [f_x, f_y, c_x, c_y, k_1, k_2, ..., ω_L→C, t_L→C]
```

**비용 함수**:

```
F(Θ) = Σ|r_circle|² + Σ|r_corner|²
```

- **r_circle**: LiDAR 원형 홀 3D 좌표를 투영한 값과 보드 기하로 계산한 2D 좌표의 차이
- **r_corner**: 체커보드 코너의 재투영 오차

### 4. Levenberg-Marquardt 최적화

**Damped Normal Equations**:

```
(J^T J + λI) ΔΘ = -J^T r
```

- λ > 0: gradient descent와 Gauss-Newton 사이를 조절
- λ가 크면 → gradient descent (안정적, 느림)
- λ가 작으면 → Gauss-Newton (빠름, 수렴 근처)
- 구현: **Ceres Solver** + AutoDiffCostFunction

## C++ 모듈 구성

| 단계 | 모듈 | 핵심 라이브러리 |
|------|------|--------------|
| Data IO | SensorBagIO | OpenCV, PCL |
| 보드 검출 | BoardDetector | OpenCV findChessboardCorners, PCL RANSAC |
| 초기 카메라 캘립 | CameraCalibrator | OpenCV calibrateCamera |
| 초기 외부파라미터 | ExtrinsicInit | 보드 정렬 또는 수동 설정 |
| 공동 최적화 | JointOptimizer | Ceres Solver |
| 검증 및 저장 | CalibResult | YAML, 오버레이 시각화 |

## 기존 방법 대비 장점

1. **내부파라미터의 미세 오류를 보정**: 2단계 방식에서 전파되는 오차를 제거
2. **서브픽셀 수준의 재투영 오차** 달성
3. **거친 초기화에도 수렴**: LiDAR-카메라 외부파라미터를 대략적으로만 설정해도 LM 최적화로 보정 가능

## 데이터 수집 시 유의사항

- 보드를 이미지의 중앙~가장자리까지 고르게 배치
- 10개 이상의 Pose에서 촬영
- 근거리와 원거리 모두 포함

</div>
