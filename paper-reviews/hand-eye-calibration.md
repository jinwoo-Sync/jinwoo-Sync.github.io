---
layout: default
title: "Hand-Eye Calibration (AX=XB)"
paper: "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group"
authors: "F. Park, B. Martin"
venue: "IEEE Transactions on Robotics and Automation, 10(5): 717-721, 1994"
tags: [Calibration, Hand-Eye, Rodrigues, SVD]
---

<div class="container py-5">
  <nav aria-label="breadcrumb">
    <ol class="breadcrumb">
      <li class="breadcrumb-item"><a href="{{ '/' | relative_url }}">홈</a></li>
      <li class="breadcrumb-item"><a href="{{ '/papers.html' | relative_url }}">논문 리뷰</a></li>
      <li class="breadcrumb-item active">Hand-Eye Calibration</li>
    </ol>
  </nav>

  <h1 class="mb-3">Hand-Eye Calibration: AX=XB 문제</h1>
  <p class="text-muted">F. Park, B. Martin — IEEE Trans. Robotics and Automation, 1994</p>

  <hr>

## 개요

Hand-Eye 캘리브레이션은 로봇 팔(Hand)과 카메라(Eye)의 상대적 위치/방향 관계를 알아내는 문제입니다. 로봇 팔 끝에 카메라가 장착된 경우, 로봇이 움직일 때 카메라도 함께 움직이는데, 이 두 움직임의 관계를 수학적으로 계산합니다.

## 수학적 정의

이 문제는 다음 방정식으로 표현됩니다:

**AX = XB**

- **A**: 로봇 팔의 상대적 움직임 (변환 행렬)
- **B**: 카메라의 상대적 움직임 (변환 행렬)
- **X**: 로봇 팔-카메라 간의 관계 (회전 R + 이동 t)

## 풀이 과정 (Park 방법)

### 1단계: 회전 R 추정

여러 쌍의 로봇/카메라 움직임에서 상대적인 회전을 계산합니다:

1. 각 쌍 (i, j)에 대해 로봇과 카메라의 상대적 회전 행렬을 구함
2. **Rodrigues 벡터**로 변환하여 회전 축과 각도를 추출
3. 외적 행렬 M을 축적: `M += b * a^T`
4. `M^T * M`의 **고유값 분해(Eigendecomposition)**로 최종 회전 행렬 R을 계산

핵심 공식:

```
R = V^T * diag(1/sqrt(eigenvalues)) * V * M^T
```

### 2단계: 이동 t 추정

회전 R이 구해진 후, 선형 방정식 시스템을 구성합니다:

```
(I - R_gij) * t = t_gij - R * t_cij
```

모든 쌍에 대해 이 방정식을 쌓아서 **SVD(특이값 분해)**로 최소제곱해를 구합니다.

## 구현 핵심 포인트

- `homogeneousInverse()`: 4x4 동차 변환 행렬의 역행렬 계산
- `Rodrigues()`: OpenCV 함수로 회전 행렬 ↔ 로드리게스 벡터 변환
- `solve(C, d, t, DECOMP_SVD)`: 과결정 시스템(overdetermined system)을 SVD로 풀기
- N개 데이터에서 K = N(N-1)/2 쌍을 생성하여 정확도 향상

## 적용 분야

- 로봇 팔 끝단 카메라 캘리브레이션
- 자율주행 센서 마운트 캘리브레이션
- MMS(Mobile Mapping System)에서의 센서 간 상대 위치 결정

</div>
