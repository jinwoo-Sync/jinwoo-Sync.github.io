---
layout: default
title: "Joint Camera-LiDAR Calibration"
paper: "Joint Camera Intrinsic and LiDAR–Camera Extrinsic Calibration"
authors: "Yan et al."
venue: "arXiv:2202.13708, 2022"
tags: [Calibration, LiDAR, Camera, Joint Optimization, Ceres]
---

<div class="container py-5">
  <nav aria-label="breadcrumb">
    <ol class="breadcrumb">
      <li class="breadcrumb-item"><a href="{{ '/' | relative_url }}">홈</a></li>
      <li class="breadcrumb-item"><a href="{{ '/papers.html' | relative_url }}">논문 리뷰</a></li>
      <li class="breadcrumb-item active">Joint Camera-LiDAR Calibration</li>
    </ol>
  </nav>

  <h1 class="mb-3">Joint Camera Intrinsic and LiDAR-Camera Extrinsic Calibration</h1>
  <p class="text-muted">Yan et al. — arXiv:2202.13708, 2022</p>

  <hr>

<div class="content" markdown="1">

## 개요

LiDAR-카메라 캘리브레이션에서는 일반적으로 카메라 내부파라미터(intrinsics)를 먼저 추정하고, 그 결과를 바탕으로 외부파라미터(extrinsics)를 추정하는 **2단계 방법**을 사용한다. 하지만 이 방식은 카메라 내부파라미터의 미세한 오류가 외부파라미터 정합에 악영향을 준다는 문제가 있다.

이 연구는 **카메라 내부 및 LiDAR-카메라 외부 파라미터를 한 번의 최적화로 공동 추정**하는 방법을 제안한다. 체커보드와 원형 홀을 결합한 캘리브레이션 타겟을 설계하여, 카메라에서는 코너 특징을, LiDAR에서는 깊이 불연속(edge)을 동시에 검출한다.

---

## 1. 수학적 표기법 및 좌표계

| 기호 | 설명 |
| :--- | :--- |
| $\mathcal{C}$ | 카메라 좌표계 |
| $\mathcal{L}$ | LiDAR 좌표계 |
| $\mathcal{B}$ | 캘리브레이션 보드 좌표계 |
| $\mathbf{K}$ | 카메라 내부파라미터 행렬 |
| $\mathbf{D}(\cdot)$ | 왜곡 모델 (Radial / Tangential) |
| $\mathbf{R}_{A \to B}, \mathbf{t}_{A \to B}$ | 프레임 A에서 B로의 회전 및 이동 변환 |

최종적으로 구하고자 하는 파라미터 벡터 $\Theta$:

$$\Theta = [\underbrace{f_x, f_y, c_x, c_y, k_1, k_2, \dots}_{\Theta_{\text{int}}}, \underbrace{\boldsymbol\omega_{L \to C}, \mathbf{t}_{L \to C}}_{\Theta_{\text{ext}}}]^{\top}$$

---

## 2. 관측 모델 (Projection)

3D 점 $\mathbf{P}^F$를 이미지 평면의 2D 좌표 $\mathbf{p}$로 투영하는 식:

$$\mathbf{p} = \begin{bmatrix}u \\\\ v \\\\ 1\end{bmatrix} = \mathbf{K} \cdot \mathbf{d}\left(\underbrace{\mathbf{R}_{F \to C} \mathbf{P}^F + \mathbf{t}_{F \to C}}_{\mathbf{P}^C}\right)$$

왜곡 연산자 $\mathbf{d}$는 $z$로 나누는 정규화 과정을 포함하며 왜곡 모델을 적용한다.

---

## 3. 잔차(Residual) 및 비용 함수

### 3.1 체커보드 코너 잔차

이미지 $k$의 코너 $(i, j)$에 대해:

$$\mathbf{r}_{k,ij}^{\text{corner}} = \mathbf{p}_{k,ij}^{\text{proj}}(\Theta) - \mathbf{p}_{k,ij}^{\text{obs}}$$

### 3.2 원형 홀(Circle-hole) 잔차

LiDAR로 검출한 3D 중심점 $\mathbf{P}_{k,h}^{\mathcal{L}}$을 이미지로 투영한 값과 보드 설계상 예상되는 2D 중심점 사이의 오차:

$$\mathbf{r}_{k,h}^{\text{circle}} = \mathbf{p}_{k,h}^{\text{proj}}(\Theta) - \mathbf{p}_{k,h}^{\text{obs}}$$

### 3.3 총 목적 함수 (비선형 최소제곱)

$$F(\Theta) = \sum_{k,h} \|\mathbf{r}_{k,h}^{\text{circle}}\|^2 + \sum_{k,i,j} \|\mathbf{r}_{k,ij}^{\text{corner}}\|^2$$

---

## 4. 최적화 알고리즘 (Levenberg-Marquardt)

잔차 벡터 $\mathbf{r} \equiv \mathbf{r}(\Theta) \in \mathbb{R}^m$, 야코비안 $\mathbf{J} = \partial\mathbf{r}/\partial\Theta \in \mathbb{R}^{m \times n}$에 대해, 매 반복마다 **댐핑된 정규 방정식**을 푼다:

$$(\mathbf{J}^{\top}\mathbf{J} + \lambda\mathbf{I})\Delta\Theta = -\mathbf{J}^{\top}\mathbf{r}$$

업데이트: $\Theta \leftarrow \Theta + \Delta\Theta$

댐핑 계수 $\lambda > 0$는 오차가 줄어들면 작게, 늘어나면 크게 조절하여 Gradient Descent와 Gauss-Newton 사이를 부드럽게 전환한다.

### 야코비안 주요 항

**내부파라미터:**

$$\frac{\partial u}{\partial f_x} = x', \quad \frac{\partial v}{\partial f_y} = y', \quad \frac{\partial u}{\partial c_x} = 1, \quad \frac{\partial v}{\partial c_y} = 1$$

**왜곡 계수** ($r^2 = x^2 + y^2$):

$$\frac{\partial u}{\partial k_1} = f_x \cdot x \cdot r^2, \quad \frac{\partial v}{\partial k_1} = f_y \cdot y \cdot r^2$$

**외부파라미터 (angle-axis):**

$$\frac{\partial \mathbf{P}^C}{\partial \boldsymbol\omega} \approx -[\mathbf{P}^C]_\times, \quad \frac{\partial \mathbf{P}^C}{\partial \mathbf{t}} = \mathbf{I}_3$$

---

## 5. 파이프라인 구조

### 5.1 캘리브레이션 보드 설계

중앙에 체커보드 패턴, 주변에 4개의 원형 구멍을 배치한 하이브리드 보드를 사용한다.
- **체커보드**: 카메라에서 코너 점을 서브픽셀 정확도로 검출
- **원형 홀**: LiDAR 포인트클라우드에서 깊이 불연속으로 검출 — 회전 방향에 무관하게 안정적

### 5.2 센서별 타겟 검출

- **카메라**: OpenCV `findChessboardCorners` → 코너 픽셀 좌표 세트
- **LiDAR**: RANSAC 평면 피팅 → 보드 평면 추출 → 원형 홀 마스크 패턴 매칭 (yaw + 2D 이동 검색) → 4개 홀 중심 3D 좌표

### 5.3 초기 파라미터 추정 (Zhang 방법)

체커보드 코너로부터 **카메라 내부파라미터** $K$와 **보드-카메라 외부파라미터** $(R_{B \to C}, t_{B \to C})$를 초기 산출한다.

### 5.4 공동 최적화

체커보드 코너 잔차 + 원형 홀 잔차를 하나의 비용 함수로 합쳐 LM 솔버로 최적화한다.

---

## 6. C++ 모듈 구조

| 단계 | 모듈 | 핵심 클래스 / 라이브러리 |
| :--- | :--- | :--- |
| 데이터 IO | `SensorBagIO` | OpenCV `cv::imread`, PCL `pcl::io::loadPCDFile` |
| 타겟 검출 | `BoardDetector` | OpenCV `findChessboardCorners`, PCL RANSAC |
| 초기 카메라 보정 | `CameraCalibrator` | OpenCV `cv::calibrateCamera` (Zhang 방법) |
| 초기 외부파라미터 | `ExtrinsicInit` | 보드 정렬 또는 하드웨어 설계값 기반 |
| 공동 최적화 | `JointOptimizer` | Ceres Solver (`AutoDiffCostFunction`) |
| 검증 및 저장 | `CalibResult` | YAML 저장 및 시각화 |

실행 순서: **load → detect → initial-calib → init-extrinsic → joint-opt → export**

---

## 7. 벤치마킹 프로토콜

1. 보드가 FOV 전체를 커버하도록 10개 이상의 포즈 수집 (근거리 + 원거리)
2. 2단계 baseline과 공동 최적화 방법을 각 5회 실행 (랜덤 초기 노이즈)
3. 보고 지표:

$$\text{RPE}_{\text{circle}} = \frac{1}{N}\sum_{k,h}\|\mathbf{r}_{k,h}^{\text{circle}}\|, \quad E_R = \angle(\mathbf{R}_{\text{est}}, \mathbf{R}_{\text{gt}}^{\top}), \quad E_t = \|\mathbf{t}_{\text{est}} - \mathbf{t}_{\text{gt}}\|$$

4. 포인트클라우드를 카메라 이미지에 오버레이하여 정성적 확인

---

## 결론

공동 최적화는 카메라 내부파라미터를 고정된 상수가 아닌 잠재 변수로 처리함으로써, LiDAR-카메라 정합 시 발생하는 시스템적 오차를 제거한다. 원형 홀과 체커보드 잔차를 동시에 사용하는 LM 솔버는 거친 초기값에서도 안정적으로 수렴하며, 실전에서 서브픽셀 수준의 재투영 오차를 달성한다.

</div>
</div>
