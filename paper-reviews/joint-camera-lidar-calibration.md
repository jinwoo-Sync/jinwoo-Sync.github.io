---
layout: default
title: "Joint Camera-LiDAR Calibration"
paper: "Joint Camera Intrinsic and LiDAR–Camera Extrinsic Calibration"
authors: "Yan et al."
venue: "arXiv:2202.13708, 2022"
tags: [Calibration, LiDAR, Camera, Joint Optimization, Ceres]
---

# Joint Camera Intrinsic & LiDAR–Camera Extrinsic Calibration

**Yan et al. — arXiv:2202.13708, 2022**

---

## 개요

다중 센서 융합을 위한 정확한 **센서 캘리브레이션**은 자율주행 등의 분야에서 매우 중요하다. 특히 **LiDAR-카메라 캘리브레이션**에서는 일반적으로 카메라의 내부파라미터(intrinsics)를 먼저 추정하고, 그 결과를 바탕으로 외부파라미터(extrinsics)를 추정하는 2단계 방법을 사용한다. 하지만 이 방식은 카메라 내부파라미터의 미세한 오류가 외부파라미터 정합에 악영향을 준다는 문제가 있다.

이 연구는 **카메라 내부 및 LiDAR-카메라 외부 파라미터를 한 번의 최적화로 공동 추정**하는 방법을 제안한다.

---

## 1. 수학적 표기법 및 좌표계

| 기호 | 설명 |
| :--- | :--- |
| $\mathcal{C}$ | 카메라 좌표계 |
| $\mathcal{L}$ | LiDAR 좌표계 |
| $\mathcal{B}$ | 캘리브레이션 보드 (체커보드) 좌표계 |
| $\mathbf{K}$ | 카메라 내부파라미터 행렬 |
| $\mathbf{D}(\cdot)$ | 왜곡 모델 (Radial / Tangential) |
| $\mathbf{R}_{A\to B}, \mathbf{t}_{A\to B}$ | 프레임 A에서 B로의 회전 및 이동 변환 |

최종적으로 구하고자 하는 파라미터 벡터 $\Theta$는 다음과 같이 정의된다:

**$$\Theta = [\underbrace{f_x, f_y, c_x, c_y, k_1, k_2, \dots}_{\Theta_{\text{int}}}, \underbrace{\boldsymbol\omega_{L\to C}, \mathbf{t}_{L\to C}}_{\Theta_{\text{ext}}}]^{\top}$$**

---

## 2. 관측 모델 (Projection)

3D 공간의 점 $\mathbf{P}^F$를 이미지 평면상의 2D 좌표 $\mathbf{p}$로 투영하는 식은 다음과 같다:

**$$\mathbf{p} = \begin{bmatrix}u \\ v \\ 1\end{bmatrix} = \mathbf{K} \cdot \mathbf{d}\left(\underbrace{\mathbf{R}_{F\to C} \mathbf{P}^F + \mathbf{t}_{F\to C}}_{\mathbf{P}^C}\right)$$**

여기서 왜곡 연산자 $\mathbf{d}$는 $z$로 나누는 정규화 과정을 포함하며 왜곡 모델을 적용한다.

---

## 3. 잔차(Residual) 및 비용 함수

### 3.1 체커보드 코너 잔차
이미지 $k$의 코너 $(i, j)$에 대해:
**$$\mathbf{r}_{k,ij}^{\text{corner}} = \mathbf{p}_{k,ij}^{\text{proj}}(\Theta) - \mathbf{p}_{k,ij}^{\text{obs}}$$**

### 3.2 원형 홀(Circle-hole) 잔차
LiDAR로 검출한 3D 중심점 $\mathbf{P}_{k,h}^{\mathcal{L}}$을 이미지로 투영한 값과 보드 설계상 예상되는 2D 중심점 $\mathbf{p}_{k,h}^{\text{obs}}$ 사이의 오차:
**$$\mathbf{r}_{k,h}^{\text{circle}} = \mathbf{p}_{k,h}^{\text{proj}}(\Theta) - \mathbf{p}_{k,h}^{\text{obs}}$$**

### 3.3 총 목적 함수 (비선형 최소제곱)
**$$F(\Theta) = \sum_{k,h} \|\mathbf{r}_{k,h}^{\text{circle}}\|^2 + \sum_{k,i,j} \|\mathbf{r}_{k,ij}^{\text{corner}}\|^2$$**

---

## 4. 최적화 알고리즘 (Levenberg–Marquardt)

Ceres Solver 등을 사용하여 **댐핑된 정규 방정식(Damped Normal Equations)**을 푼다:

**$$(\mathbf{J}^{\top}\mathbf{J} + \lambda\mathbf{I})\Delta\Theta = -\mathbf{J}^{\top}\mathbf{r}$$**

여기서 $\mathbf{J}$는 야코비안 행렬($\partial\mathbf{r}/\partial\Theta$)이며, 업데이트 규칙은 $\Theta \leftarrow \Theta + \Delta\Theta$이다. 댐핑 계수 $\lambda$는 오차가 줄어들면 작게, 늘어나면 크게 조절하여 최적해로 수렴시킨다.

---

## 5. C++ 모듈 구조

| 단계 | 모듈 | 핵심 클래스 / 라이브러리 |
| :--- | :--- | :--- |
| 데이터 IO | `SensorBagIO` | OpenCV `cv::imread`, PCL `pcl::io::loadPCDFile` |
| 타겟 검출 | `BoardDetector` | OpenCV `findChessboardCorners`, PCL RANSAC |
| 초기 카메라 보정 | `CameraCalibrator` | OpenCV `cv::calibrateCamera` (Zhang 방법) |
| 초기 외부파라미터 | `ExtrinsicInit` | 보드 정렬 또는 하드웨어 설계값 기반 |
| 공동 최적화 | `JointOptimizer` | Ceres Solver (`AutoDiffCostFunction`) |
| 검증 및 저장 | `CalibResult` | YAML 저장 및 시각화 |

---

## 결론

공동 최적화는 카메라 내부파라미터를 고정된 상수가 아닌 잠재 변수로 처리함으로써, LiDAR-카메라 정합 시 발생하는 시스템적 오차를 제거한다. 원형 홀과 체커보드 잔차를 동시에 사용하는 LM 솔버는 거친 초기값에서도 안정적으로 수렴하며, 실전에서 서브픽셀 수준의 재투영 오차를 달성한다.
