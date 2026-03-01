---
layout: post
title: "제약조건 하의 직사각형 위치 최적화 — 수학적 이론과 구현"
tags: [Optimization, Math, C++, Levenberg-Marquardt, Newton-Raphson]
---

내부 직사각형이 외부 직사각형의 경계를 침범하지 않도록 위치와 방향을 최적화하는 **제약 최적화 문제(Constrained Optimization Problem)**의 수학적 이론과 구현을 다룬다.

## 1. 문제 설정

- **목표**: 내부 직사각형의 최적 위치와 방향 결정
- **제약조건**: 내부 직사각형의 모든 점이 외부 직사각형 내부에 위치
- **최적화 변수**: 중심 좌표 $(x_c, y_c)$ 및 회전 각도 $\theta$

## 2. 수학적 정의

### 2.1 최적화 문제의 형식적 정의

$$\begin{align} \min_{\mathbf{x}} \quad & f(\mathbf{x}) \\ \text{s.t.} \quad & g_i(\mathbf{x}) \leq 0, \quad i = 1, 2, ..., m \end{align}$$

여기서:
- **결정 변수**: $\mathbf{x} = [x_c, y_c, \theta]^T \in \mathbb{R}^3$
- **목적 함수**: $f: \mathbb{R}^3 \rightarrow \mathbb{R}$
- **제약 함수**: $g_i: \mathbb{R}^3 \rightarrow \mathbb{R}$

### 2.2 패널티 방법을 통한 무제약 문제로의 변환

제약 최적화 문제를 **외부 패널티 함수(Exterior Penalty Function)** 방법으로 변환한다:

$$\min_{\mathbf{x}} \quad F(\mathbf{x}, \mu) = f(\mathbf{x}) + \mu \sum_{i=1}^{m} \max(0, g_i(\mathbf{x}))^2$$

여기서 $\mu > 0$는 패널티 파라미터이다.

## 3. 좌표 체계 및 기하학적 표현

### 3.1 좌표계 정의

```
원점 O(0,0) ────────────────> X축 (양의 방향)
     │
     │
     ▼
    Y축 (양의 방향)
```

- **원점**: 이미지의 좌상단 (0,0)
- **X축**: 오른쪽 방향 (→), **Y축**: 아래쪽 방향 (↓)
- **각도 측정**: 시계방향이 양의 각도

### 3.2 직사각형의 수학적 표현

직사각형 $\mathcal{R}$은 중심 $\mathbf{c} = (x_c, y_c)^T$, 크기 (너비 $w$, 높이 $h$), 회전 각도 $\theta$로 정의된다.

### 3.3 회전 변환

2차원 회전 행렬:

$$\mathbf{R}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

직사각형의 $i$번째 꼭지점의 전역 좌표:

$$\mathbf{v}_i^{\text{global}} = \mathbf{R}(\theta) \mathbf{v}_i^{\text{local}} + \mathbf{c}$$

로컬 좌표계의 꼭지점:

$$\mathbf{v}_1^{\text{local}} = (-w/2, -h/2)^T, \quad \mathbf{v}_2^{\text{local}} = (w/2, -h/2)^T$$
$$\mathbf{v}_3^{\text{local}} = (w/2, h/2)^T, \quad \mathbf{v}_4^{\text{local}} = (-w/2, h/2)^T$$

## 4. 목적 함수의 수학적 구조

### 4.1 목적 함수 정의

$$f(\mathbf{x}) = \sum_{i=1}^{4} P_i(\mathbf{x}) + \lambda \cdot d^2(\mathbf{x})$$

- $P_i(\mathbf{x})$: $i$번째 꼭지점의 침범 패널티
- $d^2(\mathbf{x})$: 중심 간 거리 제곱
- $\lambda$: 정규화 파라미터

### 4.2 침범 패널티 함수

각 꼭지점 $\mathbf{v}_i = (v_{i,x}, v_{i,y})^T$에 대한 경계별 패널티:

$$p_{i,L}(\mathbf{x}) = \max(0, x_{\text{left}} - v_{i,x})^2$$
$$p_{i,R}(\mathbf{x}) = \max(0, v_{i,x} - x_{\text{right}})^2$$
$$p_{i,T}(\mathbf{x}) = \max(0, y_{\text{top}} - v_{i,y})^2$$
$$p_{i,B}(\mathbf{x}) = \max(0, v_{i,y} - y_{\text{bottom}})^2$$

### 4.3 중심 정렬 항

$$d^2(\mathbf{x}) = (x_c - x_c^{\text{outer}})^2 + (y_c - y_c^{\text{outer}})^2$$

## 5. C++ 구현

```cpp
#include <cmath>

class RectangleOptimizer {
public:
    static double computeCost(const Rectangle& inner, const Rectangle& outer) {
        double penalty = 0.0;
        auto inner_vertices = inner.getVertices();

        float outer_left = outer.center.x - outer.width/2;
        float outer_right = outer.center.x + outer.width/2;
        float outer_top = outer.center.y - outer.height/2;
        float outer_bottom = outer.center.y + outer.height/2;

        for (const auto& v : inner_vertices) {
            if (v.x < outer_left) penalty += std::pow(outer_left - v.x, 2);
            if (v.x > outer_right) penalty += std::pow(v.x - outer_right, 2);
            if (v.y < outer_top) penalty += std::pow(outer_top - v.y, 2);
            if (v.y > outer_bottom) penalty += std::pow(v.y - outer_bottom, 2);
        }

        double center_align = std::pow(inner.center.x - outer.center.x, 2) +
                             std::pow(inner.center.y - outer.center.y, 2);

        return penalty + 0.1 * center_align;
    }

    static JwMatrix computeGradient(const Rectangle& inner, const Rectangle& outer,
                                     double eps = 1.0) {
        JwMatrix grad(3, 1);
        double base_cost = computeCost(inner, outer);

        Rectangle temp = inner;
        temp.center.x += eps;
        grad.at(0, 0) = (computeCost(temp, outer) - base_cost) / eps;

        temp = inner;
        temp.center.y += eps;
        grad.at(1, 0) = (computeCost(temp, outer) - base_cost) / eps;

        temp = inner;
        temp.angle += eps * 0.01;
        grad.at(2, 0) = (computeCost(temp, outer) - base_cost) / (eps * 0.01);

        return grad;
    }
};
```

## 6. 최적화 알고리즘 비교

### 6.1 Gradient Descent

반복 갱신 규칙:

$$\mathbf{x}^{(k+1)} = \mathbf{x}^{(k)} - \alpha \nabla f(\mathbf{x}^{(k)})$$

Lipschitz 연속 그래디언트 가정 하의 수렴 보장 조건: $0 < \alpha < \frac{2}{L}$

### 6.2 Newton-Raphson

2차 Taylor 근사로부터 Newton 방향:

$$\nabla^2 f(\mathbf{x}) \mathbf{p}_N = -\nabla f(\mathbf{x})$$

갱신: $\mathbf{x}^{(k+1)} = \mathbf{x}^{(k)} + \mathbf{p}_N^{(k)}$

### 6.3 Levenberg-Marquardt

수정된 시스템:

$$[\nabla^2 f(\mathbf{x}) + \mu \mathbf{I}] \mathbf{p}_{LM} = -\nabla f(\mathbf{x})$$

적응적 파라미터 조정:
- **비용 감소**: $\mu \leftarrow \mu / \rho$
- **비용 증가**: $\mu \leftarrow \mu \cdot \rho$ (일반적으로 $\rho = 10$)

## 7. 수치적 미분

### 그래디언트 (전진 차분법)

$$\frac{\partial f}{\partial x_i} \approx \frac{f(\mathbf{x} + h\mathbf{e}_i) - f(\mathbf{x})}{h}$$

### 헤시안 대각 성분

$$\frac{\partial^2 f}{\partial x_i^2} \approx \frac{f(\mathbf{x} + h\mathbf{e}_i) - 2f(\mathbf{x}) + f(\mathbf{x} - h\mathbf{e}_i)}{h^2}$$

### 헤시안 비대각 성분 (4점 공식)

$$\frac{\partial^2 f}{\partial x_i \partial x_j} \approx \frac{f_{++} - f_{+-} - f_{-+} + f_{--}}{4h^2}$$

스케일링: $h_{\text{position}} = 1.0$ (픽셀), $h_{\text{angle}} = 0.01$ (라디안)

## 8. 구체적 수치 예시 (4x4 영역)

**초기값**: $\mathbf{p}_0 = [1.0, 1.5, \pi/3]^T$ (60도 회전)

### Newton-Raphson 풀이

**반복 1**: 회전 행렬 계산 → 꼭지점 좌표 → 비용 $f_0 = 0.1295$

그래디언트: $\nabla f = [-0.2, -0.1, 0.0447]^T$

헤시안:
$$H = \begin{bmatrix} 0.2 & 0 & 0.0894 \\ 0 & 0.2 & 0.0447 \\ 0.0894 & 0.0447 & 1.8447 \end{bmatrix}$$

스텝: $\Delta \mathbf{p}_1 = [1.0417, 0.5208, 0.0208]^T$

**반복 5 수렴 결과**: $\mathbf{p}^* = [2.000, 2.000, 0.000]^T$, $f(\mathbf{p}^*) = 2.3 \times 10^{-8}$

## 9. 알고리즘 5회 반복 결과 비교

| 반복 | Gradient Descent | Newton-Raphson | Levenberg-Marquardt |
|:---:|:---:|:---:|:---:|
| **0** | `[100, 200, 0]` | `[100, 200, 0]` | `[100, 200, 0]` |
| **1** | `[239.6, 4128, 0]` | `[96.7, 220.3, 0]` | `[96.7, 220.3, 0]` |
| **2** | `[264.8, 5901.6, 0]` | `[94.0, 240.1, 0]` | `[94.0, 240.1, 0]` |
| **3** | `[278.8, 6834.8, 0]` | `[91.8, 258.9, 0]` | `[92.8, 249.0, 0]` |
| **4** | `[287.5, 7345.4, 0]` | `[90.1, 276.2, 0]` | `[91.1, 267.8, 0]` |
| **5** | `[293.0, 7627.7, 0]` | `[88.9, 292.1, 0]` | `[89.9, 285.4, 0]` |

## 10. 수렴성 분석

| 알고리즘 | 수렴 차수 | 특징 |
|---------|----------|------|
| Gradient Descent | $O(1/k)$ sublinear | 학습률에 민감, 발산 위험 |
| Newton-Raphson | Quadratic (2차) | 빠르지만 헤시안 양정치성 필요 |
| Levenberg-Marquardt | Superlinear (초선형) | $\lambda$ 조정으로 안정성 확보, 초기값에 덜 민감 |

## 결론

세 가지 최적화 알고리즘을 동일한 제약 직사각형 최적화 문제에 적용한 결과:

1. **Gradient Descent**: 학습률에 매우 민감하다. 적절한 학습률을 찾지 못하면 발산하기 쉽고, 수렴 속도가 느리다.
2. **Newton-Raphson**: 2차 미분 정보(헤시안)를 사용하여 매우 빠르게 수렴하지만, 헤시안이 양정치가 아니면 불안정해질 수 있다.
3. **Levenberg-Marquardt**: Newton과 GD를 절충한 형태로, $\lambda$ 값을 조절하여 안정성과 속도의 균형을 맞춘다. 가장 안정적으로 최적해를 찾아간다.

KKT 조건 검증:
- **정상성**: $\|\nabla f(\mathbf{x}^*)\| < 10^{-6}$
- **실행가능성**: 모든 꼭지점이 경계 내부에 위치
- **상보성**: 활성 제약에서만 패널티 발생
