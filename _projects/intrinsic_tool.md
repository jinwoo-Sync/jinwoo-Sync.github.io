---
layout: project
title: "Camera Intrinsic Calibration Tool - Key Technical Contributions"
period: "2022.03 ~ Present"
category: "Calibration"
tech: "C++, Qt, OpenCV, Eigen (LM Optimization)"
role: "Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

사내 MMS(Mobile Mapping System) 핵심 프로그램인 Replica Package의 **Camera Intrinsic Calibration Tool** 개발에 참여하며, 아래 3가지 핵심 기술을 직접 구현하거나 깊이 이해하고 유지보수한 내용을 정리합니다.

---

## 1. MTF (Modulation Transfer Function) 분석 - 수식 변형 및 UI 구현

> **기여도: 오픈소스 기반 수식 변형 및 Tool UI 통합**

오픈소스 MTF 분석 코드를 기반으로, 사내 Intrinsic Calibration Tool의 용도에 맞게 **Contrast 수식을 변형**하고 Qt UI에 통합하여 실시간 시각화 기능을 구현했습니다.

### MTF 수식 변형: Michelson Contrast → 체커보드 특화

일반적인 MTF 측정에는 Michelson Contrast 수식이 사용되지만, 캘리브레이션 Tool에서는 **야외 배경이 아닌 체커보드 내부의 흑백 패턴만을 대상**으로 하기 때문에, 분모를 `Imax`로 단순화하여 체커보드 영역의 명암 대비에 집중하도록 변형했습니다.

```
                     Imax - Imin
일반 Michelson  =  ───────────────
                     Imax + Imin


                     Imax - Imin
본 구현 (변형)  =  ───────────────
                       Imax
```

**변형 이유:**
- Michelson 수식은 전체 장면의 밝기 분포를 고려하여 `Imax + Imin`으로 정규화
- 체커보드 캘리브레이션 환경에서는 **흰색 영역(Imax)이 기준**이 되므로, `Imax`만으로 정규화하는 것이 흑백 경계의 선명도를 더 직관적으로 반영
- 결과값 범위: 0(경계 없음) ~ 1(완벽한 흑백 대비)

### 구현 코드 (ProfileDetection.cpp)

```cpp
// Imax 기준 정규화 후 Contrast 계산
minMean /= maxMean;   // normalize by Imax
maxMean /= maxMean;   // = 1.0
m_mtfValue = maxMean - minMean;  // = 1 - (Imin/Imax)
```

### UI 통합

오픈소스의 신호처리 파이프라인(Moving Average → Smoothing → Derivative → Local Max/Min Detection)을 활용하여, 변형된 수식의 결과를 Qt UI 위에 실시간으로 시각화하도록 구현했습니다.

- **녹색 곡선**: Smoothing 처리된 1D 밝기 프로파일 (이미지 중앙 수평선)
- **빨간 원**: 검출된 Local Maximum (Imax 후보)
- **노란 원**: 검출된 Local Minimum (Imin 후보)
- **label_MTF**: 계산된 MTF 값 실시간 표시

<!-- TODO: MTF 시각화 스크린샷 추가 -->
<!-- ![MTF Analysis](/assets/images/projects/intrinsic_tool/mtf_analysis.png) -->

---

## 2. Intrinsic Calibration 검증 - Projection Error 방식 구현

> **기여도: Projection Error 방식 직접 구현 / 3가지 검증 방법 모두 사내 활용**

Camera Intrinsic Parameter(K, distortion)의 품질을 검증하기 위해 사내에서 **3가지 검증 방법**을 사용했으며, 그 중 **Projection Error(재투영 오차) 시각화 방식**을 직접 구현했습니다.

### 사내 3가지 Intrinsic 검증 방법

| 방법 | 원리 | 평가 대상 |
|------|------|-----------|
| **Projection Error (XY Map)** | 3D → 2D 재투영 후 검출점과의 오차 | Intrinsic + Extrinsic 통합 정확도 |
| **Simulation Grid** | 현재 K, dist로 가상 격자를 투영하여 왜곡 패턴 확인 | 렌즈 왜곡 모델의 적합성 |
| **Homography Residual** | Undistort → Homography → 이상 격자와 비교 | Intrinsic 단독 정확도 (Extrinsic 무관) |

### [직접 구현] Projection Error 시각화

3D Object Point를 현재 Calibration Parameter로 2D에 재투영(Re-projection)한 뒤, 실제 검출된 2D Corner와의 오차를 시각화하는 방식입니다.

**수학적 원리:**

```
[World → Camera]    Xc = R · Xw + t
[Camera → Norm]     x' = Xc/Zc, y' = Yc/Zc
[Norm → Distorted]  x_d = distort(x', y')    // Pinhole / Fisheye / OmniDir
[Distorted → Pixel] (u, v) = K · (x_d, y_d, 1)^T

Error = detected_2D - projected_2D
RMS = sqrt( sum(error²) / N )
```

**시각화 방식:**
- 2000 x 2000 pixel 검정 캔버스, 중심 = (1000, 1000)
- 오차 벡터를 **100배 확대**하여 표시
- 중심에 밀집 = 좋은 캘리브레이션 / 패턴 존재 = 모델 부적합

**해석 기준:**

| 시각적 패턴 | 의미 |
|-------------|------|
| 중심 근처에 밀집 | 캘리브레이션 정확도 양호 |
| 방사형 패턴 | Radial Distortion 보정 부족 |
| 한쪽 편향 | Principal Point 추정 오류 |
| 전체 산개 | Intrinsic Parameter 전반적 부정확 |

### [사내 활용] Simulation Grid

현재 추정된 K, distortion 파라미터로 가상 3D 격자를 투영하여 렌즈 왜곡 패턴을 직관적으로 확인하는 방법입니다.

- 가상 카메라: R = Identity, t = (0, 0, focal_length)
- Z = focal 평면에 균일 간격 3D 격자 생성 후 현재 K, dist로 투영
- **직선 격자** = 왜곡 없음 / **바깥으로 볼록** = Barrel Distortion / **안으로 오목** = Pincushion

### [사내 활용] Homography Residual

Extrinsic 영향을 배제하고 Intrinsic만의 품질을 평가하는 방법입니다.

- 검출된 2D점을 Undistort → Normalized 좌표 획득
- `cv::findHomography()`로 이상적 격자 좌표와 매핑
- 잔차를 **10,000배 확대** 시각화 (분홍색 원)
- 잔차가 작을수록 K, distortion 추정 품질이 높음

<!-- TODO: 3가지 검증 방법 비교 이미지 추가 -->
<!-- ![Verification Methods](/assets/images/projects/intrinsic_tool/verification_comparison.png) -->

---

## 3. FOV Coverage Visualization (Pose Map) - 학습 및 유지보수

> **기여도: 원리 학습 및 유지보수 담당** (원 구현자는 다른 팀원)

캘리브레이션 시 체커보드가 FOV(Field of View) 전체를 고르게 커버하는지 확인하기 위한 시각화 기능입니다. 직접 구현하지는 않았지만, 해당 코드의 원리를 깊이 이해하고 유지보수를 담당했습니다.

### 핵심 원리: Sin/Cos 기반 거리별 색상 인코딩

체커보드의 촬영 거리(`tvec[2]`, Z축 Translation)에 Sin/Cos 함수를 적용하여, **같은 거리의 보드는 같은 색**, **다른 거리는 다른 색**으로 구분되도록 색상을 매핑합니다.

```cpp
float fPeriod = tvec[2] * 2;     // 거리 기반 주기
float dSin = sin(fPeriod);       // [-1, +1]
float dCos = cos(fPeriod);       // [-1, +1]

// BGR 색상 매핑 (각 채널 0~255)
B = (+dSin + 1) * 127.5;   // Sin 기반
G = (-dCos + 1) * 127.5;   // Cos 기반 (위상 차이)
R = (-dSin + 1) * 127.5;   // -Sin 기반 (보색)
```

### 왜 Sin/Cos인가?

| 특성 | 이점 |
|------|------|
| **연속적** | 거리 변화에 따라 색상이 부드럽게 전환 |
| **주기적** | 값 범위가 [-1, +1]로 제한되어 0~255 매핑에 적합 |
| **위상 차이** | Sin과 Cos의 90도 위상 차이로 R/G/B 채널 간 독립적 변화 |
| **직관적** | 가까운 거리 = 유사한 색 → 같은 보드의 점이 같은 색으로 표시 |

### 거리별 색상 변화 예시

```
거리 ≈ 0    : Sin≈0, Cos≈1  → 보라색 (B:127, G:0,   R:127)
거리 ≈ π/4  : Sin≈1, Cos≈0  → 주황색 (B:255, G:127, R:0)
거리 ≈ π/2  : Sin≈0, Cos≈-1 → 청록색 (B:127, G:255, R:127)
거리 ≈ 3π/4 : Sin≈-1, Cos≈0 → 파란색 (B:0,   G:127, R:255)
```

### 실무 활용

- 전체 이미지 위에 검출된 모든 체커보드 코너를 오버레이
- **색상 분포가 고른 경우** = FOV 전체에 다양한 거리에서 데이터 확보
- **특정 색만 편중된 경우** = 특정 거리/영역에만 데이터가 편중 → 추가 촬영 필요
- 캘리브레이션 데이터의 공간적 균일성을 한 눈에 파악 가능

<!-- TODO: Pose Map 시각화 스크린샷 추가 -->
<!-- ![Pose Map](/assets/images/projects/intrinsic_tool/pose_map.png) -->

---

## Technical Stack

| 구성요소 | 기술 |
|----------|------|
| GUI Framework | Qt5 (Widget 기반) |
| Image Processing | OpenCV (Calibration, Projection, Undistortion) |
| Optimization | Eigen (Levenberg-Marquardt) |
| Camera Models | Pinhole, Fisheye, OmniDir (CMei) |
| Board Detection | Chessboard, CirclesGrid, ChArUco |
| IPC | Shared Memory (실시간 센서 데이터 스트리밍) |
