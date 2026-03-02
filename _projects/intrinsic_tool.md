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

사내 MMS(Mobile Mapping System) 핵심 프로그램인 Replica Package의 **Camera Intrinsic Calibration Tool** 개발에 참여하며, 아래 핵심 기술을 직접 구현하거나 깊이 이해하고 유지보수한 내용을 정리합니다.

## 개발 참여 배경

2023년, 사내 Intrinsic Calibration Tool은 기존 ROS 기반 도구를 전면 배제하고 자체적으로 설계된 프로젝트로, **Levenberg-Marquardt 비선형 최적화 도입**, **체커보드 자동 Detection**, **카메라-라이다 로깅 툴과의 IPC 실시간 연동** 등 기능적/기술적으로 방대한 일감이 만들어진 프로젝트였습니다.

해당 시기 저는 이제 막 **주니어 개발자**로서 프로그램 제작에 참여하게 되었고, 연차와 경험이 부족하여 전체 시스템 아키텍처나 최적화 엔진 설계에 대한 이해가 깊지 않았습니다. 그러나 당시 팀원들의 배려로 상대적으로 진입 장벽이 낮은 부분들을 담당하게 되었으며, 그 과정에서 **MTF 수식 변형 및 UI 통합**과 **Projection Error 시각화 구현**을 직접 수행했습니다.

---

## 프로그램 전체 개요

Intrinsic Calibration Tool은 카메라 내부 파라미터(K, distortion)를 추정하고 검증하는 Qt 기반 GUI 도구입니다.

![Tool 전체 UI 구성](/assets/images/projects/intrinsic_tool/figma_design.png)
*Tool UI 구성 설계 문서 - 카메라 모델 선택(Pinhole/Fisheye/OmniDir), 보드 타입(Chessboard/Circle/ChArUco), 5x5 그리드 기반 FOV 커버리지 맵, 캘리브레이션 결과 저장/최적화 기능*

### 핵심 파이프라인

```
이미지 입력 (IPC 공유 메모리 / 파일)
    → 2D Corner Detection (체커보드 코너 검출)
    → 3D Object Point 대응 (Z=0 평면 격자)
    → OpenCV Calibration (K, dist 초기 추정)
    → Levenberg-Marquardt 비선형 최적화 (Eigen)
    → 3D→2D Re-projection + 시각화
    → Intrinsic 품질 검증 (3가지 방식)
```

### 지원 기능

| 기능 | 설명 |
|------|------|
| 카메라 모델 | Pinhole, Fisheye, OmniDir (CMei) |
| 보드 타입 | Chessboard, CirclesGrid, ChArUco |
| 왜곡 모델 | k1~k3 / k1~k2 / k1~k6+s1~s4 (Rational) |
| 실시간 연동 | IPC 공유 메모리로 로깅 툴과 실시간 스트리밍 |
| 자동 데이터 선별 | 5x5 그리드 기반 FOV 균등 배치 |
| LM 최적화 | Intrinsic + Extrinsic 동시 최적화 |

![캘리브레이션 파라미터 패널](/assets/images/projects/intrinsic_tool/tool_ui_overview.png)
*우측 패널: 보드/모델 선택, X/Y Pose Map, XY Error Map, Projection Error/FocalLength/FOV 정보 표시*

---

## 프로그램 주요 기능 스크린샷

### 왜곡 격자(Distortion Grid) 시각화

현재 추정된 K, distortion 파라미터로 가상 3D 격자를 투영하여, 렌즈 왜곡 패턴을 이미지 위에 **파란색 격자선**으로 표시합니다.

![왜곡 격자 시각화](/assets/images/projects/intrinsic_tool/distortion_grid.png)
*파란색 격자선이 이미지 위에 오버레이된 모습. 격자의 휨 정도가 렌즈 왜곡의 크기와 방향을 직관적으로 보여준다. 우측 5x5 그리드의 초록/빨강은 해당 영역의 데이터 취득 여부를 나타낸다.*

### Intrinsic이 불안정할 때

![불안정한 Intrinsic](/assets/images/projects/intrinsic_tool/unstable_intrinsic.png)
*Intrinsic이 안정화되지 않았을 때의 모습. 파란색 격자선이 극도로 왜곡되어 이미지 전체를 뒤덮고 있다. 이 상태에서는 Optimizer 기능을 통해 누적 데이터 전체로 재캘리브레이션하여 개선할 수 있다.*

### 이미지 전 영역 캘리브레이션 데이터 취득

![전 영역 데이터 취득](/assets/images/projects/intrinsic_tool/full_fov_coverage.png)
*detect area 체크 시 빨간색 영역으로 아직 데이터가 부족한 FOV 영역을 시각화. 이미지 전체 영역에 걸쳐 체커보드 데이터를 균등하게 확보해야 좋은 캘리브레이션 결과를 얻을 수 있다.*

### 캘리브레이션 진행 중

![최적화 진행 중](/assets/images/projects/intrinsic_tool/optimization_in_progress.png)
*체커보드 코너 검출(무지개색 점) + 파란색 왜곡 격자 + 분홍색 Homography 잔차 원이 동시에 표시된 모습. 우측 5x5 그리드에서 초록색 영역은 데이터가 확보된 영역, 빨간색은 미확보 영역이다.*

### 왜곡 보정 뷰 (Undistort View)

![Undistort 뷰](/assets/images/projects/intrinsic_tool/undistort_view.png)
*undistort view 체크 시 현재 K, dist로 왜곡을 보정한 이미지를 표시. 파란색 격자선이 직선으로 표시되면 왜곡 보정이 정상적으로 이루어지고 있음을 의미한다. 이미지 가장자리의 검은 영역은 왜곡 보정 과정에서 발생하는 자연스러운 현상이다.*

### 실시간 라이다 투영

![라이다 실시간 투영](/assets/images/projects/intrinsic_tool/lidar_realtime.png)
*lidar projection 체크 시 IPC 공유 메모리로 수신한 라이다 포인트 클라우드를 현재 Extrinsic으로 이미지 위에 실시간 투영. 카메라-라이다 간 Extrinsic 정합 상태를 직관적으로 확인할 수 있다.*

### 양호한 캘리브레이션 결과

![양호한 캘리브레이션](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*캘리브레이션이 잘 된 상태의 전체 화면. 이미지 위에 파란색 왜곡 격자(자연스러운 곡선), 초록색 Re-projection 원(검출점과 일치), 분홍색 Homography 잔차(중앙 밀집), 녹색 MTF 프로파일 곡선이 동시에 표시된다. 우측 하단 INFO에서 ProjectionErr: 0.2 수준의 sub-pixel 정확도를 확인할 수 있다.*

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
// Local Max/Min의 평균값 계산
float maxMean = 0, minMean = 0;
for (int i = 0; i < veclocalMax.size(); i++)
    maxMean += veclocalMax[i].y;
maxMean /= veclocalMax.size();

for (int i = 0; i < veclocalMin.size(); i++)
    minMean += veclocalMin[i].y;
minMean /= veclocalMin.size();

// Imax 기준 정규화 후 Contrast 계산
minMean /= maxMean;   // normalize by Imax → Imin/Imax
maxMean /= maxMean;   // = 1.0
m_mtfValue = maxMean - minMean;  // = 1 - (Imin/Imax)
```

### 신호처리 파이프라인

이미지 중앙 수평선을 따라 밝기 프로파일을 추출한 뒤, 다단계 신호처리를 거쳐 Local Max/Min을 검출합니다.

```
이미지 중앙 수평 스캔라인 (LineIterator)
    → Moving Average 필터 (윈도우 크기 3)
    → 1D Smoothing 필터 (커널: {1,1,1,1,1})
    → 1차 미분 필터 (커널: {0,1,-1})
    → Local Max/Min 검출 (주변 10개 샘플 대비)
    → 거리 필터 (균일하지 않은 간격의 점 제거)
    → 값 필터 (평균에서 ±5 이상 벗어나는 점 제거)
    → MTF 값 계산: (maxMean - minMean) / maxMean
```

### UI 통합 (mainwindow.cpp)

```cpp
// 이미지 중앙 수평선에서 밝기 프로파일 추출
cv::Point2f pt0(0, rgb.rows * 0.5);
cv::Point2f pt1(rgb.cols, rgb.rows * 0.5);
cv::LineIterator it(m_gray, pt0, pt1, 4);
for (int i = 0; i < it.count; i++, ++it)
    m_mtfProfile[i] = *(const uchar *)*it;

// 신호처리 및 MTF 계산
profile1.FindLocalMaxMin(m_mtfProfile);

// 시각화: 녹색 프로파일 곡선, 빨간색 Local Max 원, 노란색 Local Min 원
```

**시각화 요소:**
- **녹색 곡선**: Smoothing 처리된 1D 밝기 프로파일 (이미지 중앙 수평선)
- **빨간 원**: 검출된 Local Maximum (Imax 후보)
- **노란 원**: 검출된 Local Minimum (Imin 후보)
- **label_MTF**: 계산된 MTF 값 실시간 표시

![MTF 시각화가 포함된 캘리브레이션 화면](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*이미지 중앙에 녹색 곡선(밝기 프로파일)이 표시되고, 빨간/노란 점으로 흑백 경계의 극값이 마킹된다. 좌측 하단 MTF 값이 실시간으로 갱신된다.*

---

## 2. Intrinsic Calibration 검증 - 3가지 방식 상세

> **기여도: Projection Error 방식 직접 구현 / 3가지 검증 방법 모두 사내 활용**

Camera Intrinsic Parameter(K, distortion)의 품질을 검증하기 위해 사내에서 **3가지 검증 방법**을 사용했으며, 그 중 **Projection Error(재투영 오차) 시각화 방식**을 직접 구현했습니다.

### 사내 3가지 Intrinsic 검증 방법 요약

| 방법 | 방향 | 시각화 | 평가 대상 |
|------|------|--------|-----------|
| **Projection Error (XY Map)** | Forward (3D→2D) | 우측 XY Map 패널 (초록 점, 검정 배경) | Intrinsic + Extrinsic 통합 정확도 |
| **Simulation Grid** | Forward (3D→2D) | 메인 이미지 위 (파란색 격자선) | 렌즈 왜곡 모델의 적합성 |
| **Homography Residual** | Inverse (2D→Normalized) | 메인 이미지 위 (분홍색 원) | Intrinsic 단독 정확도 (Extrinsic 무관) |

---

### [직접 구현] Projection Error 시각화 (XY Map)

이 방식은 **알려진 3D Object Point를 현재 Calibration Parameter(K, dist, R, t)로 2D에 재투영(Re-projection)한 뒤, 실제 검출된 2D Corner와의 오차 벡터를 시각화**하는 것입니다.

#### 수학적 원리: Forward Projection Pipeline

체커보드의 3D 좌표(Z=0 평면)는 이미 알려져 있고, 캘리브레이션 과정에서 각 보드의 Extrinsic(R, t)도 추정됩니다. 이 알려진 값들을 사용하여 3D 점을 2D 이미지에 투영하는 **Forward Projection** 과정은 다음과 같습니다.

```
Step 1: World → Camera 좌표 변환
        Xc = R · Xw + t
        (체커보드 좌표계의 3D 점을 카메라 좌표계로 변환)

Step 2: Camera → Normalized 좌표 (정규화)
        x' = Xc / Zc
        y' = Yc / Zc
        (Z 성분으로 나누어 정규화 → 이것이 Z=1 평면 위의 좌표)

Step 3: Normalized → Distorted (왜곡 적용)
        Pinhole:  r² = x'² + y'²
                  cdist = 1 + k1·r² + k2·r⁴ + k3·r⁶
                  x_d = x'·cdist + p1·(2·x'·y') + p2·(r² + 2·x'²)
                  y_d = y'·cdist + p2·(2·x'·y') + p1·(r² + 2·y'²)

        Fisheye:  θ = atan(√r²)
                  ratio = (θ/√r²)·(1 + k1·θ² + k2·θ⁴ + k3·θ⁶ + k4·θ⁸)
                  x_d = x'·ratio,  y_d = y'·ratio

Step 4: Distorted → Pixel 좌표
        u = fx · x_d + skew · y_d + cx
        v = fy · y_d + cy
```

#### 오차 계산

```
Error 벡터: eᵢⱼ = detected_2D - projected_2D
             (검출된 코너 좌표 - 재투영된 좌표)

RMS Error:  RMS = √( Σᵢ Σⱼ ‖eᵢⱼ‖² / N )
```

#### 구현 코드 (UtilCamera.cpp - GetError 함수)

```cpp
void UtilCamera::GetError(cv::Mat& dst1, bool update)
{
    // 2000 x 2000 검정 캔버스 생성
    if(dst1.empty())
        dst1 = cv::Mat::zeros(2000, 2000, CV_8UC3);

    if(update)
    {
        m_ptError.clear();
        size_t totalPoints = 0;
        double totalErr = 0;

        for(int i = 0; i < m_vecCalibData.size(); i++)
        {
            // 현재 K, dist와 각 보드의 R, t로 3D→2D 재투영
            UtilCameraModel cameraModel;
            cameraModel.copy(m_mainCamera);
            cameraModel.rvec = m_vecCalibData[i]->rvec.clone();
            cameraModel.tvec = m_vecCalibData[i]->tvec.clone();
            cameraModel.ConvertWorld2Image(
                m_vecCalibData[i]->object_points, imagePoints2);

            // 오차 벡터 = 검출점 - 재투영점
            for (int j = 0; j < imagePoints2.size(); j++)
            {
                cv::Point2f ptErr =
                    m_vecCalibData[i]->image_points[j] - imagePoints2[j];
                m_ptError.push_back(ptErr);
            }

            // RMS 오차 누적
            double err = cv::norm(
                m_vecCalibData[i]->image_points, imagePoints2, cv::NORM_L2);
            totalErr += err * err;
            totalPoints += imagePoints2.size();
        }
        m_projectionErr = std::sqrt(totalErr / totalPoints);
    }

    // 시각화: 중심 (1000, 1000), 오차 벡터 100배 확대
    float eX = dst1.cols * 0.5;  // = 1000
    float ey = dst1.rows * 0.5;  // = 1000
    for (uint32_t i = 0; i < m_ptError.size(); i++)
    {
        cv::Point2f ptErr = m_ptError[i] * 100;  // 100배 확대
        ptErr.x += eX;  // 중심 기준 오프셋
        ptErr.y += ey;
        cv::circle(dst1, ptErr, 4,
                   cv::Scalar(0, 255, 0), 10, cv::LINE_AA, 0);
    }
}
```

#### 시각화 해석

2000 x 2000 pixel 검정 캔버스의 중심 (1000, 1000)이 오차 0 지점이며, 각 오차 벡터를 **100배 확대**하여 초록색 점으로 표시합니다.

```
중심 (1000, 1000) = error 0
1 pixel error → 중심에서 100px 거리에 표시
중심 반경 100px 이내 = sub-pixel 정확도 (양호)
```

| 시각적 패턴 | 의미 | 원인 |
|-------------|------|------|
| 중심 근처에 밀집 | 캘리브레이션 정확도 양호 | K, dist, R, t 모두 정확 |
| 방사형 패턴 | Radial Distortion 보정 부족 | k1, k2, k3 추정 오류 |
| 한쪽 편향 | Principal Point 추정 오류 | cx, cy 추정 부정확 |
| 전체 산개 | Intrinsic 전반적 부정확 | 데이터 부족 또는 모델 부적합 |

![Projection Error 시각화 포함 화면](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*우측 하단 검정 영역(XY Map)에 초록 점이 중앙에 밀집되어 있으면 양호한 캘리브레이션 결과. 우측 상단(Pose Map)은 검출 보드의 FOV 커버리지를 보여준다.*

#### 메인 이미지 위 Re-projection 점 (초록색 원)

XY Map과 별개로, 메인 이미지 위에서도 재투영 점을 **초록색 원**으로 표시합니다. 이 원이 OpenCV가 검출한 체커보드 코너(무지개색 점)와 시각적으로 일치할수록 캘리브레이션이 정확합니다.

```cpp
// UtilCamera.cpp - DetectBoard 함수 내부
for (int i = 0; i < calibData->image_Projection_points.size(); i++)
{
    cv::circle(image_in, calibData->image_Projection_points[i],
               10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
}
```

---

### [사내 활용] Homography Residual (분홍색 원)

Homography Residual은 Projection Error와 반대 방향의 검증 방식입니다. **Extrinsic(R, t)의 영향을 배제하고 Intrinsic(K, dist)만의 품질을 독립적으로 평가**합니다.

#### 핵심 원리: Normalized Image Plane (Z=1 평면)

이 방법의 핵심은 **2D 픽셀 좌표를 Normalized Camera Coordinate로 역변환**하는 과정입니다. 여기서 "Z=1인 Normal Plane"이라는 개념이 등장합니다.

```
Pixel 좌표 (u, v)
    │
    │  K⁻¹ 적용: [x_d, y_d, 1]ᵀ = K⁻¹ · [u, v, 1]ᵀ
    ▼
Distorted Normalized 좌표 (x_d, y_d)
    │
    │  왜곡 제거 (Newton-Raphson 반복법)
    ▼
Undistorted Normalized 좌표 (x, y)
    │
    │  이것은 카메라 좌표계에서 x = X/Z, y = Y/Z
    │  즉, Z=1 평면 위의 점 = 광선의 "방향 벡터" (x, y, 1)
    │  실제 3D 위치 (X, Y, Z)는 Z를 모르므로 복원 불가
    ▼
  방향(ray direction)만 획득
```

`cv::undistortPoints()`에 `newCameraMatrix`를 전달하지 않으면, 출력은 normalized coordinate `(x, y) = (X/Z, Y/Z)`입니다. 이것은 카메라 원점에서 해당 픽셀을 통과하는 **광선(ray)의 방향 벡터** `(x, y, 1)`과 같습니다. 3D 공간에서의 절대 위치를 알 수는 없지만, **방향 정보**는 정확히 얻을 수 있습니다.

#### 왜 Z 없이도 Intrinsic 검증이 가능한가?

체커보드는 **평면(Z=0)** 위의 물체입니다. 평면 제약 조건에 의해:

```
일반 3D→2D: s·[u,v,1]ᵀ = K·[R|t]·[X,Y,Z,1]ᵀ
Z=0 대입:   s·[u,v,1]ᵀ = K·[r₁ r₂ t]·[X,Y,1]ᵀ = H·[X,Y,1]ᵀ
```

K⁻¹ + 왜곡 제거 후:
```
s·[x,y,1]ᵀ = [r₁ r₂ t]·[X,Y,1]ᵀ = H'·[X,Y,1]ᵀ
```

**H'는 Z를 사용하지 않는 2D→2D 변환**입니다. 따라서:
- 정확한 K, dist → undistort 결과가 정확 → Homography 잔차 ≈ 0
- 부정확한 K, dist → undistort에 잔여 왜곡 → Homography가 보상하지 못함 → 잔차 ≠ 0

#### 구현 코드 (UtilCamera.cpp - DetectBoard 함수 내)

```cpp
// ① 검출된 2D 코너를 왜곡 제거 → Normalized 좌표
//    출력: object = {(X/Z, Y/Z), ...} = Z=1 평면 위의 방향 벡터
m_mainCamera.GetUndistortPoint(calibData->image_points, object);

// ② 3D object point의 X,Y를 dxdy로 나누어 이상적 격자 좌표 생성
for (float i = 0; i < calibData->object_points.size(); i++)
    object2.emplace_back(
        cv::Point2f(calibData->object_points[i].x,
                    calibData->object_points[i].y) / m_dxdy);

// ③ Normalized 좌표 → 이상적 격자 좌표로의 Homography 계산
cv::Mat h = cv::findHomography(object, object2);
cv::perspectiveTransform(object, object, h);

// ④ 잔차를 10000배 확대하여 분홍색으로 시각화
for (int i = 0; i < object.size(); i++)
{
    cv::Point2f diff = (object[i] - object2[i]) * 10000;
    cv::circle(image_in,
               cv::Point2f(diff.x + cols, diff.y + rows),
               10, cv::Scalar(255, 0, 255), 5, cv::LINE_AA, 0);
}
```

**10000배 증폭의 이유:** Normalized coordinate에서의 잔차는 매우 작은 값(예: 1pixel 오차 → 1/fx ≈ 0.001)이므로, 시각적 확인을 위해 10000배 증폭합니다.

![Homography 잔차가 중앙에 밀집된 모습](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*메인 이미지 중앙 부근에 분홍색 원이 밀집되어 있으면 K, distortion 추정이 정확함을 의미한다.*

---

### [사내 활용] Simulation Grid (파란색 격자선)

현재 추정된 K, distortion 파라미터로 가상 3D 격자를 투영하여 왜곡 패턴을 직관적으로 확인하는 방법입니다.

#### 동작 원리

```cpp
// ① 가상 카메라: R = Identity, t = (0, 0, focal)
camera2.SetExtrinsic({0,0,0}, {0,0,focal}, false);

// ② Z = focal 평면에 균일 간격 3D 격자 생성
for (y = -range; y <= range; y += 0.5)
    for (x = -range; x <= range; x += 0.5)
        simulation_point.push_back(cv::Point3f(x, y, focal));

// ③ 현재 K, dist로 3D→2D 투영
camera2.ConvertWorld2Image(simulation_point, image_point_list2);

// ④ 인접 점을 파란색 선으로 연결
cv::line(..., cv::Scalar(220, 0, 0), 2, cv::LINE_AA);
```

| 격자 형태 | 의미 |
|-----------|------|
| 직선 격자 | 왜곡 없음 (이상적) |
| 바깥으로 볼록 | Barrel Distortion (k1 < 0) |
| 안으로 오목 | Pincushion Distortion (k1 > 0) |
| 비대칭 기울어짐 | Tangential Distortion (p1, p2) |

![왜곡 격자 정상 상태](/assets/images/projects/intrinsic_tool/distortion_grid_2.png)
*정상적인 캘리브레이션 상태의 파란색 격자. 자연스러운 Barrel Distortion 곡선이 관찰된다.*

![왜곡 격자 불안정 상태](/assets/images/projects/intrinsic_tool/unstable_intrinsic.png)
*Intrinsic이 불안정할 때 파란색 격자가 극도로 왜곡된 상태. k1~k3 또는 고차 왜곡 계수의 추정이 발산했음을 나타낸다.*

---

## 3. FOV Coverage Visualization (Pose Map) - 학습 및 유지보수

> **기여도: 원리 학습 및 유지보수 담당** (원 구현자는 다른 팀원)

캘리브레이션 시 체커보드가 FOV(Field of View) 전체를 고르게 커버하는지 확인하기 위한 시각화 기능입니다. 직접 구현하지는 않았지만, 해당 코드의 원리를 깊이 이해하고 유지보수를 담당했습니다.

### 핵심 원리: Sin/Cos 기반 거리별 색상 인코딩

체커보드의 촬영 거리(`tvec[2]`, Z축 Translation)에 Sin/Cos 함수를 적용하여, **같은 거리의 보드는 같은 색**, **다른 거리는 다른 색**으로 구분되도록 색상을 매핑합니다.

```cpp
// UtilCamera.cpp - GetDetectBoard 함수
float fPeriod = m_vecCalibData[i]->tvec.at<double>(2) * 2;
float dSin = sin(fPeriod);
float dCos = cos(fPeriod);

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
| **직관적** | 같은 거리 = 같은 색 → 하나의 보드 점들이 동일 색으로 표시 |

### 실무 활용

- 우측 상단 Pose Map 패널에 누적된 모든 체커보드 코너를 오버레이
- **색상 분포가 고른 경우** = FOV 전체에 다양한 거리에서 데이터 확보 (양호)
- **특정 색만 편중된 경우** = 특정 거리/영역에만 데이터가 편중 → 추가 촬영 필요

---

## Projection Error vs Pose Map: 기술적 구분

이 두 시각화는 우측 패널에 나란히 위치하며, 역할이 명확히 다릅니다.

| | Projection Error (XY Map) | Pose Map |
|---|---|---|
| **위치** | 우측 하단 패널 | 우측 상단 패널 |
| **배경** | 2000x2000 검정 캔버스 | 원본 이미지 크기 검정 캔버스 |
| **점 색상** | 초록색 (고정) | Sin/Cos 기반 거리별 가변 색상 |
| **확대 배율** | 100배 | 없음 (원본 좌표) |
| **표시 내용** | 재투영 오차 벡터 | 누적 코너 검출 위치 |
| **평가 대상** | K, dist, R, t의 통합 정확도 | FOV 데이터 커버리지 균일성 |
| **갱신 시점** | 매 캘리브레이션마다 | 매 보드 검출마다 누적 |

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
| Parallelization | OpenMP (자코비안 계산 병렬화) |
