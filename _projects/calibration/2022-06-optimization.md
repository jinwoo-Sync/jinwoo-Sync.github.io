---
layout: project
title: Calibration Program Optimization
category: Calibration
date: 2022-06-01
period: 2022.06 - 2023.06
role: Lead Developer
tech: C++, OpenCV, PnP Algorithm, Computer Vision
thumbnail: /assets/images/calibration/optimization-thumb.png
---

## 프로젝트 개요

기존 수동/시각적 보정 기반 캘리브레이션 시스템을 PnP(Perspective-n-Point) 알고리즘을 활용한 자동화 시스템으로 업그레이드한 프로젝트입니다.

## 기술적 배경

### 이전 시스템의 한계

- 사용자가 직접 파라미터를 조절하며 시각적 정합성을 확인하는 수동 방식
- 숙련도에 따른 결과 편차 발생
- 시간 소모적인 작업 프로세스
- 재현성 및 일관성 부족

### 개선 목표

PnP 알고리즘을 활용한 2D-3D 포인트 매칭 기반 자동화 시스템을 구현하여 캘리브레이션 정확도와 효율성을 향상시키는 것을 목표로 했습니다.

## 기술적 접근

### 핵심 알고리즘

```cpp
struct CalibrationPoint {
    cv::Point2f image_point;    // 2D 이미지 좌표
    cv::Point3f world_point;    // 3D 월드 좌표
};

class AutoCalibration {
private:
    std::vector<CalibrationPoint> correspondence_points;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

public:
    bool optimize() {
        std::vector<cv::Point2f> image_pts;
        std::vector<cv::Point3f> world_pts;

        // 대응점 추출
        for (const auto& pt : correspondence_points) {
            image_pts.push_back(pt.image_point);
            world_pts.push_back(pt.world_point);
        }

        // PnP 알고리즘 실행
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(
            world_pts,
            image_pts,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_ITERATIVE
        );

        if (success) {
            refineParameters(rvec, tvec);
        }

        return success;
    }

private:
    void refineParameters(cv::Mat& rvec, cv::Mat& tvec) {
        // Levenberg-Marquardt 최적화를 통한 파라미터 정제
        // RMS 오차 최소화
    }
};
```

### 자동화 파이프라인

1. **특징점 검출**: 자동으로 이미지에서 특징점 추출
2. **대응점 매칭**: 2D 이미지 포인트와 3D 월드 좌표 매칭
3. **PnP 솔버**: 초기 외부 파라미터 추정
4. **비선형 최적화**: Reprojection Error 최소화
5. **정확도 검증**: RMS 오차 계산 및 시각화

## 성과 및 정량적 평가

### RMS 오차 분석

- **정면/후면 카메라**: RMS < 1.xxx pixel
- **바닥면 매핑**: RMS < 6 pixel
- **전체 시스템 정확도**: 기존 대비 40% 향상

### 처리 시간 단축

- **이전 시스템**: 평균 30-40분 (숙련자 기준)
- **개선 시스템**: 평균 5-10분 (자동화)
- **효율성 증가**: 약 75% 시간 단축

### 3차원 시각적 평가

정지 상태에서의 LiDAR-카메라 캘리브레이션 정확도를 다음 세 가지 방법으로 검증했습니다:

1. **정량적 평가**: RMS 오차 측정
2. **정성적 평가**: 이미지 projection 시각적 검증
3. **3차원 평가**: Point cloud overlay 정합도 확인

## 기술적 확장성

### 다양한 센서 조합 지원

프로젝트를 통해 다음과 같은 다양한 센서 조합에 대한 캘리브레이션 경험을 축적했습니다:

- **LiDAR - LiDAR**: Multi-LiDAR 상대 좌표계 정렬
- **Radar - LiDAR**: 이기종 센서 융합을 위한 외부 캘리브레이션
- **Camera - LiDAR**: 가장 일반적인 센서 융합 케이스
- **Camera - LiDAR - Radar**: 3종 센서 통합 캘리브레이션
- **Map - Camera - LiDAR**: 절대 좌표계 정렬

### 알고리즘 확장

```cpp
// 다중 센서 캘리브레이션 프레임워크
class MultiSensorCalibration {
public:
    template<typename SensorA, typename SensorB>
    TransformMatrix calibrate(
        const SensorA& sensor_a,
        const SensorB& sensor_b,
        const CalibrationMethod& method
    ) {
        auto correspondences = findCorrespondences(sensor_a, sensor_b);
        return optimizeTransform(correspondences, method);
    }
};
```

## 배운 점과 한계

### 기술적 성장

- 컴퓨터 비전 및 캘리브레이션 이론에 대한 깊은 이해
- 센서 융합 시스템의 기하학적 관계 이해
- 최적화 알고리즘 실무 적용 경험

### 프로젝트 한계

비록 사내 정식 채택은 되지 않았지만, 다음과 같은 인사이트를 얻었습니다:

- **사용자 인터페이스의 중요성**: 기술적 우수성만큼 사용자 경험이 중요
- **알고리즘 정확도 vs 실용성**: 완벽한 자동화보다는 적절한 수준의 자동화와 사용자 제어의 균형 필요
- **검증 프로세스**: 자동화 시스템의 신뢰도 확보를 위한 충분한 검증 필요

### 후속 프로젝트로의 연결

이 프로젝트에서 얻은 경험과 피드백은 후속 프로젝트인 "현대자동차 캘리브레이션 툴" 개발에 직접적으로 반영되어, 사용자 친화적이면서도 정확한 시스템 구축으로 이어졌습니다.

## 관련 기술

- Computer Vision
- Camera Calibration Theory
- PnP (Perspective-n-Point) Algorithm
- Non-linear Optimization
- Sensor Fusion
- OpenCV
- C++
