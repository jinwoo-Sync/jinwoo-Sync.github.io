---
layout: project
title: "Camera Intrinsic Calibration Tool"
period: "2022.03 ~ Present"
category: "Calibration"
tech: "C++, Qt, OpenCV, Eigen (LM Optimization)"
role: "Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

사내 MMS(Mobile Mapping System) 핵심 파이프라인 중 하나인 **Camera Intrinsic Calibration Tool**의 고도화 및 유지보수를 담당했습니다. 기존 ROS 기반 도구의 한계를 극복하기 위해 자체 설계된 C++ 기반 시스템에서 핵심 검증 모듈과 신호 처리 로직을 구현하여 캘리브레이션의 신뢰성을 극대화했습니다.

---

## 💡 주요 업무 및 기술적 성과

### 1. FOV Coverage 시각화 파이프라인 최적화 (Pose Map)
캘리브레이션 데이터의 신뢰성을 확보하기 위해, 체커보드가 카메라의 FOV(Field of View) 전체를 고르게 덮고 있는지 직관적으로 확인하는 시각화 모듈을 관리 및 최적화했습니다.

*   **Sin/Cos 기반 거리 인코딩 연동:** 기존 모듈의 수학적 핵심 로직(Z축 Translation 값을 Sin/Cos 함수로 인코딩하여 거리에 따른 고유 색상 부여)을 정밀하게 분석하여 메인 UI 시스템에 통합했습니다.
*   **실시간 분포 모니터링 구축:** 캘리브레이션 작업자가 촬영된 체커보드들의 공간적 분포(상하좌우 및 깊이)를 2D 캔버스 상에서 실시간으로 파악할 수 있도록 리팩토링하여 데이터 수집의 편향성을 사전에 방지했습니다.

### 2. MTF (Modulation Transfer Function) 신호 처리 및 UI 통합
이미지 선명도를 정량적으로 평가하기 위해 오픈소스 기반의 MTF 분석 알고리즘을 도입하고, 캘리브레이션 특성에 맞게 수학적 수식을 변형하여 통합했습니다.

*   **체커보드 특화 Contrast 수식 변형:** 일반적인 Michelson Contrast 수식을 체커보드의 흑백 패턴(Imax 중심)에 맞게 정규화 수식으로 변형하여, 명암 대비 측정의 정확도를 높였습니다.
*   **다단계 신호 처리 파이프라인 구현:**
    *   **시간축 노이즈 제거 (Moving Average):** 프레임 간 미세 밝기 떨림과 센서 노이즈를 억제.
    *   **공간축 고주파 제거 (Spatial Smoothing):** 단일 프레임 내 픽셀 노이즈를 박스 필터로 평활화.
    *   **유한 차분 기반 엣지 검출 (1D Derivative):** 1차 미분을 통해 원본 신호에서 밝기 변화가 가장 극심한 흑백 경계선(Local Max/Min)만을 선별적으로 추출.
*   **실시간 UI 피드백:** 처리된 1D 밝기 프로파일 곡선과 극값을 Qt UI 상에 실시간으로 오버레이하여, 렌즈 초점 조절 시 직관적인 피드백을 제공했습니다.

### 3. Re-projection Error (재투영 오차) 시각화 시스템 구축
추정된 Camera Intrinsic Parameter(K, dist)와 Extrinsic(R, t)의 통합 정확도를 검증하기 위해, 3D 점을 2D 이미지 평면으로 재투영하는 수학적 파이프라인을 직접 구현했습니다.

*   **Forward Projection 연산 구현:** `3D 공간 좌표 → 카메라 좌표(R, t 적용) → 정규화 평면(Z축 나눔) → 왜곡 모델(Pinhole/Fisheye 적용) → 최종 픽셀 좌표`로 이어지는 투영 모델을 수식화하여 적용했습니다.
*   **오차 벡터 증폭 시각화 (XY Map):** 검출된 실제 2D 코너와 계산된 재투영 2D 코너 간의 오차 벡터(Error Vector)를 계산하고, 이를 100배 확대하여 별도의 2000x2000 캔버스 중앙에 시각화했습니다.
*   **오차 패턴 분석 시스템:** 점들이 중앙에 밀집하면 정상, 방사형으로 퍼지면 왜곡 계수(Radial Distortion) 오류, 한쪽으로 치우치면 주점(Principal Point) 오류로 직관적으로 진단할 수 있는 검증 기준을 확립했습니다.

### 4. 수학적 모델 기반의 교차 검증 도구 도입
재투영 오차 외에도 Intrinsic 파라미터의 신뢰성을 교차 검증하기 위해 두 가지 추가적인 수학적 시각화 기법을 도입 및 활용했습니다.

*   **Simulation Grid (파란색 격자선):**
    *   추정된 왜곡 모델을 기반으로 FOV 전체에 가상의 평면 격자를 투영하여, 렌즈 주변부(특히 광각/어안 렌즈)에서 왜곡 보정이 물리적으로 타당하게 이루어지고 있는지 육안으로 검증하는 기능을 구현했습니다.
*   **Homography Residual (분홍색 원):**
    *   Extrinsic(R, t)의 영향을 완전히 배제하고 오직 Intrinsic 파라미터만의 품질을 평가하기 위해 Normalized Image Plane(Z=1 평면) 개념을 활용했습니다.
    *   픽셀 좌표를 왜곡이 제거된 정규 좌표로 역변환(`K⁻¹` 및 왜곡 제거)한 뒤, 평면(Z=0) 제약 조건을 바탕으로 이상적인 체커보드 격자와의 Homography 변환을 수행하여 순수 렌즈 왜곡의 잔차를 검증했습니다.

---

## 📈 프로젝트 성과 및 결과물

*   복잡한 ROS 의존성을 제거하고 **독립 실행 가능한 가벼운 Windows/Linux 크로스 플랫폼 툴**로 완성.
*   MTF 모니터링과 다중 오차 시각화(FOV, Reprojection, Homography)를 통해 **비전문가도 캘리브레이션 품질을 즉각 판단할 수 있는 UX** 제공.
*   이러한 신뢰성 검증 로직들을 바탕으로 현대자동차 등 외부 클라이언트의 까다로운 품질 검수 기준을 완벽하게 통과.

### 시스템 구동 화면

![MTF 및 Projection Error 시각화 환경](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*좌측 하단: MTF 측정 프로파일 곡선 / 우측 상단: FOV Coverage(Pose Map) / 우측 하단: Re-projection Error 100배 확대(XY Map)*

![전체 FOV 커버리지 진행 상황](/assets/images/projects/intrinsic_tool/full_fov_coverage.png)
*FOV 전역을 고르게 커버하도록 유도하는 시각화 인터페이스*

![Simulation Grid를 통한 렌즈 왜곡 검증](/assets/images/projects/intrinsic_tool/distortion_grid.png)
*가상 격자(파란선)를 투영하여 물리적 왜곡 모델의 적합성을 교차 검증하는 모습*