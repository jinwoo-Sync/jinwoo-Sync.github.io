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

## 프로젝트 개요

사내 MMS(Mobile Mapping System) 핵심 파이프라인 중 하나인 Camera Intrinsic Calibration Tool의 고도화 및 유지보수를 담당. 기존 ROS 기반 도구의 한계를 극복하기 위해 자체 설계된 C++ 기반 시스템에서 핵심 검증 모듈과 신호 처리 로직을 구현하여 캘리브레이션의 신뢰성을 극대화.

---

## 역할 및 기여 범위 (R&R)

| 구분 | 내용 |
|------|------|
| 포지션 | 1인 개발 및 유지보수 담당 |
| 담당 범위 | FOV Coverage 시각화, MTF 신호 처리 파이프라인, Re-projection Error 시각화, 교차 검증 도구 전반 |
| 협업 | 캘리브레이션 작업자(비전문가)의 피드백을 반영한 UX 개선, 현대자동차 등 외부 납품 품질 검수 대응 |

---

## 주요 업무

### 1. FOV Coverage 시각화 파이프라인 최적화

체커보드가 카메라 FOV 전체를 고르게 덮고 있는지 직관적으로 확인하는 시각화 모듈을 관리 및 최적화.

- Z축 Translation 값을 Sin/Cos 함수로 인코딩하여 거리에 따른 고유 색상을 부여하는 수학적 로직을 정밀 분석 후, 메인 UI 시스템에 안정적으로 통합
- 촬영된 체커보드들의 공간적 분포(상하좌우 및 깊이)를 2D 캔버스 상에서 실시간으로 모니터링할 수 있도록 리팩토링하여, 데이터 수집 단계에서의 편향성을 사전 방지

### 2. MTF(Modulation Transfer Function) 신호 처리 및 UI 통합

이미지 선명도를 정량적으로 평가하기 위해 오픈소스 기반 MTF 분석 알고리즘을 도입하고, 캘리브레이션 특성에 맞게 수식을 변형하여 시스템에 통합.

- Michelson Contrast 수식을 체커보드 흑백 패턴에 맞게 정규화 수식으로 변형하여 명암 대비 측정 정확도 향상
- 다단계 신호 처리 파이프라인 구현
    - 시간축 노이즈 제거: Moving Average 적용으로 프레임 간 미세 밝기 떨림 및 센서 노이즈 억제
    - 공간축 고주파 제거: 박스 필터를 통한 단일 프레임 내 픽셀 노이즈 평활화
    - 유한 차분 기반 엣지 검출: 1차 미분으로 흑백 경계선만 선별적 추출
- 처리된 1D 밝기 프로파일 곡선과 극값을 Qt UI에 실시간 오버레이하여 렌즈 초점 조절 시 직관적 피드백 제공

### 3. Re-projection Error(재투영 오차) 시각화 시스템 구축

추정된 Intrinsic/Extrinsic Parameter의 통합 정확도를 검증하기 위해 3D→2D 재투영 수학적 파이프라인을 직접 구현.

- 3D 공간 좌표 → 카메라 좌표 변환 → 정규화 평면 투영 → 왜곡 모델(Pinhole/Fisheye) 적용 → 최종 픽셀 좌표로 이어지는 Forward Projection 전체 연산 수식화 및 적용
- 실제 검출 2D 코너와 재투영 2D 코너 간 오차 벡터를 100배 확대하여 별도 2000x2000 캔버스에 시각화
- 오차 패턴별 진단 기준 확립: 중앙 밀집(정상) / 방사형 확산(왜곡 계수 오류) / 편향(주점 오류)

### 4. 수학적 모델 기반 교차 검증 도구 도입

재투영 오차 외 Intrinsic 파라미터의 신뢰성을 교차 검증하기 위한 두 가지 시각화 기법을 도입.

- **Simulation Grid:** 추정된 왜곡 모델 기반으로 FOV 전체에 가상 격자를 투영하여, 광각/어안 렌즈 주변부의 왜곡 보정이 물리적으로 타당한지 육안 검증
- **Homography Residual:** Extrinsic 영향을 완전히 배제하고, Normalized Image Plane 개념을 활용하여 픽셀 좌표를 왜곡 제거된 정규 좌표로 역변환 후, 이상적인 체커보드 격자와의 Homography 변환으로 순수 렌즈 왜곡 잔차를 검증

---

## 성과

| 항목 | 내용 |
|------|------|
| 플랫폼 독립화 | ROS 의존성 제거, Windows/Linux 크로스 플랫폼 독립 실행 가능한 툴로 완성 |
| 외부 납품 품질 검수 | 현대자동차 등 외부 클라이언트의 품질 검수 기준 통과 |
| 캘리브레이션 소요 시간 | <!-- TODO: 수치 기입 --> |
| 작업 오류율 감소 | <!-- TODO: 수치 기입 --> |
| 기타 정량 지표 | <!-- TODO: 수치 기입 --> |

---

## 시스템 구동 화면

![MTF 및 Projection Error 시각화 환경](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*좌측 하단: MTF 측정 프로파일 곡선 / 우측 상단: FOV Coverage(Pose Map) / 우측 하단: Re-projection Error 100배 확대(XY Map)*

![전체 FOV 커버리지 진행 상황](/assets/images/projects/intrinsic_tool/full_fov_coverage.png)
*FOV 전역을 고르게 커버하도록 유도하는 시각화 인터페이스*

![Simulation Grid를 통한 렌즈 왜곡 검증](/assets/images/projects/intrinsic_tool/distortion_grid.png)
*가상 격자를 투영하여 물리적 왜곡 모델의 적합성을 교차 검증하는 모습*