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

## 프로젝트 개요 (Overview)
- **프로젝트명**: Camera Intrinsic Calibration Tool (사내 코어 툴 고도화)
- **기간**: 2022.03 ~ Present
- **역할**: Developer (수학적 검증 모델 구현 및 데이터 시각화 파이프라인 개발)
- **기술 (Tech Stack)**: C++, Qt, OpenCV, Eigen (LM Optimization)

## 주요 성과 (Key Achievements)
- 캘리브레이션 검수 및 불량 데이터 색출 소요 시간 기존 대비 [수치 입력]% 단축
- 비전문가도 직관적으로 작업 품질을 확인할 수 있도록 시각화 모니터링 환경 완비
- 다중 수학적 교차 검증 시스템(3종)을 통해 현대자동차 등 외부 클라이언트 납품 품질 표준 100% 통과

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. Re-projection Error (재투영 오차) 시각화 시스템 구축
- **문제 상황/목표**: 산출된 Intrinsic(K, dist) 및 Extrinsic(R, t) 파라미터가 실제 환경에서 얼마나 픽셀 오차를 가지는지 작업자가 직관적으로 판단할 수단이 부재.
- **해결 방안 (Action)**: 3D 공간 좌표를 카메라 프레임, 정규화 평면을 거쳐 렌즈 왜곡 모델(Pinhole/Fisheye)을 씌워 2D 픽셀로 변환하는 Forward Projection 수식 파이프라인을 직접 코드로 구현. 오차 벡터를 100배로 증폭하여 2000x2000 캔버스에 산점도로 렌더링.
- **결과 (Result)**: 점의 분포(방사형, 편향성 등)만으로 렌즈 왜곡 계수나 주점(Principal Point)의 최적화 실패 여부를 즉시 진단 가능한 시각적 품질 기준 확립.

### 2. MTF (Modulation Transfer Function) 신호 처리 및 UI 연동
- **문제 상황/목표**: 카메라 초점(Focus)을 맞출 때 육안에만 의존하여 발생하는 정밀도 저하 문제 해결.
- **해결 방안 (Action)**: 오픈소스 MTF 분석 로직을 캘리브레이션 타겟(체커보드)에 맞게 수식 변형(Michelson Contrast 재정의). Moving Average(시간축)와 박스 필터(공간축), 1차 미분 기반 극값 검출 등 다단계 신호 처리 파이프라인을 구축해 노이즈 억제.
- **결과 (Result)**: 렌즈 초점 조정 시 선명도를 나타내는 MTF 값과 1D 프로파일 곡선을 Qt UI 상에서 실시간 지표로 제공하여 정밀한 포커싱 작업 보장.

### 3. FOV Coverage 파이프라인 최적화 (Pose Map)
- **문제 상황/목표**: 캘리브레이션 데이터의 신뢰성은 체커보드가 FOV(Field of View) 전체를 고르게 커버할 때 확보됨. 데이터 편향 여부를 실시간 추적 필요.
- **해결 방안 (Action)**: 타겟의 Z축(깊이) 값을 Sin/Cos 함수로 인코딩하여 거리에 따른 고유 색상을 부여하는 기존 모듈의 수학적 로직을 분석, 이를 메인 애플리케이션의 렌더링 시스템에 안정적으로 통합 및 리팩토링.
- **결과 (Result)**: 작업자가 실시간으로 화면의 비어있는 화각(FOV) 영역을 파악하고 촬영을 유도할 수 있어 캘리브레이션 알고리즘의 발산 가능성 사전 차단.

### 4. 수학적 모델 기반 추가 교차 검증 도구 도입
- **문제 상황/목표**: 재투영 오차 외에도, 추정된 모델 자체가 물리적으로 타당한지 다각도로 교차 검증 필요.
- **해결 방안 (Action)**: 
  1. **Simulation Grid**: 추정된 렌즈 왜곡 모델을 활용해 가상의 격자를 FOV 전체에 투영, 주변부 왜곡 보정이 시각적으로 타당한지 검증하는 기능 추가.
  2. **Homography Residual**: Z=1 (Normalized Image Plane) 개념을 도입해 Extrinsic(R, t) 영향을 배제하고 역변환(K⁻¹)을 통해 순수 Intrinsic 오차만을 평가하는 기능 도입.
- **결과 (Result)**: 입체적인 검증 파이프라인 구축으로 캘리브레이션 모델의 구조적 신뢰성 극대화.

---

### 부록: 시스템 구동 화면

![MTF 및 Projection Error 시각화 환경](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*좌측 하단: MTF 측정 프로파일 곡선 / 우측 상단: FOV Coverage(Pose Map) / 우측 하단: Re-projection Error 100배 확대(XY Map)*

![전체 FOV 커버리지 진행 상황](/assets/images/projects/intrinsic_tool/full_fov_coverage.png)
*FOV 전역을 고르게 커버하도록 유도하는 시각화 인터페이스*

![Simulation Grid를 통한 렌즈 왜곡 검증](/assets/images/projects/intrinsic_tool/distortion_grid.png)
*가상 격자를 투영하여 물리적 왜곡 모델의 적합성을 교차 검증하는 모습*