---
layout: project
title: "Camera Intrinsic Calibration Tool"
period: "2022.11 ~ 2023.04"
category: "Calibration"
tech: "C++, Qt, OpenCV"
role: "Developer"
company: "Mobiltech (Internal R&D)"
order: 4
---

## 프로젝트 개요 (Overview)
- **프로젝트명**: Camera Intrinsic Calibration Tool (사내 코어 툴 고도화)
- **기간**: 2022.11 ~ 2023.04
- **역할**: Developer (수학적 검증 모델 구현 및 데이터 시각화 파이프라인 개발)
- **기술 (Tech Stack)**: C++, Qt, OpenCV

## 주요 성과 (Key Achievements)
- **비전문가 업무 이관 및 R&D 효율화**: 현장 오퍼레이터도 즉각적인 품질 판단이 가능한 직관적 시각화 환경을 완비하여, 캘리브레이션 실무를 타 부서로 성공적으로 이관하고 코어 개발자들의 R&D 집중 시간 대폭 확보
- **데이터 재작업률(불량률) 0% 근접**: FOV Coverage 실시간 모니터링을 통해 촬영 편향 및 초점 불량을 원천 차단. 이는 높은 Re-projection Error로 인한 알고리즘 발산 및 데이터 재촬영 이슈를 근본적으로 해결하는 핵심 지표로 작용

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. Re-projection Error (재투영 오차) 시각화 시스템 구축
- **문제 상황/목표**: 산출된 Intrinsic(K, dist) 및 Extrinsic(R, t) 파라미터가 실제 환경에서 얼마나 픽셀 오차를 가지는지 작업자가 직관적으로 판단할 수단이 부재.
- **해결 방안 (Action)**: 3D 공간 좌표를 카메라 프레임, 정규화 평면을 거쳐 렌즈 왜곡 모델(Pinhole/Fisheye)을 씌워 2D 픽셀로 변환하는 Forward Projection 수식 파이프라인을 직접 코드로 구현. 오차 벡터를 100배로 증폭하여 2000x2000 캔버스에 산점도로 렌더링.
- **결과 (Result)**: 점의 분포(방사형, 편향성 등)만으로 렌즈 왜곡 계수나 주점(Principal Point)의 최적화 실패 여부를 즉시 진단 가능한 시각적 품질 기준 확립.

### 2. MTF (Modulation Transfer Function) 포커싱 지표 연동
- **문제 상황/목표**: 카메라 초점(Focus)을 맞출 때 육안에만 의존하여 발생하는 정밀도 저하 문제 해결.
- **해결 방안 (Action)**: 기존 오픈소스 MTF 분석 모듈을 캘리브레이션 현장 환경에 적용. 체커보드는 발광이 아닌 반사 타겟이므로 표준 Michelson Contrast(`(Lmax−Lmin)/(Lmax+Lmin)`)보다 `Lmax` 단독 정규화(`(Lmax−Lmin)/Lmax`)가 초점 변화에 더 민감하게 반응함을 확인하고 수식을 재정의. 산출된 MTF 값과 1D 프로파일 곡선을 Qt UI에 실시간 연동.
- **결과 (Result)**: 렌즈 초점 조정 시 선명도를 수치(MTF)로 즉시 확인할 수 있어 정밀한 포커싱 작업 보장.

### 3. FOV Coverage 실시간 모니터링 (Pose Map)
- **문제 상황/목표**: 캘리브레이션 정확도는 체커보드가 FOV(Field of View) 전체를 고르게 커버할 때 확보됨. 데이터 편향 여부를 촬영 중 실시간으로 파악할 수단이 부재.
- **해결 방안 (Action)**: 각 캡처 이미지에 대해 solvePnP로 산출되는 카메라-보드 간 거리(extrinsic t_z)를 sin/cos 함수로 색상 인코딩하는 기존 모듈의 수학적 로직을 분석하고, 이를 메인 애플리케이션의 실시간 렌더링 시스템에 통합. 검출된 코너 좌표를 이미지 해상도와 1:1 대응하는 검정 캔버스에 색상 원으로 누적 렌더링.
- **결과 (Result)**: 작업자가 실시간으로 화면의 비어있는 FOV 영역을 파악하고 촬영을 유도할 수 있어 캘리브레이션 알고리즘의 발산 가능성을 사전 차단.

---

### 부록: 시스템 구동 화면

![MTF 및 Projection Error 시각화 환경](/assets/images/projects/intrinsic_tool/calib_good_mtf.png)
*좌측 전체 이미지 파란 직선: MTF 측정 프로파일 곡선 / 우측 상단 작은 검은색 공간 : FOV Coverage(Pose Map) / 우측 하단 작은 검은색 공간 : Re-projection Error 100배 확대(XY Map)*

![전체 FOV 커버리지 진행 상황](/assets/images/projects/intrinsic_tool/full_fov_coverage.png)
*FOV 전역을 고르게 커버하도록 유도하는 시각화 인터페이스*

![Simulation Grid를 통한 렌즈 왜곡 검증](/assets/images/projects/intrinsic_tool/distortion_grid.png)
*가상 격자를 투영하여 물리적 왜곡 모델의 적합성을 교차 검증하는 모습*
