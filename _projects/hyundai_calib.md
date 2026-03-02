---
layout: project
title: "Camera-Camera and Camera-Map Calibration Tool"
period: "2023.04 ~ 2024.09"
category: "Calibration"
tech: "C#, OpenCV, PnP Algorithm"
role: "Project Manager & Lead Developer"
company: "Mobiltech (Hyundai Motors Contract)"
order: 2
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: Camera-Camera / Camera-Map Calibration Tool (현대자동차 외주 프로젝트)
- **기간**: 2023.04 ~ 2024.09
- **역할**: Project Manager & Lead Developer (시스템 아키텍처 설계 및 핵심 알고리즘 개발 주도)
- **기술 (Tech Stack)**: C#, OpenCV, PnP Algorithm, WinForms / WPF

## 주요 성과 (Key Achievements)
- 현대자동차 내부 품질 표준 테스트 [수치 입력]개 항목 완벽 통과 및 성공적 납품 달성
- 수동 캘리브레이션 대비 작업 소요 시간 [수치 입력]% 단축 및 정밀도 향상
- 현대자동차 내 3개 타 부서의 각기 다른 요구사항을 [수치 입력]개의 일원화된 파이프라인으로 통합 설계

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 카메라 및 지도 캘리브레이션 통합 시스템 설계
- **문제 상황/목표**: 차량 주변을 커버하는 다중 카메라 간의 정합성(Camera-Camera)과, 추출된 피처를 실제 지도 좌표계(Camera-Map)에 맵핑하는 자동화 툴 필요.
- **해결 방안 (Action)**: C#과 OpenCV를 결합하여 다중 뷰의 피처를 추출하고 PnP(Perspective-n-Point) 알고리즘을 기반으로 최적의 변환 행렬을 도출하는 통합 UI 애플리케이션 직접 설계 및 개발.
- **결과 (Result)**: 현장 작업자가 복잡한 수식 연산 없이 직관적인 UI 환경에서 고정밀 캘리브레이션을 수행할 수 있는 워크플로우 구축.

### 2. 품질 및 표준 준수 검증 모듈 구현
- **문제 상황/목표**: 납품되는 결과물이 현대자동차의 까다로운 내부 기술 표준과 파라미터 규격을 완벽하게 만족해야 함.
- **해결 방안 (Action)**: 연산된 캘리브레이션 파라미터가 고객사 시스템의 포맷 및 수학적 제약 조건을 벗어나지 않도록 변환 및 역변환 정합성을 상시 검증하는 별도의 내부 모듈 구축.
- **결과 (Result)**: 납품 시 데이터 포맷 충돌이나 변환 오차에 의한 클레임 발생률 0% 달성 및 시스템 신뢰도 확보.

### 3. 다중 부서 요구사항 통합 아키텍처 (확장성 확보)
- **문제 상황/목표**: 현대자동차 내 여러 팀에서 각기 다른 방식의 센서 구성 및 뷰(View) 튜닝 방식을 요구하여 파편화된 개발이 우려됨.
- **해결 방안 (Action)**: 개별적인 프로그램을 여러 개 만드는 대신, 하나의 거대한 파이프라인 안에서 각 팀이 원하는 기능(예: 가이드라인 튜닝, 개별 뷰 렌더링 등)을 모듈식으로 활성화할 수 있는 확장성 높은 구조 채택.
- **결과 (Result)**: 3개 팀의 요구사항을 모두 충족하면서도 개발 공수 및 유지보수 비용을 획기적으로 절감.

---

### 부록: 시스템 구동 화면

[ 메인 화면 ]
![](/assets/images/projects/hyundai_calib/main_screen.png)

[ 가상환경 ]
![](/assets/images/projects/hyundai_calib/virtual_env.png)

[ 튜닝 환경 ]
![](/assets/images/projects/hyundai_calib/tuning_env.png)

[ 가이드라인 튜닝 및 전체 lay out view ]
![](/assets/images/projects/hyundai_calib/guideline_tuning.png)

[ Auto Detection ]
![](/assets/images/projects/hyundai_calib/auto_detection.png)

[ 뷰 튜닝 창 개별 프로그램화 ]
![](/assets/images/projects/hyundai_calib/view_tuning.png)

[ 탑뷰 이미지 ]
![](/assets/images/projects/hyundai_calib/topview1.png)
![](/assets/images/projects/hyundai_calib/topview2.png)