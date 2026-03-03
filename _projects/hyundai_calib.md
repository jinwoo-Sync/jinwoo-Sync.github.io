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
- 기존 개별 프로그램 기반의 반복 작업(카메라 모델 선정 → 시뮬레이션 → 실제 데이터 취득 → 품질 수치 검증 → 미달 시 재진행)을 통합하여 약 **85~90%의 작업 소요 시간 단축** 및 유지보수 효율화 달성
- 현대자동차 내 부서 간에 서로 다른 카메라 배치·좌표 체계를 사용하는 환경에서, 부서별 Vehicle-Camera Extrinsic(R/T)·Intrinsic 파라미터를 공통 기준으로 정의하고 맵-카메라 간 이종 좌표계를 단일 변환 체계로 통일함으로써, 기존 부서별로 파편화되어 있던 시뮬레이션·검증 프로세스를 **하나의 일원화된 캘리브레이션 파이프라인으로 통합 설계**

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

### 4. 개발 리딩 및 대외 커뮤니케이션 주도
- **문제 상황/목표**: 현대자동차 내 3개 팀에서 각기 다른 요구사항이 동시다발적으로 쏟아지는 상황이었으나, 사내 PM이 타 대형 프로젝트(모비스 등) 투입으로 인해 본 프로젝트에 할당할 수 있는 시간이 극도로 부족해짐.
- **해결 방안 (Action)**: 재무/계약 등 핵심 의사결정을 제외한 프로젝트의 실무 전권을 위임받아 개발 리드 역할을 수행. 매월 1회의 정기 대면 미팅을 주관하고, 이를 위해 2일간 개발 중간 발표 자료(PPT) 준비 및 요구사항 명세화 작업을 전담하여 월 최소 3일 이상 고객사와의 핵심 커뮤니케이션에 집중.
- **결과 (Result)**: 명확한 요구사항 조율로 고객사와의 싱크를 맞추는 동시에, 사내 신규 주니어 개발자들의 일감 할당, 코드 리뷰, Merge 작업까지 모두 병행하며 차질 없이 프로젝트를 완수. 단순한 '개발자'를 넘어 대외 소통과 내부 리딩이 모두 가능한 엔지니어링 역량을 입증.

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