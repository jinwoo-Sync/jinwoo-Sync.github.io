---
layout: default
title: "LiDAR-IMU Calibration 검증 방법"
tags: [Calibration, LiDAR, IMU, SLAM, Validation]
---

# LiDAR-IMU Calibration 및 검증 전략

**R&D팀 캘리브레이션 연구 및 검증 방법 정리**

---

## 1. LiDAR-IMU Calibration 방법론

### 1.1 타겟 기반 (Target-based)
특정 타겟의 위치 및 자세를 활용하여 LiDAR와 IMU 간의 상대 변환 행렬($R|t$)을 계산한다.
- **체커보드 및 평면 타겟**: LiDAR로 타겟을 감지하고, IMU의 중력 벡터 기반 자세 추정치를 결합하여 상대적인 $R|t$를 도출한다.
- **레이저 트래커**: 고정밀 레이저 장치를 사용하여 LiDAR와 IMU의 상대 위치를 직접 측정한다.

### 1.2 모션 기반 (Motion-based)
두 센서에 동일한 움직임을 부여하고, 각각의 데이터를 결합해 변환 관계를 추정한다.
- **최적화 방법**: LiDAR 오도메트리와 IMU 데이터를 EKF 또는 Levenberg-Marquardt 알고리즘으로 최적화하여 $R|t$를 추정한다.
- **지면 평면 제약 (Ground Plane Constraints)**: 지면이 고정된 평면임을 가정하고, LiDAR의 지면 감지와 IMU의 중력 벡터를 비교하여 자세 관계를 추정한다.
- **NDT (Normal Distributions Transform)**: LiDAR 정합 움직임과 IMU 움직임을 비교한다.

---

## 2. 주요 오픈소스 도구

| 구분 | 도구 명 | 특징 |
| :--- | :--- | :--- |
| 타겟 기반 | **Kalibr** | 카메라/IMU/LiDAR 통합 캘리브레이션 지원 |
| 타겟리스 (초기화) | **LI_init** | LiDAR-IMU 초기화 및 캘리브레이션 (HKU-Mars) |
| 타겟리스 (모션) | **GRIL_Calib** | 지면 평면 제약을 활용한 주행 기반 캘리브레이션 |
| 종합 패키지 | **OpenCalib** | LiDAR2INS, LiDAR2Cam 등 다양한 센서 간 보정 지원 |

---

## 3. 논문별 검증 방법 분석

### 3.1 LI_Calib (IROS 2020)
- **방법**: 연속 시간 배치 추정(Continuous-time Batch Estimation) 기반.
- **검증**: 10회의 몬테카를로 시뮬레이션 수행.
- **지표**: 포인트클라우드 정합 오차(Point cloud registration error) 및 모션 모델 일관성 평가.

### 3.2 LiDAR2INS (Arxiv 2022)
- **방법**: 실제 주행 환경에서 LiDAR와 GNSS/INS 데이터를 수집하여 최적화.
- **기준(GT)**: CAD 모델 및 하드웨어 설계값.
- **지표**: RMSE (Root Mean Square Error) 및 회전/변환 오차.

---

## 4. 실전 검증 시나리오 제안 (회사 적용)

단순 연구를 넘어 실제 제품의 신뢰성을 확보하기 위한 보수적인 검증 시나리오다.

1.  **시뮬레이션 및 설계값 비교 (GT 확보)**
    - 정확한 설계상의 $R|t$ 값을 얻을 수 있는 시뮬레이션 환경(Isaac Sim 등) 구축.
    - 환경 변화와 노이즈를 추가하며 알고리즘의 한계를 테스트한다.
2.  **실측 데이터 $R|t$ 비교 검증**
    - IMU 위치만을 의도적으로 변경(예: y축으로 5cm 이동)하여 데이터를 취득하고, 캘리브레이션 결과가 해당 변화를 정확히 찾아내는지 확인한다.
3.  **재투영 오차 시각화 (Project to Image)**
    - LiDAR-IMU 캘리브레이션 결과가 적용된 포인트를 카메라 이미지에 재투영한다. (LiDAR-Cam 캘리브레이션이 완벽하다는 가정 하에 수행)
4.  **타겟 배치 및 CAD 모델링 시뮬레이션**
    - 캘리브레이션 룸(Calib-shop) 구성 시 기준이 될 수 있는 타겟(가로/세로 봉 등)을 배치하고 이를 시뮬레이션과 실측 데이터에서 비교 평가한다.
