---
layout: project
title: "충북대 학부생 자율주행 경진대회"
period: "2020.03 ~ 2021.10"
category: "Academic & Competition"
tech: "C++, ROS, Camera, GPS/GNSS, LiDAR, IMU"
role: "Team Leader & Member"
institution: "Chungbuk National University"
thumbnail: /assets/images/projects/autonomous_competition/kcity_field.svg
order: 7
---

##  프로젝트 개요 (Overview)
- **프로젝트명**: 국제 대학생 창작 자동차 경진대회 참가 (충북대학교 자율주행 팀)
- **기간**: 2020.03 ~ 2021.10 (팀원 1년 + 팀장 1년)
- **역할**: Team Member(1년) → Team Leader(1년) (차량 제어, 경로 계획, 센서 융합 전체 파이프라인 설계 및 구현)
- **기술 (Tech Stack)**: C++, ROS, Camera, GPS/GNSS, LiDAR, IMU

충북대학교 학부 자율주행 팀으로 국제 대학생 창작 자동차 경진대회(2020~2021)에 참가하여, ROS 기반 주행 소프트웨어를 구성하고 카메라·LiDAR·GPS·IMU 데이터 융합으로 센서-지도 정합 및 상태 추정 알고리즘을 구축. K-City 및 충북대학교 오창 캠퍼스 실도로 환경에서 주행 테스트를 수행.

## 주요 성과 (Key Achievements)
- 학부 수준에서 자율주행 전체 파이프라인(인지-판단-제어)을 자체 구축하고 실차 주행 검증 완료
- 16개 가상 차선 후보 기반 Local Path Planning 알고리즘 설계 및 구현
- Camera-LiDAR 캘리브레이션, 센서 융합, local path planing 구현
<!-- TODO: 대회 순위/수상 실적이 있다면 기입 -->

## 상세 업무 및 기여 (Responsibilities & Contributions)

### 1. 자율주행 제어 안정화
- **문제 상황/목표**: 실차 환경에서 안정적인 횡/종 방향 차량 제어가 필요.
- **해결 방안 (Action)**: PI 제어기를 직접 구현하여 횡/종 방향 제어를 안정화하고, 응답성 개선을 위한 실제 충북대 오창 캠퍼스 자율주행 트랙에서 파라미터 튜닝을 반복적으로 수행.
- **결과 (Result)**: 실도로 환경에서 안정적인 차량 제어 달성.

### 2. Global + Local Path Planning 구현
- **문제 상황/목표**: GPS 기반 Global Path만으로는 동적 장애물 회피가 불가능하여 Local Path Planning이 필요.
- **해결 방안 (Action)**: GPS 기반 Global Path를 작성하고, Camera-LiDAR 캘리브레이션 값을 활용해 LiDAR 점군을 이미지 평면에 투영하여 전방 장애물 후보를 추출하는 Cost Map 생성. 장애물 양쪽에 4개 기준점을 정의한 뒤 16개 가상 차선을 생성하고, Cost Map 상의 이동 벡터와 가장 가까운 차선을 선택하여 Local Path를 재구성하는 알고리즘 설계.
- **결과 (Result)**: 동적 장애물 회피 시나리오에서 안전한 차선 변경 및 경로 재생성 동작 구현.

### 3. 센서 캘리브레이션 및 융합
- **문제 상황/목표**: 미션 구역 인식 및 정밀 제어를 위해 카메라 라이다 Calibration 필요성 대두.
- **해결 방안 (Action)**: Camera-LiDAR 캘리브레이션과 센서 융합을 수행.
- **결과 (Result)**: 다중 센서 융합 기반의 미션 구역 확인을 위한 안정적인 상태 추정 달성.

---

### 부록: 프로젝트 결과물

![Local Path Planning Flow](/assets/images/projects/autonomous_competition/path_planning.png)

![Team Photo / Awards](/assets/images/projects/autonomous_competition/team_photo.png)