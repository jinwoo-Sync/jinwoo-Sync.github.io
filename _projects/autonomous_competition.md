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

## 프로젝트 개요

- 충북대학교 학부 자율주행 팀으로 **국제 대학생 창작 자동차 경진대회(2020~2021)**에 참가하여 차량 제어, 경로 계획, 센서 융합 전체 파이프라인을 자체 개발
- ROS 기반 주행 소프트웨어를 구성하고, 카메라·LiDAR·GPS·IMU 데이터 융합으로 **센서-지도 정합 및 상태 추정 알고리즘** 구축
- K-City 실도로 환경에서 주행 테스트를 수행하여 **학부 수준에서의 End-to-End 자율주행 시스템 완성도**를 증명

## 주요 역할 및 기여

- **자율주행 제어 안정화:** PID 제어기를 직접 구현하여 횡/종 방향 제어를 안정화하고, 응답성 개선을 위한 파라미터 튜닝을 반복적으로 수행
- **Global + Local Path Planning:** GPS 기반 global path를 작성하고 cost map을 이용한 local path planning을 추가하여 동적 장애물 회피 시나리오를 처리
- **센서 캘리브레이션 & 융합:** Camera-LiDAR 캘리브레이션 도구를 제작하고 IMU-GPS 칼만 필터 기반 융합 시스템을 구성하여 **K-City 실측 데이터**로 검증
- **멀티 센서 로깅 파이프라인:** Hikvision 카메라, Velodyne/Hesai LiDAR, 고정밀 GNSS, DMI/IMU 센서를 동기화하여 데이터셋을 구축하고 분석 가능하게 정리

## Local Path Planning 세부 구조

1. **Calibration 기반 Cost Map 생성**  
   - 실시간 Camera-LiDAR 캘리브레이션 값을 활용해 LiDAR 점군을 이미지 평면에 투영하고 전방 장애물 후보를 추출
2. **장애물 경계 지점 선정**  
   - 차량 진행 벡터 기준으로 장애물 양쪽에 4개의 기준점을 생성하여 정사면체 형태의 안전 구역(사이드 존)을 정의
3. **16개 Lane 후보 생성**  
   - Global path와 현재 장애물 위치, 미션 포인트를 모두 알고 있는 상태에서 16개의 가상 차선을 생성하고, cost map 상의 이동 벡터와 가장 가까운 차선을 선택
4. **Local Path 재생성 & 전환**  
   - 선택된 차선을 따라 local path를 즉시 재구성하고, 차량의 Pose와 속도 조건을 만족할 경우 안전하게 차선 변경을 수행

이 과정 덕분에 대회 미션 구간에서 예측 불가능한 장애물을 만나도 차량이 **실시간으로 대체 경로를 계산하여 안전하게 통과**할 수 있었습니다.

## Photos & Evidence


![Local Path Planning Flow](/assets/images/projects/autonomous_competition/path_planning.png)

![Team Photo / Awards](/assets/images/projects/autonomous_competition/team_photo.png)
