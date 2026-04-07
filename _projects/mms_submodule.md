---
layout: project
title: "MMS (Mobile Mapping System) sub module develop"
period: "2024.10 ~ 2025.07"
category: "Mobile Mapping System"
tech: "C++, LiDAR SDK, IMU, GPS"
role: "Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 3
---

## 주요 활동

- **이전 기술:** 글로벌 맵 ( 후처리된 gps 데이터 ) 및 오픈소스 SLAM 워크플로우에 극단적으로 의존하는 전통적인 MMS/SLAM 프로세스

- **개발한 기술의 특별한 점:** LiDAR, 카메라, GPS, IMU를 활용하는 기존 MMS 매핑 프로그램에 **DMI 로그 데이터를 도입**하여, DMI tick 기반의 로컬 맵 생성 모듈을 개발하였다. 터널 구간과 같이 GNSS가 튀거나 손실되는 환경에서 LiDAR SLAM(GICP) 만으로 누적 맵을 생성할 경우, 특징점이 부족한 구간에서 ICP가 터널 뒤쪽으로 역방향 매칭되는 오류가 발생한다. 이를 억제하기 위해 **DMI 엔코더 값을 진행 방향의 초기 추정값으로 강제 주입**하여, 특징점 부재 구간에서도 안정적인 맵 누적이 가능하도록 하였다.

  - **한계점:** 회전 구간에서 DMI tick 기반 초기값을 적용할 경우 기존 SLAM 시스템과의 융합이 불완전하여 맵이 튀는 현상이 발생하며, GNSS 신호가 유효한 구간에서는 동작하지 않고 **GNSS 값이 소실되는 구간에 한해서만 적용 가능**하다는 제약이 있음.

- **성과**: MMS/SLAM 코드들을 **모듈화**하고 빌드 순서와 각기 다른 라이브러리 버전을 통일하여, 통합된 빌드 시스템 구축을 통해 코드 재사용성을 높임.

## 프로젝트 결과물

![](/assets/images/projects/mms/module_architecture.png)

![](/assets/images/projects/mms/mms.gif)

![](/assets/images/projects/mms/gnss_mss이미지.png)

### GNSS 한계점

![](/assets/images/projects/mms/gnss한계점.png)
