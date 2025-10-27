---
layout: project
title: "MMS (Mobile Mapping System) sub module develop"
period: "2025.01 ~ 2025.07"
category: "Mobile Mapping System"
tech_stack: "C++"
role: "Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 3
---

## 주요 활동

- **이전 기술:** 글로벌 맵 ( 후처리된 gps 데이터 ) 및 오픈소스 SLAM 워크플로우에 극단적으로 의존하는 전통적인 MMS/SLAM 프로세스

- **개발한 기술의 특별한 점:** LiDAR, 카메라, GPS, IMU, dmi 로그 데이터를 활용해 단위 거리 기반 로컬 맵을 생성하는 모듈을 개발하여, 기존 방식의 한계를 극복하고 더 유연하고 효율적인 데이터 처리 방식을 도입

- **성과**: MMS/SLAM 코드들을 **모듈화**하고 빌드 순서와 각기 다른 라이브러리 버전을 통일하여, 통합된 빌드 시스템 구축을 통해 코드 재사용성을 높임.

## 프로젝트 결과물

![](/assets/images/projects/mms/module_architecture.png)

![](/assets/images/projects/mms/mms.gif)

![](/assets/images/projects/mms/gnss_mss이미지.png)

### GNSS 한계점

![](/assets/images/projects/mms/gnss한계점.png)
