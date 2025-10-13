---
layout: project
title: "Data Synchronization (Mapping System)"
period: "2023.02 ~ Present"
category: "Time Synchronization"
tech_stack: "C++"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 6
---

## 주요 활동

- lidar에서 trigger time을 지정한 사내 예전 버전의 구형 동기화 보드를 신형 동기화 보드로 업그레이드에 기여.

- lidar(hw)에서 직접적인 rmc 신호를 받을 수 있게 지원하는 라이다에서만 시간 동기화가 가능했었던 시스템에서 hw / sw ptp를 이용한 시스템으로 확장에 기여.

### **ptp 시스템 구축 :**

- sw ptp를 통해 초기 sw timestamp ptp 시스템을 구축. ( us 단위 동기화 )

- ( hw ptp )라즈베리파이 xxx 제품으로 어느 정도 구축된 시스템에 gps time이 분기마다 변경되는 이슈 확인 및 수정으로 제품 안정화에 기여. ( ns 단위 동기화 )

![](/assets/images/projects/ptp_system/sc300_board.png)

![](/assets/images/projects/ptp_system/ptp_architecture.png)

### **gprmc _ local time :**

- 기존 시스템은 gps 신호가 끊기면 lidar - gps - trigger time - local time 간에 시간 동기화가 풀리고, gps 신호가 다시 들어온 데이터까지 데이터를 버려야 하는 문제 발생.

- 신규 개발한 SC-400보드는 GPS 신호가 없는 환경에서 가상 GPS 신호를 생성.

- GPS 초당 펄스(PPS)와 미국 해양 전자 협회(NMEA)를 생성 가능.

- hw 적인 요소보다 gprmc 파싱과 데이터 검증에 크게 기여.

- 2025년도 부터 해당 제품들의 판매 진행.

![](/assets/images/projects/ptp_system/sc400_gprmc.png)

![](/assets/images/projects/ptp_system/sc400_system.png)

### **trigger time 성능 평가 :**

![](/assets/images/projects/ptp_system/trigger_performance.png)
