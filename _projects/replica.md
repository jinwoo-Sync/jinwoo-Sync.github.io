---
layout: project
title: "Replica Program (Mobile Mapping System) Tool Development"
period: "2022.03 ~ Present"
category: "Mobile Mapping System"
tech_stack: "C++"
role: "Team Member"
company: "Mobiltech (Core System)"
order: 5
---

## 주요 활동

사내 핵심 프로그램인 모바일 매핑 시스템(MMS)의 핵심 기능을 지원하는 로깅, 추출 및 센서 인터페이스 툴 개발 및 유지보수. 다양한 고성능 센서 데이터의 안정적인 수집 및 처리를 위한 기반 시스템을 구축

### ** 센서 인터페이스 개발:**

- **카메라 인터페이스**: **Hik**, **E-con MIPI**, **E-con GMSL** 과 같은 머신비전 카메라에 대한 인터페이스를 직접 담당하여, Hikvision, Vimba, Pylon, Ozray, Flycapture와 같은 다양한 머신 비전 카메라 sdk 기반의 데이터 연동 모듈을 개발 기여. 고해상도 이미지 데이터의 안정적인 캡처 및 전송 시스템 및 저장 시스템 구축에 기여.

- **LiDAR 인터페이스**: 신규 Hesai LiDAR SDK로 sdk를 업데이트하여 기존 LiDAR 센서 데이터 로깅 파이프라인에 큰 에러 없이 연동.

### **MMS 로깅 및 추출 툴 개발/유지보수:**

- 모바일 매핑 시스템에서 수집된 대량의 센서 데이터를 효율적으로 로깅(Logging)하고, 특정 조건에 따라 필요한 데이터를 추출(Extraction)하는 툴을 개발 및 유지보수.

- 툴의 성능 최적화 및 안정성 확보를 위한 지속적인 코드 리팩토링 및 버그 수정 작업 진행.

- **트리거 보드 및 하드웨어 제어**: NVIDIA Orin Nano의 GPIO를 활용한 트리거 보드 인터페이스를 개발하여, 카메라 및 LiDAR 등 여러 센서 간의 정확한 시각 동기화 및 제어 시스템을 구축

- 데이터 추출 툴과 intrinsic tool / mms mapping tool 간에 ipc 동기화 부분의 안정화 후 프로그램 pipline 구축에 기여. [ 단일 카메라 / 라이다 만 ipc로 공유 -> 다중 카메라와 다중 라이다 ipc 확장 ]

## 프로젝트 결과물

![](/assets/images/projects/replica/system_diagram.png)

![](/assets/images/projects/replica/sensor_interface.png)

[ 저가 센스 xsens mms 매핑 결과물 ]

![](/assets/images/projects/replica/mapping_result.png)
