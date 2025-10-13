---
layout: project
title: "MMS Submodule Development"
period: "2021.11 ~ Present"
category: "Mobile Mapping System"
tech_stack: "C++"
role: "Lead Developer"
company: "Mobiltech (Internal R&D)"
order: 3
---

## 프로젝트 개요
글로벌 맵과 오픈소스 SLAM에 의존하던 전통적인 MMS/SLAM 프로세스를 개선하여, 단위 거리 기반 로컬 맵 생성 모듈 개발

## 주요 활동

### [기존 시스템의 한계]
- **글로벌 맵 의존성**: 후처리된 GPS 데이터에 극단적으로 의존
- **오픈소스 SLAM 워크플로우**: 유연성 부족 및 커스터마이징 한계
- **통합 빌드 시스템 부재**: 각기 다른 라이브러리 버전 및 빌드 순서 문제

### [신규 개발 기술]
- **다중 센서 융합**: LiDAR, Camera, GPS, IMU, DMI 로그 데이터 통합
- **단위 거리 기반 로컬 맵**: 글로벌 맵 의존성 제거
- **유연한 데이터 처리**: 실시간 및 후처리 모드 지원
- **효율적인 메모리 관리**: 대용량 센서 데이터 처리 최적화

### [모듈화 및 통합]
- **MMS/SLAM 코드 모듈화**: 재사용 가능한 컴포넌트 설계
- **통합 빌드 시스템**: 라이브러리 버전 통일 및 빌드 순서 표준화
- **코드 재사용성 향상**: 다양한 프로젝트에 적용 가능한 범용 모듈

## 기술적 성과
- 전통적인 MMS 워크플로우의 한계 극복
- 유연하고 효율적인 데이터 처리 시스템 구축
- 코드 재사용성 및 유지보수성 향상
- 다양한 센서 조합에 대한 확장성 확보

## 프로젝트 결과물

### 모듈 아키텍처
![MMS Module Architecture](/assets/images/projects/mms/module_architecture.png)
*단위 거리 기반 로컬 맵 생성 모듈의 전체 아키텍처*
