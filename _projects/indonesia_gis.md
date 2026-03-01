---
layout: project
title: "대규모 도시급 항공 GIS 상용 툴 개발"
period: "2025.07 ~ Present"
category: "GIS & Data Pipeline"
tech: "C#, Python, PDAL, GDAL, SAM, EfficientNet, Nuitka"
role: "Lead Developer"
company: "Mobiltech (Indonesia Project)"
order: 1
---

## 프로젝트 개요

인도네시아 도시 단위의 대규모 항공 측량 데이터(항공 카메라, 항공 LiDAR)를 처리하여, 현지 사용자가 직접 운용할 수 있는 **상용 GIS 프로그램**을 개발하는 프로젝트다. 단순한 연구 단계의 코드가 아닌, 실제 고객에게 배포되어 상업적으로 운용되는 소프트웨어를 만들어내는 것이 목표다.

## 나의 역할

- 딥러닝 연구팀이 분할 개발한 Python 스크립트들을 **C# 기반 통합 GIS 프로그램**으로 설계 및 구현
- 데이터 처리 시퀀스 정의, 알고리즘 최적화, 배포 시스템 구축까지 **파이프라인 전체를 주도**
- C# 프로그램 내에서 Python 스크립트를 실행하는 하이브리드 아키텍처 설계 및 구현

## 데이터 처리 파이프라인

### 1. SHP 파일 생성
TIF → PNG 변환 → Airborne + Global Building + OSM 데이터 결합 → `.shp` 생성

### 2. LAS 핸들링
포인트클라우드 추출 → Polygon 기반 분할 + Multi-layer 분할 → `.las` 출력

### 3. OBJ 메쉬 생성
LOD1 Mesh / LOD2 Mesh → `.obj` 출력

### 4. DTM(Digital Terrain Model) 생성
건물 옥상 제거(Voxel + Block 통계 분석) → 지면 점 격자화 → 수역 특징 보존 보간

## 성능 최적화 결과

| 항목 | 기존 (PDAL CSF) | 개선 후 (C# 파이프라인) |
|------|-----------------|----------------------|
| 데이터 크기 | 69 GB | 321 MB (전처리 후) |
| 처리 시간 | 2,160분 (36시간) | 1분 |
| 처리 속도 | 32.71 MB/min | 307.00 MB/min |
| **성능 비율** | 기준 | **약 10배 개선** |

## 배포 시스템 구축 (Nuitka)

상용 소프트웨어로 배포하기 위해 소스코드 보안과 실행 환경 일관성 문제를 해결해야 했다.

- **문제**: 기존 `.pyc` 배포 방식은 `pycdc` 역컴파일로 소스코드가 거의 복구 가능
- **해결**: Nuitka를 사용하여 Python 코드를 C → Native Binary로 컴파일
- **결과**: 13개 스크립트 중 9개 빌드 성공, PyTorch 포함 스크립트는 standalone 모드로 전환
- **환경 관리**: UV 기반 가상환경 일괄 관리로 빌드 재현성 확보

## 결과물

- 인도네시아 현지에 배포되어 실제 운용 중인 상용 GIS 프로그램
- 69GB 규모의 도시급 항공 데이터를 자동으로 처리하는 엔드투엔드 파이프라인
- 역컴파일 방지가 적용된 보안 배포 시스템

## 기술 스택

- **Languages**: C#, Python
- **GIS Tools**: PDAL, GDAL/OGR, Rasterio
- **AI/DL**: SAM (Segment Anything Model), EfficientNet
- **Build/Deploy**: Nuitka, UV
- **Data Formats**: LAS, TIF, SHP, OBJ, PNG
