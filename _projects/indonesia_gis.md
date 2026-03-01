---
layout: project
title: "인도네시아 GIS 데이터 처리 파이프라인"
period: "2025.07 ~ Present"
category: "GIS & Data Pipeline"
tech: "Python, C#, PDAL, SAM, EfficientNet"
role: "Lead Developer"
company: "Mobiltech (Indonesia Project)"
order: 1
---

## 프로젝트 개요

인도네시아 프로젝트에서 항공 측량 데이터(LAS, TIF, SHP)를 처리하는 GIS 파이프라인을 구축했습니다. 딥러닝 연구팀에서 개발한 Python 코드를 C#으로 컨버팅하고, GIS 툴에서 Python 스크립트를 직접 실행할 수 있는 기능을 구현하여 전체 워크플로우를 통합했습니다.

## 주요 역할

- **Python → C# 코드 컨버팅**: 연구팀의 분할 개발된 Python 코드를 C#으로 변환하고, 동작 시퀀스를 정리 및 통일
- **GIS 툴 내 Python 실행 환경 구현**: C# 프로그램 배포 시 Python 가상환경도 함께 배포 및 연결
- **DTM(Digital Terrain Model) 생성 파이프라인 최적화**: 기존 PDAL CSF 방식 대비 3배 성능 개선 (9분 → 3분)
- **대규모 데이터 처리 최적화**: 69GB 데이터 기준 처리 속도 33배 개선

## 데이터 처리 시퀀스

### 1. SHP 파일 생성
TIF → PNG 변환 → Airborne + Global Building + OSM 데이터 결합 → `.shp` 생성

### 2. LAS 핸들링
포인트클라우드 추출 → Polygon 기반 분할 + Multi-layer 분할 → `.las` 출력

### 3. OBJ 메쉬 생성
LOD1 Mesh / LOD2 Mesh → `.obj` 출력

## DTM 생성 알고리즘

1. **건물 옥상 제거**: Voxel과 Block 단위의 통계 분석을 통해 '높은 평면' (건물 옥상 등)을 제거
2. **지면 점 격자화**: 추출된 지면 점을 격자화하고, 수역(강, 논)의 특징을 보존하면서 빈 공간을 보간

## 성과

| 항목 | 기존 (PDAL CSF) | 개선 후 (C# 파이프라인) |
|------|-----------------|----------------------|
| 데이터 크기 | 69 GB | 321 MB (전처리 후) |
| 처리 시간 | 2,160분 (36시간) | 1분 |
| 처리 속도 | 32.71 MB/min | 307.00 MB/min |
| **성능 비율** | 기준 | **약 10배 개선** |

## 기술 스택

- **Languages**: Python, C#
- **GIS Tools**: PDAL, GDAL/OGR
- **AI/DL**: SAM (Segment Anything Model), EfficientNet
- **Data Formats**: LAS, TIF, SHP, OBJ, PNG
