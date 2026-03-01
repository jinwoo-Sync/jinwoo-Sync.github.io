---
layout: project
title: "대규모 도시급 항공 GIS 상용 툴 개발"
period: "2025.07 ~ Present"
category: "GIS & Data Pipeline"
tech: "C#, Python, PDAL, GDAL, SAM, EfficientNet, Nuitka"
role: "Lead Developer"
company: "Mobiltech (Indonesia Project)"
thumbnail: "/assets/images/projects/indonesia_gis/shp_overlap_detection.png"
order: 1
---

## 프로젝트 개요

인도네시아 도시 단위의 대규모 항공 측량 데이터(항공 카메라, 항공 LiDAR)를 처리하여, 현지 사용자가 직접 운용할 수 있는 **상용 GIS 프로그램**을 개발하는 프로젝트다. 단순한 연구 단계의 코드가 아닌, 실제 고객에게 배포되어 상업적으로 운용되는 소프트웨어를 만들어내는 것이 목표다.

## 나의 역할

- 딥러닝 연구팀이 분할 개발한 Python 스크립트들을 **C# 기반 통합 GIS 프로그램**으로 설계 및 구현
- 데이터 처리 시퀀스 정의, 알고리즘 최적화, 배포 시스템 구축까지 **파이프라인 전체를 주도**
- C# 프로그램 내에서 Python 스크립트를 실행하는 하이브리드 아키텍처 설계 및 구현

---

## 데이터 처리 파이프라인

### 1. SHP 파일 생성
TIF → PNG 변환 → Airborne + Global Building + OSM 데이터 결합 → `.shp` 생성

SHP 생성 과정에서 겹치는 영역을 자동 감지하고 정리하는 기능을 구현했다.

![SHP 중복 영역 감지 및 정리]({{ '/assets/images/projects/indonesia_gis/shp_overlap_detection.png' | relative_url }})
*GIS 툴에서 중복 영역이 빨간색으로 표시된 모습*

### 2. LAS 핸들링 및 DTM 최적화
포인트클라우드 추출 → Polygon 기반 분할 + Multi-layer 분할 → **알고리즘 기반 지면 점군 추출 및 DTM 생성** → `.las` / `.tif` 출력

이 파이프라인의 핵심은 단순히 높이값 기반의 DSM(Digital Surface Model)을 만드는 것이 아니라, 복잡한 지형지물을 제거하고 **순수 지면 높이를 나타내는 고정밀 DTM(Digital Terrain Model)**을 생성하는 데 있다. 이는 추후 **지붕과 바닥 높이 차를 이용한 .obj 자동 모델링 생성**에 반드시 필요한 필수 공정이다.

---

## DTM(Digital Terrain Model) 생성 알고리즘 상세

기존 PDAL CSF 방식 대비 연산 속도와 정밀도를 획기적으로 개선한 커스텀 DTM 생성 파이프라인을 4단계로 구현했다.

### Step 1: 지면 점군 추출 (Ground Extraction)
단순 임계치 방식이 아닌, 두 단계의 필터링을 통해 건물 및 인공 구조물을 정교하게 제거한다.
- **Voxel 기반 분석 (ExtractGroundPointsVoxelBased)**: 전체 영역을 5m x 5m Voxel로 분할한다. 각 Voxel 내 점들의 높이 분포를 분석해 최저점 클러스터를 식별하고, 지면에서 약 2~3m 이상 떨어진 점들을 1차 제거한다.
- **Block 단위 평면 분석 (FilterElevatedFlatSurfaces_BlockLevel)**: 25m 단위 Block으로 확장하여 통계(Median Z, Normal Z, StdZ)를 산출한다. 주변 Block과의 **경계 높이 차이(Edge Height)**가 크고 표면이 평탄한(Normal Z) 경우, 이를 건물 옥상으로 판단하여 최종 제거한다.

### Step 2: 초기 그리드 생성 (BuildSeedMinZ_Parallel)
추출된 지면 점군을 격자(Grid) 시스템에 매핑한다.
- **병렬 처리 최적화**: CPU 코어 수만큼 로컬 그리드를 독립적으로 생성하여 계산한 뒤 병합하는 방식을 채택하여 최적화했다.
- **MinZ 할당**: 각 셀에는 해당 영역 내의 최저 Z값(MinZ)을 할당하여 지형의 가장 낮은 바닥면을 정의한다.

### Step 3: Voxel 기반 구멍 메우기 (FillHolesVoxelBased_Parallel)
건물 제거 부위나 데이터 공백을 채우는 핵심 과정으로, 기존 반경 탐색보다 효율적인 **$O(Width \times Height)$** 복잡도로 동작한다.
- **지형 전파**: 5~10m 단위의 대형 Voxel 참조 그리드를 통해 주변 지면의 최소 높이 값을 전파받아 빈 공간을 채운다.
- **수역 보존 (waterDepthOffset)**: 하천이나 논과 같은 수역 영역의 구멍을 메울 때 주변보다 약 0.5m 낮게 설정하여 자연스러운 지형 특징을 유지한다.

### Step 4: 스무딩 및 데이터 출력 (SmoothMean_Parallel)
- **평균값 스무딩**: 인접한 8개 셀의 가중 평균을 산출하여 지형 경계를 부드럽게 보정한다.
- **GeoTiff 저장**: 32비트 실수형(Float32) TIFF 포맷으로 저장하며, 좌표 정렬을 위해 `.tfw` (World File)를 동시에 생성하여 GIS 호환성을 확보한다.

---

## 성능 최적화 결과

| 항목 | 기존 (PDAL CSF) | 개선 후 (C# 파이프라인) |
|------|-----------------|----------------------|
| 데이터 크기 | 69 GB | 321 MB (전처리 후) |
| 처리 시간 | 2,160분 (36시간) | 1분 |
| 처리 속도 | 32.71 MB/min | 307.00 MB/min |
| **성능 비율** | 기준 | **약 10배 개선** |

---

## 관련 기술 블로그
- **[Nuitka를 이용한 Python 프로젝트 보안 배포](https://jinwoo-sync.github.io/2026/03/01/nuitka-python-deployment.html)**: 상용 소프트웨어 배포를 위한 소스코드 보안 및 실행 환경 최적화 과정 정리

---

## 결과물

- 인도네시아 현지에 배포되어 실제 운용 중인 상용 GIS 프로그램
- 69GB 규모의 도시급 항공 데이터를 자동으로 처리하는 엔드투엔드 파이프라인

## 기술적 특징 요약
- **DTM Generation**: Voxel/Block 통계 분석 기반 고정밀 지면 추출 알고리즘 (DSM → DTM 변환)
- **Automation**: 지붕-지면 높이차 분석을 통한 .obj 메쉬 자동 생성 자동화의 핵심 토대 마련
- **Performance**: 병렬 그리드 처리 및 $O(N)$ 복잡도 보간 알고리즘을 통한 10배 이상의 속도 향상
