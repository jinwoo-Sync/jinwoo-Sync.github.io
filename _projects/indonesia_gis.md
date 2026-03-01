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

### 2. LAS 핸들링
포인트클라우드 추출 → Polygon 기반 분할 + Multi-layer 분할 → `.las` 출력

![3D 포인트클라우드 뷰]({{ '/assets/images/projects/indonesia_gis/pointcloud_3d_view.png' | relative_url }})
*도시급 항공 LiDAR 데이터의 3D 포인트클라우드 (CloudCompare)*

### 3. OBJ 메쉬 생성
LOD1 Mesh / LOD2 Mesh → `.obj` 출력

---

## DTM(Digital Terrain Model) 생성

기존 PDAL CSF 방식 대비 약 10배의 성능 개선을 달성한 커스텀 DTM 생성 알고리즘을 설계했다.

![DTM 생성 파이프라인]({{ '/assets/images/projects/indonesia_gis/dtm_pipeline_flowchart.png' | relative_url }})
*DTM 생성 전체 파이프라인 — Raw Points에서 최종 DTM.tif까지*

### A. 지면 점군 추출

Voxel과 Block 단위의 통계 분석을 통해 건물 옥상 같은 '높은 평면'을 제거한다.

![지면 추출 알고리즘]({{ '/assets/images/projects/indonesia_gis/ground_extraction_algorithm.png' | relative_url }})
*Voxel 기반 높이 분석 → Block 단위 평면 필터링 → 건물 옥상 제거*

![건물 판별 Scoring]({{ '/assets/images/projects/indonesia_gis/building_detection_scoring.png' | relative_url }})
*Suspicion Score 기반 건물/지면 판별 — Edge Height, Flatness, StdZ 3가지 기준으로 판정*

### B. DTM 그리드 생성 및 보정

추출된 지면 점을 격자화하고, 수역(강, 논)의 특징을 보존하면서 빈 공간을 보간한다.

![DTM 그리드 생성]({{ '/assets/images/projects/indonesia_gis/dtm_grid_generation.png' | relative_url }})
*MinZ 초기 그리드 → Voxel 기반 Hole Filling → 스무딩*

![그리드 빈 공간 보간]({{ '/assets/images/projects/indonesia_gis/grid_hole_filling.png' | relative_url }})
*NoData 픽셀 탐지 → 최저 높이 결정 → 수역 보정(Water Offset) → 특징 보존 DTM 완성*

### DTM 결과 비교

| PDAL CSF 방식 | C# 커스텀 알고리즘 |
|:---:|:---:|
| ![PDAL CSF DTM]({{ '/assets/images/projects/indonesia_gis/dtm_pdal_csf.png' | relative_url }}) | ![C# DTM]({{ '/assets/images/projects/indonesia_gis/dtm_csharp.png' | relative_url }}) |

![지면 추출 결과]({{ '/assets/images/projects/indonesia_gis/ground_surface_detail.png' | relative_url }})
*DTM 생성 전 바닥면 추출 결과*

---

## 성능 최적화 결과

| 항목 | 기존 (PDAL CSF) | 개선 후 (C# 파이프라인) |
|------|-----------------|----------------------|
| 데이터 크기 | 69 GB | 321 MB (전처리 후) |
| 처리 시간 | 2,160분 (36시간) | 1분 |
| 처리 속도 | 32.71 MB/min | 307.00 MB/min |
| **성능 비율** | 기준 | **약 10배 개선** |

---

## 배포 시스템 구축 (Nuitka)

상용 소프트웨어로 배포하기 위해 소스코드 보안과 실행 환경 일관성 문제를 해결해야 했다.

### 기존 방식의 보안 문제

기존 `.pyc` 파일은 `pycdc` 역컴파일로 소스코드가 거의 복구 가능했다.

![pyc 역컴파일 비교]({{ '/assets/images/projects/indonesia_gis/pycdc_decompile_comparison.png' | relative_url }})
*좌: 원본 Python 코드 / 우: pycdc로 복원된 코드 — 거의 동일하게 복구됨*

### Nuitka 네이티브 바이너리 컴파일

Python 코드를 C → Native Binary로 컴파일하여 역컴파일을 원천 차단했다.

![Nuitka 빌드 결과]({{ '/assets/images/projects/indonesia_gis/nuitka_build_result.png' | relative_url }})
*13개 스크립트 빌드 결과 — 9개 성공, 4개 OSGEO/GDAL 이슈로 실패*

![컴파일된 exe 목록]({{ '/assets/images/projects/indonesia_gis/compiled_exe_list.png' | relative_url }})
*최종 배포된 13개의 네이티브 바이너리 파일*

- PyTorch 포함 스크립트(3.7GB+)는 standalone 모드로 전환
- UV 기반 가상환경 일괄 관리로 빌드 재현성 확보

---

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
