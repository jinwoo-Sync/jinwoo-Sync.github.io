---
layout: project
title: "Python 배포 시스템 (Nuitka)"
period: "2025.12 ~ Present"
category: "GIS & Data Pipeline"
tech: "Python, Nuitka, PyTorch, UV, GDAL"
role: "Lead Developer"
company: "Mobiltech (Indonesia Project)"
order: 2
---

## 프로젝트 개요

인도네시아 GIS 프로젝트에서 개발한 13개의 Python 스크립트를 Nuitka를 활용하여 `.exe`로 컴파일하고, 보안이 강화된 형태로 배포하는 시스템을 구축했습니다. 기존의 `.pyc` 배포 방식이 역컴파일에 취약한 문제를 해결하기 위해 네이티브 바이너리 컴파일 방식을 도입했습니다.

## 기존 배포 방식의 문제점

| 방식 | Python 3.6 이전 | Python 3.7 이후 |
|------|----------------|----------------|
| `.pyc` → `pycdc` 역컴파일 | 모든 구문 복구 가능 | try-catch 제외 복구 가능 |

기존의 `pyc` 변환 방식은 `pycdc` 라이브러리를 사용하면 대부분의 소스코드가 복구 가능하여 보안상 문제가 있었습니다.

## Nuitka 컴파일 접근 방식

모든 Python 스크립트와 가상환경을 하나의 `.exe`로 컴파일하여 배포를 시도했습니다.

### 빌드 결과
- **빌드 성공**: 9개 스크립트
- **빌드 실패**: 4개 스크립트 (OSGEO/GDAL 패키지 관련 이슈)

### 해결한 주요 에러

| 에러 | 원인 | 해결 방법 |
|------|------|----------|
| `No process is associated` | onefile exe 실행 실패 | standalone 모드 전환 |
| `ModuleNotFoundError: mt_progress` | 빌드 출력 경로에 모듈 미포함 | `--follow-import-to` 옵션 추가 |
| `exit code 0xC0000005` | onefile PE 로더 메모리 접근 위반 | PyTorch 포함 시 onefile 비활성화 |
| `ImportError: torch.distributed excluded` | Nuitka nofollow로 필수 모듈 제외 | 제외 목록 정밀 조정 |
| `ImportError: torch._functorch excluded` | 핵심 torch 모듈을 제외함 | `torch.autograd` 의존성 분석 후 복원 |

### PyTorch 빌드 최적화

PyTorch 포함 스크립트(3.7GB+)는 onefile PE 로더의 한계를 초과하므로, standalone 모드로 전환하고 불필요한 서브패키지를 선별적으로 제외하여 빌드 시간을 단축했습니다:

- **제외 가능**: `torch._dynamo`, `torch._inductor`, `torch.testing`, `torch.onnx`, `torch.profiler` 등
- **제외 불가**: `torch._functorch` (torch.autograd가 import), `torch.distributed` (dataloader가 import)

## 기술 스택

- **Build Tool**: Nuitka (Python → C → Native Binary)
- **Package Manager**: UV (가상환경 관리)
- **AI/ML**: PyTorch, SAM, EfficientNet
- **GIS**: GDAL, OSGEO, Rasterio
- **배포 환경**: Windows
