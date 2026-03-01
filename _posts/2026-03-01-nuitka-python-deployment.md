---
layout: post
title: "Nuitka로 PyTorch 포함 Python 프로젝트 배포하기"
tags: [Python, Nuitka, Deployment, PyTorch]
---

인도네시아 GIS 프로젝트에서 13개의 Python 스크립트를 Nuitka로 컴파일하여 배포한 경험을 정리한다. 특히 PyTorch가 포함된 스크립트의 빌드에서 겪은 에러들과 해결 과정을 공유한다.

## 왜 Nuitka인가?

### 기존 배포 방식의 문제

사내에서 사용하던 Python 배포 방식은 `.pyc` 파일 변환이었다.

| Python 버전 | pycdc 역컴파일 결과 |
|-------------|-------------------|
| 3.6 이전 | 모든 구문 100% 복구 가능 |
| 3.7 이후 | try-catch 제외 대부분 복구 가능 |

`pycdc` 라이브러리를 사용하면 `.pyc` 파일에서 거의 원본에 가까운 소스코드를 복구할 수 있어 보안상 문제가 있었다.

### Nuitka 선택

Nuitka는 Python 코드를 C로 트랜스파일한 뒤 네이티브 바이너리로 컴파일한다. 역컴파일이 사실상 불가능하고, 실행 속도도 개선되는 장점이 있다.

## 빌드 구성

13개 스크립트를 각각 독립적으로 빌드하되, 가상환경과 의존성을 통합 관리하는 빌드 스크립트를 작성했다.

핵심 설정 구조:

```python
MAIN_SCRIPTS = {
    "tif2png_overlay_summary": {
        "file": "tif2png_overlay_summary.py",
        "env": "uv_env",
        "follow": ["mt_progress"],
        "plugins": [],
        "packages": ["rasterio", "PIL"],
    },
    "airborne_building_extraction": {
        "file": "airborne_building_extraction_Indonesia_multi.py",
        "env": "uv_env",
        "onefile": False,  # PyTorch 3.7GB+ → onefile 한계 초과
        "follow": ["mt_progress", "mt_raster_utils"],
        "plugins": ["numpy", "torch"],
        "packages": ["segment_anything", "efficientnet_pytorch", ...],
    },
}
```

## 빌드 결과

- **성공**: 9개 스크립트
- **실패**: 4개 스크립트 (OSGEO/GDAL 관련)

## 에러 해결 과정

### 1. `No process is associated` — onefile exe 실행 실패

**원인**: onefile 모드로 빌드된 exe가 실행 시 내부 프로세스를 찾지 못함

**해결**: standalone 모드로 전환하여 디렉토리 구조로 배포

### 2. `ModuleNotFoundError: mt_progress` — 커스텀 모듈 누락

**원인**: Python으로 실행했는데 `mt_progress.py`가 빌드 출력 경로에 포함되지 않음

**해결**: `--follow-import-to=mt_progress` 옵션으로 명시적 포함

### 3. `exit code 0xC0000005` — 메모리 접근 위반

**원인**: PyTorch 포함 시 3.7GB+가 되어 onefile PE 로더의 한계를 초과했다.

**해결**: `onefile: False`로 설정하여 standalone 모드 사용

### 4. `ImportError: torch.distributed excluded`

**원인**: Nuitka의 `--nofollow-import-to`로 `torch.distributed`를 제외했는데, 실제로 `torch.utils.data.dataloader`가 이를 import한다.

**해결**: `torch.distributed`를 제외 목록에서 복원

### 5. `ImportError: torch._functorch excluded`

**원인**: `torch._functorch`를 제외했는데 `torch.autograd.function`이 이를 import한다.

**해결**: `torch._functorch`를 제외 목록에서 복원

## PyTorch Nuitka 빌드 최적화 전략

PyTorch는 내부 서브패키지가 매우 많아 빌드 시간이 오래 걸린다. 의존성 분석을 통해 **제외 가능한 모듈**과 **제외 불가능한 모듈**을 분류했다.

### 제외 가능 (빌드 시간 단축)

```
torch._dynamo, torch._inductor, torch.testing,
torch.utils.tensorboard, torch.utils.benchmark,
torch.ao, torch.export, torch.onnx, torch.profiler,
torch.compiler, torch.fx
torchvision.datasets, torchvision.io.video
IPython, matplotlib, pandas, scipy, sympy, unittest, tkinter
```

### 제외 불가 (런타임 에러 발생)

```
torch._functorch  → torch.autograd.function이 import
torch.distributed → torch.utils.data.dataloader가 import
```

### 에러 처리 옵션

```
--no-deployment-flag=excluded-module-usage
```

이 옵션을 추가하면, 제외한 모듈이 lazy import될 때 hard crash 대신 일반 `ImportError`로 처리되어 안정성이 개선된다.

## 교훈

1. **PyTorch + Nuitka onefile은 현실적으로 어렵다**: 3.7GB+ 바이너리는 PE 로더 한계를 초과한다. standalone 모드를 사용하라.
2. **의존성 트리를 반드시 분석하라**: `--nofollow-import-to`로 무분별하게 제외하면 런타임에 crash가 발생한다.
3. **UV로 가상환경을 관리하면 빌드 재현성이 높아진다**: UV의 lockfile 기반 환경 관리가 Nuitka 빌드와 잘 맞는다.
4. **GDAL/OSGEO는 Nuitka와 호환성이 낮다**: C 확장 모듈이 많아 빌드 실패 가능성이 높다.
