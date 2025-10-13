---
layout: project
title: Camera-Camera and Camera-Map Calibration Tool for Hyundai Motors
category: Calibration
date: 2023-04-01
period: 2023.04 - 2024.03
role: Project Manager & Lead Developer
tech: C#, WPF, OpenCV, Computer Vision, UI/UX Design
thumbnail: /assets/images/calibration/hyundai-thumb.png
---

## 프로젝트 개요

현대자동차의 내부 표준을 준수하는 Camera-Camera 및 Camera-Map 캘리브레이션 UI 프로그램을 설계 및 개발한 프로젝트입니다. 세 개의 다른 팀(AVM, ADAS, 자율주행)의 요구사항을 단일 파이프라인에서 처리할 수 있도록 설계했습니다.

## 프로젝트 배경

### 비즈니스 요구사항

현대자동차 내 여러 팀에서 각각 다른 캘리브레이션 도구를 사용하면서 발생하는 문제들:

- 팀 간 파라미터 포맷 불일치
- 중복 개발로 인한 리소스 낭비
- 일관성 없는 사용자 경험
- 유지보수 비용 증가

### 솔루션 접근

통합된 단일 플랫폼을 구축하여 모든 팀의 요구사항을 충족하면서도 각 팀의 고유한 워크플로우를 지원하는 확장 가능한 아키텍처를 설계했습니다.

## 시스템 아키텍처

### 모듈식 설계

C# WPF 기반의 모듈식 아키텍처로 설계하여 확장성과 유지보수성을 확보했습니다.

```csharp
public interface ICalibrationModule {
    bool Validate();
    CalibrationResult Execute();
    void ExportParameters(string path);
    Dictionary<string, object> GetMetadata();
}

public class CameraToMapCalibration : ICalibrationModule {
    private readonly CameraParameters cameraParams;
    private readonly MapCoordinateSystem mapCoords;
    private readonly HyundaiStandardsValidator validator;

    public CalibrationResult Execute() {
        // 변환 행렬 계산
        var transformMatrix = ComputeTransformation();

        // 현대자동차 표준 검증
        if (!validator.ValidateTransform(transformMatrix)) {
            return CalibrationResult.Failed("표준 불일치");
        }

        return new CalibrationResult {
            Transform = transformMatrix,
            RMSError = CalculateRMSError(),
            Status = CalibrationStatus.Success,
            Metadata = GenerateMetadata()
        };
    }

    private Matrix4x4 ComputeTransformation() {
        // 현대자동차 표준 좌표계 변환 로직
        return TransformationEngine.Compute(
            cameraParams,
            mapCoords,
            HyundaiStandards.CoordinateSystem
        );
    }

    private double CalculateRMSError() {
        var projectedPoints = ProjectPoints(transformMatrix);
        return RMSCalculator.Compute(groundTruthPoints, projectedPoints);
    }
}
```

### 플러그인 아키텍처

```csharp
public class CalibrationPipeline {
    private List<ICalibrationModule> modules = new List<ICalibrationModule>();

    public void RegisterModule(ICalibrationModule module) {
        modules.Add(module);
    }

    public async Task<PipelineResult> ExecuteAsync() {
        var results = new List<CalibrationResult>();

        foreach (var module in modules) {
            if (!module.Validate()) {
                return PipelineResult.ValidationFailed(module);
            }

            var result = await Task.Run(() => module.Execute());
            results.Add(result);

            if (result.Status != CalibrationStatus.Success) {
                return PipelineResult.ExecutionFailed(module, result);
            }
        }

        return PipelineResult.Success(results);
    }
}
```

## 주요 기능

### 1. 통합 대시보드

메인 화면에서 전체 캘리브레이션 프로세스를 관리할 수 있습니다:

- 프로젝트 생성 및 로드
- 센서 구성 설정
- 캘리브레이션 진행 상태 모니터링
- 결과 검증 및 내보내기

**주요 UI 컴포넌트:**
- 센서 구성 시각화
- 실시간 진행 상황 표시
- 오류 및 경고 알림
- 파라미터 미리보기

### 2. 가상 환경 시뮬레이션

실제 차량 배치 전 가상 환경에서 캘리브레이션을 검증할 수 있습니다:

```csharp
public class VirtualEnvironment {
    private Scene3D virtualScene;
    private List<VirtualSensor> sensors;

    public SimulationResult RunSimulation(CalibrationParameters params) {
        // 3D 환경 설정
        SetupVirtualEnvironment();

        // 센서 배치
        PlaceSensors(params);

        // 시뮬레이션 실행
        var measurements = GenerateSyntheticMeasurements();

        // 결과 검증
        return ValidateAgainstGroundTruth(measurements);
    }

    private void SetupVirtualEnvironment() {
        // 가상 도로, 차량, 장애물 생성
        virtualScene = new Scene3D();
        virtualScene.LoadEnvironment("highway_scenario.xml");
    }
}
```

**기능:**
- 다양한 시나리오 테스트 (주차, 고속도로, 도심)
- 센서 배치 최적화
- 예상 정확도 사전 평가
- 엣지 케이스 검증

### 3. 실시간 튜닝 인터페이스

실시간 파라미터 조정과 즉각적인 피드백을 제공합니다:

```csharp
public class RealTimeTuner : INotifyPropertyChanged {
    private CalibrationParameters parameters;

    public void OnParameterChanged(string paramName, object value) {
        // 파라미터 업데이트
        UpdateParameter(paramName, value);

        // 실시간 재계산
        var result = RecalculateTransformation();

        // UI 업데이트
        UpdateVisualization(result);

        // 오차 계산
        var error = CalculateError(result);
        NotifyErrorChanged(error);
    }

    private void UpdateVisualization(TransformResult result) {
        // 3D 뷰어 업데이트
        viewer3D.UpdateOverlay(result.Transform);

        // 2D 이미지 오버레이
        imageViewer.UpdateProjection(result.ProjectedPoints);
    }
}
```

**인터페이스 구성:**
- 슬라이더 및 수치 입력
- 실시간 3D 프리뷰
- 오차 그래프 (실시간)
- 히스토그램 분석

### 4. AI 기반 자동 검출

YOLOv5 기반 특징점 자동 검출 기능을 구현했습니다:

```csharp
public class AutoDetector {
    private readonly YoloDetector detector;
    private readonly FeatureClassifier classifier;

    public AutoDetector() {
        detector = new YoloDetector("models/feature_detector.onnx");
        classifier = new FeatureClassifier();
    }

    public List<FeaturePoint> DetectFeatures(Image input) {
        // YOLO 검출 실행
        var detections = detector.Detect(input);

        // 신뢰도 필터링
        var filtered = detections
            .Where(d => d.Confidence > 0.85)
            .ToList();

        // 특징점 분류
        return filtered.Select(d => new FeaturePoint {
            Position = d.BoundingBox.Center,
            Type = classifier.ClassifyFeatureType(d.Class),
            Confidence = d.Confidence,
            BoundingBox = d.BoundingBox
        }).ToList();
    }

    public List<FeaturePoint> RefineDetections(
        List<FeaturePoint> initial,
        Image input
    ) {
        // Sub-pixel 정밀도 개선
        return initial.Select(fp =>
            RefineToSubPixel(fp, input)
        ).ToList();
    }
}
```

**검출 대상:**
- 체스보드 패턴
- ArUco 마커
- 차선 마킹
- 구조화된 환경 특징점

## 팀별 커스터마이징

### AVM (Around View Monitor) 팀

```csharp
public class AVMCalibrationProfile : ICalibrationProfile {
    public CoordinateSystem GetCoordinateSystem() {
        return CoordinateSystem.VehicleCentric;
    }

    public List<Camera> GetCameras() {
        return new List<Camera> {
            new Camera("front", 190, FisheyeModel.Kannala),
            new Camera("rear", 190, FisheyeModel.Kannala),
            new Camera("left", 190, FisheyeModel.Kannala),
            new Camera("right", 190, FisheyeModel.Kannala)
        };
    }

    public ExportFormat GetExportFormat() {
        return new AVMParameterFormat {
            CoordinateOrder = "XYZ-RPY",
            Units = "mm-degrees",
            FileFormat = "JSON"
        };
    }
}
```

### ADAS 팀

```csharp
public class ADASCalibrationProfile : ICalibrationProfile {
    public CoordinateSystem GetCoordinateSystem() {
        return CoordinateSystem.ISO8855;
    }

    public List<Sensor> GetSensors() {
        return new List<Sensor> {
            new Camera("front", 60, PinholeModel.Standard),
            new Radar("front_long_range", 77GHz),
            new LiDAR("front", 32channels)
        };
    }

    public ExportFormat GetExportFormat() {
        return new ADASParameterFormat {
            CoordinateOrder = "ISO8855",
            Units = "m-radians",
            FileFormat = "XML"
        };
    }
}
```

### 자율주행 팀

```csharp
public class AutonomousCalibrationProfile : ICalibrationProfile {
    public CoordinateSystem GetCoordinateSystem() {
        return CoordinateSystem.ROS_REP103;
    }

    public List<Sensor> GetSensors() {
        return new List<Sensor> {
            new Camera("front", 120),
            new Camera("rear", 120),
            new LiDAR("roof_center", 64channels),
            new IMU("imu_link"),
            new GPS("gps_link")
        };
    }

    public ExportFormat GetExportFormat() {
        return new ROSParameterFormat {
            Format = "URDF + launch",
            TFTree = true,
            FileFormat = "YAML"
        };
    }
}
```

## 품질 보증 시스템

### 파라미터 검증

현대자동차 내부 표준을 자동으로 검증하는 모듈을 구현했습니다:

```csharp
public class HyundaiStandardsValidator {
    public ValidationResult Validate(CalibrationParameters params) {
        var results = new List<ValidationCheck>();

        // 1. 좌표계 검증
        results.Add(CheckCoordinateSystem(params));

        // 2. 회전 제약 조건
        results.Add(CheckRotationConstraints(params));

        // 3. 이동 변환 범위
        results.Add(CheckTranslationBounds(params));

        // 4. 왜곡 계수 범위
        results.Add(CheckDistortionCoefficients(params));

        // 5. 센서 간 기하학적 관계
        results.Add(CheckSensorGeometry(params));

        return new ValidationResult {
            Checks = results,
            IsValid = results.All(r => r.Passed),
            Warnings = results.Where(r => r.IsWarning).ToList()
        };
    }

    private ValidationCheck CheckRotationConstraints(
        CalibrationParameters params
    ) {
        // 물리적으로 가능한 회전 범위 확인
        var roll = params.Rotation.Roll;
        var pitch = params.Rotation.Pitch;
        var yaw = params.Rotation.Yaw;

        bool valid =
            Math.Abs(roll) < 45 &&   // 롤 제한
            Math.Abs(pitch) < 30 &&  // 피치 제한
            true;                     // 요는 제한 없음

        return new ValidationCheck {
            Name = "Rotation Constraints",
            Passed = valid,
            Message = valid ? "OK" : "회전 각도가 물리적 제약을 벗어남"
        };
    }
}
```

### 자동 리포트 생성

```csharp
public class CalibrationReportGenerator {
    public Report Generate(CalibrationResult result) {
        var report = new Report();

        // 1. 요약 정보
        report.AddSection(GenerateSummary(result));

        // 2. 파라미터 상세
        report.AddSection(GenerateParameterDetails(result));

        // 3. 정확도 분석
        report.AddSection(GenerateAccuracyAnalysis(result));

        // 4. 시각화
        report.AddSection(GenerateVisualizations(result));

        // 5. 검증 결과
        report.AddSection(GenerateValidationResults(result));

        return report;
    }
}
```

## 프로젝트 성과

### 정량적 성과

- ✅ **정해진 기한 내 성공적 납품** 및 현장 배포
- ✅ **캘리브레이션 시간 70% 단축** (평균 40분 → 12분)
- ✅ **사용자 오류율 95% 감소** (자동화 및 검증 시스템)
- ✅ **3개 팀 통합 지원** (AVM, ADAS, 자율주행)

### 정성적 성과

- 현대자동차 내부 표준 완벽 준수
- 사용자 친화적 인터페이스로 교육 시간 단축
- 확장 가능한 아키텍처로 향후 센서 추가 용이
- 자동 검증 시스템으로 신뢰성 확보

### 비즈니스 임팩트

- 팀 간 협업 효율성 향상
- 중복 개발 비용 절감
- 일관된 품질 표준 확립
- 유지보수 비용 감소

## 기술적 도전과 해결

### 도전 1: 서로 다른 좌표계 통합

**문제**: 세 팀이 각각 다른 좌표계 규약 사용

**해결**:
```csharp
public class CoordinateTransformer {
    public Matrix4x4 Transform(
        Matrix4x4 transform,
        CoordinateSystem from,
        CoordinateSystem to
    ) {
        var intermediateTransform = ToStandardCoordinate(transform, from);
        return FromStandardCoordinate(intermediateTransform, to);
    }
}
```

### 도전 2: 실시간 성능 요구사항

**문제**: 파라미터 조정 시 즉각적인 피드백 필요 (< 100ms)

**해결**:
- GPU 가속 (CUDA)
- 다중 스레드 처리
- 증분 계산 (전체 재계산 회피)

### 도전 3: 다양한 카메라 모델 지원

**문제**: Pinhole, Fisheye, Omnidirectional 등 다양한 모델

**해결**: Strategy 패턴으로 카메라 모델 추상화

## 사용된 기술 스택

### 프론트엔드
- C# / WPF (XAML)
- MVVM 패턴
- Material Design

### 백엔드
- OpenCV (Computer Vision)
- Accord.NET (수치 최적화)
- MathNet.Numerics (선형대수)

### AI/ML
- ONNX Runtime
- YOLOv5 (특징점 검출)

### 테스팅
- NUnit
- Moq (Mocking)
- FluentAssertions

## 배운 점

### 기술적 학습

- 대규모 WPF 애플리케이션 아키텍처 설계
- 플러그인 기반 확장 가능 시스템 구축
- 실시간 컴퓨터 비전 최적화
- AI 모델 통합 및 배포

### 프로젝트 관리

- 다중 이해관계자 요구사항 조율
- 단계별 배포 전략 (Agile)
- 사용자 피드백 기반 개선
- 문서화 및 교육 자료 작성

### 소프트웨어 공학

- 확장 가능한 아키텍처의 중요성
- 사용자 경험과 기술적 정확도의 균형
- 자동화된 테스트의 가치
- 지속 가능한 유지보수 설계

## 향후 확장 계획

- 클라우드 기반 협업 기능
- 딥러닝 기반 자동 캘리브레이션
- 모바일 앱 연동
- 실시간 센서 데이터 모니터링
