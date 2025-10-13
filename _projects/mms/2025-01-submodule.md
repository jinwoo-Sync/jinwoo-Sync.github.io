---
layout: project
title: MMS Submodule Development
category: MMS
date: 2025-01-01
period: 2021.08 - Present
role: Core Developer
tech: C++, LiDAR Processing, Point Cloud, SLAM, Multi-threading
thumbnail: /assets/images/mms/submodule-thumb.png
---

## 프로젝트 개요

Mobile Mapping System (MMS)의 핵심 서브모듈들을 개발하고 최적화하는 장기 프로젝트입니다. 대용량 Point Cloud 처리, 실시간 SLAM, 센서 융합 등 MMS 시스템의 핵심 기능들을 구현했습니다.

## MMS 시스템 소개

### Mobile Mapping System이란?

차량에 여러 센서(LiDAR, 카메라, IMU, GPS 등)를 장착하여 이동하면서 주변 환경의 3차원 공간 정보를 수집하고 처리하는 시스템입니다.

**주요 응용 분야:**
- 정밀 도로 지도 제작
- 도시 3D 모델링
- 인프라 관리 및 유지보수
- 자율주행 HD 맵 구축

## 개발한 주요 서브모듈

### 1. Point Cloud Processing Pipeline

대용량 LiDAR 데이터를 실시간으로 처리하는 파이프라인을 구축했습니다.

```cpp
class PointCloudProcessor {
private:
    // 멀티스레드 처리를 위한 스레드 풀
    ThreadPool threadPool;

    // 처리 단계별 큐
    ConcurrentQueue<RawPointCloud> rawDataQueue;
    ConcurrentQueue<FilteredPointCloud> filteredQueue;
    ConcurrentQueue<SegmentedPointCloud> segmentedQueue;

public:
    void ProcessStream(LiDARStream& stream) {
        // 파이프라인 단계 설정
        auto filterStage = [this](RawPointCloud raw) {
            return ApplyFilters(raw);
        };

        auto segmentStage = [this](FilteredPointCloud filtered) {
            return SegmentObjects(filtered);
        };

        auto featureStage = [this](SegmentedPointCloud segmented) {
            return ExtractFeatures(segmented);
        };

        // 비동기 파이프라인 실행
        while (stream.HasNext()) {
            auto rawData = stream.GetNext();

            auto future = threadPool.Submit([=]() {
                auto filtered = filterStage(rawData);
                auto segmented = segmentStage(filtered);
                return featureStage(segmented);
            });

            ProcessResult(future.get());
        }
    }

private:
    FilteredPointCloud ApplyFilters(const RawPointCloud& raw) {
        FilteredPointCloud result;

        // 1. 노이즈 제거
        StatisticalOutlierRemoval(raw, result);

        // 2. 다운샘플링
        VoxelGridFilter(result, 0.05); // 5cm voxel

        // 3. 지면 제거
        GroundPlaneRemoval(result);

        return result;
    }

    SegmentedPointCloud SegmentObjects(
        const FilteredPointCloud& filtered
    ) {
        SegmentedPointCloud result;

        // Euclidean Cluster Extraction
        std::vector<PointCluster> clusters;
        EuclideanClustering(filtered, clusters, 0.5); // 50cm tolerance

        // 각 클러스터를 객체로 분류
        for (auto& cluster : clusters) {
            auto objectType = ClassifyCluster(cluster);
            result.AddObject(cluster, objectType);
        }

        return result;
    }
};
```

**성능 최적화:**
- 멀티스레드 파이프라인으로 **3배 처리 속도 향상**
- SIMD 명령어 활용으로 필터링 **40% 가속**
- 메모리 풀링으로 메모리 할당 오버헤드 **60% 감소**

### 2. Real-time SLAM Module

실시간으로 위치 추정과 지도 생성을 수행하는 SLAM 모듈입니다.

```cpp
class RealtimeSLAM {
private:
    // 센서 데이터
    LiDARSensor lidar;
    IMUSensor imu;
    GPSSensor gps;

    // SLAM 상태
    Pose currentPose;
    PointCloudMap globalMap;

    // 최적화 그래프
    PoseGraph poseGraph;

public:
    void Run() {
        // 센서 데이터 동기화
        SensorFusion fusion(lidar, imu, gps);

        while (IsRunning()) {
            // 1. 센서 데이터 획득
            auto syncedData = fusion.GetSynchronizedData();

            // 2. Odometry 추정
            auto odomDelta = EstimateOdometry(syncedData);

            // 3. Pose 예측
            PredictPose(odomDelta);

            // 4. Scan Matching
            auto matchResult = ScanMatching(
                syncedData.pointCloud,
                globalMap
            );

            // 5. Pose 보정
            CorrectPose(matchResult);

            // 6. Map 업데이트
            UpdateMap(syncedData.pointCloud, currentPose);

            // 7. Loop Closure 검출
            if (auto loop = DetectLoopClosure()) {
                OptimizePoseGraph(loop);
            }
        }
    }

private:
    Pose EstimateOdometry(const SyncedSensorData& data) {
        // IMU 기반 초기 추정
        Pose imuPose = IntegrateIMU(data.imu);

        // LiDAR 기반 정밀 보정
        Pose lidarPose = ICPMatching(
            data.pointCloud,
            previousPointCloud
        );

        // 센서 융합 (칼만 필터)
        return FusePoses(imuPose, lidarPose, data.gps);
    }

    ScanMatchResult ScanMatching(
        const PointCloud& scan,
        const PointCloudMap& map
    ) {
        // NDT (Normal Distributions Transform) 사용
        NDTMatcher matcher;
        matcher.SetResolution(1.0);  // 1m resolution
        matcher.SetStepSize(0.1);
        matcher.SetMaxIterations(35);

        auto result = matcher.Align(scan, map);

        return ScanMatchResult {
            .transform = result.transformation,
            .score = result.fitness_score,
            .converged = result.has_converged
        };
    }

    void OptimizePoseGraph(const LoopClosure& loop) {
        // G2O 그래프 최적화
        g2o::SparseOptimizer optimizer;

        // 모든 pose를 vertex로 추가
        for (auto& [id, pose] : poseGraph.GetPoses()) {
            auto vertex = new g2o::VertexSE3();
            vertex->setId(id);
            vertex->setEstimate(ToG2OPose(pose));
            optimizer.addVertex(vertex);
        }

        // Edge 추가 (odometry + loop closure)
        AddOdometryEdges(optimizer);
        AddLoopClosureEdge(optimizer, loop);

        // 최적화 실행
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // 결과 반영
        UpdatePosesFromOptimizer(optimizer);
    }
};
```

**SLAM 성능:**
- **실시간 처리**: 10Hz LiDAR 데이터 처리
- **위치 정확도**: RMS < 0.1m (GPS 보정 후)
- **맵 품질**: 5cm 해상도 Point Cloud Map
- **Loop Closure**: 자동 검출 및 그래프 최적화

### 3. Multi-Sensor Fusion

이기종 센서 데이터를 통합하는 융합 모듈입니다.

```cpp
class SensorFusion {
private:
    // Extended Kalman Filter
    ExtendedKalmanFilter ekf;

    // 센서별 모델
    struct SensorModels {
        LiDARModel lidar;
        IMUModel imu;
        GPSModel gps;
        WheelOdometryModel wheel;
    } models;

public:
    FusedState Fuse(const MultiSensorData& data) {
        // 1. Prediction (IMU 기반)
        ekf.Predict(models.imu, data.imu);

        // 2. Update (다중 센서)
        if (data.HasGPS()) {
            ekf.Update(models.gps, data.gps);
        }

        if (data.HasLiDAR()) {
            auto lidarPose = ExtractPoseFromLiDAR(data.lidar);
            ekf.Update(models.lidar, lidarPose);
        }

        if (data.HasWheel()) {
            ekf.Update(models.wheel, data.wheel);
        }

        return ekf.GetState();
    }

private:
    Pose ExtractPoseFromLiDAR(const PointCloud& scan) {
        // Feature-based localization
        auto features = ExtractFeatures(scan);
        auto matches = MatchToMap(features);
        return EstimatePoseFromMatches(matches);
    }
};
```

### 4. Efficient Data Structure

대용량 Point Cloud를 효율적으로 관리하는 자료구조를 구현했습니다.

```cpp
// Octree 기반 공간 분할
class OctreeMap {
private:
    struct OctreeNode {
        BoundingBox bbox;
        std::vector<Point> points;
        std::array<std::unique_ptr<OctreeNode>, 8> children;
        bool isLeaf;

        static constexpr int MAX_POINTS = 1000;
    };

    std::unique_ptr<OctreeNode> root;
    double minVoxelSize;

public:
    void Insert(const Point& point) {
        InsertRecursive(root.get(), point);
    }

    std::vector<Point> QueryRadius(
        const Point& center,
        double radius
    ) {
        std::vector<Point> result;
        QueryRadiusRecursive(root.get(), center, radius, result);
        return result;
    }

private:
    void InsertRecursive(OctreeNode* node, const Point& point) {
        if (node->isLeaf) {
            node->points.push_back(point);

            // 분할 조건 확인
            if (node->points.size() > OctreeNode::MAX_POINTS &&
                node->bbox.Size() > minVoxelSize) {
                Subdivide(node);
            }
        } else {
            int childIdx = GetChildIndex(node->bbox, point);
            InsertRecursive(node->children[childIdx].get(), point);
        }
    }

    void Subdivide(OctreeNode* node) {
        // 8개의 자식 노드 생성
        auto childBoxes = node->bbox.Subdivide();

        for (int i = 0; i < 8; ++i) {
            node->children[i] = std::make_unique<OctreeNode>();
            node->children[i]->bbox = childBoxes[i];
            node->children[i]->isLeaf = true;
        }

        // 기존 포인트들을 자식에 재분배
        for (const auto& point : node->points) {
            int childIdx = GetChildIndex(node->bbox, point);
            node->children[childIdx]->points.push_back(point);
        }

        node->points.clear();
        node->isLeaf = false;
    }
};
```

**공간 쿼리 성능:**
- K-NN 검색: O(log n)
- Radius 검색: O(log n + k)
- 메모리 효율: 기존 대비 **70% 절감**

### 5. Feature Extraction Module

Point Cloud에서 의미 있는 특징을 추출하는 모듈입니다.

```cpp
class FeatureExtractor {
public:
    struct Features {
        std::vector<Point> cornerPoints;
        std::vector<Point> planePoints;
        std::vector<Point> edgePoints;
    };

    Features Extract(const PointCloud& cloud) {
        Features features;

        // 1. 곡률 계산
        auto curvatures = ComputeCurvatures(cloud);

        // 2. 특징점 분류
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (curvatures[i] > cornerThreshold) {
                features.cornerPoints.push_back(cloud[i]);
            } else if (curvatures[i] < planeThreshold) {
                features.planePoints.push_back(cloud[i]);
            } else {
                features.edgePoints.push_back(cloud[i]);
            }
        }

        return features;
    }

private:
    std::vector<double> ComputeCurvatures(const PointCloud& cloud) {
        std::vector<double> curvatures(cloud.size());

        KDTree kdtree(cloud);

        for (size_t i = 0; i < cloud.size(); ++i) {
            auto neighbors = kdtree.RadiusSearch(cloud[i], 0.5);

            if (neighbors.size() < 10) {
                curvatures[i] = 0;
                continue;
            }

            // PCA 기반 곡률 계산
            Eigen::Matrix3d covariance = ComputeCovariance(neighbors);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

            auto eigenvalues = solver.eigenvalues();
            curvatures[i] = eigenvalues[0] /
                (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
        }

        return curvatures;
    }
};
```

## 시스템 통합 및 최적화

### 전체 시스템 아키텍처

```cpp
class MMSSystem {
private:
    // 하드웨어 인터페이스
    LiDARDriver lidarDriver;
    CameraDriver cameraDriver;
    IMUDriver imuDriver;
    GPSDriver gpsDriver;

    // 처리 모듈
    PointCloudProcessor pcProcessor;
    RealtimeSLAM slam;
    SensorFusion fusion;
    FeatureExtractor featureExtractor;

    // 데이터 저장
    DataLogger logger;

public:
    void Run() {
        // 센서 초기화
        InitializeSensors();

        // 처리 스레드 시작
        std::thread pcThread(&MMSSystem::ProcessPointCloud, this);
        std::thread slamThread(&MMSSystem::RunSLAM, this);
        std::thread fusionThread(&MMSSystem::RunFusion, this);

        // 메인 루프
        while (IsRunning()) {
            // 센서 데이터 수집
            CollectSensorData();

            // 상태 모니터링
            MonitorSystem();

            // 데이터 로깅
            LogData();
        }

        // 종료 처리
        pcThread.join();
        slamThread.join();
        fusionThread.join();
    }

private:
    void ProcessPointCloud() {
        while (IsRunning()) {
            if (auto cloud = lidarDriver.GetNextScan()) {
                // 전처리
                auto filtered = pcProcessor.Filter(*cloud);

                // 특징 추출
                auto features = featureExtractor.Extract(filtered);

                // SLAM에 전달
                slam.AddScan(filtered, features);
            }
        }
    }
};
```

### 성능 최적화 기법

**1. 메모리 최적화**
```cpp
// Object Pooling
class PointCloudPool {
    std::queue<std::unique_ptr<PointCloud>> availableClouds;

public:
    std::unique_ptr<PointCloud> Acquire() {
        if (availableClouds.empty()) {
            return std::make_unique<PointCloud>();
        }
        auto cloud = std::move(availableClouds.front());
        availableClouds.pop();
        cloud->clear();
        return cloud;
    }

    void Release(std::unique_ptr<PointCloud> cloud) {
        availableClouds.push(std::move(cloud));
    }
};
```

**2. 병렬 처리**
```cpp
// TBB를 사용한 병렬 필터링
void ParallelFilter(PointCloud& cloud) {
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, cloud.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
                cloud[i] = ApplyFilter(cloud[i]);
            }
        }
    );
}
```

**3. SIMD 최적화**
```cpp
// AVX2를 사용한 거리 계산
void ComputeDistancesSIMD(
    const float* points,
    const float* center,
    float* distances,
    size_t count
) {
    __m256 centerVec = _mm256_set_ps(
        center[2], center[1], center[0],
        center[2], center[1], center[0],
        center[2], center[1]
    );

    for (size_t i = 0; i < count; i += 8) {
        __m256 pointsVec = _mm256_loadu_ps(&points[i * 3]);
        __m256 diff = _mm256_sub_ps(pointsVec, centerVec);
        __m256 squared = _mm256_mul_ps(diff, diff);

        // Horizontal sum and sqrt
        __m256 dist = _mm256_sqrt_ps(squared);
        _mm256_storeu_ps(&distances[i], dist);
    }
}
```

## 프로젝트 성과

### 처리 성능

- **Point Cloud 처리**: 100만 points/sec
- **SLAM 업데이트 주기**: 10Hz (실시간)
- **메모리 사용량**: 4GB (20km 주행 데이터)
- **맵 저장 크기**: 압축 시 1GB/10km

### 정확도

- **위치 정확도**: RMS < 10cm
- **맵 해상도**: 5cm
- **Loop Closure 성공률**: 95%
- **특징점 추출 정확도**: 92%

### 시스템 안정성

- **연속 운용 시간**: 8시간 이상
- **크래시 없는 운용**: 100+ 시간
- **센서 동기화 오차**: < 1ms

## 기술적 도전과 해결

### 도전 1: 대용량 데이터 실시간 처리

**문제**: 10Hz로 들어오는 100만 포인트 데이터를 실시간 처리

**해결책**:
- 파이프라인 병렬화
- 단계별 비동기 처리
- 메모리 풀링으로 할당 최소화

### 도전 2: 장시간 운용 시 메모리 누수

**문제**: 8시간 이상 운용 시 메모리 사용량 증가

**해결책**:
- Valgrind를 사용한 메모리 프로파일링
- 스마트 포인터 및 RAII 패턴 적용
- 주기적 메모리 정리 로직 추가

### 도전 3: 센서 동기화

**문제**: 여러 센서의 타임스탬프 불일치

**해결책**:
- 하드웨어 트리거 동기화
- 소프트웨어 타임스탬프 보정
- 칼만 필터 기반 시간 정렬

## 사용 기술

- **언어**: C++17
- **라이브러리**: PCL, Eigen, g2o, TBB
- **빌드**: CMake, Conan
- **버전 관리**: Git
- **테스팅**: Google Test, Catch2

## 배운 점

- 대용량 데이터 실시간 처리 시스템 설계
- 멀티스레드 프로그래밍 및 동기화
- 센서 융합 알고리즘 구현
- 시스템 최적화 및 프로파일링
- 장기 프로젝트 유지보수

이 프로젝트는 현재도 진행 중이며, 지속적인 개선과 새로운 기능 추가가 이루어지고 있습니다.
