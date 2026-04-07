---
layout: post
title: "ASAN·TSan·UBSan·clang-tidy로 발견한 실제 버그 사례"
tags: [C++, Sanitizer, ASAN, TSan, UBSan, clang-tidy, ARM]
---

실제 개발 중 Redmine에 기록한 이슈를 기반으로, 정적/동적 분석 도구(ASAN·TSan·UBSan·clang-tidy)를 활용한 SW 품질 관리 사례를 정리한다.

---

## 1. ASAN — `delete` vs `delete[]` 불일치

> 빌드 플래그: `-fsanitize=address -fno-omit-frame-pointer`

| 항목 | 내용 |
|------|------|
| **문제** | `new uint8_t[size]`로 배열 할당 후 `delete`(스칼라)로 해제 → UB, 힙 메타데이터 손상 |
| **탐지** | ASAN — `alloc-dealloc-mismatch` |
| **파일** | `CoreModule/src/util/UtilSaveQueue.h` |

```cpp
// 이전: 배열을 스칼라 delete로 해제
data = new uint8_t[size];
delete data;       // ← UB

// 이후
delete[] data;     // 올바른 배열 해제
```

`delete`/`delete[]` 불일치는 힙 메타데이터를 깨뜨려 나중에 전혀 무관한 위치에서 크래시가 터진다. 원인 추적이 매우 어려운 유형으로, ASAN 없이는 발견하기 어렵다.

---

## 2. TSan — 멀티스레드 공유 변수 보호 누락

> 빌드 플래그: `-fsanitize=thread`

| 항목 | 내용 |
|------|------|
| **문제** | GPIO 트리거 클래스에서 두 스레드가 `mutex`/`atomic` 없이 공유 변수 동시 접근 |
| **탐지** | TSan — `DATA RACE on m_running`, `DATA RACE on m_triggerTime` |
| **파일** | `CoreModule/src/trigger/OrinNano.h` |

```cpp
// 이전: 보호 없는 공유 변수
bool   m_running;      // 스레드 종료 플래그 — atomic 아님
double m_triggerTime;  // 트리거 시간 — mutex 없음

// 이후
std::atomic<bool>   m_running{false};
std::atomic<double> m_triggerTime{-1.0};
```

기존 SC400, CameraEconRoute 클래스는 동일 패턴에서 `mutex`/`atomic`을 올바르게 쓰고 있었는데, ARM 포팅 시 빠르게 작성한 OrinNano에만 누락됐다.

**ARM에서 특히 중요한 이유**: x86은 TSO(Total Store Order) 메모리 모델이라 레이스가 잠자는 경우가 있지만, ARM64는 약한 메모리 모델이라 Jetson에서 확정적으로 터진다. ARM64에서 `std::atomic`은 `ldar`/`stlr`(Load-Acquire/Store-Release) 명령어로 컴파일되어 메모리 배리어가 보장된다.

---

## 3. UBSan — 두 가지 사례

> 빌드 플래그: `-fsanitize=undefined`

### 3-1. printf 포맷 스트링 불일치

| 항목 | 내용 |
|------|------|
| **문제** | `double` 변수를 `%ld`(long int)로 출력 → ARM64 ABI에서 FP/정수 레지스터 분리로 쓰레기값 출력 |
| **탐지** | UBSan — `type mismatch in format string` |
| **파일** | `CoreModule/src/trigger/OrinNano.cpp` |

```cpp
// 이전: 타입 불일치
printf("m_triggerTime = %ld\n", m_triggerTime);  // double을 %ld로 → UB

// 이후
printf("m_triggerTime = %f\n", m_triggerTime);
```

x86_64에서는 double/long 모두 64비트라 우연히 넘어가기도 했지만, ARM64 AArch64 ABI에서는 FP 인자가 `d0~d7`, 정수 인자가 `x0~x7`로 완전히 분리된다. `%ld`가 전혀 다른 레지스터를 읽어 쓰레기값을 출력한다.

---

### 3-2. Eigen 메모리 정렬 위반 (ARM 포팅)

| 항목 | 내용 |
|------|------|
| **문제** | Eigen 고정 크기 행렬은 32바이트 정렬 요구 → ARM64에서 `new` 기본 정렬 8바이트라 Assertion 즉시 실패 |
| **탐지** | UBSan `-fsanitize=alignment` — `misaligned address requires 32 byte alignment` |
| **파일** | `CoreModule/src/io/Mt_format.h` |

```cpp
// 이전: 정렬 매크로 없음
struct CalibLidarCamera {
    Eigen::Matrix<double,3,1> translationMatrix;  // 32바이트 정렬 요구
    Eigen::Quaterniond quaternion;
};
std::vector<CalibLidarCamera> m_vecCalib;         // 일반 allocator

// 이후
struct CalibLidarCamera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW               // new를 정렬 보장 버전으로 오버로드
    Eigen::Matrix<double,3,1> translationMatrix;
    Eigen::Quaterniond quaternion;
};
std::vector<CalibLidarCamera,
    Eigen::aligned_allocator<CalibLidarCamera>> m_vecCalib;
```

x86_64은 `new` 기본 정렬이 16바이트라 Eigen 요구(32바이트)를 우연히 맞추는 경우가 많아 assertion이 안 터진다. ARM64는 기본 정렬이 8바이트 → Jetson에 올리는 순간 캘리브 로드 시점에 즉시 abort.

---

## 4. clang-tidy — `struct timespec` 혼용 제거

| 항목 | 내용 |
|------|------|
| **문제** | 시간 API가 `struct timespec` 혼용 → 플랫폼별 멤버 크기 차이로 IPC 패킷 레이아웃 불안정 |
| **탐지** | clang-tidy — `bugprone-narrowing-conversions` |
| **범위** | 16개 파일 일괄 변경 |

```cpp
// 이전: struct timespec 혼용
struct timespec now = UtilTime::GetTime(true);
double diff = (a.tv_sec - b.tv_sec) + (double)(a.tv_nsec - b.tv_nsec) * 1e-9;

// 이후: uint64_t 나노초 단일 값으로 통일
uint64_t now = UtilTime::get(true);
double diff = (a - b) * 1e-9;
```

`struct timespec`의 `tv_sec`(time_t), `tv_nsec`(long)은 플랫폼/커널 버전에 따라 크기가 달라질 수 있다. `uint64_t`는 아키텍처 무관하게 항상 8바이트로 고정되어 IPC 패킷 바이너리 레이아웃이 안정된다.

---

## 도구별 특성 요약

| 도구 | 탐지 범주 | 한계 |
|------|-----------|------|
| **ASAN** | 힙/스택 버퍼 오버플로우, UAF, delete/delete[] 불일치 | 스레드 레이스·UB는 탐지 불가 |
| **TSan** | 멀티스레드 데이터 레이스 | 메모리 오류·UB 탐지 불가 |
| **UBSan** | 포맷 불일치·정렬 위반·다운캐스트 UB | 설계 결함·레이스 탐지 불가 |
| **clang-tidy** | 코드 품질·API 불일치·캐스트 패턴 | 런타임 동작은 탐지 불가 |

각 도구가 탐지하는 범주가 겹치지 않기 때문에 조합해서 사용해야 한다.
