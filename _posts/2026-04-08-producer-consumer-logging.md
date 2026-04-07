---
layout: post
title: "생산자-소비자 패턴으로 카메라 데이터 로깅하기 — Android와 C++ 구현 비교"
tags: [C++, Android, Kotlin, MediaCodec, Queue, Logging, SSD, MIB, MIC]
---

카메라 콜백은 빠르고, 파일 저장은 느리다. 둘을 같은 스레드에서 처리하면 카메라 콜백이 저장 시간만큼 블로킹된다. 생산자-소비자 패턴으로 이 속도 불일치를 해결한다. Android 앱과 C++ 임베디드 로거에서 각각 구현한 내용을 비교한다.

---

## 왜 생산자-소비자가 필요한가

```
[카메라 콜백]  30fps = 33ms마다 프레임 도착
[파일 저장]    JPEG 인코딩 + 디스크 쓰기 = 가변 시간

콜백 스레드에서 직접 저장하면:
  → 저장이 33ms 초과 시 다음 프레임 놓침
  → 저장 지연이 카메라 타이밍에 영향
```

큐를 사이에 두면 카메라 콜백은 큐에 넣고 즉시 리턴하고, 별도 저장 스레드가 큐에서 꺼내 처리한다.

---

## C++ 구현 — MIB/MIC 바이너리 포맷

### 큐 구조

```cpp
// UtilRing: 고정 크기 원형 버퍼
UtilRing<DataFormatCamera::CameraData, 50> cameraQ; // 생산자가 밀어넣음

// 카메라 콜백 (생산자)
void DeviceCallback::CameraCallback(EventImageCallback* pImage, ...) {
    CameraData data;
    data.frame = pImage->frame.clone();
    data.triggerTime = pImage->triggerTime;
    cameraQ.push(data); // 큐에 넣고 리턴
}

// 저장 스레드 (소비자)
while (running) {
    CameraData* data = cameraQ.pop();
    if (data) {
        m_MtIO.save_Data(savePath, data->grabberTime, data->frame, ...);
    }
}
```

### MIB/MIC 바이너리 저장 포맷

파일마다 저장하지 않고 하나의 바이너리 파일에 이어붙인다.

```
YYMMDDHHMMSS.mib  ← 이미지 데이터 binary concat
YYMMDDHHMMSS.txt  ← 인덱스 (offset, size, timestamp)

파일 구조:
┌──────────┬──────────┬──────────┬──────────┐
│ JPEG 프레임1 │ JPEG 프레임2 │ JPEG 프레임3 │   ...    │
└──────────┴──────────┴──────────┴──────────┘
  offset=0    offset=12345  offset=24100

인덱스:
index=1, offset=0,     size=12345, posix=..., gps_time=...
index=2, offset=12345, size=11755, posix=..., gps_time=...
```

읽을 때는 인덱스에서 offset을 꺼내 `fseek` → `fread`로 해당 프레임만 추출한다.

**MIB vs MIC 차이:**

| | MIB | MIC |
|--|-----|-----|
| 이미지 | Bayer raw (8UC1, 1채널) | Color (8UC3, 3채널, demosaic 후) |
| 인덱스 | `.txt` | `.mic.json` |

MIB는 센서 raw 그대로, MIC는 demosaic 완료된 컬러 이미지다.

### SSD 저장 최적화

현재 코드는 매 프레임마다 `fopen` → `fwrite` → `fclose`를 반복한다. 파일 포인터를 세션 동안 유지하고 `fallocate`로 공간을 미리 확보하면 I/O 지연 편차를 줄일 수 있다.

```cpp
// 세션 시작 시 한 번만
int fd = open("data.mib", O_WRONLY | O_CREAT, 0644);
fallocate(fd, 0, 0, 4ULL * 1024 * 1024 * 1024); // 4GB 사전 확보
FILE* fp = fdopen(fd, "wb");

// 매 프레임: open/close 없이 바로 쓰기
fwrite(jpegData, dataSize, 1, fp);

// 세션 종료 시 한 번만
fclose(fp);
truncate("data.mib", actualWrittenSize); // 실제 쓴 만큼으로 조정
```

`fallocate`는 `std::vector::reserve()`와 동일한 개념이다. 파일시스템이 연속된 블록을 미리 확보하므로 매 write마다 블록 할당 오버헤드가 사라진다.

---

## Android 구현 — MediaCodec + MediaMuxer

### 큐 구조

```
Camera → addFrame(bitmap)         ← 생산자 (카메라 콜백)
          ↓
    MediaCodec 내부 입력 버퍼     ← 큐 역할
          ↓
    encodingJob (IO Coroutine)     ← 소비자
    drainEncoder() → MediaMuxer
          ↓
    video_YYYYMMDD.mp4
```

### bitmap 풀 — CircularQueue

YUV 변환 시 매번 `ByteArray`를 새로 할당하면 GC 압박이 생긴다. 작업용 bitmap을 `CircularQueue`로 재사용한다.

```kotlin
// 고정 크기 원형 버퍼 - 용량 초과 시 오래된 항목 자동 제거
class CircularQueue<T>(private val capacity: Int) {
    private val deque = ArrayDeque<T>(capacity)

    fun push(item: T) {
        synchronized(deque) {
            if (deque.size >= capacity) deque.removeFirst() // 가득 차면 오래된 것 제거
            deque.addLast(item)
        }
    }

    fun poll(): T? = synchronized(deque) {
        if (deque.isEmpty()) null else deque.removeFirst()
    }
}

// YUV 변환용 bitmap 풀 (capacity=2)
private val workingBitmaps = CircularQueue<Bitmap>(capacity = 2)
```

```kotlin
// 사용 패턴
fun convertToYUV(originalBitmap: Bitmap): ByteArray {
    val workingBitmap = workingBitmaps.poll() ?: Bitmap.createBitmap(...)
    // 원본 복사 후 YUV 변환
    val canvas = Canvas(workingBitmap)
    canvas.drawBitmap(originalBitmap, 0f, 0f, null)
    val yuv = bitmapToColorYUV420(workingBitmap)
    workingBitmaps.push(workingBitmap) // 풀에 반환
    return yuv
}
```

### 인코딩 파이프라인

```kotlin
// 생산자: 카메라 콜백에서 프레임 투입
fun addFrame(bitmap: Bitmap): Boolean {
    val inputBufferIndex = mediaCodec!!.dequeueInputBuffer(20000)
    if (inputBufferIndex >= 0) {
        val yuvData = convertBitmapDirectly(bitmap)
        val inputBuffer = mediaCodec!!.getInputBuffer(inputBufferIndex)!!
        inputBuffer.put(yuvData)
        mediaCodec!!.queueInputBuffer(inputBufferIndex, 0, yuvData.size, presentationTimeUs, 0)
        frameIndex++
    }
    return inputBufferIndex >= 0
}

// 소비자: 별도 IO 코루틴에서 인코딩 결과를 mp4에 기록
private fun drainEncoder(bufferInfo: MediaCodec.BufferInfo) {
    val outputBufferIndex = encoder.dequeueOutputBuffer(bufferInfo, 10000)
    if (outputBufferIndex >= 0) {
        val outputBuffer = encoder.getOutputBuffer(outputBufferIndex)!!
        muxer.writeSampleData(videoTrackIndex, outputBuffer, bufferInfo) // mp4에 기록
        encoder.releaseOutputBuffer(outputBufferIndex, false)
    }
}
```

---

## 두 구현 비교

| | C++ (Replica) | Android (Kotlin) |
|--|---------------|-----------------|
| 큐 | `UtilRing<T>` (원형 버퍼) | `MediaCodec` 내부 버퍼 |
| bitmap 풀 | - | `CircularQueue<Bitmap>` |
| 소비자 스레드 | 별도 저장 스레드 | IO Coroutine (`Dispatchers.IO`) |
| 출력 포맷 | `.mib` / `.mic` 바이너리 | `.mp4` (H.264) |
| 인덱스 | `.txt` / `.json` | mp4 컨테이너 내부 |

구현 언어와 플랫폼은 다르지만 핵심 구조는 동일하다. 카메라 콜백은 큐에 밀어넣고 즉시 리턴, 별도 스레드가 큐에서 꺼내 저장한다.
