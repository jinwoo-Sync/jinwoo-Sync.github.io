---
layout: post
title: "V4L2로 MIPI/GMSL 카메라 직접 연결하기 — Jetson Orin 전용"
tags: [C++, V4L2, Camera, MIPI, GMSL, Jetson, Linux, mmap]
---

HIK 카메라는 SDK가 버퍼와 스레드를 알아서 관리해줬다. E-con MIPI/GMSL 카메라는 그런 SDK가 없었고, Linux V4L2(Video4Linux2) 커널 인터페이스를 직접 구현해야 했다. Jetson Orin 보드에서만 동작하는 구성이다.

---

## V4L2가 뭔가

리눅스 커널이 카메라·비디오 장치를 추상화하는 인터페이스다. 카메라가 `/dev/video0`, `/dev/video1` 같은 파일로 보이고, `ioctl()`로 제어한다. MIPI와 GMSL은 Jetson 내부 버스로 연결되어 각각 다른 `/dev/videoN` 번지에 매핑된다.

---

## 초기화 흐름

```
Refresh()
  → /dev/video0, video1... 순서대로 open() 시도
  → InitDevice()
      → VIDIOC_S_PARM   : 60fps 설정
      → VIDIOC_S_FMT    : 1920×1080, UYVY 포맷
      → VIDIOC_REQBUFS  : 커널에 mmap 버퍼 4개 요청
      → VIDIOC_QUERYBUF : 각 버퍼 정보 조회
      → mmap()          : 커널 버퍼를 유저 공간에 매핑
      → VIDIOC_QBUF     : 버퍼 큐에 넣기 (카메라가 채우도록)
      → VIDIOC_STREAMON : 스트리밍 시작
```

`VIDIOC_REQBUFS`로 커널에 버퍼를 요청하면 커널이 DMA 가능한 메모리를 할당한다. `mmap()`으로 그 버퍼를 유저 공간에 매핑해두면, 카메라 프레임이 도착할 때 커널이 직접 그 메모리에 써준다. `memcpy` 없이 데이터가 채워지는 구조다.

```cpp
// 버퍼 4개 요청
struct v4l2_requestbuffers req = {};
req.count  = 4;
req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
req.memory = V4L2_MEMORY_MMAP;
ioctl(fd, VIDIOC_REQBUFS, &req);

// 각 버퍼를 유저 공간에 매핑
buffer.start = mmap(nullptr, buf.length,
                    PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, buf.m.offset);

// 버퍼를 큐에 넣어 카메라가 채우도록
ioctl(fd, VIDIOC_QBUF, &buf);

// 스트리밍 시작
ioctl(fd, VIDIOC_STREAMON, &type);
```

---

## 캡처 루프

`Start(n)` 호출 시 디바이스 index `n`을 인자로 `CaptureLoop` 스레드를 생성한다. 카메라가 여러 대면 스레드도 여러 개.

```cpp
// CaptureLoop(device) 내부
while (m_start) {
    // 프레임 도착 대기 (16.67ms = 60fps 기준)
    select(fd + 1, &fds, NULL, NULL, &tv);

    // 채워진 버퍼 꺼냄
    ioctl(fd, VIDIOC_DQBUF, &buf);

    // 뮤텍스 밖에서 로컬 버퍼로 복사
    memcpy(localBuffer.data(), bufferStart, 1920 * 1080 * 2);

    // 버퍼 즉시 반납 ← 드롭 방지 핵심
    ioctl(fd, VIDIOC_QBUF, &buf);

    // 색상 변환 (UYVY → BGR)
    cv::cvtColor(uyvyFrame, m_bgrFrames[device], cv::COLOR_YUV2BGR_UYVY);

    // 콜백 호출
    m_callbackFn[device](&image, m_pCallbackData[device]);
}
```

---

## DQBUF와 QBUF 사이를 왜 최소화해야 하나

커널이 관리하는 버퍼 4개는 카메라와 앱이 번갈아 쓰는 링 버퍼다.

```
커널 버퍼 풀: [buf0][buf1][buf2][buf3]
                ↑
         카메라가 순서대로 채움

앱이 DQBUF로 꺼내서 쓰고 → QBUF로 반납
반납 안 하면 카메라가 쓸 버퍼가 없어짐 → 프레임 드롭
```

그래서 `memcpy()`로 로컬 버퍼에 복사한 직후 바로 `VIDIOC_QBUF`로 반납한다. `cvtColor` 같은 무거운 처리는 반납 이후에 한다.

**뮤텍스 밖에서 `memcpy()`를 하는 이유도 같다.** 뮤텍스를 잡고 있는 시간이 길어지면 다른 카메라 스레드가 블로킹되어 그쪽에서도 드롭이 날 수 있다.

```cpp
// 뮤텍스는 버퍼 포인터 조회할 때만 잡음
{
    std::lock_guard<std::mutex> lock(m_bufferMutex);
    bufferStart = m_deviceBuffers[device][buf.index].start;
}
// 뮤텍스 해제 후 memcpy
memcpy(localBuffer.data(), bufferStart, 1920 * 1080 * 2);
// 그 다음 즉시 QBUF
ioctl(fd, VIDIOC_QBUF, &buf);
```

---

## MIPI와 GMSL의 차이

코드는 완전히 동일하다. `/dev/videoN` 디바이스 번지만 다르다. Jetson 내부적으로 MIPI CSI 레인과 GMSL 디시리얼라이저가 각각 다른 video 노드에 매핑된다.

```
MIPI  → /dev/video0, video1
GMSL  → /dev/video2, video3
```

`Refresh()`가 `/dev/video0`부터 순서대로 열어보고 `InitDevice()`가 성공하면 등록한다. 어떤 번지에 무엇이 붙어있는지 신경 쓸 필요 없다.

---

## CameraModule 인터페이스와의 관계

HIK, MIPI, GMSL 모두 `CameraModule`을 상속한다.

```cpp
class CameraEcon_Mipi : public CameraModule {
    int Refresh() override;
    void Start(int n) override;
    void Stop(int n) override;
    void SetCameraCallback(int id, CAMERA_CALLBACK fn, const void* data) override;
};
```

상위 코드는 `CameraModule*` 포인터만 쓰므로 어떤 카메라가 붙어도 교체 가능하다. `extern "C" create()`로 동적 로딩을 지원해 런타임에 어떤 카메라 모듈을 쓸지 결정할 수도 있다.
