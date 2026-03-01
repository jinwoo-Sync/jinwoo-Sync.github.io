---
layout: project
title: "무인이송 시스템"
period: "2025.07 ~ Present"
category: "Autonomous System"
tech: "C++, Hesai LiDAR SDK, Linux, NUC"
role: "Developer"
company: "Mobiltech"
order: 3
---

## 프로젝트 개요

LiDAR 기반 무인이송 시스템을 개발하고 현장에 배포하는 프로젝트입니다. Intel NUC 기반 하드웨어에 Hesai LiDAR SDK를 통합하고, Ubuntu 환경에서 시스템 안정화 및 현장 테스트를 수행했습니다.

## 주요 역할

### Hesai LiDAR SDK 통합
- 신규 Hesai LiDAR SDK를 기존 무인이송 시스템에 연동
- 기존 LiDAR 데이터 로깅 파이프라인과 호환되도록 인터페이스 구현

### Linux WiFi/네트워크 시스템 구축 및 디버깅
Intel NUC의 BE201 WiFi 칩에서 발생한 커널 레벨 버그(Bug #220034)를 19단계에 걸쳐 추적하고 해결한 과정:

1. **초기 증상**: NUC Hotspot이 야외 테스트 중 예고 없이 다운됨
2. **1차 시도**: NetworkManager → hostapd/dnsmasq 직접 관리로 전환
3. **2차 시도**: Channel, hw_mode, 보안 설정 직접 구성
4. **로그 분석**: `dmesg` 에러 체인 확인 — wireless API → 펌웨어 crash → reprobe → PCIe reset → USB 실패 → SSD 다운
5. **핵심 에러**: `Failed to send LINK_CONFIG_CMD (-5)` — 드라이버 내부 펌웨어 명령 거부
6. **펌웨어 업데이트**: 최신 ucode(98) 설치했으나 커널이 여전히 구버전(96) 로드
7. **심볼릭 링크 시도**: 98.ucode → 96.ucode 이름으로 바꿔치기 시도
8. **Backports 빌드**: `backport-iwlwifi` 소스를 직접 clone하여 빌드
9. **커널 다운그레이드**: 여러 커널 버전으로 테스트
10. **최종 확인**: Linux 커널 공식 버그(Bug #220034)로 등재된 이슈임을 확인. 커널 6.14.4 이상에서 해결 예정

### 현장 배포 및 테스트
- 양재, 등촌 등 실제 현장 환경에서 데이터 취득 및 시스템 테스트 수행
- WiFi 이슈 발생 시 키보드/마우스 직접 연결 방식으로 대체하여 취득 진행

## 기술 스택

- **Language**: C++
- **Hardware**: Intel NUC, Hesai LiDAR
- **OS**: Ubuntu 24.04 / 20.04
- **Network**: hostapd, dnsmasq, NetworkManager
- **Kernel**: Linux 커널 모듈 빌드, iwlwifi 드라이버 디버깅
