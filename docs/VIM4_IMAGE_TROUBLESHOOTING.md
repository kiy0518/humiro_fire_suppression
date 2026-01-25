# VIM4 이미지 백업/복원 문제 해결 가이드

## 개요

VIM4에 Ubuntu 22.04 Server를 설치하고 woowow로 이미지를 백업/복원할 때 발생할 수 있는 문제들과 해결 방법을 정리합니다.

---

## 문제 1: 이미지 복원 후 부팅 실패

### 증상
- woowow로 이미지를 다른 VIM4에 복원했으나 부팅이 안됨
- 버전 업그레이드 후 만든 이미지에서 부팅 실패

### 원인
`apt upgrade` 시 **커널 및 U-Boot 부트로더**가 업그레이드되면서 발생합니다.

특히 아래 패키지들이 문제를 일으킵니다:
- `linux-u-boot-vim4-vendor` (U-Boot 부트로더)
- `linux-image-amlogic-5.15` (커널)
- `linux-dtb-amlogic-5.15` (Device Tree)

### 해결 방법: 커널/부트로더 버전 고정

안정적으로 동작하는 버전에서 아래 명령으로 패키지를 고정합니다:

```bash
sudo apt-mark hold linux-image-amlogic-5.15 \
    linux-headers-amlogic-5.15 \
    linux-dtb-amlogic-5.15 \
    linux-u-boot-vim4-vendor \
    linux-board-package-jammy-vim4 \
    khadas-vim4-linux-5.4-dt-overlays
```

### 고정 상태 확인

```bash
apt-mark showhold
```

출력 예시:
```
khadas-vim4-linux-5.4-dt-overlays
linux-board-package-jammy-vim4
linux-dtb-amlogic-5.15
linux-headers-amlogic-5.15
linux-image-amlogic-5.15
linux-u-boot-vim4-vendor
```

### 고정 해제 (필요 시)

```bash
sudo apt-mark unhold linux-image-amlogic-5.15 \
    linux-headers-amlogic-5.15 \
    linux-dtb-amlogic-5.15 \
    linux-u-boot-vim4-vendor \
    linux-board-package-jammy-vim4 \
    khadas-vim4-linux-5.4-dt-overlays
```

### 안정 버전 정보 (2026-01 기준)

| 패키지 | 안정 버전 |
|--------|----------|
| linux-image-amlogic-5.15 | 1.6.9 |
| linux-headers-amlogic-5.15 | 1.6.9 |
| linux-dtb-amlogic-5.15 | 1.6.9 |
| linux-u-boot-vim4-vendor | 1.6.9-2019.01 |
| linux-board-package-jammy-vim4 | 1.6.9 |

---

## 문제 2: 부팅 시 모니터(HDMI) 출력 안됨

### 증상
- SSH 접속은 정상적으로 됨
- 모니터를 연결해도 부팅 프롬프트/로그인 화면이 안 나옴
- 화면이 검은색으로 유지됨

### 원인

`/boot/uEnv.txt`에 HDMI 출력 비활성화 설정이 있음:

```
bootargs=nomodeset video=HDMI-A-1:d
```

| 설정 | 의미 |
|------|------|
| `nomodeset` | 커널 그래픽 모드 설정 비활성화 |
| `video=HDMI-A-1:d` | HDMI 출력 비활성화 (d = disabled) |

### 진단 방법

```bash
# 현재 부트 파라미터 확인
cat /proc/cmdline

# 프레임버퍼 확인 (없으면 문제)
ls -la /dev/fb*

# uEnv.txt 확인
cat /boot/uEnv.txt
```

### 해결 방법

#### 방법 1: 문제 줄 주석 처리

```bash
# 백업 생성
sudo cp /boot/uEnv.txt /boot/uEnv.txt.backup

# 문제 줄 주석 처리
sudo sed -i 's/^bootargs=nomodeset video=HDMI-A-1:d/#bootargs=nomodeset video=HDMI-A-1:d/' /boot/uEnv.txt

# 확인
cat /boot/uEnv.txt

# 재부팅
sudo reboot
```

#### 방법 2: 수동 편집

```bash
sudo nano /boot/uEnv.txt
```

아래 줄을 찾아서:
```
bootargs=nomodeset video=HDMI-A-1:d
```

주석 처리하거나 삭제:
```
#bootargs=nomodeset video=HDMI-A-1:d
```

저장 후 재부팅:
```bash
sudo reboot
```

### 복원 방법 (HDMI 다시 비활성화)

```bash
sudo cp /boot/uEnv.txt.backup /boot/uEnv.txt
sudo reboot
```

---

## 이미지 백업 전 체크리스트

이미지를 백업하기 전에 아래 사항을 확인하세요:

- [ ] 커널/부트로더 패키지 고정 (`apt-mark showhold`로 확인)
- [ ] HDMI 출력 설정 확인 (`/boot/uEnv.txt`에 `video=HDMI-A-1:d` 없는지)
- [ ] 시스템 정상 부팅 확인
- [ ] SSH 접속 정상 확인
- [ ] 필요한 서비스 자동 시작 설정 (`systemctl is-enabled`)

---

## 관련 파일

| 파일 | 용도 |
|------|------|
| `/boot/uEnv.txt` | 부트 환경 변수 설정 |
| `/boot/extlinux/extlinux.conf` | 부트로더 설정 |
| `/etc/fstab` | 파일시스템 마운트 설정 |

---

## 참고 명령어

```bash
# 현재 커널 버전 확인
uname -r

# 설치된 커널 패키지 확인
dpkg -l | grep linux-image

# 업그레이드 가능한 커널 패키지 확인
apt list --upgradable 2>/dev/null | grep -E "linux|u-boot"

# 부트 파티션 UUID 확인
blkid

# 시스템 부팅 로그 확인
journalctl -b
```

---

**작성일**: 2026-01-24
**테스트 환경**: VIM4 + Ubuntu 22.04 Server + Kernel 5.15.119
