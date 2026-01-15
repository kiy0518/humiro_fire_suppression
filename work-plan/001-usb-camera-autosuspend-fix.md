# 001 - USB 카메라 자동절전 문제 해결

**작성일:** 2026-01-13
**대상 기체:** 1번 (192.168.100.11)
**문제:** USB 허브를 통해 연결된 카메라들이 Linux에서 인식되지 않음

---

## 문제 상황

### 증상
- USB 웹캠과 열화상 카메라를 USB 허브에 연결
- Windows PC에서는 정상 동작
- Linux에서 `lsusb` 명령 시 허브만 보이고 카메라는 안 보임

### 초기 확인
```bash
lsusb
# Bus 001 Device 013: ID 0424:2514 USB 2.0 Hub (허브만 인식)

lsusb -t
# 허브 아래에 하위 장치 없음
```

---

## 원인 분석

### 1. 커널 에러 로그
```bash
dmesg | grep -i usb | tail -30
```
**결과:** `device descriptor read/64, error -71` 반복 발생

- **Error -71 (EPROTO)**: USB 프로토콜 에러
- 장치와 통신 불가 상태

### 2. USB 허브 전원 상태
```bash
cat /sys/bus/usb/devices/1-1.1/power/runtime_status
# 결과: suspended

cat /sys/bus/usb/devices/1-1.1/power/control
# 결과: auto
```

### 3. 근본 원인 확인
**Linux USB Autosuspend 기능이 원인**
- USB 허브가 자동으로 절전 모드 진입
- 절전 모드에서는 하위 포트 전원 차단
- 카메라들이 전원을 받지 못해 enumeration 실패

---

## 해결 방법

### 임시 해결 (현재 세션)
```bash
# USB 허브 autosuspend 비활성화
echo on | sudo tee /sys/bus/usb/devices/1-1.1/power/control

# 확인
sleep 3
lsusb
```

**결과:**
```
Bus 001 Device 014: ID 01da:5875 DSJ USB Camera (열화상)
Bus 001 Device 016: ID 1e4e:0100 Cubeternet WebCam (웹캠)
```
✅ 카메라 2개 모두 인식됨

### 영구 해결 (udev 규칙)

**파일 생성:** `/etc/udev/rules.d/50-usb-no-autosuspend.rules`

```bash
# USB 허브 autosuspend 비활성화
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0424", ATTR{idProduct}=="2514", TEST=="power/control", ATTR{power/control}="on"

# 열화상 카메라 autosuspend 비활성화
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="01da", ATTR{idProduct}=="5875", TEST=="power/control", ATTR{power/control}="on"

# 웹캠 autosuspend 비활성화
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1e4e", ATTR{idProduct}=="0100", TEST=="power/control", ATTR{power/control}="on"
```

**규칙 적용:**
```bash
sudo udevadm control --reload-rules
```

---

## 적용 결과

### USB 장치 목록
| Device ID | 제조사/모델 | 용도 | Video Device |
|-----------|------------|------|--------------|
| 0424:2514 | Microchip USB 2.0 Hub | USB 허브 | - |
| 01da:5875 | DSJ USB Camera | 열화상 카메라 | /dev/video2, video3 |
| 1e4e:0100 | Cubeternet WebCam | RGB 웹캠 | /dev/video0, video1 |

### Video 장치 확인
```bash
cat /sys/class/video4linux/video*/name | grep -E "USB Camera|Thermal"
```
결과:
- `video0, video1`: USB Camera (웹캠)
- `video2, video3`: HumiroThermal (fw:v1.3.1)

---

## 기술 배경

### USB Autosuspend란?
**개념:**
- Linux 커널의 USB 전원 관리 기능
- 미사용 USB 장치를 자동으로 절전 모드 전환
- 배터리 절약 목적

**설정 값:**
- `auto`: 자동 절전 활성화 (기본값)
- `on`: 절전 비활성화 (항상 켜짐)

### 왜 카메라가 안 보였나?

**절전 모드 진입 과정:**
1. USB 허브가 일정 시간 미사용
2. 커널이 허브를 `suspended` 상태로 전환
3. 허브가 하위 포트 전원 차단
4. 카메라들이 전원을 받지 못함
5. USB enumeration 실패
6. 시스템이 카메라를 발견하지 못함

**Error -71의 의미:**
- 절전 상태의 장치와 통신 시도 시 발생
- Device Descriptor 읽기 실패
- 장치가 응답하지 않음

---

## 주의사항 및 영향

### 전력 소비
- autosuspend 비활성화로 약간의 전력 소비 증가
- USB 카메라 자체 소비 전력이 크지 않아 영향 미미
- 드론 환경: 카메라 안정성 > 배터리 절약

### 재부팅 후
- udev 규칙 자동 적용
- USB 장치 연결 시 autosuspend 자동 비활성화
- 추가 조치 불필요

### 다른 기체 적용
- 동일한 USB 허브/카메라 사용 시 같은 문제 발생 가능
- 모든 기체에 동일한 udev 규칙 적용 권장

---

## 트러블슈팅

### Q: 재부팅 후에도 카메라가 인식 안 됨
```bash
# 1. udev 규칙 파일 확인
cat /etc/udev/rules.d/50-usb-no-autosuspend.rules

# 2. 규칙 재적용
sudo udevadm control --reload-rules

# 3. USB 물리적 재연결
# (케이블 뽑았다 꽂기)
```

### Q: 허브 포트 번호가 바뀜
- 포트 번호(1-1.1, 1-1.4)는 물리적 연결 위치에 따라 변경됨
- udev 규칙은 Vendor ID/Product ID로 식별
- 어느 포트에 연결해도 자동 적용됨

### Q: 다른 USB 장치도 영향 받나?
- 아니오. 규칙에 명시된 Vendor/Product ID만 적용
- 다른 USB 장치는 기본 autosuspend 동작 유지

---

## 체크리스트

- [x] USB 허브 인식 확인
- [x] 카메라 2개 인식 확인
- [x] Video 장치 생성 확인
- [x] udev 규칙 파일 생성
- [x] udev 규칙 적용
- [ ] 다른 기체에도 동일 규칙 적용 (필요 시)
- [ ] 재부팅 후 자동 적용 테스트

---

## 참고 자료

### 관련 파일
- `/etc/udev/rules.d/50-usb-no-autosuspend.rules`: udev 규칙 파일
- `/sys/bus/usb/devices/*/power/control`: USB 장치별 절전 설정
- `/sys/bus/usb/devices/*/power/runtime_status`: USB 장치 전원 상태

### 유용한 명령어
```bash
# USB 장치 목록
lsusb

# USB 트리 구조
lsusb -t

# 특정 장치 상세 정보
lsusb -v -d 01da:5875

# 커널 USB 로그
dmesg | grep -i usb

# Video 장치 목록
ls -la /dev/video*

# Video 장치 이름
cat /sys/class/video4linux/video*/name
```
