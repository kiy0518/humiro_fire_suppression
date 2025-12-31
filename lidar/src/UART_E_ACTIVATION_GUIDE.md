# VIM4 UART_E 활성화 가이드

## 개요

VIM4에서 UART_E를 활성화하려면 `/boot/overlays/kvim4.dtb.overlay.env` 파일의 `fdt_overlays` 노드에 `uart_e`를 추가해야 합니다.

## 빠른 활성화 (권장)

### 자동 활성화 스크립트 사용

```bash
cd ~/humiro_fire_suppression/lidar/src
./enable_uart_e.sh
```

이 스크립트가 자동으로:
1. 현재 UART 상태 확인
2. 설정 파일 확인 및 백업 생성
3. `fdt_overlays=uart_e` 추가
4. 재부팅 안내

## 수동 활성화

### 1. 설정 파일 확인

```bash
# 설정 파일 위치 확인
ls -l /boot/overlays/kvim4.dtb.overlay.env
# 또는
ls -l /boot/dtb/amlogic/kvim4.dtb.overlay.env
```

### 2. 현재 내용 확인

```bash
cat /boot/overlays/kvim4.dtb.overlay.env
```

**현재 상태 예시:**
```
fdt_overlays=
```

### 3. 설정 파일 편집

```bash
sudo nano /boot/overlays/kvim4.dtb.overlay.env
```

### 4. uart_e 추가

**케이스 1: fdt_overlays가 비어있는 경우**
```
fdt_overlays=uart_e
```

**케이스 2: 다른 overlay가 이미 있는 경우**
```
fdt_overlays=existing_overlay uart_e
```

**예시:**
```
# 이전
fdt_overlays=spi1

# 이후
fdt_overlays=spi1 uart_e
```

### 5. 저장 및 종료

- `Ctrl+O` (저장)
- `Enter` (확인)
- `Ctrl+X` (종료)

## 재부팅 및 확인

### 1. 재부팅

```bash
sudo reboot
```

### 2. 재부팅 후 확인

```bash
# UART_E 포트 확인
ls -l /dev/ttyS4

# 출력 예시:
# crw-rw---- 1 root dialout 506, 4 Jan  1  1970 /dev/ttyS4
```

### 3. 권한 설정

```bash
# 권한 설정
sudo chmod 666 /dev/ttyS4

# 또는 사용자를 dialout 그룹에 추가 (권장)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### 4. 테스트

```bash
cd ~/humiro_fire_suppression/lidar/src/build
./lidar_test -g /dev/ttyS4 0 -n
```

## 설정 확인

### 활성화 상태 확인

```bash
# 설정 파일 확인
cat /boot/overlays/kvim4.dtb.overlay.env

# UART_E 포트 확인
ls -l /dev/ttyS4

# 활성화 스크립트로 확인
./activate_uart_e.sh
```

## 문제 해결

### 문제: 재부팅 후에도 /dev/ttyS4가 없음

**원인:**
- 설정 파일 경로가 잘못됨
- 설정 파일 형식 오류
- 커널이 overlay를 인식하지 못함

**해결:**
1. 설정 파일 확인:
   ```bash
   cat /boot/overlays/kvim4.dtb.overlay.env
   ```

2. 형식 확인:
   - 올바른 형식: `fdt_overlays=uart_e`
   - 잘못된 형식: `fdt_overlays = uart_e` (공백 있음)

3. 백업에서 복원 후 다시 시도:
   ```bash
   sudo cp /boot/overlays/kvim4.dtb.overlay.env.backup /boot/overlays/kvim4.dtb.overlay.env
   ./enable_uart_e.sh
   ```

### 문제: 설정 파일을 찾을 수 없음

**해결:**
```bash
# 실제 파일 위치 확인
find /boot -name "kvim4.dtb.overlay.env" 2>/dev/null

# 심볼릭 링크 확인
ls -l /boot/dtb/amlogic/kvim4.dtb.overlay.env
```

## 설정 되돌리기

### 백업에서 복원

```bash
# 백업 파일 확인
ls -l /boot/overlays/kvim4.dtb.overlay.env.backup

# 복원
sudo cp /boot/overlays/kvim4.dtb.overlay.env.backup /boot/overlays/kvim4.dtb.overlay.env

# 재부팅
sudo reboot
```

### 수동으로 제거

```bash
sudo nano /boot/overlays/kvim4.dtb.overlay.env

# uart_e 제거
# 이전: fdt_overlays=uart_e
# 이후: fdt_overlays=

# 또는 다른 overlay만 남기기
# 이전: fdt_overlays=spi1 uart_e
# 이후: fdt_overlays=spi1
```

## 참고

- 설정 파일 경로: `/boot/overlays/kvim4.dtb.overlay.env`
- 심볼릭 링크: `/boot/dtb/amlogic/kvim4.dtb.overlay.env` → `/boot/overlays/kvim4.dtb.overlay.env`
- 재부팅 필수: 설정 변경 후 반드시 재부팅 필요
- 백업 자동 생성: `enable_uart_e.sh` 스크립트가 자동으로 백업 생성

## 관련 파일

- 활성화 스크립트: `enable_uart_e.sh`
- 확인 스크립트: `activate_uart_e.sh`
- 설정 파일: `/boot/overlays/kvim4.dtb.overlay.env`
- 백업 파일: `/boot/overlays/kvim4.dtb.overlay.env.backup`

