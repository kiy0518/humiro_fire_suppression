# Humiro Fire Suppression 자동 실행 가이드

개발 중인 `humiro_fire_suppression` 프로그램을 systemd 서비스로 자동 실행하는 방법입니다.

## 빠른 시작

### 1. 서비스 설치 및 시작

```bash
# 서비스 파일 설치
./scripts/runtime/service-control.sh install

# 서비스 시작 (부팅 시 자동 시작 안 함)
./scripts/runtime/service-control.sh start
```

### 2. 부팅 시 자동 시작 활성화 (선택사항)

테스트 후 문제없으면 부팅 시 자동 시작을 활성화할 수 있습니다:

```bash
./scripts/runtime/service-control.sh enable
```

## 주요 명령어

### 서비스 제어

```bash
# 서비스 시작
./scripts/runtime/service-control.sh start

# 서비스 중지
./scripts/runtime/service-control.sh stop

# 서비스 재시작
./scripts/runtime/service-control.sh restart

# 서비스 상태 확인
./scripts/runtime/service-control.sh status
```

### 로그 확인

```bash
# 최근 50줄 로그 보기
./scripts/runtime/service-control.sh logs

# 최근 100줄 로그 보기
./scripts/runtime/service-control.sh logs 100

# 실시간 로그 모니터링
./scripts/runtime/service-control.sh follow

# systemd 저널 로그 (더 자세한 정보)
sudo journalctl -u humiro-fire-suppression -f
```

### 부팅 시 자동 시작 관리

```bash
# 부팅 시 자동 시작 활성화
./scripts/runtime/service-control.sh enable

# 부팅 시 자동 시작 비활성화
./scripts/runtime/service-control.sh disable
```

### 서비스 제거

```bash
# 서비스 중지 및 제거
./scripts/runtime/service-control.sh remove
```

## 개발 중 사용 팁

### 권장 워크플로우

1. **개발 중**: `start`만 사용 (부팅 시 자동 시작 안 함)
   ```bash
   ./scripts/runtime/service-control.sh start
   ```

2. **테스트 완료 후**: `enable`로 부팅 시 자동 시작 설정
   ```bash
   ./scripts/runtime/service-control.sh enable
   ```

3. **코드 수정 후**: 재시작
   ```bash
   ./scripts/runtime/service-control.sh restart
   ```

### 로그 파일 위치

- 표준 출력: `logs/humiro-service.log`
- 에러 로그: `logs/humiro-service-error.log`

### 문제 해결

#### 서비스가 시작되지 않는 경우

```bash
# 상태 확인
./scripts/runtime/service-control.sh status

# 상세 로그 확인
sudo journalctl -u humiro-fire-suppression -n 100

# 로그 파일 확인
tail -f logs/humiro-service-error.log
```

#### 실행 파일 경로 확인

서비스 파일에서 실행 파일 경로를 확인하세요:
- `/home/khadas/humiro_fire_suppression/application/build/humiro_fire_suppression`

빌드 후 실행 파일이 이 위치에 있는지 확인하세요.

#### 환경 변수 확인

서비스는 다음 파일에서 환경 변수를 로드합니다:
- `/home/khadas/humiro_fire_suppression/config/device_config.env`

필요한 환경 변수가 설정되어 있는지 확인하세요.

## 서비스 파일 위치

- 서비스 파일: `scripts/runtime/humiro-fire-suppression.service`
- 시스템 설치 위치: `/etc/systemd/system/humiro-fire-suppression.service`

## 주의사항

1. **개발 중**: `Restart=no`로 설정되어 있어서 프로그램이 종료되면 자동으로 재시작하지 않습니다.
   - 프로덕션 환경에서는 `Restart=always`로 변경할 수 있습니다.

2. **로그 파일**: 로그 파일은 계속 쌓이므로 주기적으로 정리하거나 로그 로테이션을 설정하세요.

3. **권한**: 서비스는 `khadas` 사용자로 실행됩니다. 필요한 하드웨어 접근 권한이 있는지 확인하세요.

## 수동 실행과 비교

### 수동 실행
```bash
cd /home/khadas/humiro_fire_suppression/application/build
./humiro_fire_suppression
```

### 서비스 실행 (자동)
```bash
./scripts/runtime/service-control.sh start
```

서비스를 사용하면:
- ✅ 백그라운드에서 실행
- ✅ 로그 파일에 자동 저장
- ✅ 부팅 시 자동 시작 가능
- ✅ 상태 모니터링 용이
- ✅ 재시작 간편

