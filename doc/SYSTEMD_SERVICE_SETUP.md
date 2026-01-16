# Systemd 서비스 자동 실행 설정

부팅 시 Humiro Fire Suppression 시스템을 자동으로 실행하도록 설정하는 방법입니다.

---

## 📋 설정 완료 상태

현재 시스템에는 다음과 같이 설정되어 있습니다:

- **서비스 이름**: `humiro-fire-suppression.service`
- **위치**: `/etc/systemd/system/humiro-fire-suppression.service`
- **상태**: 부팅 시 자동 시작 **활성화** ✅

---

## 🔧 서비스 파일 내용

### 서비스 파일: `/etc/systemd/system/humiro-fire-suppression.service`

```ini
[Unit]
Description=Humiro Fire Suppression System
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/khadas
ExecStart=/home/khadas/humiro_fire_suppression/application/build/humiro_fire_suppression
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

# 환경 변수 설정
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

[Install]
WantedBy=multi-user.target
```

### 설정 설명

| 항목 | 값 | 설명 |
|------|-----|------|
| `Description` | Humiro Fire Suppression System | 서비스 설명 |
| `After` | network.target | 네트워크가 준비된 후 실행 |
| `Type` | simple | 단순 실행 타입 |
| `User` | root | root 권한으로 실행 (카메라/LiDAR 접근) |
| `WorkingDirectory` | /home/khadas | 작업 디렉토리 |
| `ExecStart` | 실행 파일 경로 | 실제 실행할 프로그램 |
| `Restart` | on-failure | 실패 시 자동 재시작 |
| `RestartSec` | 5 | 재시작 대기 시간 (초) |
| `StandardOutput` | journal | 출력을 systemd 로그에 저장 |
| `StandardError` | journal | 에러를 systemd 로그에 저장 |

---

## 🎮 서비스 제어 명령어

### 기본 제어

#### 서비스 시작
```bash
sudo systemctl start humiro-fire-suppression
```

#### 서비스 중지
```bash
sudo systemctl stop humiro-fire-suppression
```

#### 서비스 재시작
```bash
sudo systemctl restart humiro-fire-suppression
```

#### 서비스 상태 확인
```bash
sudo systemctl status humiro-fire-suppression
```

**출력 예시**:
```
● humiro-fire-suppression.service - Humiro Fire Suppression System
     Loaded: loaded (/etc/systemd/system/humiro-fire-suppression.service; enabled; vendor preset: enabled)
     Active: active (running) since Thu 2026-01-16 10:30:45 KST; 1min 23s ago
   Main PID: 12345 (humiro_fire_sup)
      Tasks: 15 (limit: 4915)
     Memory: 45.2M
```

### 자동 시작 제어

#### 부팅 시 자동 시작 활성화
```bash
sudo systemctl enable humiro-fire-suppression
```

#### 부팅 시 자동 시작 비활성화
```bash
sudo systemctl disable humiro-fire-suppression
```

#### 자동 시작 상태 확인
```bash
sudo systemctl is-enabled humiro-fire-suppression
```
- `enabled`: 자동 시작 활성화
- `disabled`: 자동 시작 비활성화

---

## 📊 로그 확인

### 실시간 로그 보기
```bash
sudo journalctl -u humiro-fire-suppression -f
```

**종료**: `Ctrl + C`

### 최근 로그 확인
```bash
# 최근 100줄
sudo journalctl -u humiro-fire-suppression -n 100

# 최근 50줄
sudo journalctl -u humiro-fire-suppression -n 50

# 전체 로그
sudo journalctl -u humiro-fire-suppression
```

### 특정 시간대 로그
```bash
# 오늘 로그
sudo journalctl -u humiro-fire-suppression --since today

# 최근 1시간
sudo journalctl -u humiro-fire-suppression --since "1 hour ago"

# 특정 날짜
sudo journalctl -u humiro-fire-suppression --since "2026-01-16 09:00:00"
```

### 로그 검색
```bash
# "ERROR" 포함된 로그
sudo journalctl -u humiro-fire-suppression | grep ERROR

# "INDOOR" 포함된 로그
sudo journalctl -u humiro-fire-suppression | grep INDOOR
```

---

## 🔄 서비스 설정 수정

### 1. 서비스 파일 편집
```bash
sudo nano /etc/systemd/system/humiro-fire-suppression.service
```

### 2. 변경사항 적용
```bash
# systemd 데몬 리로드
sudo systemctl daemon-reload

# 서비스 재시작
sudo systemctl restart humiro-fire-suppression

# 상태 확인
sudo systemctl status humiro-fire-suppression
```

### 자주 수정하는 항목

#### 실행 파일 경로 변경
```ini
[Service]
ExecStart=/new/path/to/humiro_fire_suppression
```

#### 작업 디렉토리 변경
```ini
[Service]
WorkingDirectory=/new/working/directory
```

#### 재시작 정책 변경
```ini
[Service]
Restart=always              # 항상 재시작
# Restart=on-failure        # 실패 시만 재시작
# Restart=no                # 재시작 안 함
RestartSec=10               # 10초 후 재시작
```

#### 환경 변수 추가
```ini
[Service]
Environment="DRONE_ID=1"
Environment="LOG_LEVEL=DEBUG"
```

---

## 🚀 초기 설치 방법

새로운 시스템에 서비스를 설치하려면:

### 1. 서비스 파일 생성
```bash
sudo nano /etc/systemd/system/humiro-fire-suppression.service
```

위의 [서비스 파일 내용](#서비스-파일-etcsystemdsystemhumiro-fire-suppressionservice)을 복사하여 붙여넣기

### 2. systemd 리로드
```bash
sudo systemctl daemon-reload
```

### 3. 서비스 활성화
```bash
sudo systemctl enable humiro-fire-suppression
```

### 4. 서비스 시작
```bash
sudo systemctl start humiro-fire-suppression
```

### 5. 상태 확인
```bash
sudo systemctl status humiro-fire-suppression
```

---

## 🔍 문제 해결

### 서비스가 시작되지 않을 때

#### 1. 상태 확인
```bash
sudo systemctl status humiro-fire-suppression
```

#### 2. 상세 로그 확인
```bash
sudo journalctl -u humiro-fire-suppression -n 50 --no-pager
```

#### 3. 일반적인 문제

**실행 파일을 찾을 수 없음**
```
ExecStart=/home/khadas/humiro_fire_suppression/application/build/humiro_fire_suppression (code=exited, status=203/EXEC)
```

**해결**:
```bash
# 실행 파일 존재 확인
ls -la /home/khadas/humiro_fire_suppression/application/build/humiro_fire_suppression

# 실행 권한 확인
chmod +x /home/khadas/humiro_fire_suppression/application/build/humiro_fire_suppression
```

**권한 문제**
```
Permission denied
```

**해결**:
- 서비스 파일에 `User=root` 설정 확인
- 실행 파일 권한 확인

**포트 충돌**
```
Address already in use
```

**해결**:
```bash
# 포트 사용 프로세스 확인
sudo lsof -i :14553
sudo lsof -i :8080

# 기존 프로세스 종료
sudo killall humiro_fire_suppression
```

---

## ⚙️ 고급 설정

### 서비스 의존성 추가
```ini
[Unit]
Description=Humiro Fire Suppression System
After=network.target multi-user.target
Requires=network.target
```

### 메모리 제한
```ini
[Service]
MemoryLimit=500M
MemoryMax=1G
```

### CPU 제한
```ini
[Service]
CPUQuota=50%
```

### 프로세스 우선순위
```ini
[Service]
Nice=-10        # 높은 우선순위 (-20 ~ 19)
```

### 타임아웃 설정
```ini
[Service]
TimeoutStartSec=60     # 시작 타임아웃 60초
TimeoutStopSec=30      # 중지 타임아웃 30초
```

---

## 📝 체크리스트

설정 완료 후 확인사항:

- [ ] 서비스 파일이 올바른 위치에 있는가? (`/etc/systemd/system/`)
- [ ] systemd 데몬이 리로드되었는가? (`daemon-reload`)
- [ ] 서비스가 활성화되었는가? (`enable`)
- [ ] 서비스가 정상 실행되는가? (`status`)
- [ ] 로그가 정상적으로 출력되는가? (`journalctl`)
- [ ] 재부팅 후에도 자동 실행되는가?

---

## 🔗 관련 문서

- [systemd 공식 문서](https://www.freedesktop.org/software/systemd/man/systemd.service.html)
- [systemd 튜토리얼](https://www.digitalocean.com/community/tutorials/understanding-systemd-units-and-unit-files)

---

## 📅 변경 이력

| 날짜 | 버전 | 변경사항 |
|------|------|----------|
| 2026-01-16 | 1.0 | 초기 문서 작성 및 서비스 설치 |

---

**작성일**: 2026-01-16
**작성자**: Claude Sonnet 4.5
