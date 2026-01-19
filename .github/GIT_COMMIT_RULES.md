# Git 커밋 및 태그 규칙

## 버전 규칙 (Semantic Versioning)

```
v{MAJOR}.{MINOR}.{PATCH}
```

- **MAJOR**: 호환되지 않는 큰 변경 (v1.0.0, v2.0.0)
- **MINOR**: 새로운 기능 추가 (v0.10.0, v0.11.0)
- **PATCH**: 버그 수정, 작은 개선 (v0.10.1, v0.10.2)

### 현재 버전 상태
- 개발 중: v0.x.x
- 안정 버전: v1.0.0 이상 (아직 미도달)

## 커밋 메시지 형식

```
{type}: {간단한 설명} (v{버전})

{상세 설명}

기체: {N}번

Co-Authored-By: Claude {모델명} <noreply@anthropic.com>
```

### Type 종류
- `feat`: 새로운 기능
- `fix`: 버그 수정
- `refactor`: 리팩토링 (기능 변경 없음)
- `docs`: 문서 변경
- `style`: 코드 스타일 변경
- `test`: 테스트 추가/수정
- `chore`: 빌드, 설정 변경

### 상세 설명 필수 포함 항목
1. **문제/배경**: 왜 이 변경이 필요했는지
2. **해결 방법**: 어떻게 해결했는지
3. **변경된 파일**: 주요 변경 파일 목록
4. **테스트 환경**: 테스트한 기체 번호

## 태그 메시지 형식

```
{간단한 설명} - 기체 {N}번

주요 변경:
- 변경사항 1
- 변경사항 2

테스트: 기체 {N}번에서 검증됨
```

## 필수 확인 사항 (커밋 전 체크리스트)

- [ ] 기체 번호 확인 (`cat config/device_config.env | grep DRONE_ID`)
- [ ] 버전 번호 확인 (`git tag --list | sort -V | tail -1`)
- [ ] 변경된 파일 확인 (`git diff --stat`)
- [ ] 커밋 메시지에 기체 번호 포함
- [ ] 태그 메시지에 기체 번호 포함

## 예시

### 좋은 커밋 메시지
```
fix: 다중 드론 MAVLink 패킷 충돌 방지 (v0.10.0)

문제:
- QGC에서 1번과 3번 기체의 모드(Hold/Stabilized)가 번갈아 표시됨
- 원인: mavlink-router가 0.0.0.0으로 모든 인터페이스에서 수신하여
  다른 드론의 브로드캐스트 MAVLink 패킷도 함께 수신

해결:
1. mavlink-router FC 엔드포인트 수정 (003-apply_config.sh)
   - Address = 0.0.0.0 → Address = $ETH0_IP
   - 자신의 eth0 IP로만 FC 패킷 수신하여 다른 드론 패킷 차단

2. GUI FC 하트비트 필터링 추가 (gui/app.py)
   - wait_fc_heartbeat() 함수 신규 추가
   - 자기 자신 sysid=255 제외하고 실제 FC 하트비트만 대기
   - api_fc_version(), api_fc_reboot() API에 적용

변경 파일:
- scripts/install/003-apply_config.sh (mavlink-router 설정)
- gui/app.py (FC 하트비트 필터링)

기체: 3번

Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>
```

### 나쁜 커밋 메시지
```
fix 버그 수정
```
(기체 번호 없음, 상세 설명 없음, 버전 없음)
