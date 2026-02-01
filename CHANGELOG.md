# Changelog

프로젝트의 모든 주목할 만한 변경사항이 이 파일에 문서화됩니다.

형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/)를 기반으로 하며,
이 프로젝트는 [시맨틱 버저닝](https://semver.org/lang/ko/)을 준수합니다.

---

## [v0.12.8] - 2026-02-01

### Added
- **docs/CUSTOM_MESSAGE_EXECUTION_CHECKLIST.md**
  - 커스텀 메시지 실행을 위한 상세 체크리스트
  - PX4 커스텀 메시지 설정 및 디버깅 가이드
  - 실행 단계별 검증 절차 포함

- **img/humiro_logo.png**
  - Humiro 프로젝트 공식 로고 이미지
  - 문서 및 프레젠테이션용 리소스

### Notes
- 어제 저장되지 않았던 문서 파일과 이미지 리소스 추가
- 커스텀 메시지 실행 프로세스 문서화 완료
- 프로젝트 브랜딩 리소스 추가

---

## [v0.12.6] - 2026-01-31

### Fixed
- RTL abort 버그 수정
- 상태 전환 중 hold-position 기능 추가

### Changed
- 중간 웨이포인트 방식을 통한 순항 속도 적용

---

## [v0.12.5] - 2026-01-31

### Changed
- 일반적인 개선사항 및 버그 수정

---

## [v0.12.4] - 2026-01-30

### Changed
- 일반적인 개선사항 및 버그 수정

---

## [v0.12.3] - 2026-01-29

### Changed
- 일반적인 개선사항 및 버그 수정

---

## [v0.12.2] - 2026-01-29

### Fixed
- vehicle_command_pub double free 버그 수정

### Changed
- MAVLink 송신 target_system을 MAV_SYS_ID 기반으로 변경

---

## [v0.12.1] - 2026-01-29

### Changed
- GUI 하드코딩 IP/ID/포트를 device_config.env 기반 동적 설정으로 전환
- 설정 관리 개선

---

## [v0.12.0] - 2026-01-28

### Changed
- **중요:** PX4 펌웨어 v1.15.0 → v1.16.0 업그레이드
- 백업 지점으로 중요한 릴리스

---

## [v0.11.8] - 2026-01-28

### Fixed
- MAVLink Wire Format 정렬 수정
- 메시지 ID 통일

---

## [v0.11.7] - 2026-01-27

### Fixed
- testMission3 Yaw/Heading 드리프트 문제 해결

### Removed
- 중복된 changelog 파일 제거

---

## [v0.11.6] - 2026-01-27

### Changed
- testMission3 비동기 상태 머신 리팩토링
- ⚠️ 테스트 필요

---

## [v0.11.5] - 2026-01-26

### Fixed
- OffboardManager 상태 리셋 버그 수정
- 미션 실패 후 재시도 가능하도록 개선

---

## [v0.11.4] - 2026-01-26

### Changed
- 커스텀 메시지 MSG ID를 60000번대로 변경
- OFFBOARD 모드 플래그 리셋 버그 수정

---

## [v0.11.3] - 2026-01-25

### Changed
- 커스텀 메시지 포트 개선
- 메시지 필터링 개선

---

## [v0.11.2] - 2026-01-25

### Added
- OFFBOARD 모드 안정화
- 착륙 복귀 기능 추가

---

## 버전 관리 규칙

이 프로젝트는 [CLAUDE.md](docs/CLAUDE_v1.0_backup.md)에 정의된 GitHub 규칙을 따릅니다:

- 모든 주요 변경사항은 문서화
- 시맨틱 버저닝 사용 (MAJOR.MINOR.PATCH)
- 의미 있는 커밋 메시지 작성
- 버전별 태그 생성

---

**형식 가이드:**

- `Added`: 새로운 기능
- `Changed`: 기존 기능의 변경사항
- `Deprecated`: 곧 제거될 기능
- `Removed`: 제거된 기능
- `Fixed`: 버그 수정
- `Security`: 보안 관련 변경사항
