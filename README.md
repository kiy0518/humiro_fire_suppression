# Humiro Fire Suppression System

소방 드론 시스템 프로젝트

## 프로젝트 구조

```
humiro_fire_suppression/
├── docs/                    # 문서
├── config/                  # 설정 파일
├── scripts/                 # 스크립트
├── workspaces/              # ROS2 워크스페이스
├── common/                  # 공통 유틸리티
├── thermal/                 # 열화상 카메라
├── targeting/               # 타겟팅 시스템
├── throwing_mechanism/      # 투척 메커니즘
├── navigation/              # 네비게이션
├── qgc_custom/             # QGroundControl 커스터마이징
├── deployment/              # 배포 관련
├── tests/                   # 테스트
├── logs/                    # 로그
└── data/                    # 데이터
```

## 빠른 시작

### 1. 저장소 클론

```bash
git clone <repository-url> ~/humiro_fire_suppression
cd ~/humiro_fire_suppression
```

### 2. 환경 변수 설정

```bash
source ~/humiro_fire_suppression/setup_env.sh
```

또는 `~/.bashrc`에 추가:

```bash
echo "source ~/humiro_fire_suppression/setup_env.sh" >> ~/.bashrc
```

### 3. 설치

```bash
cd ~/humiro_fire_suppression
sudo ./scripts/install/000-install_all.sh
```

**참고**: 설치 스크립트가 자동으로 ROS2 워크스페이스를 빌드합니다.

### 4. 검증

```bash
./scripts/check/103-verify_installation.sh
```

## 워크스페이스 관리

### 중요: GitHub 저장소에는 워크스페이스 빌드 결과물이 포함되지 않습니다

이 프로젝트는 GitHub에 업로드될 때:
- ✅ **포함됨**: 소스 코드, 스크립트, 설정 파일
- ❌ **제외됨**: `workspaces/*/build/`, `workspaces/*/install/`, `workspaces/*/log/`

### 워크스페이스 재빌드

저장소를 클론한 후 워크스페이스를 빌드해야 합니다:

```bash
# Micro-ROS 워크스페이스
cd ~/humiro_fire_suppression/workspaces/micro_ros_ws
colcon build

# PX4 ROS2 워크스페이스
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws
colcon build
```

또는 설치 스크립트가 자동으로 처리합니다.

## 주요 환경 변수

- `PROJECT_ROOT`: 프로젝트 루트 디렉토리
- `MICRO_ROS_WS`: Micro-ROS 워크스페이스 경로
- `PX4_ROS2_WS`: PX4 ROS2 워크스페이스 경로
- `MAVLINK_ROUTER_DIR`: MAVLink Router 디렉토리

전체 환경 변수 목록은 `setup_env.sh` 참조

## 문서

- [설치 가이드](docs/installation/)
- [기술 문서](docs/technical/)

## 라이선스

[라이선스 정보 추가]
