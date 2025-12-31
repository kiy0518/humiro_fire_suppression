# 워크스페이스 관리 가이드

## 워크스페이스가 GitHub에 포함되지 않는 이유

ROS2 워크스페이스(`workspaces/`)는 다음과 같은 이유로 GitHub에 빌드 결과물이 포함되지 않습니다:

1. **용량 문제**: `build/`, `install/`, `log/` 디렉토리는 수 GB에 달할 수 있습니다
2. **플랫폼 의존성**: 빌드 결과물은 특정 아키텍처/OS에 종속적입니다
3. **불필요**: 소스 코드만 있으면 어디서든 빌드할 수 있습니다

## 포함되는 것 vs 제외되는 것

### ✅ GitHub에 포함됨
- `workspaces/micro_ros_ws/src/` - Micro-ROS Agent 소스
- `workspaces/px4_ros2_ws/src/` - PX4 메시지 패키지 소스
- `workspaces/mavlink-router/` - MAVLink Router 소스 (용량 확인 필요)

### ❌ GitHub에 제외됨 (`.gitignore`)
- `workspaces/*/build/` - 빌드 결과물
- `workspaces/*/install/` - 설치 결과물
- `workspaces/*/log/` - 빌드 로그

## 워크스페이스 설정 방법

### 방법 1: 설치 스크립트 사용 (권장)

```bash
cd ~/humiro_fire_suppression
sudo ./scripts/install/000-install_all.sh
```

이 스크립트가 자동으로:
1. 워크스페이스 소스 다운로드 (없는 경우)
2. 의존성 설치
3. 워크스페이스 빌드

### 방법 2: 수동 빌드

#### Micro-ROS 워크스페이스

```bash
cd ~/humiro_fire_suppression/workspaces/micro_ros_ws

# 소스가 없으면 클론
if [ ! -d "src" ]; then
    mkdir -p src
    cd src
    git clone https://github.com/micro-ROS/micro-ROS-Agent.git
    cd ..
fi

# 빌드
colcon build
source install/setup.bash
```

#### PX4 ROS2 워크스페이스

```bash
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws

# 소스가 없으면 클론
if [ ! -d "src" ]; then
    mkdir -p src
    cd src
    git clone https://github.com/PX4/px4_msgs.git
    cd ..
fi

# 빌드
colcon build
source install/setup.bash
```

#### MAVLink Router

```bash
cd ~/humiro_fire_suppression/workspaces/mavlink-router

# 소스가 없으면 클론
if [ ! -d ".git" ]; then
    git clone https://github.com/mavlink-router/mavlink-router.git .
fi

# 빌드
git submodule update --init --recursive
meson setup build .
ninja -C build
sudo ninja -C build install
```

## 워크스페이스 소스를 Git에 포함할지 결정

### 옵션 1: 소스 포함 (현재 설정)
- **장점**: 모든 것이 한 저장소에 있음
- **단점**: 저장소 크기가 커짐 (수백 MB ~ 수 GB)

### 옵션 2: Git Submodule 사용
- **장점**: 저장소 크기 작음, 각 프로젝트 독립 관리
- **단점**: submodule 관리 필요

`.gitmodules` 예시:
```ini
[submodule "workspaces/micro_ros_ws/src/micro-ROS-Agent"]
    path = workspaces/micro_ros_ws/src/micro-ROS-Agent
    url = https://github.com/micro-ROS/micro-ROS-Agent.git

[submodule "workspaces/px4_ros2_ws/src/px4_msgs"]
    path = workspaces/px4_ros2_ws/src/px4_msgs
    url = https://github.com/PX4/px4_msgs.git
```

### 옵션 3: 설치 스크립트로 자동 다운로드
- **장점**: 저장소 크기 최소화
- **단점**: 초기 설정 시간 소요

## 권장 사항

1. **소스 코드는 포함**: 워크스페이스의 `src/` 디렉토리는 포함
2. **빌드 결과물은 제외**: `.gitignore`로 `build/`, `install/`, `log/` 제외
3. **설치 스크립트 제공**: 자동으로 소스 다운로드 및 빌드
4. **문서화**: README에 빌드 방법 명시

## 저장소 크기 최적화

```bash
# 저장소 크기 확인
du -sh ~/humiro_fire_suppression/workspaces/*/src 2>/dev/null

# 큰 디렉토리 확인
du -h ~/humiro_fire_suppression/workspaces/*/src 2>/dev/null | sort -h
```

만약 소스가 너무 크다면:
- Git LFS 사용 고려
- 또는 submodule로 전환
