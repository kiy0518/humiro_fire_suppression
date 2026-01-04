# 워크스페이스 재빌드 가이드

## 왜 재빌드가 필요한가?

프로젝트 구조를 재정비하면서 워크스페이스가 새 위치로 이동했습니다:
- 이전: `~/projects/Cluster_Drone/micro_ros_ws/`
- 현재: `~/humiro_fire_suppression/workspaces/micro_ros_ws/`

ROS2 워크스페이스의 빌드 결과물(`build/`, `install/`)에는 절대 경로가 포함되어 있어, 경로가 변경되면 재빌드가 필요합니다.

## 재빌드 방법

### 방법 1: 자동 재빌드 스크립트 (권장)

```bash
cd ~/humiro_fire_suppression
source setup_env.sh
./scripts/install/004-rebuild_workspaces.sh
```

이 스크립트가 자동으로:
1. 기존 빌드 결과물 삭제
2. Micro-ROS 워크스페이스 재빌드
3. PX4 ROS2 워크스페이스 재빌드
4. 소유권 설정

### 방법 2: 수동 재빌드

#### Micro-ROS 워크스페이스

```bash
cd ~/humiro_fire_suppression/workspaces/micro_ros_ws

# 기존 빌드 결과물 삭제
rm -rf build install log

# 재빌드
colcon build --symlink-install
source install/setup.bash
```

#### PX4 ROS2 워크스페이스

```bash
cd ~/humiro_fire_suppression/workspaces/px4_ros2_ws

# 기존 빌드 결과물 삭제
rm -rf build install log

# 재빌드
colcon build --symlink-install
source install/setup.bash
```

## 재빌드 후 필수 작업

워크스페이스를 재빌드한 후에는 드론 통신 설정도 업데이트해야 합니다:

```bash
# 1. 환경 변수 로드
source ~/humiro_fire_suppression/setup_env.sh

# 2. 드론 통신 설정 업데이트
sudo ./scripts/install/003-apply_config.sh

# 3. 연결 확인
./scripts/check/101-check_px4_connection.sh
```

## 빌드 시간

- Micro-ROS 워크스페이스: 약 5-10분
- PX4 ROS2 워크스페이스: 약 2-5분

## 문제 해결

### 빌드 실패 시

1. 의존성 확인:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. 빌드 로그 확인:
   ```bash
   cat /tmp/micro_ros_build.log
   cat /tmp/px4_ros2_build.log
   ```

3. 완전히 정리 후 재빌드:
   ```bash
   rm -rf build install log
   colcon build --symlink-install
   ```

## 체크리스트

- [ ] 워크스페이스 재빌드 완료
- [ ] 드론 통신 설정 업데이트 (003-apply_config.sh)
- [ ] 서비스 재시작 확인
- [ ] PX4 연결 확인
