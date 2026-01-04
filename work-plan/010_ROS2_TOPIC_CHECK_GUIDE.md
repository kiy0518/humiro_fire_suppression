# 열화상/라이다 ROS2 토픽 확인 가이드

## 사전 확인

### 1. ROS2 설치 확인

```bash
# ROS2 설치 확인
which ros2
ros2 --version

# 출력 예시:
# ros2 version 3.3.2
```

ROS2가 설치되어 있지 않으면 설치가 필요합니다.

### 2. ROS2 환경 설정

```bash
# ROS2 환경 설정 (일반적으로)
source /opt/ros/humble/setup.bash

# 또는 설치 경로에 따라
source /opt/ros/foxy/setup.bash  # Foxy인 경우
```

## ROS2 활성화 빌드

### 1. ROS2 활성화 모드로 빌드

```bash
# 방법 1: 빌드 스크립트 사용 (권장, ROS2 활성화는 CMake 옵션으로)
cd ~/humiro_fire_suppression
# build.sh를 수정하여 -DENABLE_ROS2=ON 추가하거나
# application/build/CMakeLists.txt에서 직접 설정

# 방법 2: 수동 빌드
# 1. 라이브러리 빌드 (ROS2 비활성화로 충분)
cd ~/humiro_fire_suppression/thermal/src/build
cmake .. && make -j$(nproc) thermal_lib

cd ~/humiro_fire_suppression/targeting/src/build
cmake .. && make -j$(nproc) targeting_lib

cd ~/humiro_fire_suppression/streaming/src/build
cmake .. && make -j$(nproc) streaming_lib

# 2. 메인 애플리케이션 빌드 (ROS2 활성화)
cd ~/humiro_fire_suppression/application/build
rm -rf CMakeCache.txt CMakeFiles/
cmake .. -DENABLE_ROS2=ON
make -j$(nproc)
```

**주의**: ROS2 패키지가 설치되어 있어야 빌드가 성공합니다.

필요한 ROS2 패키지:
- `ros-humble-rclcpp` 또는 `ros-foxy-rclcpp`
- `ros-humble-sensor-msgs` 또는 `ros-foxy-sensor-msgs`
- `ros-humble-std-msgs` 또는 `ros-foxy-std-msgs`
- `ros-humble-geometry-msgs` 또는 `ros-foxy-geometry-msgs`
- `ros-humble-cv-bridge` 또는 `ros-foxy-cv-bridge`
- `ros-humble-pcl-conversions` 또는 `ros-foxy-pcl-conversions`

### 2. 빌드 오류 발생 시

ROS2 패키지가 없으면 빌드가 실패합니다. 이 경우:
- ROS2 설치가 필요하거나
- ROS2 비활성화 모드로 사용 (토픽 확인 불가)

## 실행 및 토픽 확인

### 1. 터미널 1: 애플리케이션 실행

```bash
cd ~/humiro_fire_suppression/application/build

# ROS2 환경 설정 (필요한 경우)
source /opt/ros/humble/setup.bash

# 애플리케이션 실행
./humiro_fire_suppression
```

실행 시 다음과 같은 메시지가 나타나면 ROS2가 활성화된 것입니다:
```
  ✓ ROS2 토픽 발행 활성화
```

### 2. 터미널 2: ROS2 토픽 확인

#### 모든 토픽 목록 확인

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 모든 토픽 목록
ros2 topic list
```

**예상 출력**:
```
/fmu/out/vehicle_status        # PX4 토픽 (Micro-ROS)
/fmu/out/vehicle_odometry      # PX4 토픽 (Micro-ROS)
/lidar/front_distance          # 라이다 토픽 (VIM4 내부)
/lidar/points                  # 라이다 토픽 (VIM4 내부)
/thermal/center                # 열화상 토픽 (VIM4 내부)
/thermal/hotspot               # 열화상 토픽 (VIM4 내부)
/thermal/image                 # 열화상 토픽 (VIM4 내부)
/thermal/max_temperature       # 열화상 토픽 (VIM4 내부)
/thermal/min_temperature       # 열화상 토픽 (VIM4 내부)
```

#### 특정 토픽 내용 확인

**열화상 데이터**:

```bash
# 최대 온도 확인
ros2 topic echo /thermal/max_temperature

# 출력 예시:
# data: 45.2

# 최소 온도 확인
ros2 topic echo /thermal/min_temperature

# 중심점 좌표 확인
ros2 topic echo /thermal/center

# 출력 예시:
# x: 240.0
# y: 165.0
# z: 0.0

# Hotspot 좌표 확인
ros2 topic echo /thermal/hotspot

# 열화상 이미지 확인 (한 번만)
ros2 topic echo /thermal/image --once
```

**라이다 데이터**:

```bash
# 전방 거리 확인
ros2 topic echo /lidar/front_distance

# 출력 예시:
# data: 2.5

# 포인트 클라우드 확인 (한 번만, 데이터가 큼)
ros2 topic echo /lidar/points --once
```

#### 토픽 정보 확인

```bash
# 토픽 타입 확인
ros2 topic info /thermal/max_temperature

# 출력 예시:
# Type: std_msgs/msg/Float32
# Publisher count: 1
# Subscription count: 0

# 토픽 타입 확인
ros2 topic info /lidar/points

# 출력 예시:
# Type: sensor_msgs/msg/PointCloud2
# Publisher count: 1
# Subscription count: 0

# 토픽 빈도 확인
ros2 topic hz /thermal/max_temperature

# 출력 예시:
# average rate: 8.234
#   min: 0.120s max: 0.125s std dev: 0.00141s window: 100
```

#### 토픽 데이터 타입 확인

```bash
# 메시지 타입 상세 확인
ros2 interface show std_msgs/msg/Float32

# 출력 예시:
# float32 data

ros2 interface show sensor_msgs/msg/PointCloud2 | head -20
```

## 문제 해결

### 1. 토픽이 보이지 않을 때

**확인 사항**:
1. 애플리케이션이 실행 중인가?
   ```bash
   ps aux | grep humiro_fire_suppression
   ```

2. ROS2 활성화 모드로 빌드되었는가?
   ```bash
   # 빌드 시 ROS2 활성화 확인
   cd ~/humiro_fire_suppression/application/build
   cat CMakeCache.txt | grep ENABLE_ROS2
   ```

3. ROS2 환경이 설정되었는가?
   ```bash
   source /opt/ros/humble/setup.bash
   echo $ROS_DOMAIN_ID  # 기본값: 0
   ```

4. 같은 ROS2 도메인에 있는가?
   ```bash
   # 터미널 1과 터미널 2에서 같은 도메인 ID 확인
   echo $ROS_DOMAIN_ID
   ```

### 2. ROS2 패키지 없음 오류

ROS2가 설치되어 있지 않으면:
- ROS2 설치 필요 (Ubuntu 22.04: ROS2 Humble)
- 또는 ROS2 비활성화 모드로 사용 (토픽 확인 불가)

### 3. 빌드 실패

ROS2 패키지가 없어서 빌드가 실패하면:
```bash
# ROS2 비활성화 모드로 빌드
cd ~/humiro_fire_suppression/application/build
cmake .. -DENABLE_ROS2=OFF
make -j$(nproc)
```

## 빠른 참조

### 토픽 확인 명령어 요약

```bash
# 모든 토픽 목록
ros2 topic list

# 특정 토픽 내용 확인
ros2 topic echo /thermal/max_temperature
ros2 topic echo /lidar/front_distance

# 토픽 정보 확인
ros2 topic info /thermal/max_temperature

# 토픽 빈도 확인
ros2 topic hz /thermal/max_temperature
```

### 발행되는 토픽 목록

**열화상 토픽**:
- `/thermal/image` - sensor_msgs/Image
- `/thermal/max_temperature` - std_msgs/Float32
- `/thermal/min_temperature` - std_msgs/Float32
- `/thermal/center` - geometry_msgs/Point
- `/thermal/hotspot` - geometry_msgs/Point

**라이다 토픽**:
- `/lidar/points` - sensor_msgs/PointCloud2
- `/lidar/front_distance` - std_msgs/Float32

