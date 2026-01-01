# 열화상/라이다 ROS2 토픽 빠른 확인 가이드

## 단계 1: ROS2 환경 설정

```bash
# ROS2 Humble 환경 설정
source /opt/ros/humble/setup.bash

# 또는 Foxy인 경우
source /opt/ros/foxy/setup.bash

# 확인
ros2 --version
```

## 단계 2: ROS2 활성화 모드로 빌드

```bash
# 라이브러리 먼저 빌드 (ROS2 비활성화로 충분)
cd ~/humiro_fire_suppression/thermal/src/build
cmake .. && make -j$(nproc) thermal_lib

cd ~/humiro_fire_suppression/targeting/src/build
cmake .. && make -j$(nproc) targeting_lib

cd ~/humiro_fire_suppression/streaming/src/build
cmake .. && make -j$(nproc) streaming_lib

# 메인 애플리케이션을 ROS2 활성화 모드로 빌드
cd ~/humiro_fire_suppression/application/build
cmake .. -DENABLE_ROS2=ON
make -j$(nproc)
```

**주의**: ROS2 패키지가 없으면 빌드 실패 → ROS2 비활성화 모드로 사용

## 단계 3: 실행 및 토픽 확인

### 터미널 1: 애플리케이션 실행

```bash
cd ~/humiro_fire_suppression/application/build

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 실행
./humiro_fire_suppression
```

실행 시 "✓ ROS2 토픽 발행 활성화" 메시지가 보이면 성공!

### 터미널 2: 토픽 확인

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 모든 토픽 목록
ros2 topic list

# 열화상 토픽 확인
ros2 topic echo /thermal/max_temperature
ros2 topic echo /thermal/min_temperature
ros2 topic echo /thermal/center

# 라이다 토픽 확인
ros2 topic echo /lidar/front_distance
```

## 발행되는 토픽

### 열화상 토픽
- `/thermal/image` - 열화상 이미지
- `/thermal/max_temperature` - 최대 온도 (Float32)
- `/thermal/min_temperature` - 최소 온도 (Float32)
- `/thermal/center` - 중심점 좌표 (Point)
- `/thermal/hotspot` - Hotspot 좌표 (Point)

### 라이다 토픽
- `/lidar/points` - 포인트 클라우드 (PointCloud2)
- `/lidar/front_distance` - 전방 거리 (Float32, 미터)

## 빠른 명령어

```bash
# 토픽 목록
ros2 topic list | grep -E "(thermal|lidar)"

# 최대 온도 확인
ros2 topic echo /thermal/max_temperature

# 전방 거리 확인
ros2 topic echo /lidar/front_distance

# 토픽 정보 확인
ros2 topic info /thermal/max_temperature
ros2 topic info /lidar/front_distance

# 토픽 빈도 확인
ros2 topic hz /thermal/max_temperature
ros2 topic hz /lidar/front_distance
```

