# LiDAR 기반 타겟 거리 측정 및 타겟팅 가이드

## 개요

전방 LiDAR를 사용하여 타겟까지의 정확한 거리를 측정하고, 이를 바탕으로 소화탄 발사 각도를 계산하는 시스템입니다.

**핵심 기능:**
- ✅ 타겟까지의 정확한 거리 측정
- ✅ 발사 각도 자동 계산
- ✅ 타겟팅 정밀도 향상
- ✅ 안전 거리 확인

---

## LiDAR 하드웨어 선택

### 권장 하드웨어

**RPLIDAR A2 (권장)**
- 스캔 범위: 360도
- 측정 거리: 0.15m ~ 12m
- 스캔 주기: 10Hz
- 가격: 약 $150
- 인터페이스: USB, UART

**YDLIDAR X4**
- 스캔 범위: 270도
- 측정 거리: 0.12m ~ 12m
- 스캔 주기: 10Hz
- 가격: 약 $120
- 인터페이스: USB, UART

**TFMini Plus (ToF 센서, 단순)**
- 측정 거리: 0.1m ~ 12m
- 단일 거리 측정
- 가격: 약 $30
- 인터페이스: UART, I2C

**권장:** RPLIDAR A2 (360도 스캔으로 더 유연함)

---

## LiDAR 설치

### 설치 위치

**전방 설치:**
- 드론 전방에 설치
- 타겟 방향을 향하도록 고정
- 진동 최소화 (진동 댐퍼 사용)
- 각도 조정 가능하도록 마운트

**설치 각도:**
- 수평: 0도 (전방 정면)
- 또는 약간 아래 (-10도) - 타겟이 낮을 경우

### 하드웨어 연결

**Pixhawk 연결:**
```
LiDAR UART → Pixhawk TELEM 포트
또는
LiDAR USB → Khadas VIM4 USB 포트
```

**전원:**
- 5V 전원 공급
- Pixhawk AUX 전원 또는 별도 전원

---

## ROS2 통합

### LiDAR 드라이버 설치

**RPLIDAR A2:**
```bash
cd ~/projects/Cluster_Drone
mkdir -p lidar_ws/src
cd lidar_ws/src

# RPLIDAR ROS2 드라이버 클론
git clone https://github.com/robopeak/rplidar_ros2.git

cd ..
source /opt/ros/humble/setup.bash
colcon build
```

**YDLIDAR X4:**
```bash
git clone https://github.com/YDLIDAR/ydlidar_ros2.git
```

### LiDAR 드라이버 실행

```bash
# RPLIDAR A2 실행
ros2 run rplidar_ros rplidar_node \
    --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200 \
    -p frame_id:=lidar_link \
    -p angle_compensate:=true

# LiDAR 스캔 데이터 확인
ros2 topic echo /scan
```

---

## 타겟 거리 측정

### 열화상 카메라 + LiDAR 통합

**시스템 구조:**
```
열화상 카메라 → 타겟 각도 (라디안)
         ↓
    LiDAR → 타겟 방향 거리 (미터)
         ↓
    통합 → 정밀 타겟 좌표 (GPS)
```

**ROS2 노드:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class TargetDistanceNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'target_distance_{drone_id}')
        self.drone_id = drone_id
        
        # 열화상 카메라 타겟 각도 구독
        self.thermal_angle_sub = self.create_subscription(
            Float32,
            f'/drone_{drone_id}/fire_target/angle',
            self.thermal_angle_callback,
            10
        )
        
        # LiDAR 스캔 구독
        self.lidar_sub = self.create_subscription(
            LaserScan,
            f'/drone_{drone_id}/lidar/scan',
            self.lidar_callback,
            10
        )
        
        # 타겟 거리 발행
        self.distance_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/target_distance',
            10
        )
        
        # 타겟 3D 위치 발행
        self.target_3d_pub = self.create_publisher(
            PointStamped,
            f'/drone_{drone_id}/target_3d',
            10
        )
        
        self.target_angle = 0.0
        self.lidar_scan = None
    
    def thermal_angle_callback(self, msg):
        """
        열화상 카메라에서 타겟 각도 수신
        """
        self.target_angle = msg.data
        self.process_target_distance()
    
    def lidar_callback(self, msg):
        """
        LiDAR 스캔 데이터 수신
        """
        self.lidar_scan = msg
        self.process_target_distance()
    
    def process_target_distance(self):
        """
        타겟 방향의 거리 추출
        """
        if self.lidar_scan is None:
            return
        
        # 타겟 각도에 해당하는 LiDAR 거리 추출
        target_distance = self.extract_distance_at_angle(
            self.lidar_scan,
            self.target_angle
        )
        
        if target_distance > 0:
            # 타겟 거리 발행
            distance_msg = Float32()
            distance_msg.data = target_distance
            self.distance_pub.publish(distance_msg)
            
            # 3D 위치 계산
            target_3d = self.calculate_3d_position(target_distance)
            self.target_3d_pub.publish(target_3d)
    
    def extract_distance_at_angle(self, scan, target_angle):
        """
        특정 각도에서의 거리 추출
        """
        # LiDAR 스캔 각도 범위
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # 타겟 각도에 해당하는 인덱스 계산
        # LiDAR는 보통 전방이 0도 (또는 180도)
        # 열화상 카메라 각도를 LiDAR 좌표계로 변환
        lidar_angle = self.convert_to_lidar_frame(target_angle)
        
        # 인덱스 계산
        index = int((lidar_angle - angle_min) / angle_increment)
        
        # 인덱스 범위 확인
        if 0 <= index < len(scan.ranges):
            distance = scan.ranges[index]
            
            # 유효한 거리인지 확인
            if scan.range_min <= distance <= scan.range_max:
                return distance
        
        return -1.0
    
    def convert_to_lidar_frame(self, camera_angle):
        """
        카메라 좌표계 각도를 LiDAR 좌표계로 변환
        """
        # 카메라와 LiDAR의 상대 각도 고려
        # 예: 카메라가 전방, LiDAR도 전방이면 동일
        # 실제 설치 각도에 따라 조정 필요
        return camera_angle
    
    def calculate_3d_position(self, distance):
        """
        거리와 각도로 3D 위치 계산
        """
        from math import cos, sin
        
        # 3D 위치 계산
        target_3d = PointStamped()
        target_3d.header.frame_id = f'drone_{self.drone_id}_base_link'
        target_3d.header.stamp = self.get_clock().now().to_msg()
        
        # 카메라 좌표계에서 3D 위치
        target_3d.point.x = distance * cos(self.target_angle)
        target_3d.point.y = distance * sin(self.target_angle)
        target_3d.point.z = 0.0  # 수평 가정
        
        return target_3d
```

---

## 발사 각도 계산

### 포물선 운동 기반 계산

**물리 공식:**
```
x = v₀ * cos(θ) * t
y = v₀ * sin(θ) * t - (1/2) * g * t²

여기서:
- x: 수평 거리 (LiDAR 거리)
- y: 수직 거리 (고도 차이)
- v₀: 초기 속도
- θ: 발사 각도
- g: 중력 가속도 (9.81 m/s²)
```

**ROS2 노드:**
```python
class LaunchAngleCalculatorNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'launch_angle_calculator_{drone_id}')
        self.drone_id = drone_id
        
        # 타겟 거리 구독
        self.distance_sub = self.create_subscription(
            Float32,
            f'/drone_{drone_id}/target_distance',
            self.distance_callback,
            10
        )
        
        # 드론 고도 구독
        self.altitude_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.altitude_callback,
            10
        )
        
        # 발사 각도 발행
        self.angle_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/launch_angle',
            10
        )
        
        # 발사 속도 발행
        self.velocity_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/launch_velocity',
            10
        )
        
        self.target_distance = 0.0
        self.drone_altitude = 0.0
        self.target_altitude = 0.0  # 타겟 고도 (추정 또는 측정)
    
    def distance_callback(self, msg):
        self.target_distance = msg.data
        self.calculate_launch_parameters()
    
    def altitude_callback(self, msg):
        self.drone_altitude = msg.altitude
        self.calculate_launch_parameters()
    
    def calculate_launch_parameters(self):
        """
        발사 각도 및 속도 계산
        """
        if self.target_distance <= 0:
            return
        
        # 고도 차이
        altitude_diff = self.target_altitude - self.drone_altitude
        
        # 발사 각도 계산 (최적 각도)
        launch_angle = self.optimize_launch_angle(
            self.target_distance,
            altitude_diff
        )
        
        # 발사 속도 계산
        launch_velocity = self.calculate_velocity(
            self.target_distance,
            altitude_diff,
            launch_angle
        )
        
        # 발행
        angle_msg = Float32()
        angle_msg.data = launch_angle
        self.angle_pub.publish(angle_msg)
        
        velocity_msg = Float32()
        velocity_msg.data = launch_velocity
        self.velocity_pub.publish(velocity_msg)
    
    def optimize_launch_angle(self, distance, altitude_diff):
        """
        최적 발사 각도 계산
        """
        from math import atan, degrees
        
        # 기본 각도 (45도 근처가 최적)
        base_angle = 45.0
        
        # 고도 차이 보정
        if altitude_diff != 0:
            # tan(θ) ≈ (y + sqrt(y² + x²)) / x
            correction = degrees(atan(altitude_diff / distance))
            base_angle += correction
        
        # 거리 기반 보정
        if distance < 10.0:
            # 가까우면 각도 증가
            base_angle += 10.0
        elif distance > 30.0:
            # 멀면 각도 감소
            base_angle -= 5.0
        
        return base_angle
    
    def calculate_velocity(self, distance, altitude_diff, angle):
        """
        필요한 발사 속도 계산
        """
        from math import radians, sin, cos, sqrt
        
        g = 9.81
        angle_rad = radians(angle)
        
        # 포물선 운동 공식
        # v = sqrt((g * x²) / (x * sin(2θ) - 2y * cos²(θ)))
        
        x = distance
        y = altitude_diff
        
        numerator = g * x**2
        denominator = x * sin(2 * angle_rad) - 2 * y * cos(angle_rad)**2
        
        if denominator > 0:
            velocity = sqrt(numerator / denominator)
        else:
            # 기본 속도
            velocity = 15.0  # m/s
        
        # 최소/최대 속도 제한
        velocity = max(10.0, min(velocity, 30.0))
        
        return velocity
```

---

## 통합 타겟팅 시스템

### 열화상 카메라 + LiDAR 통합

**통합 제어 노드:**
```python
class IntegratedTargetingNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'integrated_targeting_{drone_id}')
        self.drone_id = drone_id
        
        # 열화상 카메라 타겟 각도
        self.thermal_angle = None
        
        # LiDAR 타겟 거리
        self.lidar_distance = None
        
        # RTK GPS 위치
        self.drone_position = None
        
        # 구독
        self.create_subscription(
            Float32,
            f'/drone_{drone_id}/fire_target/angle',
            self.thermal_callback,
            10
        )
        
        self.create_subscription(
            Float32,
            f'/drone_{drone_id}/target_distance',
            self.lidar_callback,
            10
        )
        
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.gps_callback,
            10
        )
        
        # 정밀 타겟 발행
        self.precise_target_pub = self.create_publisher(
            FireTarget,
            f'/drone_{drone_id}/precise_target',
            10
        )
        
        # 통합 제어 루프
        self.create_timer(0.1, self.integrated_targeting)
    
    def thermal_callback(self, msg):
        self.thermal_angle = msg.data
    
    def lidar_callback(self, msg):
        self.lidar_distance = msg.data
    
    def gps_callback(self, msg):
        self.drone_position = msg
    
    def integrated_targeting(self):
        """
        열화상 카메라 + LiDAR 통합 타겟팅
        """
        if None in [self.thermal_angle, self.lidar_distance, self.drone_position]:
            return
        
        # 타겟 GPS 좌표 계산
        target_lat, target_lon = self.calculate_target_gps(
            self.drone_position.latitude,
            self.drone_position.longitude,
            self.thermal_angle,
            self.lidar_distance
        )
        
        # 정밀 타겟 정보 발행
        target = FireTarget()
        target.latitude = target_lat
        target.longitude = target_lon
        target.altitude = self.drone_position.altitude
        target.distance = self.lidar_distance
        target.angle = self.thermal_angle
        target.accuracy = 0.1  # LiDAR 정밀도 (10cm)
        
        self.precise_target_pub.publish(target)
    
    def calculate_target_gps(self, lat, lon, angle, distance):
        """
        드론 위치 + 각도 + 거리로 타겟 GPS 계산
        """
        from math import radians, cos, sin
        
        # 거리를 위도/경도로 변환
        lat_offset = distance * cos(angle) / 111320.0
        lon_offset = distance * sin(angle) / (111320.0 * cos(radians(lat)))
        
        return lat + lat_offset, lon + lon_offset
```

---

## 소화탄 발사 제어

### LiDAR 거리 기반 발사

**발사 제어 노드:**
```python
class FireExtinguisherLaunchNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_extinguisher_launch_{drone_id}')
        self.drone_id = drone_id
        
        # 정밀 타겟 구독
        self.create_subscription(
            FireTarget,
            f'/drone_{drone_id}/precise_target',
            self.target_callback,
            10
        )
        
        # 발사 각도 구독
        self.create_subscription(
            Float32,
            f'/drone_{drone_id}/launch_angle',
            self.angle_callback,
            10
        )
        
        # MAVLink 서보 제어 클라이언트
        self.servo_client = self.create_client(
            CommandLong,
            f'/drone_{drone_id}/mavros/cmd/command'
        )
        
        self.target_distance = 0.0
        self.launch_angle = 0.0
    
    def target_callback(self, msg):
        self.target_distance = msg.distance
    
    def angle_callback(self, msg):
        self.launch_angle = msg.data
    
    def launch(self):
        """
        소화탄 발사
        """
        # 안전 확인
        if not self.safety_check():
            return False
        
        # 발사 각도 설정 (서보 모터)
        self.set_launch_angle(self.launch_angle)
        
        # 발사 실행 (솔레노이드 또는 서보)
        self.trigger_launch()
        
        return True
    
    def set_launch_angle(self, angle):
        """
        발사 각도 설정 (서보 모터)
        """
        # 각도를 PWM 값으로 변환
        # 0도 = 1000, 90도 = 2000
        pwm_value = 1000 + (angle / 90.0) * 1000
        
        cmd = CommandLong.Request()
        cmd.command = 183  # MAV_CMD_DO_SET_SERVO
        cmd.param1 = 6.0   # 서보 채널 (AUX6)
        cmd.param2 = pwm_value
        
        self.servo_client.call_async(cmd)
    
    def trigger_launch(self):
        """
        소화탄 격발
        """
        cmd = CommandLong.Request()
        cmd.command = 183  # MAV_CMD_DO_SET_SERVO
        cmd.param1 = 7.0   # 발사 서보 채널 (AUX7)
        cmd.param2 = 2000  # 격발 PWM 값
        
        self.servo_client.call_async(cmd)
        
        # 격발 후 원위치
        self.create_timer(1.0, lambda: self.reset_launcher())
    
    def safety_check(self):
        """
        안전 확인
        """
        # 최소 거리 확인
        if self.target_distance < 5.0:
            self.get_logger().warn("Too close to target!")
            return False
        
        # 최대 거리 확인
        if self.target_distance > 50.0:
            self.get_logger().warn("Too far from target!")
            return False
        
        return True
```

---

## 테스트 방법

### LiDAR 거리 측정 테스트

```bash
# LiDAR 드라이버 실행
ros2 run rplidar_ros rplidar_node

# LiDAR 스캔 데이터 확인
ros2 topic echo /scan

# 타겟 거리 확인
ros2 topic echo /drone_1/target_distance
```

### 통합 타겟팅 테스트

```bash
# 통합 타겟팅 노드 실행
ros2 run firefighting_targeting integrated_targeting_node --ros-args -p drone_id:=1

# 정밀 타겟 확인
ros2 topic echo /drone_1/precise_target
```

---

## 요약

**LiDAR의 역할:**
- ✅ 타겟까지의 정확한 거리 측정
- ✅ 발사 각도 계산에 필수
- ✅ 타겟팅 정밀도 향상
- ✅ 안전 거리 확인

**통합 시스템:**
- 열화상 카메라 → 타겟 각도
- LiDAR → 타겟 거리
- 통합 → 정밀 타겟 좌표 + 발사 각도

**필요한 하드웨어:**
- 전방 LiDAR (RPLIDAR A2 권장)
- 서보 모터 (발사 각도 제어)
- 솔레노이드 또는 서보 (격발)

