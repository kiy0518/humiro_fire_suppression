# 화재 진압 군집 드론 시스템 기술 스택

## 프로젝트 개요

**목표:** 화재 발생 시 3대의 드론이 자동으로 발화점으로 이동하여 열화상 카메라로 타겟팅하고, 소화탄을 순차적으로 격발한 후 복귀하는 완전 자동화 시스템

**핵심 기능:**
1. 화재 감지 및 위치 파악
2. 자동 경로 계획 및 군집 비행
3. 열화상 카메라 기반 타겟팅
4. 소화탄 순차 격발
5. 자동 복귀 및 착륙

---

## 1. 기본 인프라 기술 (현재 구축됨)

### 1.1 통신 인프라

**ROS2 XRCE-DDS 통신:**
- ✅ PX4 ↔ SBC 통신 (이더넷)
- ✅ Micro-ROS Agent (XRCE-DDS 브리지)
- ✅ ROS2 토픽 기반 메시지 교환
- ✅ 멀티캐스트를 통한 다중 기체 통신

**MAVLink 통신:**
- ✅ QGroundControl 모니터링
- ✅ 비행 제어 명령 전송
- ✅ 상태 모니터링

**네트워크 구성:**
- ✅ 각 기체별 고유 IP (10.0.0.11, 10.0.0.21, 10.0.0.31)
- ✅ ROS2 도메인 분리 (도메인 1, 2, 3)
- ✅ WiFi 네트워크를 통한 중앙 제어

### 1.2 하드웨어 플랫폼

**Flight Controller:**
- Pixhawk 6X (Standard v2A PM02D)
- PX4 펌웨어 v1.16.0

**SBC:**
- Khadas VIM4 (Ubuntu 22.04 ARM64)
- ROS2 Humble

**지상 제어:**
- QGroundControl v5.0.8 (64-bit)

---

## 2. 화재 감지 및 위치 파악 기술

### 2.1 화재 감지 방법

#### 방법 1: 지상 센서 네트워크 (권장)

**구성:**
- 열화상 영상
- WiFi 통신
- 중앙 모니터링 시스템


**ROS2 토픽:**
```python
/fire_detection/fire_alert        # 화재 알림
/fire_detection/fire_location    # 발화점 좌표 (GPS)
/fire_detection/fire_intensity    # 화재 강도
```

#### 방법 2: 드론 기반 초기 탐색

**구성:**
- 드론이 정찰 비행
- 열화상 카메라로 화재 탐지
- 컴퓨터 비전 기반 화재 인식

**기술:**
- YOLO 또는 TensorFlow Lite (화재 객체 인식)
- 열화상 이미지 처리
- GPS 좌표 추출

### 2.2 위치 정보 처리

**필요한 정보:**
- 발화점 GPS 좌표 (위도, 경도, 고도)
- 화재 강도 및 범위
- 바람 방향 및 속도
- 주변 장애물 정보

**ROS2 메시지:**
```python
# px4_msgs 또는 커스텀 메시지
fire_location_msgs/FireAlert:
    float64 latitude
    float64 longitude
    float64 altitude
    float32 intensity
    float32 radius
    float32 wind_direction
    float32 wind_speed
```

---

## 3. 열화상 카메라 통합 기술

### 3.1 하드웨어

**열화상 카메라 옵션:**

**옵션 1: FLIR Lepton3.5 (경량, 저가)**
- 해상도: 160x120
- 인터페이스: SPI, I2C
- ROS2 드라이버: `flir_lepton` 패키지


### 3.2 ROS2 통합

**필요한 패키지:**
```bash
# 열화상 카메라 드라이버
ros-humble-flir-camera-driver
# 또는 커스텀 드라이버

# 이미지 처리
ros-humble-image-transport
ros-humble-cv-bridge
ros-humble-image-geometry

# LiDAR 처리
ros-humble-rplidar-ros          # RPLIDAR 드라이버
ros-humble-ydlidar-ros          # YDLIDAR 드라이버
ros-humble-laser-filters        # LiDAR 데이터 필터링
ros-humble-laser-geometry       # LiDAR 기하학 변환
ros-humble-laser-proc           # LiDAR 데이터 처리
```

**ROS2 토픽:**
```python
/drone_X/thermal_camera/image_raw      # 원본 열화상 이미지
/drone_X/thermal_camera/image_rect    # 보정된 이미지
/drone_X/thermal_camera/temperature   # 온도 데이터
/drone_X/thermal_camera/hotspot       # 핫스팟 좌표
```

### 3.3 화재 타겟팅 알고리즘

**컴퓨터 비전 처리:**

```python
# 1. 열화상 이미지 전처리
- 노이즈 제거
- 온도 임계값 필터링 (예: 100°C 이상)

# 2. 핫스팟 탐지
- 온도 그라디언트 분석
- 연결된 구성 요소 분석 (Connected Components)
- 최고 온도 지점 식별

# 3. 타겟 좌표 계산
- 이미지 좌표 → 카메라 좌표
- 카메라 좌표 → 드론 좌표
- 드론 좌표 → GPS 좌표
```

### 3.4 LiDAR 기반 거리 측정 (타겟팅 정밀도 향상)

**LiDAR의 역할:**
- 타겟까지의 정확한 거리 측정
- 소화탄 발사 각도 계산
- 타겟팅 정밀도 향상

**필요한 하드웨어:**

**옵션 1: 2D LiDAR (경량, 저가)**
- RPLIDAR A1/A2 (360도 스캔)
- YDLIDAR X4 (270도 스캔)
- 전방 거리 측정에 충분

**옵션 2: ToF 센서 (단순)**
- VL53L0X (단일 거리)
- TFMini Plus (단일 거리)
- 가볍지만 제한적

**권장: RPLIDAR A2 또는 YDLIDAR X4** (2D LiDAR, 전방 거리 측정에 충분)

**LiDAR 설치 위치:**
- 드론 전방에 설치
- 타겟 방향을 향하도록 고정
- 진동 최소화 (진동 댐퍼 사용)

**ROS2 통합:**

**필요한 패키지:**
```bash
ros-humble-rplidar-ros          # RPLIDAR 드라이버
ros-humble-ydlidar-ros          # YDLIDAR 드라이버
ros-humble-laser-filters        # LiDAR 데이터 필터링
ros-humble-laser-geometry       # LiDAR 기하학 변환
```

**LiDAR 드라이버 노드:**
```python
class LiDARDriverNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'lidar_driver_{drone_id}')
        self.drone_id = drone_id
        
        # LiDAR 스캔 데이터 구독
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/drone_{drone_id}/lidar/scan',
            self.scan_callback,
            10
        )
        
        # 타겟 방향 거리 발행
        self.target_distance_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/target_distance',
            10
        )
        
        # 타겟 각도 (열화상 카메라에서)
        self.target_angle_sub = self.create_subscription(
            Float32,
            f'/drone_{drone_id}/fire_target/angle',
            self.target_angle_callback,
            10
        )
        
        self.target_angle = 0.0  # 타겟 각도 (라디안)
    
    def target_angle_callback(self, msg):
        """
        열화상 카메라에서 타겟 각도 수신
        """
        self.target_angle = msg.data
    
    def scan_callback(self, msg):
        """
        LiDAR 스캔 데이터 처리
        """
        # 타겟 방향의 거리 추출
        target_distance = self.extract_target_distance(msg, self.target_angle)
        
        if target_distance > 0:
            # 타겟 거리 발행
            distance_msg = Float32()
            distance_msg.data = target_distance
            self.target_distance_pub.publish(distance_msg)
    
    def extract_target_distance(self, scan, target_angle):
        """
        타겟 방향의 거리 추출
        """
        # LiDAR 스캔 각도 범위
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        
        # 타겟 각도에 해당하는 인덱스 계산
        target_index = int((target_angle - angle_min) / angle_increment)
        
        # 인덱스 범위 확인
        if 0 <= target_index < len(scan.ranges):
            distance = scan.ranges[target_index]
            
            # 유효한 거리인지 확인 (최소/최대 범위)
            if scan.range_min <= distance <= scan.range_max:
                return distance
        
        return -1.0  # 유효하지 않음
```

**타겟팅 + LiDAR 통합:**

**정밀 타겟팅 노드:**
```python
class PreciseTargetingNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'precise_targeting_{drone_id}')
        self.drone_id = drone_id
        
        # 열화상 카메라 타겟 각도 구독
        self.thermal_angle_sub = self.create_subscription(
            Float32,
            f'/drone_{drone_id}/fire_target/angle',
            self.thermal_angle_callback,
            10
        )
        
        # LiDAR 거리 구독
        self.lidar_distance_sub = self.create_subscription(
            Float32,
            f'/drone_{drone_id}/target_distance',
            self.lidar_distance_callback,
            10
        )
        
        # 정밀 타겟 정보 발행
        self.precise_target_pub = self.create_publisher(
            FireTarget,
            f'/drone_{drone_id}/precise_target',
            10
        )
        
        self.target_angle = 0.0
        self.target_distance = 0.0
    
    def thermal_angle_callback(self, msg):
        self.target_angle = msg.data
        self.update_precise_target()
    
    def lidar_distance_callback(self, msg):
        self.target_distance = msg.data
        self.update_precise_target()
    
    def update_precise_target(self):
        """
        열화상 카메라 각도 + LiDAR 거리로 정밀 타겟 계산
        """
        if self.target_distance <= 0:
            return
        
        # 현재 드론 위치 (RTK GPS)
        drone_pos = self.get_drone_position()
        
        # 타겟 위치 계산
        target_lat, target_lon = self.calculate_target_position(
            drone_pos.latitude,
            drone_pos.longitude,
            self.target_angle,
            self.target_distance
        )
        
        # 정밀 타겟 정보 발행
        target = FireTarget()
        target.latitude = target_lat
        target.longitude = target_lon
        target.altitude = drone_pos.altitude  # 같은 고도 가정
        target.distance = self.target_distance
        target.angle = self.target_angle
        target.accuracy = 0.1  # LiDAR 정밀도 (10cm)
        
        self.precise_target_pub.publish(target)
    
    def calculate_target_position(self, lat, lon, angle, distance):
        """
        드론 위치 + 각도 + 거리로 타겟 GPS 좌표 계산
        """
        from math import radians, cos, sin
        
        # 거리를 위도/경도로 변환
        lat_offset = distance * cos(angle) / 111320.0
        lon_offset = distance * sin(angle) / (111320.0 * cos(radians(lat)))
        
        return lat + lat_offset, lon + lon_offset
```

**ROS2 노드 예시:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireTargetingNode(Node):
    def __init__(self):
        super().__init__('fire_targeting')
        self.bridge = CvBridge()
        
        # 열화상 이미지 구독
        self.thermal_sub = self.create_subscription(
            Image,
            '/drone_1/thermal_camera/image_raw',
            self.thermal_callback,
            10
        )
        
        # 타겟 좌표 발행
        self.target_pub = self.create_publisher(
            FireTarget,
            '/drone_1/fire_target',
            10
        )
    
    def thermal_callback(self, msg):
        # 이미지 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        
        # 온도 임계값 필터링
        threshold = 100  # 100°C
        hot_mask = cv_image > threshold
        
        # 핫스팟 탐지
        contours, _ = cv2.findContours(
            hot_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # 최대 온도 지점 찾기
        max_temp = 0
        target_pixel = None
        for contour in contours:
            for point in contour:
                temp = cv_image[point[0][1], point[0][0]]
                if temp > max_temp:
                    max_temp = temp
                    target_pixel = point[0]
        
        # 타겟 좌표 발행
        if target_pixel:
            # 픽셀 좌표 → GPS 좌표 변환
            target_gps = self.pixel_to_gps(target_pixel)
            self.publish_target(target_gps, max_temp)
```

---

## 4. 소화탄 격발 시스템

### 4.1 하드웨어 구성

**소화탄 발사 장치:**
- 솔레노이드 밸브
- 발사 제어 보드 (Arduino 또는 Pixhawk AUX 출력)
- 안전 장치 (격발 전 안전 확인)

**하드웨어 인터페이스:**
```
Pixhawk AUX 출력 → 발사 제어 보드 → 소화탄 발사 장치
```

### 4.2 ROS2 통합

**필요한 패키지:**
```bash
# PX4 AUX 출력 제어
ros-humble-mavros
# 또는 커스텀 노드
```

**ROS2 서비스:**
```python
# 소화탄 격발 서비스
firefighting_msgs/FireExtinguisherLaunch:
    uint8 drone_id
    float64 target_latitude
    float64 target_longitude
    float64 target_altitude
    ---
    bool success
    string message
```

**ROS2 노드 예시:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from firefighting_msgs.srv import FireExtinguisherLaunch

class FireExtinguisherNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_extinguisher_{drone_id}')
        self.drone_id = drone_id
        
        # MAVLink 명령 서비스 클라이언트
        self.mav_cmd_client = self.create_client(
            CommandLong,
            f'/drone_{drone_id}/mavros/cmd/command'
        )
        
        # 소화탄 격발 서비스
        self.launch_service = self.create_service(
            FireExtinguisherLaunch,
            f'/drone_{drone_id}/fire_extinguisher/launch',
            self.launch_callback
        )
    
    def launch_callback(self, request, response):
        # 안전 확인
        if not self.safety_check():
            response.success = False
            response.message = "Safety check failed"
            return response
        
        # AUX 출력 활성화 (소화탄 격발)
        # MAV_CMD_DO_SET_SERVO 사용
        cmd = CommandLong.Request()
        cmd.command = 183  # MAV_CMD_DO_SET_SERVO
        cmd.param1 = 5.0   # 서보 채널 (AUX5)
        cmd.param2 = 2000  # PWM 값 (격발)
        
        future = self.mav_cmd_client.call_async(cmd)
        # 결과 대기 및 처리
        
        response.success = True
        response.message = "Fire extinguisher launched"
        return response
    
    def safety_check(self):
        # 안전 확인 로직
        # - 드론 위치 확인
        # - 타겟 거리 확인
        # - 이전 격발 시간 확인
        return True
```

### 4.3 LiDAR 기반 발사 각도 계산

**소화탄 발사 각도 계산:**

타겟까지의 정확한 거리를 알면 발사 각도를 계산할 수 있습니다.

```python
class FireExtinguisherAimingNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_extinguisher_aiming_{drone_id}')
        self.drone_id = drone_id
        
        # 정밀 타겟 정보 구독
        self.precise_target_sub = self.create_subscription(
            FireTarget,
            f'/drone_{drone_id}/precise_target',
            self.target_callback,
            10
        )
        
        # 발사 각도 발행
        self.launch_angle_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/launch_angle',
            10
        )
        
        # 발사 속도 발행
        self.launch_velocity_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/launch_velocity',
            10
        )
    
    def target_callback(self, msg):
        """
        타겟 정보로 발사 각도 계산
        """
        target_distance = msg.distance  # LiDAR 거리
        target_altitude = msg.altitude
        drone_altitude = self.get_drone_altitude()
        
        # 고도 차이
        altitude_diff = target_altitude - drone_altitude
        
        # 발사 각도 계산 (포물선 운동)
        launch_angle = self.calculate_launch_angle(
            target_distance,
            altitude_diff
        )
        
        # 발사 속도 계산
        launch_velocity = self.calculate_launch_velocity(
            target_distance,
            altitude_diff,
            launch_angle
        )
        
        # 발사 파라미터 발행
        angle_msg = Float32()
        angle_msg.data = launch_angle
        self.launch_angle_pub.publish(angle_msg)
        
        velocity_msg = Float32()
        velocity_msg.data = launch_velocity
        self.launch_velocity_pub.publish(velocity_msg)
    
    def calculate_launch_angle(self, distance, altitude_diff):
        """
        발사 각도 계산 (포물선 운동)
        """
        from math import atan, sqrt
        
        # 중력 가속도
        g = 9.81
        
        # 최적 발사 각도 계산
        # tan(θ) = (v² + sqrt(v⁴ - g(gx² + 2yv²))) / (gx)
        # 단순화: 45도 근처에서 최적
        
        # 고도 차이 고려
        if altitude_diff > 0:
            # 타겟이 위에 있으면 각도 증가
            base_angle = 45.0 + (altitude_diff / distance) * 10.0
        else:
            # 타겟이 아래에 있으면 각도 감소
            base_angle = 45.0 + (altitude_diff / distance) * 10.0
        
        return base_angle
    
    def calculate_launch_velocity(self, distance, altitude_diff, angle):
        """
        발사 속도 계산
        """
        from math import radians, sin, cos, sqrt
        
        g = 9.81
        angle_rad = radians(angle)
        
        # 포물선 운동 공식
        # v = sqrt((g * x²) / (x * sin(2θ) - 2y * cos²(θ)))
        
        numerator = g * distance**2
        denominator = distance * sin(2 * angle_rad) - 2 * altitude_diff * cos(angle_rad)**2
        
        if denominator > 0:
            velocity = sqrt(numerator / denominator)
        else:
            velocity = 15.0  # 기본 속도 (m/s)
        
        return velocity
```

### 4.4 순차 격발 제어 (LiDAR 거리 기반)

**군집 제어 노드:**
```python
class SwarmFireControl(Node):
    def __init__(self):
        super().__init__('swarm_fire_control')
        self.drones = [1, 2, 3]
        self.launch_sequence = []
        
    def sequential_launch(self, targets):
        """
        순차적으로 소화탄 격발 (LiDAR 거리 기반)
        """
        for i, target in enumerate(targets):
            drone_id = self.drones[i % len(self.drones)]
            
            # 타겟 거리 확인 (LiDAR)
            target_distance = target.distance
            
            # 거리 기반 안전 확인
            if not self.safety_check(drone_id, target_distance):
                self.get_logger().warn(
                    f'Drone {drone_id}: Safety check failed for distance {target_distance}m'
                )
                continue
            
            # 격발 요청
            client = self.create_client(
                FireExtinguisherLaunch,
                f'/drone_{drone_id}/fire_extinguisher/launch'
            )
            
            request = FireExtinguisherLaunch.Request()
            request.drone_id = drone_id
            request.target_latitude = target.latitude
            request.target_longitude = target.longitude
            request.target_altitude = target.altitude
            request.target_distance = target_distance  # LiDAR 거리
            request.launch_angle = target.launch_angle  # 계산된 각도
            
            # 격발 실행
            future = client.call_async(request)
            
            # 다음 격발까지 대기 (안전 간격)
            time.sleep(2.0)  # 2초 간격
    
    def safety_check(self, drone_id, distance):
        """
        거리 기반 안전 확인
        """
        # 최소 거리 확인 (너무 가까우면 위험)
        min_distance = 5.0  # 5m
        if distance < min_distance:
            return False
        
        # 최대 거리 확인 (너무 멀면 정확도 저하)
        max_distance = 50.0  # 50m
        if distance > max_distance:
            return False
        
        return True
```

---

## 5. 군집 제어 알고리즘

### 5.1 경로 계획 (Path Planning)

**필요한 알고리즘:**

**A* 또는 RRT* (Rapidly-exploring Random Tree):**
- 장애물 회피
- 최적 경로 탐색
- 다중 목표점 처리

**ROS2 패키지:**
```bash
ros-humble-nav2          # 네비게이션 스택
ros-humble-planner      # 경로 계획
ros-humble-costmap-2d   # 비용 맵
```

**커스텀 경로 계획 노드:**
```python
class SwarmPathPlanner(Node):
    def __init__(self):
        super().__init__('swarm_path_planner')
        
    def plan_path(self, start, goal, obstacles):
        """
        경로 계획
        - 시작점: 현재 드론 위치
        - 목표점: 발화점
        - 장애물: 건물, 다른 드론 등
        """
        # A* 알고리즘 구현
        path = self.a_star(start, goal, obstacles)
        return path
    
    def plan_formation(self, fire_location, num_drones=3):
        """
        군집 형성 경로 계획
        - 발화점 주변에 형성 배치
        - 각 드론의 목표 위치 계산
        """
        formation_positions = []
        radius = 10.0  # 발화점으로부터 10m 반경
        
        for i in range(num_drones):
            angle = 2 * np.pi * i / num_drones
            x = fire_location.x + radius * np.cos(angle)
            y = fire_location.y + radius * np.sin(angle)
            z = fire_location.z + 5.0  # 고도 5m
            
            formation_positions.append((x, y, z))
        
        return formation_positions
```

### 5.2 RTK GPS 기반 충돌 방지 (RTK GPS Collision Avoidance)

**RTK GPS의 장점:**
- ✅ 센티미터 단위 정밀도 (일반 GPS: 미터 단위)
- ✅ 실시간 정밀 위치 추적
- ✅ 기체 간 정확한 거리 측정
- ✅ 안전한 편대 비행 가능

**RTK GPS 구성:**

```
RTK 기지국 (Base Station)
    │
    │ RTCM 보정 데이터 (무선 전송)
    │
    ├─→ Drone 1 (Rover)
    ├─→ Drone 2 (Rover)
    └─→ Drone 3 (Rover)
```

**필요한 하드웨어:**

**RTK GPS 모듈:**
- u-blox ZED-F9P (권장)
- Here+ RTK GPS
- Here3 RTK GPS
- Pixhawk와 직접 연결 가능

**RTK 기지국:**
- 고정된 위치에 설치
- RTCM 보정 데이터 전송
- 무선 링크 (LoRa, WiFi, 4G)

**PX4 RTK GPS 설정:**

```bash
# PX4 파라미터 설정
EKF2_AID_MASK = 24        # GPS + RTK 활성화
GPS_1_CONFIG = GPS
GPS_1_GNSS = 71           # GPS + GLONASS + Galileo
GPS_1_PROTOCOL = 4        # u-blox
GPS_YAW_OFFSET = 0
RTK_TYPE = 1              # RTK 활성화
```

**RTK GPS 통합:**

**ROS2 메시지 정의:**
```python
# rtk_gps_msgs/RTKStatus.msg
bool rtk_fix_available
uint8 rtk_fix_type        # 0=No RTK, 1=RTK Float, 2=RTK Fixed
float64 rtk_accuracy      # 정밀도 (미터)
```

**RTK GPS 드라이버 노드:**
```python
class RTKGPSDriverNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_gps_driver_{drone_id}')
        self.drone_id = drone_id
        
        # PX4 GPS 데이터 구독
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/gps',
            self.gps_callback,
            10
        )
        
        # RTCM 보정 데이터 구독 (기지국에서)
        self.rtcm_sub = self.create_subscription(
            RTCM,
            '/rtk_base/rtcm',
            self.rtcm_callback,
            10
        )
        
        # RTK GPS 위치 발행
        self.rtk_pub = self.create_publisher(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            10
        )
        
        # RTK 상태 발행
        self.status_pub = self.create_publisher(
            RTKStatus,
            f'/drone_{drone_id}/rtk_status',
            10
        )
    
    def gps_callback(self, msg):
        """
        GPS 데이터에 RTK 보정 적용
        """
        if self.rtk_fix_available:
            rtk_msg = NavSatFix()
            rtk_msg.latitude = msg.latitude + self.rtk_correction.lat
            rtk_msg.longitude = msg.longitude + self.rtk_correction.lon
            rtk_msg.altitude = msg.altitude + self.rtk_correction.alt
            
            # RTK FIX 정밀도 (센티미터)
            rtk_msg.position_covariance_type = 1  # RTK FIX
            rtk_msg.position_covariance = [
                0.01, 0, 0,      # 1cm 정밀도
                0, 0.01, 0,
                0, 0, 0.01
            ]
            
            self.rtk_pub.publish(rtk_msg)
```

**RTK GPS 기반 충돌 방지 노드:**
```python
class RTKCollisionAvoidanceNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_collision_avoidance_{drone_id}')
        self.drone_id = drone_id
        
        # 자신의 RTK GPS 위치
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.update_self_position,
            10
        )
        
        # 다른 드론의 RTK GPS 위치
        self.other_drones_rtk = {}
        for other_id in [1, 2, 3]:
            if other_id != drone_id:
                self.create_subscription(
                    NavSatFix,
                    f'/drone_{other_id}/rtk_gps',
                    lambda msg, id=other_id: self.update_other_rtk(id, msg),
                    10
                )
        
        # 충돌 회피 속도 발행
        self.avoidance_pub = self.create_publisher(
            Twist,
            f'/drone_{drone_id}/collision_avoidance/velocity',
            10
        )
        
        # 충돌 감지 타이머 (10Hz)
        self.create_timer(0.1, self.check_collision)
        
        self.min_safe_distance = 3.0  # 최소 안전 거리 3m
        self.collision_risk = False
    
    def update_self_position(self, msg):
        self.self_position = msg
    
    def update_other_rtk(self, drone_id, msg):
        self.other_drones_rtk[drone_id] = msg
    
    def check_collision(self):
        """
        RTK GPS 기반 정밀 충돌 감지
        """
        if not hasattr(self, 'self_position'):
            return
        
        self.collision_risk = False
        
        for other_id, other_pos in self.other_drones_rtk.items():
            # RTK GPS 좌표 간 정밀 거리 계산
            horizontal_distance = self.calculate_horizontal_distance(
                self.self_position.latitude,
                self.self_position.longitude,
                other_pos.latitude,
                other_pos.longitude
            )
            
            # 고도 차이
            altitude_diff = abs(self.self_position.altitude - other_pos.altitude)
            
            # 3D 거리
            distance_3d = np.sqrt(horizontal_distance**2 + altitude_diff**2)
            
            # 충돌 위험 판단
            if altitude_diff < 2.0 and horizontal_distance < self.min_safe_distance:
                self.collision_risk = True
                self.trigger_avoidance(other_id, other_pos, horizontal_distance, altitude_diff)
    
    def calculate_horizontal_distance(self, lat1, lon1, lat2, lon2):
        """
        두 RTK GPS 좌표 간 수평 거리 계산 (미터, 정밀)
        """
        from math import radians, cos, sin, asin, sqrt
        
        R = 6371000  # 지구 반지름 (미터)
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        return R * c
    
    def trigger_avoidance(self, other_id, other_pos, h_dist, v_dist):
        """
        충돌 회피 동작 실행
        """
        # 회피 방향 계산
        avoidance_vector = self.compute_avoidance_vector(other_pos, h_dist, v_dist)
        
        # 회피 속도 발행
        avoidance_velocity = Twist()
        avoidance_velocity.linear.x = avoidance_vector[0]
        avoidance_velocity.linear.y = avoidance_vector[1]
        avoidance_velocity.linear.z = avoidance_vector[2]
        
        self.avoidance_pub.publish(avoidance_velocity)
        
        self.get_logger().warn(
            f'⚠️ RTK Collision risk with drone {other_id}! '
            f'Horizontal: {h_dist:.2f}m, Vertical: {v_dist:.2f}m'
        )
    
    def compute_avoidance_vector(self, other_pos, h_dist, v_dist):
        """
        RTK GPS 기반 회피 벡터 계산
        """
        # 자신의 위치에서 다른 드론 방향으로의 벡터
        dlat = other_pos.latitude - self.self_position.latitude
        dlon = other_pos.longitude - self.self_position.longitude
        
        # 정규화
        norm = np.sqrt(dlat**2 + dlon**2)
        if norm > 0:
            dlat /= norm
            dlon /= norm
        
        # 반대 방향으로 회피 (수평)
        avoidance_x = -dlon * 2.0  # 회피 속도 (m/s)
        avoidance_y = -dlat * 2.0
        
        # 고도 회피 (위로 상승)
        if v_dist < 1.0:  # 고도 차이가 1m 미만
            avoidance_z = 1.0  # 위로 상승
        else:
            avoidance_z = 0.0
        
        return (avoidance_x, avoidance_y, avoidance_z)
    
    def is_collision_risk(self):
        return self.collision_risk
```

### 5.3 편대 비행 (Formation Flight)

**편대 비행이란:**
- 여러 드론이 일정한 형상(formation)을 유지하며 함께 비행
- 리더 드론을 기준으로 상대적 위치 유지
- 군집 비행의 핵심 기술

**편대 형상 옵션:**

**1. 삼각형 편대 (Triangle Formation):**
```
        Leader
       /     \
    Follower1  Follower2
```

**2. 일렬 편대 (Line Formation):**
```
Leader → Follower1 → Follower2
```

**3. V자 편대 (V Formation):**
```
    Follower1
         \
          Leader
         /
    Follower2
```

**4. 원형 편대 (Circle Formation):**
```
    Follower1
         |
Leader - O - Follower2
```

**필요한 알고리즘:**

**Leader-Follower 방식:**
- 리더 드론이 경로를 따라감
- 팔로워 드론이 리더를 따라 형성 유지
- RTK GPS로 정밀 위치 추적

**Consensus 기반 방식:**
- 모든 드론이 동등하게 협력
- 분산 제어
- 더 복잡하지만 더 안정적

**ROS2 노드:**
```python
class FormationFlightNode(Node):
    def __init__(self, drone_id, formation_type='triangle', role='follower'):
        super().__init__(f'formation_flight_{drone_id}')
        self.drone_id = drone_id
        self.formation_type = formation_type
        self.role = role  # 'leader' or 'follower'
        
        # RTK GPS 위치 구독
        self.rtk_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.rtk_callback,
            10
        )
        
        # 다른 드론 위치 구독 (RTK GPS)
        self.other_drones_pos = {}
        for other_id in [1, 2, 3]:
            if other_id != drone_id:
                self.create_subscription(
                    NavSatFix,
                    f'/drone_{other_id}/rtk_gps',
                    lambda msg, id=other_id: self.update_other_position(id, msg),
                    10
                )
        
        # 편대 위치 명령 발행
        self.formation_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/formation_setpoint',
            10
        )
    
    def rtk_callback(self, msg):
        """
        RTK GPS 위치 업데이트
        """
        self.current_position = msg
        if self.role == 'leader':
            self.publish_formation_command()
        else:
            self.follow_formation()
    
    def update_other_position(self, drone_id, msg):
        """
        다른 드론의 RTK GPS 위치 업데이트
        """
        self.other_drones_pos[drone_id] = msg
    
    def calculate_formation_offset(self):
        """
        편대 형상에 따른 오프셋 계산
        """
        if self.formation_type == 'triangle':
            if self.drone_id == 1:  # Leader
                return (0.0, 0.0, 0.0)
            elif self.drone_id == 2:  # Follower 1
                return (-5.0, 5.0, 0.0)  # 왼쪽 뒤
            elif self.drone_id == 3:  # Follower 2
                return (5.0, 5.0, 0.0)   # 오른쪽 뒤
        
        elif self.formation_type == 'line':
            if self.drone_id == 1:  # Leader
                return (0.0, 0.0, 0.0)
            elif self.drone_id == 2:
                return (0.0, -10.0, 0.0)  # 뒤
            elif self.drone_id == 3:
                return (0.0, -20.0, 0.0)  # 더 뒤
        
        elif self.formation_type == 'v_formation':
            if self.drone_id == 1:  # Leader
                return (0.0, 0.0, 0.0)
            elif self.drone_id == 2:
                return (-10.0, -5.0, 0.0)  # 왼쪽 뒤
            elif self.drone_id == 3:
                return (10.0, -5.0, 0.0)   # 오른쪽 뒤
        
        elif self.formation_type == 'circle':
            angle = 2 * np.pi * (self.drone_id - 1) / 3
            radius = 10.0
            return (
                radius * np.cos(angle),
                radius * np.sin(angle),
                0.0
            )
    
    def follow_formation(self):
        """
        리더를 따라 편대 유지
        """
        if 1 not in self.other_drones_pos:  # Leader 위치 없음
            return
        
        leader_pos = self.other_drones_pos[1]
        offset = self.calculate_formation_offset()
        
        # RTK GPS 좌표에서 오프셋 적용
        target_lat, target_lon = self.apply_offset(
            leader_pos.latitude,
            leader_pos.longitude,
            offset[0],
            offset[1]
        )
        
        # 편대 위치 명령 발행
        setpoint = TrajectorySetpoint()
        setpoint.position = [target_lat, target_lon, leader_pos.altitude + offset[2]]
        setpoint.yaw = self.calculate_formation_yaw(leader_pos)
        
        self.formation_pub.publish(setpoint)
    
    def apply_offset(self, lat, lon, offset_x, offset_y):
        """
        GPS 좌표에 미터 단위 오프셋 적용
        """
        # 미터를 위도/경도로 변환
        lat_offset = offset_y / 111320.0  # 1도 ≈ 111.32km
        lon_offset = offset_x / (111320.0 * np.cos(np.radians(lat)))
        
        return lat + lat_offset, lon + lon_offset
    
    def calculate_formation_yaw(self, leader_pos):
        """
        편대 방향에 따른 요각 계산
        """
        # 리더의 이동 방향을 따라 요각 설정
        # 실제로는 리더의 속도 벡터를 사용
        return 0.0  # 예시
```

### 5.4 RTK GPS 기반 충돌 방지

**RTK GPS란:**
- Real-Time Kinematic GPS
- 센티미터 단위 정밀도 (일반 GPS는 미터 단위)
- 기지국(base station)과 로버(rover) 간 상대 측위

**RTK GPS 구성:**

```
기지국 (Base Station)
    │
    │ RTCM 보정 데이터
    │
로버 (Rover) - 각 드론
```

**필요한 하드웨어:**

**옵션 1: RTK GPS 모듈 (권장)**
- u-blox ZED-F9P
- Pixhawk와 직접 연결 가능
- RTCM 데이터 수신 (무선 또는 지상국)

**옵션 2: RTK GPS 보드**
- Here+ RTK GPS
- Here3 RTK GPS
- Pixhawk와 호환

**RTK GPS 설정:**

**PX4 파라미터:**
```
EKF2_AID_MASK = 24        # GPS + RTK 활성화
GPS_1_CONFIG = GPS
GPS_1_GNSS = 71           # GPS + GLONASS + Galileo
GPS_1_PROTOCOL = 4        # u-blox
GPS_YAW_OFFSET = 0        # GPS 요각 오프셋
```

**RTCM 데이터 수신:**
- 무선 링크 (LoRa, WiFi)
- 또는 지상국에서 직접 전송

**ROS2 통합:**

**RTK GPS 드라이버 노드:**
```python
class RTKGPSNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_gps_{drone_id}')
        self.drone_id = drone_id
        
        # RTK GPS 데이터 구독 (PX4에서)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/gps',
            self.gps_callback,
            10
        )
        
        # RTCM 보정 데이터 구독 (기지국에서)
        self.rtcm_sub = self.create_subscription(
            RTCM,
            '/rtk_base/rtcm',
            self.rtcm_callback,
            10
        )
        
        # RTK GPS 위치 발행
        self.rtk_pub = self.create_publisher(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            10
        )
        
        # RTK 상태 발행
        self.status_pub = self.create_publisher(
            RTKStatus,
            f'/drone_{drone_id}/rtk_status',
            10
        )
    
    def gps_callback(self, msg):
        """
        GPS 데이터 처리
        """
        # RTK 보정 적용
        if self.rtk_fix_available:
            rtk_msg = NavSatFix()
            rtk_msg.latitude = msg.latitude + self.rtk_correction.lat
            rtk_msg.longitude = msg.longitude + self.rtk_correction.lon
            rtk_msg.altitude = msg.altitude + self.rtk_correction.alt
            rtk_msg.position_covariance_type = 1  # RTK FIX
            rtk_msg.position_covariance = [
                0.01, 0, 0,      # 센티미터 정밀도
                0, 0.01, 0,
                0, 0, 0.01
            ]
            self.rtk_pub.publish(rtk_msg)
    
    def rtcm_callback(self, msg):
        """
        RTCM 보정 데이터 처리
        """
        # RTCM 데이터를 GPS 모듈로 전송
        self.send_rtcm_to_gps(msg.data)
```

**RTK GPS 기반 충돌 방지:**

**정밀 위치 기반 충돌 감지:**
```python
class RTKCollisionAvoidanceNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_collision_avoidance_{drone_id}')
        self.drone_id = drone_id
        
        # 자신의 RTK GPS 위치
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.update_self_position,
            10
        )
        
        # 다른 드론의 RTK GPS 위치
        self.other_drones_rtk = {}
        for other_id in [1, 2, 3]:
            if other_id != drone_id:
                self.create_subscription(
                    NavSatFix,
                    f'/drone_{other_id}/rtk_gps',
                    lambda msg, id=other_id: self.update_other_rtk(id, msg),
                    10
                )
        
        # 충돌 회피 속도 발행
        self.avoidance_pub = self.create_publisher(
            Twist,
            f'/drone_{drone_id}/collision_avoidance/velocity',
            10
        )
        
        # 충돌 감지 타이머
        self.create_timer(0.1, self.check_collision)  # 10Hz
    
    def update_self_position(self, msg):
        self.self_position = msg
    
    def update_other_rtk(self, drone_id, msg):
        self.other_drones_rtk[drone_id] = msg
    
    def check_collision(self):
        """
        RTK GPS 기반 충돌 감지
        """
        if not hasattr(self, 'self_position'):
            return
        
        min_safe_distance = 3.0  # 최소 안전 거리 3m
        
        for other_id, other_pos in self.other_drones_rtk.items():
            # RTK GPS 좌표 간 거리 계산
            distance = self.calculate_distance(
                self.self_position.latitude,
                self.self_position.longitude,
                other_pos.latitude,
                other_pos.longitude
            )
            
            # 고도 차이 고려
            altitude_diff = abs(self.self_position.altitude - other_pos.altitude)
            
            # 수평 거리만 고려 (고도 차이가 충분하면 안전)
            if altitude_diff < 2.0:  # 고도 차이 2m 미만
                if distance < min_safe_distance:
                    # 충돌 위험! 회피 동작
                    self.trigger_avoidance(other_id, other_pos, distance)
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        두 GPS 좌표 간 거리 계산 (미터)
        """
        from math import radians, cos, sin, asin, sqrt
        
        # 하버사인 공식
        R = 6371000  # 지구 반지름 (미터)
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        return R * c
    
    def trigger_avoidance(self, other_id, other_pos, distance):
        """
        충돌 회피 동작 실행
        """
        # 회피 방향 계산
        avoidance_vector = self.compute_avoidance_vector(other_pos)
        
        # 회피 속도 발행
        avoidance_velocity = Twist()
        avoidance_velocity.linear.x = avoidance_vector[0]
        avoidance_velocity.linear.y = avoidance_vector[1]
        avoidance_velocity.linear.z = avoidance_vector[2]  # 고도 변경
        
        self.avoidance_pub.publish(avoidance_velocity)
        
        self.get_logger().warn(
            f'Collision risk with drone {other_id}! '
            f'Distance: {distance:.2f}m'
        )
    
    def compute_avoidance_vector(self, other_pos):
        """
        회피 벡터 계산
        """
        # 자신의 위치에서 다른 드론 방향으로의 벡터
        dx = other_pos.latitude - self.self_position.latitude
        dy = other_pos.longitude - self.self_position.longitude
        
        # 반대 방향으로 회피
        avoidance_x = -dx * 2.0  # 회피 속도
        avoidance_y = -dy * 2.0
        
        # 고도 회피 (위로 상승)
        avoidance_z = 1.0
        
        return (avoidance_x, avoidance_y, avoidance_z)
```

**편대 비행 + RTK GPS 통합:**

**통합 제어 노드:**
```python
class IntegratedFormationControlNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'integrated_formation_{drone_id}')
        self.drone_id = drone_id
        
        # 편대 비행 노드
        self.formation_node = FormationFlightNode(drone_id)
        
        # RTK 충돌 회피 노드
        self.collision_node = RTKCollisionAvoidanceNode(drone_id)
        
        # 통합 제어 명령 발행
        self.integrated_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/integrated_setpoint',
            10
        )
        
        # 통합 제어 루프
        self.create_timer(0.1, self.integrated_control)  # 10Hz
    
    def integrated_control(self):
        """
        편대 비행 + 충돌 회피 통합 제어
        """
        # 편대 비행 목표 위치
        formation_setpoint = self.formation_node.get_setpoint()
        
        # 충돌 회피 보정
        avoidance_correction = self.collision_node.get_avoidance_correction()
        
        # 통합 명령 계산
        integrated_setpoint = TrajectorySetpoint()
        integrated_setpoint.position[0] = formation_setpoint.position[0] + avoidance_correction[0]
        integrated_setpoint.position[1] = formation_setpoint.position[1] + avoidance_correction[1]
        integrated_setpoint.position[2] = formation_setpoint.position[2] + avoidance_correction[2]
        
        # 우선순위: 충돌 회피 > 편대 유지
        if self.collision_node.is_collision_risk():
            # 충돌 위험 시 편대보다 충돌 회피 우선
            integrated_setpoint = self.collision_node.get_emergency_setpoint()
        
        self.integrated_pub.publish(integrated_setpoint)
```

---

## 6. RTK GPS 기반 정밀 호버링 (Precision Hovering)

### 6.1 RTK GPS를 사용한 정밀 위치 유지

**"말뚝처럼 고정" 호버링:**
- RTK GPS의 센티미터 단위 정밀도로 위치 유지
- 바람이나 외부 요인에도 위치 고정
- 정밀 타겟팅 및 발사에 필수

**일반 GPS vs RTK GPS:**
- 일반 GPS: 미터 단위 정밀도 (1-3m 오차)
- RTK GPS: 센티미터 단위 정밀도 (1-3cm 오차)
- RTK GPS로 "말뚝처럼 고정" 가능!

**PX4 RTK GPS 설정:**

```bash
# PX4 파라미터
EKF2_AID_MASK = 24        # GPS + RTK 활성화
GPS_1_CONFIG = GPS
GPS_1_GNSS = 71           # GPS + GLONASS + Galileo
GPS_1_PROTOCOL = 4        # u-blox
RTK_TYPE = 1              # RTK 활성화

# 위치 제어 정밀도
MPC_XY_P = 0.95           # 위치 제어 게인 (높을수록 정밀)
MPC_Z_P = 1.0             # 고도 제어 게인
MPC_XY_VEL_P = 0.1        # 속도 제어 게인
MPC_Z_VEL_P = 0.1
```

**ROS2 정밀 호버링 노드:**
```python
class PrecisionHoveringNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'precision_hovering_{drone_id}')
        self.drone_id = drone_id
        
        # RTK GPS 위치 구독
        self.rtk_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.rtk_callback,
            10
        )
        
        # RTK 상태 구독
        self.rtk_status_sub = self.create_subscription(
            RTKStatus,
            f'/drone_{drone_id}/rtk_status',
            self.rtk_status_callback,
            10
        )
        
        # 목표 위치 설정
        self.target_position = None
        
        # 위치 제어 명령 발행
        self.position_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/trajectory_setpoint',
            10
        )
        
        # 정밀 호버링 제어 루프 (20Hz - 빠른 반응)
        self.create_timer(0.05, self.precision_hover_control)
        
        self.rtk_fix_available = False
        self.current_position = None
    
    def rtk_callback(self, msg):
        """
        RTK GPS 위치 업데이트
        """
        self.current_position = msg
    
    def rtk_status_callback(self, msg):
        """
        RTK 상태 업데이트
        """
        self.rtk_fix_available = msg.rtk_fix_available
        
        if msg.rtk_fix_type == 2:  # RTK Fixed
            self.get_logger().info(f'RTK Fixed - Precision: {msg.rtk_accuracy:.3f}m')
        elif msg.rtk_fix_type == 1:  # RTK Float
            self.get_logger().warn('RTK Float - Reduced precision')
        else:
            self.get_logger().error('RTK Fix lost!')
    
    def set_target_position(self, lat, lon, alt):
        """
        목표 위치 설정 (호버링 위치)
        """
        self.target_position = {
            'latitude': lat,
            'longitude': lon,
            'altitude': alt
        }
    
    def precision_hover_control(self):
        """
        정밀 호버링 제어 (RTK GPS 기반)
        """
        if not self.rtk_fix_available:
            self.get_logger().warn('RTK Fix not available - Cannot hover precisely')
            return
        
        if self.target_position is None or self.current_position is None:
            return
        
        # RTK GPS 위치 오차 계산
        position_error = self.calculate_position_error()
        
        # 위치 보정 명령 계산
        correction = self.compute_position_correction(position_error)
        
        # 위치 제어 명령 발행
        setpoint = TrajectorySetpoint()
        setpoint.position = [
            self.target_position['latitude'] + correction['lat'],
            self.target_position['longitude'] + correction['lon'],
            self.target_position['altitude'] + correction['alt']
        ]
        setpoint.velocity = [0.0, 0.0, 0.0]  # 속도 0 (호버링)
        setpoint.acceleration = [0.0, 0.0, 0.0]
        setpoint.yaw = 0.0  # 요각 고정
        
        self.position_pub.publish(setpoint)
    
    def calculate_position_error(self):
        """
        RTK GPS 기반 위치 오차 계산 (센티미터 단위)
        """
        error = {
            'lat': self.current_position.latitude - self.target_position['latitude'],
            'lon': self.current_position.longitude - self.target_position['longitude'],
            'alt': self.current_position.altitude - self.target_position['altitude']
        }
        
        # 미터 단위로 변환
        error['lat_m'] = error['lat'] * 111320.0
        error['lon_m'] = error['lon'] * 111320.0 * np.cos(np.radians(self.current_position.latitude))
        error['alt_m'] = error['alt']
        
        return error
    
    def compute_position_correction(self, error):
        """
        위치 보정 계산 (PID 제어)
        """
        # PID 게인
        Kp = 2.0  # 비례 게인
        Ki = 0.1  # 적분 게인
        Kd = 0.5  # 미분 게인
        
        # 적분 누적 (오차 누적)
        if not hasattr(self, 'error_integral'):
            self.error_integral = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        
        self.error_integral['lat'] += error['lat_m']
        self.error_integral['lon'] += error['lon_m']
        self.error_integral['alt'] += error['alt_m']
        
        # 미분 (오차 변화율)
        if not hasattr(self, 'prev_error'):
            self.prev_error = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        
        error_derivative = {
            'lat': error['lat_m'] - self.prev_error['lat'],
            'lon': error['lon_m'] - self.prev_error['lon'],
            'alt': error['alt_m'] - self.prev_error['alt']
        }
        
        self.prev_error = error.copy()
        
        # PID 제어 출력
        correction = {
            'lat': -(Kp * error['lat_m'] + Ki * self.error_integral['lat'] + Kd * error_derivative['lat']),
            'lon': -(Kp * error['lon_m'] + Ki * self.error_integral['lon'] + Kd * error_derivative['lon']),
            'alt': -(Kp * error['alt_m'] + Ki * self.error_integral['alt'] + Kd * error_derivative['alt'])
        }
        
        # 보정값을 위도/경도로 변환
        correction['lat'] /= 111320.0
        correction['lon'] /= (111320.0 * np.cos(np.radians(self.current_position.latitude)))
        
        return correction
```

**호버링 모드 활성화:**
```python
class HoveringModeNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'hovering_mode_{drone_id}')
        self.drone_id = drone_id
        
        # 현재 위치에서 호버링 시작
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.start_hovering,
            10
        )
        
        self.hovering_node = PrecisionHoveringNode(drone_id)
    
    def start_hovering(self, msg):
        """
        현재 위치에서 호버링 시작
        """
        self.hovering_node.set_target_position(
            msg.latitude,
            msg.longitude,
            msg.altitude
        )
```

### 6.2 바람 보상 (Wind Compensation)

**바람 영향 보상:**
- RTK GPS로 위치 오차 감지
- 바람으로 인한 드리프트 보정
- 지속적인 위치 유지

**ROS2 노드:**
```python
class WindCompensationNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'wind_compensation_{drone_id}')
        self.drone_id = drone_id
        
        # RTK GPS 위치 추적
        self.position_history = []
        
        # 바람 속도 추정
        self.wind_velocity = {'x': 0.0, 'y': 0.0}
        
        # 위치 제어 루프
        self.create_timer(0.1, self.wind_compensation)
    
    def wind_compensation(self):
        """
        바람 보상 계산
        """
        if len(self.position_history) < 10:
            return
        
        # 위치 드리프트 계산
        drift = self.calculate_drift()
        
        # 바람 속도 추정
        self.estimate_wind_velocity(drift)
        
        # 바람 보상 명령 발행
        compensation = self.compute_wind_compensation()
        self.publish_compensation(compensation)
    
    def calculate_drift(self):
        """
        위치 드리프트 계산 (RTK GPS 기반)
        """
        # 최근 위치 이력 분석
        recent_positions = self.position_history[-10:]
        
        # 드리프트 방향 및 속도 계산
        drift_x = recent_positions[-1].x - recent_positions[0].x
        drift_y = recent_positions[-1].y - recent_positions[0].y
        
        return {'x': drift_x, 'y': drift_y}
    
    def estimate_wind_velocity(self, drift):
        """
        바람 속도 추정
        """
        # 드리프트가 바람의 영향으로 가정
        # 실제로는 더 복잡한 필터링 필요
        self.wind_velocity['x'] = drift['x'] * 0.5
        self.wind_velocity['y'] = drift['y'] * 0.5
    
    def compute_wind_compensation(self):
        """
        바람 보상 계산
        """
        # 바람 속도와 반대 방향으로 보정
        compensation = {
            'x': -self.wind_velocity['x'],
            'y': -self.wind_velocity['y']
        }
        
        return compensation
```

---

## 7. 자동 복귀 시스템

### 7.1 복귀 경로 계획

**필요한 정보:**
- 홈 위치 (GPS 좌표)
- 현재 드론 위치 (RTK GPS)
- 배터리 잔량
- 착륙 가능 여부

**ROS2 서비스:**
```python
firefighting_msgs/ReturnHome:
    uint8 drone_id
    ---
    bool success
    string message
```

### 7.2 착륙 시퀀스

**단계:**
1. 홈 위치로 이동 (RTK GPS 정밀 제어)
2. 착륙 지점 접근
3. 수직 하강 (RTK GPS 고도 제어)
4. 착륙 완료

**ROS2 노드:**
```python
class ReturnHomeNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'return_home_{drone_id}')
        self.drone_id = drone_id
        self.home_position = self.get_home_position()
        
        # 정밀 호버링 노드 사용
        self.hovering_node = PrecisionHoveringNode(drone_id)
        
    def return_home(self):
        """
        자동 복귀 (RTK GPS 정밀 제어)
        """
        # 현재 위치 확인 (RTK GPS)
        current_pos = self.get_current_position_rtk()
        
        # 홈으로 경로 계획
        path = self.plan_path(current_pos, self.home_position)
        
        # 경로 따라 이동 (RTK GPS 정밀 제어)
        for waypoint in path:
            # 정밀 위치 제어로 이동
            self.hovering_node.set_target_position(
                waypoint.latitude,
                waypoint.longitude,
                waypoint.altitude
            )
            self.wait_for_arrival_rtk()  # RTK GPS로 도착 확인
        
        # 착륙 (RTK GPS 고도 제어)
        self.precision_land()
    
    def precision_land(self):
        """
        RTK GPS 기반 정밀 착륙
        """
        # 수직 하강 (RTK GPS 고도 제어)
        current_alt = self.get_current_altitude_rtk()
        target_alt = self.home_position['altitude']
        
        while current_alt > target_alt + 0.1:  # 10cm 정밀도
            current_alt -= 0.5  # 0.5m씩 하강
            self.hovering_node.set_target_position(
                self.home_position['latitude'],
                self.home_position['longitude'],
                current_alt
            )
            time.sleep(1.0)
        
        # 착륙 명령
        self.land()
```

---

## 7. 안전 시스템

### 7.1 안전 장치

**필수 안전 기능:**

1. **배터리 모니터링:**
   - 배터리 잔량 확인
   - 자동 복귀 트리거 (20% 이하)

2. **GPS 신호 확인:**
   - GPS 품질 모니터링
   - GPS 손실 시 안전 착륙

3. **통신 연결 확인:**
   - 지상국 연결 모니터링
   - 연결 끊김 시 자동 복귀

4. **장애물 감지:**
   - LiDAR 또는 초음파 센서
   - 비상 정지

5. **비상 착륙:**
   - 비정상 상황 감지 시 즉시 착륙

**ROS2 노드:**
```python
class SafetyMonitorNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'safety_monitor_{drone_id}')
        self.drone_id = drone_id
        
        # 배터리 모니터링
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            f'/drone_{drone_id}/battery_status',
            self.battery_callback,
            10
        )
        
        # GPS 모니터링
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/gps',
            self.gps_callback,
            10
        )
    
    def battery_callback(self, msg):
        if msg.percentage < 0.2:  # 20% 이하
            self.trigger_return_home()
    
    def gps_callback(self, msg):
        if msg.status.status < 2:  # GPS 품질 낮음
            self.trigger_safe_landing()
```

---

## 8. 중앙 제어 시스템

### 8.1 중앙 제어 노드

**기능:**
- 모든 드론 상태 모니터링
- 화재 감지 정보 수집
- 군집 제어 명령 발행
- 순차 격발 제어

**ROS2 노드 구조:**
```python
class CentralControlNode(Node):
    def __init__(self):
        super().__init__('central_control')
        
        # 각 드론 상태 구독
        for drone_id in [1, 2, 3]:
            self.create_subscription(
                VehicleStatus,
                f'/drone_{drone_id}/vehicle_status',
                lambda msg, id=drone_id: self.update_status(id, msg),
                10
            )
        
        # 화재 감지 구독
        self.fire_sub = self.create_subscription(
            FireAlert,
            '/fire_detection/fire_alert',
            self.fire_alert_callback,
            10
        )
        
        # 제어 명령 발행
        self.command_pub = {}
        for drone_id in [1, 2, 3]:
            self.command_pub[drone_id] = self.create_publisher(
                TrajectorySetpoint,
                f'/drone_{drone_id}/trajectory_setpoint',
                10
            )
    
    def fire_alert_callback(self, msg):
        """
        화재 알림 수신 시 처리
        """
        # 1. 경로 계획
        paths = self.plan_swarm_paths(msg.location)
        
        # 2. 각 드론에 명령 발행
        for drone_id, path in enumerate(paths, 1):
            self.send_path_command(drone_id, path)
        
        # 3. 도착 대기
        self.wait_for_arrival()
        
        # 4. 타겟팅 및 격발
        self.execute_firefighting_sequence()
    
    def execute_firefighting_sequence(self):
        """
        소화탄 순차 격발 시퀀스
        """
        # 타겟 좌표 수집
        targets = self.collect_fire_targets()
        
        # 순차 격발
        for i, target in enumerate(targets):
            drone_id = (i % 3) + 1
            self.launch_extinguisher(drone_id, target)
            time.sleep(2.0)  # 안전 간격
```

---

## 9. 필요한 ROS2 패키지 목록

### 9.1 핵심 패키지

```bash
# 기본 통신
ros-humble-px4-msgs              # PX4 메시지
ros-humble-mavros                # MAVLink 통신
ros-humble-micro-ros-agent       # XRCE-DDS 브리지

# 이미지 처리
ros-humble-cv-bridge             # OpenCV 브리지
ros-humble-image-transport       # 이미지 전송
ros-humble-image-geometry        # 이미지 기하학

# 경로 계획
ros-humble-nav2                  # 네비게이션 스택
ros-humble-planner               # 경로 계획
ros-humble-costmap-2d           # 비용 맵

# 제어
ros-humble-control-msgs         # 제어 메시지
ros-humble-trajectory-msgs      # 궤적 메시지
```

### 9.2 커스텀 패키지 (개발 필요)

```bash
firefighting_msgs/               # 화재 진압 메시지 정의
firefighting_control/            # 중앙 제어 노드
firefighting_targeting/          # 타겟팅 노드
firefighting_precise_targeting/  # LiDAR 기반 정밀 타겟팅
firefighting_formation/          # 편대 비행 제어
firefighting_rtk/                # RTK GPS 통합
firefighting_collision/           # RTK GPS 기반 충돌 방지
firefighting_safety/             # 안전 모니터링
thermal_camera_driver/           # 열화상 카메라 드라이버
lidar_driver/                    # LiDAR 드라이버
fire_extinguisher_control/       # 소화탄 제어 (LiDAR 거리 기반)
fire_extinguisher_aiming/        # 발사 각도 계산
rtk_gps_msgs/                    # RTK GPS 메시지 정의
rtk_gps_driver/                  # RTK GPS 드라이버
```

---

## 10. 하드웨어 구성

### 10.1 드론 하드웨어

**각 드론 구성:**

1. **Flight Controller:**
   - Pixhawk 6X (Standard v2A PM02D)
   - PX4 펌웨어 v1.16.0

2. **SBC:**
   - Khadas VIM4
   - Ubuntu 22.04 ARM64

3. **RTK GPS 모듈 (필수):**
   - u-blox ZED-F9P (권장)
   - 또는 Here+ RTK GPS
   - 또는 Here3 RTK GPS
   - Pixhawk와 직접 연결
   - 센티미터 단위 정밀도

4. **열화상 카메라:**
   - FLIR Lepton 또는 Seek Thermal
   - ROS2 드라이버 필요

5. **소화탄 발사 장치:**
   - 서보 모터 또는 솔레노이드
   - Pixhawk AUX 출력 제어

6. **센서:**
   - RTK GPS (편대 비행 및 충돌 방지용)
   - IMU (Pixhawk 내장)
   - 배터리 모니터링
   - LiDAR 또는 초음파 (선택, 추가 안전)

### 10.2 지상 시스템

1. **중앙 제어 PC:**
   - ROS2 Humble 설치
   - 중앙 제어 노드 실행
   - QGroundControl 실행

2. **RTK GPS 기지국 (Base Station):**
   - 고정 위치에 설치
   - RTCM 보정 데이터 생성 및 전송
   - 무선 링크 (LoRa, WiFi, 4G)
   - 모든 드론에 RTCM 데이터 전송

3. **화재 감지 시스템:**
   - 지상 센서 네트워크
   - 또는 드론 기반 탐지

---

## 11. 소프트웨어 아키텍처

### 11.1 시스템 구조

```
┌─────────────────────────────────────────┐
│         중앙 제어 시스템                 │
│  (Central Control Node)                 │
│  - 화재 감지 수신                        │
│  - 군집 제어 명령 발행                    │
│  - 순차 격발 제어                        │
└──────────────┬──────────────────────────┘
               │ ROS2 Topics
       ┌───────┴───────┐
       │               │
┌──────▼──────┐  ┌─────▼──────┐  ┌──────────┐
│  Drone 1    │  │  Drone 2   │  │ Drone 3  │
│             │  │            │  │          │
│ - 경로 계획  │  │ - 경로 계획 │  │ - 경로 계획│
│ - 타겟팅    │  │ - 타겟팅    │  │ - 타겟팅 │
│ - 격발 제어  │  │ - 격발 제어 │  │ - 격발 제어│
│ - 안전 모니터│  │ - 안전 모니터│  │ - 안전 모니터│
└─────────────┘  └────────────┘  └──────────┘
```

### 11.2 ROS2 노드 구조

**각 드론별 노드:**

```
/drone_X/
├── path_planner              # 경로 계획
├── collision_avoidance       # 충돌 회피
├── formation_control         # 군집 형성
├── thermal_camera_driver     # 열화상 카메라
├── fire_targeting            # 화재 타겟팅
├── fire_extinguisher         # 소화탄 제어
├── return_home               # 자동 복귀
└── safety_monitor            # 안전 모니터링
```

---

## 12. 개발 단계

### Phase 1: 기본 인프라 (완료)
- ✅ ROS2 통신 구축
- ✅ PX4 연결
- ✅ 다중 기체 통신

### Phase 2: 화재 감지 시스템
- [ ] 화재 감지 센서 통합
- [ ] 화재 위치 파악 알고리즘
- [ ] ROS2 메시지 정의

### Phase 3: 열화상 카메라 통합
- [ ] 열화상 카메라 하드웨어 선택 및 구매
- [ ] ROS2 드라이버 개발
- [ ] 이미지 처리 파이프라인 구축

### Phase 4: 타겟팅 시스템
- [ ] 화재 핫스팟 탐지 알고리즘
- [ ] 좌표 변환 (이미지 → GPS)
- [ ] LiDAR 하드웨어 설치 및 통합
- [ ] LiDAR 드라이버 개발
- [ ] 열화상 카메라 + LiDAR 통합 타겟팅
- [ ] 타겟 거리 측정 및 발사 각도 계산

### Phase 5: 소화탄 격발 시스템
- [ ] 하드웨어 설계 및 제작
- [ ] Pixhawk AUX 출력 제어
- [ ] 안전 시스템 구현

### Phase 6: 편대 비행 및 RTK GPS 통합
- [ ] RTK GPS 하드웨어 설치 및 설정
- [ ] RTK GPS 드라이버 개발
- [ ] 편대 비행 알고리즘 구현
- [ ] RTK GPS 기반 충돌 방지 알고리즘
- [ ] RTK GPS 기반 정밀 호버링 구현
- [ ] 바람 보상 알고리즘
- [ ] 편대 비행 + 충돌 방지 + 호버링 통합 제어

### Phase 7: 자동 복귀
- [ ] 복귀 경로 계획
- [ ] 착륙 시퀀스 구현
- [ ] 안전 착륙 보장

### Phase 8: 통합 테스트
- [ ] 전체 시스템 통합
- [ ] 실전 테스트
- [ ] 성능 최적화

---

## 13. 보안 및 안전 고려사항

### 13.1 안전 시스템

- **비상 정지:** 언제든지 모든 드론 정지 가능
- **자동 복귀:** 배터리 부족 시 자동 복귀
- **GPS 손실 대응:** GPS 손실 시 안전 착륙
- **통신 끊김 대응:** 통신 끊김 시 자동 복귀

### 13.2 법적 고려사항

- **드론 비행 허가:** 관련 기관 허가 필요
- **화재 진압 허가:** 소방서 협의 필요
- **공역 사용 허가:** 항공 당국 허가 필요

---

## 14. 예상 비용

### 하드웨어
- 드론 플랫폼: 3대 × 약 $500 = $1,500
- RTK GPS 모듈: 3대 × 약 $200 = $600 (u-blox ZED-F9P)
- RTK GPS 기지국: 1대 × 약 $300 = $300
- 열화상 카메라: 3대 × 약 $300 = $900
- 전방 LiDAR: 3대 × 약 $150 = $450 (RPLIDAR A2)
- 소화탄 발사 장치: 3대 × 약 $100 = $300
- 기타 센서 및 부품: 약 $500
- **총 하드웨어:** 약 $4,550

### 소프트웨어
- 오픈소스 기반 (무료)
- 개발 시간: 약 6-12개월

---

## 15. 참고 자료

- [PX4 개발 문서](https://dev.px4.io/)
- [ROS2 네비게이션](https://navigation.ros.org/)
- [열화상 카메라 통합](https://github.com/ros-drivers/flir_camera_driver)
- [군집 로봇 제어](https://github.com/ethz-asl/mav_control_rw)
- [RPLIDAR ROS2 드라이버](https://github.com/robopeak/rplidar_ros2)
- [RTK GPS 통합](https://docs.px4.io/main/en/gps_compass/rtk_gps.html)

---

## 요약

**필요한 핵심 기술:**

1. ✅ **기본 인프라** (완료)
   - ROS2 통신
   - PX4 연결
   - 다중 기체 통신

2. 🔨 **RTK GPS 시스템** (우선순위 높음)
   - RTK GPS 하드웨어 설치
   - RTK 기지국 구축
   - RTK GPS 드라이버 개발
   - 센티미터 단위 정밀 위치 추적
   - **정밀 호버링** ("말뚝처럼 고정")
   - 바람 보상 시스템

3. 🔨 **편대 비행**
   - 편대 형상 정의 (삼각형, V자, 일렬 등)
   - Leader-Follower 알고리즘
   - RTK GPS 기반 정밀 형성 유지

4. 🔨 **RTK GPS 기반 충돌 방지**
   - 정밀 거리 측정 (센티미터 단위)
   - 실시간 충돌 감지
   - 자동 회피 알고리즘

5. 🔨 **화재 감지**
   - 지상 센서 또는 드론 탐지
   - 위치 파악 알고리즘

6. 🔨 **열화상 카메라**
   - 하드웨어 통합
   - 이미지 처리
   - 타겟팅 알고리즘

7. 🔨 **전방 LiDAR 통합** (타겟 거리 측정)
   - LiDAR 하드웨어 설치
   - ROS2 드라이버 개발
   - 타겟 방향 거리 측정
   - 열화상 카메라 + LiDAR 통합

8. 🔨 **소화탄 격발**
   - 하드웨어 제작
   - LiDAR 거리 기반 발사 각도 계산
   - 제어 시스템
   - 안전 시스템

8. 🔨 **경로 계획**
   - A* 또는 RRT* 알고리즘
   - 장애물 회피

9. 🔨 **자동 복귀**
   - 복귀 경로 계획
   - 착륙 시퀀스

10. 🔨 **중앙 제어**
    - 통합 제어 시스템
    - 모니터링
    - 순차 격발 제어

**예상 개발 기간:** 8-14개월 (RTK GPS 통합 포함)

