# RTK GPS 기반 편대 비행 및 충돌 방지 가이드

## 개요

RTK GPS를 사용하여 센티미터 단위 정밀도로 편대 비행을 수행하고, 기체 간 충돌을 방지하는 시스템입니다.

**핵심 기능:**
- ✅ 센티미터 단위 정밀 위치 추적
- ✅ 정밀 편대 형성 유지
- ✅ 실시간 충돌 감지 및 회피
- ✅ 안전한 군집 비행

---

## RTK GPS 시스템 구성

### 하드웨어 구성

```
                    RTK 기지국 (Base Station)
                    고정 위치 설치
                           │
                           │ RTCM 보정 데이터
                           │ (무선 전송)
        ┌──────────────────┼──────────────────┐
        │                  │                  │
     Drone 1            Drone 2            Drone 3
    (Rover 1)          (Rover 2)          (Rover 3)
        │                  │                  │
        └──────────────────┴──────────────────┘
                  RTK GPS 정밀 위치 추적
```

### 필요한 하드웨어

**RTK GPS 모듈 (각 드론):**
- u-blox ZED-F9P (권장)
- Here+ RTK GPS
- Here3 RTK GPS
- Pixhawk와 직접 연결 가능

**RTK GPS 기지국:**
- 고정 위치에 설치
- RTCM 보정 데이터 생성
- 무선 전송 (LoRa, WiFi, 4G)

---

## RTK GPS 설정

### PX4 파라미터 설정

**각 드론의 PX4에서:**

```bash
# RTK GPS 활성화
EKF2_AID_MASK = 24        # GPS + RTK 활성화
GPS_1_CONFIG = GPS
GPS_1_GNSS = 71           # GPS + GLONASS + Galileo
GPS_1_PROTOCOL = 4        # u-blox
GPS_YAW_OFFSET = 0
RTK_TYPE = 1              # RTK 활성화

# RTK 정밀도 설정
EKF2_GPS_P_NOISE = 0.5    # GPS 위치 노이즈 (낮을수록 정밀)
```

### RTK 기지국 설정

**RTCM 데이터 전송:**

```python
# RTK 기지국 노드
class RTKBaseStationNode(Node):
    def __init__(self):
        super().__init__('rtk_base_station')
        
        # RTCM 데이터 읽기 (GPS 모듈에서)
        self.rtcm_data = self.read_rtcm_from_gps()
        
        # RTCM 데이터 발행
        self.rtcm_pub = self.create_publisher(
            RTCM,
            '/rtk_base/rtcm',
            10
        )
        
        # 주기적으로 RTCM 데이터 전송
        self.create_timer(1.0, self.publish_rtcm)
    
    def publish_rtcm(self):
        """
        RTCM 보정 데이터 전송
        """
        rtcm_msg = RTCM()
        rtcm_msg.data = self.rtcm_data
        self.rtcm_pub.publish(rtcm_msg)
```

---

## 편대 비행 구현

### 편대 형상 정의

**삼각형 편대:**
```python
formation_offsets = {
    1: (0.0, 0.0, 0.0),      # Leader (중앙)
    2: (-5.0, -5.0, 0.0),   # Follower 1 (왼쪽 뒤)
    3: (5.0, -5.0, 0.0)     # Follower 2 (오른쪽 뒤)
}
```

**V자 편대:**
```python
formation_offsets = {
    1: (0.0, 0.0, 0.0),      # Leader (앞)
    2: (-10.0, -5.0, 0.0),  # Follower 1 (왼쪽 뒤)
    3: (10.0, -5.0, 0.0)    # Follower 2 (오른쪽 뒤)
}
```

**일렬 편대:**
```python
formation_offsets = {
    1: (0.0, 0.0, 0.0),      # Leader
    2: (0.0, -10.0, 0.0),   # Follower 1 (뒤)
    3: (0.0, -20.0, 0.0)    # Follower 2 (더 뒤)
}
```

### 편대 비행 제어

**ROS2 노드:**
```python
class FormationFlightNode(Node):
    def __init__(self, drone_id, formation_type='triangle'):
        super().__init__(f'formation_flight_{drone_id}')
        self.drone_id = drone_id
        self.formation_type = formation_type
        
        # RTK GPS 위치 구독
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.update_self_rtk,
            10
        )
        
        # 리더 드론 RTK GPS 위치 구독
        if drone_id != 1:  # Follower만
            self.create_subscription(
                NavSatFix,
                '/drone_1/rtk_gps',
                self.update_leader_rtk,
                10
            )
        
        # 편대 위치 명령 발행
        self.formation_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/formation_setpoint',
            10
        )
        
        # 편대 제어 루프 (10Hz)
        self.create_timer(0.1, self.formation_control)
    
    def update_self_rtk(self, msg):
        self.self_rtk = msg
    
    def update_leader_rtk(self, msg):
        self.leader_rtk = msg
    
    def formation_control(self):
        """
        편대 비행 제어
        """
        if self.drone_id == 1:
            # Leader는 경로를 따라감
            return
        
        if not hasattr(self, 'leader_rtk'):
            return
        
        # 편대 오프셋 계산
        offset = self.get_formation_offset()
        
        # RTK GPS 좌표에 오프셋 적용
        target_lat, target_lon = self.apply_rtk_offset(
            self.leader_rtk.latitude,
            self.leader_rtk.longitude,
            offset[0],
            offset[1]
        )
        
        # 편대 위치 명령 발행
        setpoint = TrajectorySetpoint()
        setpoint.position = [
            target_lat,
            target_lon,
            self.leader_rtk.altitude + offset[2]
        ]
        
        self.formation_pub.publish(setpoint)
    
    def apply_rtk_offset(self, lat, lon, offset_x, offset_y):
        """
        RTK GPS 좌표에 미터 단위 오프셋 적용 (정밀)
        """
        from math import radians, cos
        
        # 미터를 위도/경도로 변환 (정밀)
        lat_offset = offset_y / 111320.0
        lon_offset = offset_x / (111320.0 * cos(radians(lat)))
        
        return lat + lat_offset, lon + lon_offset
```

---

## RTK GPS 기반 충돌 방지

### 충돌 감지 알고리즘

**정밀 거리 계산:**
```python
def calculate_rtk_distance(pos1, pos2):
    """
    두 RTK GPS 좌표 간 정밀 거리 계산 (센티미터 단위)
    """
    from math import radians, cos, sin, asin, sqrt
    
    R = 6371000  # 지구 반지름 (미터)
    
    lat1, lon1 = radians(pos1.latitude), radians(pos1.longitude)
    lat2, lon2 = radians(pos2.latitude), radians(pos2.longitude)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    
    distance = R * c  # 미터 단위
    
    return distance
```

### 충돌 회피 시스템

**통합 충돌 방지 노드:**
```python
class RTKCollisionAvoidanceNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_collision_avoidance_{drone_id}')
        self.drone_id = drone_id
        
        # 자신의 RTK GPS
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.update_self_rtk,
            10
        )
        
        # 다른 드론의 RTK GPS
        for other_id in [1, 2, 3]:
            if other_id != drone_id:
                self.create_subscription(
                    NavSatFix,
                    f'/drone_{other_id}/rtk_gps',
                    lambda msg, id=other_id: self.update_other_rtk(id, msg),
                    10
                )
        
        # 충돌 회피 명령 발행
        self.avoidance_pub = self.create_publisher(
            Twist,
            f'/drone_{drone_id}/collision_avoidance',
            10
        )
        
        # 충돌 감지 (20Hz - 빠른 반응)
        self.create_timer(0.05, self.check_collision)
        
        self.min_safe_distance = 3.0  # 최소 안전 거리 3m
        self.collision_risk = False
    
    def check_collision(self):
        """
        RTK GPS 기반 정밀 충돌 감지
        """
        if not hasattr(self, 'self_rtk'):
            return
        
        self.collision_risk = False
        
        for other_id, other_rtk in self.other_drones_rtk.items():
            # 수평 거리 계산
            h_dist = self.calculate_horizontal_distance(
                self.self_rtk.latitude,
                self.self_rtk.longitude,
                other_rtk.latitude,
                other_rtk.longitude
            )
            
            # 고도 차이
            v_dist = abs(self.self_rtk.altitude - other_rtk.altitude)
            
            # 충돌 위험 판단
            if v_dist < 2.0 and h_dist < self.min_safe_distance:
                self.collision_risk = True
                self.trigger_avoidance(other_id, other_rtk, h_dist, v_dist)
    
    def trigger_avoidance(self, other_id, other_rtk, h_dist, v_dist):
        """
        충돌 회피 실행
        """
        # 회피 벡터 계산
        avoidance = self.compute_avoidance_vector(other_rtk, h_dist, v_dist)
        
        # 회피 명령 발행
        avoidance_cmd = Twist()
        avoidance_cmd.linear.x = avoidance[0]
        avoidance_cmd.linear.y = avoidance[1]
        avoidance_cmd.linear.z = avoidance[2]
        
        self.avoidance_pub.publish(avoidance_cmd)
        
        self.get_logger().warn(
            f'⚠️ RTK Collision risk! Drone {other_id}, '
            f'H: {h_dist:.2f}m, V: {v_dist:.2f}m'
        )
```

---

## 편대 비행 + 충돌 방지 통합

### 통합 제어 노드

```python
class IntegratedFormationControlNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'integrated_formation_{drone_id}')
        self.drone_id = drone_id
        
        # 편대 비행 노드
        self.formation_node = FormationFlightNode(drone_id)
        
        # RTK 충돌 회피 노드
        self.collision_node = RTKCollisionAvoidanceNode(drone_id)
        
        # 통합 명령 발행
        self.integrated_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/integrated_setpoint',
            10
        )
        
        # 통합 제어 루프 (10Hz)
        self.create_timer(0.1, self.integrated_control)
    
    def integrated_control(self):
        """
        편대 비행 + 충돌 회피 통합 제어
        """
        # 편대 비행 목표 위치
        formation_setpoint = self.formation_node.get_setpoint()
        
        # 충돌 회피 보정
        avoidance_correction = self.collision_node.get_avoidance_correction()
        
        # 우선순위: 충돌 회피 > 편대 유지
        if self.collision_node.is_collision_risk():
            # 충돌 위험 시 편대보다 충돌 회피 우선
            integrated_setpoint = self.collision_node.get_emergency_setpoint()
        else:
            # 정상 시 편대 유지
            integrated_setpoint = TrajectorySetpoint()
            integrated_setpoint.position[0] = formation_setpoint.position[0] + avoidance_correction[0]
            integrated_setpoint.position[1] = formation_setpoint.position[1] + avoidance_correction[1]
            integrated_setpoint.position[2] = formation_setpoint.position[2] + avoidance_correction[2]
        
        self.integrated_pub.publish(integrated_setpoint)
```

---

## 테스트 방법

### RTK GPS 정밀도 확인

```bash
# RTK GPS 상태 확인
ros2 topic echo /drone_1/rtk_status

# RTK GPS 위치 확인
ros2 topic echo /drone_1/rtk_gps

# 정밀도 확인 (position_covariance 값이 작을수록 정밀)
```

### 편대 비행 테스트

```bash
# 편대 비행 노드 실행
ros2 run firefighting_formation formation_flight_node --ros-args -p drone_id:=1 -p formation_type:=triangle

# 편대 위치 확인
ros2 topic echo /drone_1/formation_setpoint
ros2 topic echo /drone_2/formation_setpoint
ros2 topic echo /drone_3/formation_setpoint
```

### 충돌 방지 테스트

```bash
# 충돌 방지 노드 실행
ros2 run firefighting_collision rtk_collision_avoidance_node --ros-args -p drone_id:=1

# 충돌 감지 확인
ros2 topic echo /drone_1/collision_avoidance
```

---

## 요약

**RTK GPS의 장점:**
- ✅ 센티미터 단위 정밀도
- ✅ 정밀 편대 비행 가능
- ✅ 정확한 충돌 감지
- ✅ 안전한 군집 비행

**편대 비행:**
- 삼각형, V자, 일렬 등 다양한 형상
- Leader-Follower 방식
- RTK GPS 기반 정밀 형성 유지

**충돌 방지:**
- RTK GPS 기반 정밀 거리 측정
- 실시간 충돌 감지 (20Hz)
- 자동 회피 알고리즘
- 충돌 회피 우선순위

