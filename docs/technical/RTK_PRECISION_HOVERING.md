# RTK GPS 기반 정밀 호버링 가이드

## 개요

RTK GPS를 사용하여 **"말뚝처럼 고정"**된 정밀 호버링을 구현합니다. 센티미터 단위 정밀도로 위치를 유지하여 정확한 타겟팅과 소화탄 발사가 가능합니다.

**핵심 기능:**
- ✅ 센티미터 단위 정밀 위치 유지
- ✅ 바람 보상
- ✅ 안정적인 호버링
- ✅ 정밀 타겟팅 및 발사 가능

---

## RTK GPS를 사용한 정밀 호버링

### 왜 RTK GPS가 필요한가?

**일반 GPS:**
- 정밀도: 1-3미터 오차
- 호버링: 위치가 계속 변함
- 타겟팅: 부정확

**RTK GPS:**
- 정밀도: 1-3센티미터 오차
- 호버링: 위치 고정 가능
- 타겟팅: 매우 정확

**결론:** RTK GPS 없이는 "말뚝처럼 고정"된 호버링이 불가능합니다!

---

## PX4 RTK GPS 설정

### 필수 파라미터

```bash
# RTK GPS 활성화
EKF2_AID_MASK = 24        # GPS + RTK 활성화
GPS_1_CONFIG = GPS
GPS_1_GNSS = 71           # GPS + GLONASS + Galileo
GPS_1_PROTOCOL = 4        # u-blox
RTK_TYPE = 1              # RTK 활성화

# 위치 제어 정밀도 (호버링용)
MPC_XY_P = 0.95           # 위치 제어 게인 (높을수록 정밀)
MPC_Z_P = 1.0             # 고도 제어 게인
MPC_XY_VEL_P = 0.1        # 속도 제어 게인
MPC_Z_VEL_P = 0.1

# 호버링 모드 설정
MPC_XY_VEL_MAX = 2.0      # 최대 수평 속도 (낮을수록 안정적)
MPC_Z_VEL_MAX_DN = 1.0    # 최대 하강 속도
MPC_Z_VEL_MAX_UP = 3.0    # 최대 상승 속도
```

---

## ROS2 정밀 호버링 구현

### 정밀 호버링 노드

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import Float32
import numpy as np

class RTKPrecisionHoveringNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'rtk_precision_hovering_{drone_id}')
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
        
        # 목표 위치 (호버링 위치)
        self.target_position = None
        
        # 위치 제어 명령 발행
        self.position_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/trajectory_setpoint',
            10
        )
        
        # 위치 오차 발행 (모니터링용)
        self.error_pub = self.create_publisher(
            Float32,
            f'/drone_{drone_id}/position_error',
            10
        )
        
        # 정밀 호버링 제어 루프 (20Hz)
        self.create_timer(0.05, self.precision_hover_control)
        
        # 상태 변수
        self.rtk_fix_available = False
        self.rtk_fix_type = 0  # 0=No RTK, 1=Float, 2=Fixed
        self.current_position = None
        
        # PID 제어 변수
        self.error_integral = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.prev_error = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        
        # PID 게인
        self.Kp = 2.0  # 비례 게인
        self.Ki = 0.1  # 적분 게인
        self.Kd = 0.5  # 미분 게인
    
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
        self.rtk_fix_type = msg.rtk_fix_type
        
        if msg.rtk_fix_type == 2:  # RTK Fixed
            self.get_logger().info(
                f'RTK Fixed - Precision: {msg.rtk_accuracy:.3f}m'
            )
        elif msg.rtk_fix_type == 1:  # RTK Float
            self.get_logger().warn(
                f'RTK Float - Reduced precision: {msg.rtk_accuracy:.3f}m'
            )
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
        self.get_logger().info(
            f'Target position set: ({lat:.7f}, {lon:.7f}, {alt:.2f})'
        )
        
        # 적분 초기화
        self.error_integral = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.prev_error = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
    
    def precision_hover_control(self):
        """
        정밀 호버링 제어 (RTK GPS 기반)
        """
        # RTK Fix 확인
        if not self.rtk_fix_available or self.rtk_fix_type < 1:
            self.get_logger().warn_throttle(
                5.0,
                'RTK Fix not available - Cannot hover precisely'
            )
            return
        
        if self.target_position is None or self.current_position is None:
            return
        
        # RTK GPS 위치 오차 계산 (센티미터 단위)
        position_error = self.calculate_position_error()
        
        # 위치 보정 계산 (PID 제어)
        correction = self.compute_pid_correction(position_error)
        
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
        setpoint.yawspeed = 0.0
        
        self.position_pub.publish(setpoint)
        
        # 위치 오차 모니터링
        total_error = np.sqrt(
            position_error['lat_m']**2 +
            position_error['lon_m']**2 +
            position_error['alt_m']**2
        )
        
        error_msg = Float32()
        error_msg.data = total_error
        self.error_pub.publish(error_msg)
    
    def calculate_position_error(self):
        """
        RTK GPS 기반 위치 오차 계산 (센티미터 단위)
        """
        from math import radians, cos
        
        error = {
            'lat': self.current_position.latitude - self.target_position['latitude'],
            'lon': self.current_position.longitude - self.target_position['longitude'],
            'alt': self.current_position.altitude - self.target_position['altitude']
        }
        
        # 위도/경도를 미터 단위로 변환
        error['lat_m'] = error['lat'] * 111320.0  # 1도 ≈ 111.32km
        error['lon_m'] = error['lon'] * 111320.0 * cos(
            radians(self.current_position.latitude)
        )
        error['alt_m'] = error['alt']
        
        return error
    
    def compute_pid_correction(self, error):
        """
        PID 제어 기반 위치 보정 계산
        """
        # 적분 누적 (오차 누적)
        self.error_integral['lat'] += error['lat_m']
        self.error_integral['lon'] += error['lon_m']
        self.error_integral['alt'] += error['alt_m']
        
        # 적분 제한 (윈드업 방지)
        max_integral = 10.0
        self.error_integral['lat'] = np.clip(
            self.error_integral['lat'], -max_integral, max_integral
        )
        self.error_integral['lon'] = np.clip(
            self.error_integral['lon'], -max_integral, max_integral
        )
        self.error_integral['alt'] = np.clip(
            self.error_integral['alt'], -max_integral, max_integral
        )
        
        # 미분 (오차 변화율)
        error_derivative = {
            'lat': error['lat_m'] - self.prev_error['lat'],
            'lon': error['lon_m'] - self.prev_error['lon'],
            'alt': error['alt_m'] - self.prev_error['alt']
        }
        
        self.prev_error = {
            'lat': error['lat_m'],
            'lon': error['lon_m'],
            'alt': error['alt_m']
        }
        
        # PID 제어 출력
        correction_m = {
            'lat': -(self.Kp * error['lat_m'] +
                    self.Ki * self.error_integral['lat'] +
                    self.Kd * error_derivative['lat']),
            'lon': -(self.Kp * error['lon_m'] +
                    self.Ki * self.error_integral['lon'] +
                    self.Kd * error_derivative['lon']),
            'alt': -(self.Kp * error['alt_m'] +
                    self.Ki * self.error_integral['alt'] +
                    self.Kd * error_derivative['alt'])
        }
        
        # 보정값을 위도/경도로 변환
        from math import radians, cos
        
        correction = {
            'lat': correction_m['lat'] / 111320.0,
            'lon': correction_m['lon'] / (
                111320.0 * cos(radians(self.current_position.latitude))
            ),
            'alt': correction_m['alt']
        }
        
        return correction
```

---

## 호버링 모드 사용

### 현재 위치에서 호버링 시작

```python
class HoveringModeNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'hovering_mode_{drone_id}')
        self.drone_id = drone_id
        
        # 정밀 호버링 노드
        self.hovering_node = RTKPrecisionHoveringNode(drone_id)
        
        # 현재 위치 구독
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.start_hovering_at_current_position,
            10
        )
        
        # 호버링 시작 서비스
        self.hover_service = self.create_service(
            HoveringCommand,
            f'/drone_{drone_id}/start_hovering',
            self.start_hovering_callback
        )
    
    def start_hovering_at_current_position(self, msg):
        """
        현재 위치에서 자동 호버링 시작
        """
        if self.hovering_node.target_position is None:
            self.hovering_node.set_target_position(
                msg.latitude,
                msg.longitude,
                msg.altitude
            )
            self.get_logger().info('Started hovering at current position')
    
    def start_hovering_callback(self, request, response):
        """
        지정된 위치에서 호버링 시작
        """
        self.hovering_node.set_target_position(
            request.latitude,
            request.longitude,
            request.altitude
        )
        
        response.success = True
        response.message = "Hovering started"
        return response
```

---

## 바람 보상 (Wind Compensation)

### 바람 영향 보상

RTK GPS로 위치 드리프트를 감지하고 바람을 보상합니다.

```python
class WindCompensationNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'wind_compensation_{drone_id}')
        self.drone_id = drone_id
        
        # RTK GPS 위치 추적
        self.position_history = []
        self.max_history = 20  # 최근 20개 위치 저장
        
        # 바람 속도 추정
        self.wind_velocity = {'x': 0.0, 'y': 0.0}
        
        # RTK GPS 위치 구독
        self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/rtk_gps',
            self.update_position_history,
            10
        )
        
        # 바람 보상 계산 (10Hz)
        self.create_timer(0.1, self.wind_compensation)
        
        # 바람 보상 발행
        self.wind_compensation_pub = self.create_publisher(
            Twist,
            f'/drone_{drone_id}/wind_compensation',
            10
        )
    
    def update_position_history(self, msg):
        """
        위치 이력 업데이트
        """
        # 미터 단위 위치로 변환
        pos_m = {
            'x': msg.longitude * 111320.0 * np.cos(np.radians(msg.latitude)),
            'y': msg.latitude * 111320.0,
            'z': msg.altitude,
            'time': self.get_clock().now()
        }
        
        self.position_history.append(pos_m)
        
        # 이력 크기 제한
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
    
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
        
        comp_msg = Twist()
        comp_msg.linear.x = compensation['x']
        comp_msg.linear.y = compensation['y']
        comp_msg.linear.z = 0.0
        
        self.wind_compensation_pub.publish(comp_msg)
    
    def calculate_drift(self):
        """
        위치 드리프트 계산 (RTK GPS 기반)
        """
        if len(self.position_history) < 10:
            return {'x': 0.0, 'y': 0.0}
        
        # 최근 위치와 이전 위치 비교
        recent = self.position_history[-1]
        old = self.position_history[-10]
        
        # 시간 차이
        time_diff = (recent['time'] - old['time']).nanoseconds / 1e9
        
        if time_diff == 0:
            return {'x': 0.0, 'y': 0.0}
        
        # 드리프트 속도
        drift_x = (recent['x'] - old['x']) / time_diff
        drift_y = (recent['y'] - old['y']) / time_diff
        
        return {'x': drift_x, 'y': drift_y}
    
    def estimate_wind_velocity(self, drift):
        """
        바람 속도 추정
        """
        # 드리프트가 바람의 영향으로 가정
        # 실제로는 칼만 필터 등 사용 가능
        alpha = 0.3  # 필터 계수
        
        self.wind_velocity['x'] = alpha * drift['x'] + (1 - alpha) * self.wind_velocity['x']
        self.wind_velocity['y'] = alpha * drift['y'] + (1 - alpha) * self.wind_velocity['y']
    
    def compute_wind_compensation(self):
        """
        바람 보상 계산
        """
        # 바람 속도와 반대 방향으로 보정
        compensation = {
            'x': -self.wind_velocity['x'] * 0.5,  # 보정 게인
            'y': -self.wind_velocity['y'] * 0.5
        }
        
        return compensation
```

---

## 통합 호버링 시스템

### 정밀 호버링 + 바람 보상 통합

```python
class IntegratedHoveringNode(Node):
    def __init__(self, drone_id):
        super().__init__(f'integrated_hovering_{drone_id}')
        self.drone_id = drone_id
        
        # 정밀 호버링 노드
        self.precision_hovering = RTKPrecisionHoveringNode(drone_id)
        
        # 바람 보상 노드
        self.wind_compensation = WindCompensationNode(drone_id)
        
        # 통합 위치 제어 명령 발행
        self.integrated_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/integrated_setpoint',
            10
        )
        
        # 바람 보상 구독
        self.create_subscription(
            Twist,
            f'/drone_{drone_id}/wind_compensation',
            self.wind_compensation_callback,
            10
        )
        
        # 통합 제어 루프
        self.create_timer(0.05, self.integrated_control)
        
        self.wind_compensation_cmd = {'x': 0.0, 'y': 0.0}
    
    def wind_compensation_callback(self, msg):
        self.wind_compensation_cmd = {
            'x': msg.linear.x,
            'y': msg.linear.y
        }
    
    def integrated_control(self):
        """
        정밀 호버링 + 바람 보상 통합 제어
        """
        # 정밀 호버링 목표 위치
        target_pos = self.precision_hovering.target_position
        
        if target_pos is None:
            return
        
        # 바람 보상 적용
        from math import radians, cos
        
        wind_lat_offset = self.wind_compensation_cmd['y'] / 111320.0
        wind_lon_offset = self.wind_compensation_cmd['x'] / (
            111320.0 * cos(radians(target_pos['latitude']))
        )
        
        # 통합 위치 명령
        setpoint = TrajectorySetpoint()
        setpoint.position = [
            target_pos['latitude'] + wind_lat_offset,
            target_pos['longitude'] + wind_lon_offset,
            target_pos['altitude']
        ]
        setpoint.velocity = [0.0, 0.0, 0.0]
        setpoint.acceleration = [0.0, 0.0, 0.0]
        
        self.integrated_pub.publish(setpoint)
```

---

## 테스트 방법

### RTK GPS 정밀도 확인

```bash
# RTK GPS 상태 확인
ros2 topic echo /drone_1/rtk_status

# RTK GPS 위치 확인
ros2 topic echo /drone_1/rtk_gps

# 위치 오차 확인
ros2 topic echo /drone_1/position_error
```

### 호버링 테스트

```bash
# 정밀 호버링 노드 실행
ros2 run firefighting_hovering rtk_precision_hovering_node --ros-args -p drone_id:=1

# 현재 위치에서 호버링 시작
ros2 service call /drone_1/start_hovering firefighting_msgs/HoveringCommand "{latitude: 37.1234567, longitude: 127.1234567, altitude: 10.0}"

# 위치 오차 모니터링 (센티미터 단위로 작아야 함)
ros2 topic echo /drone_1/position_error
```

---

## 성능 목표

### 호버링 정밀도

**RTK Fixed 모드:**
- 위치 오차: < 5cm
- 고도 오차: < 10cm
- 위치 유지: 안정적

**RTK Float 모드:**
- 위치 오차: < 20cm
- 고도 오차: < 30cm
- 위치 유지: 양호

**일반 GPS 모드:**
- 위치 오차: 1-3m
- 위치 유지: 불안정 (호버링 불가)

---

## 요약

**RTK GPS를 사용한 정밀 호버링:**

- ✅ **가능합니다!** RTK GPS는 센티미터 단위 정밀도 제공
- ✅ "말뚝처럼 고정"된 호버링 구현 가능
- ✅ 바람 보상으로 안정적인 위치 유지
- ✅ 정밀 타겟팅 및 소화탄 발사에 필수

**핵심 기술:**
- RTK GPS 정밀 위치 추적
- PID 제어 기반 위치 보정
- 바람 보상 알고리즘
- 실시간 위치 오차 모니터링

**필수 요구사항:**
- RTK GPS 모듈 (u-blox ZED-F9P 권장)
- RTK GPS 기지국
- RTK Fixed 모드 (최고 정밀도)

