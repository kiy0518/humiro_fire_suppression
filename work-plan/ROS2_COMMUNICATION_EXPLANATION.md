# ROS2 통신 구조 설명

## 두 가지 ROS2 통신 경로

현재 시스템에는 **두 가지 다른 ROS2 통신 경로**가 있습니다:

### 1. PX4 ↔ VIM4 통신 (uxrce-dds / Micro-ROS)

**목적**: PX4 Flight Controller와 VIM4 간 비행 제어 데이터 통신

**경로**:
```
PX4 Flight Controller
    ↓ (UXRCE-DDS)
Micro-ROS Agent (VIM4에서 실행)
    ↓ (ROS2 DDS)
ROS2 토픽 (VIM4)
    - /fmu/out/vehicle_status
    - /fmu/out/vehicle_odometry
    - /fmu/in/vehicle_command
    - 등등...
```

**특징**:
- **Micro-ROS** 사용 (경량 ROS2)
- **uxrce-dds** 프로토콜 사용
- **비행 제어 데이터** (자세, 위치, 속도, 명령 등)
- Micro-ROS Agent가 DDS 브리지 역할

**설정 위치**:
- `workspaces/micro_ros_ws/` - Micro-ROS Agent
- `workspaces/px4_ros2_ws/` - PX4 ROS2 메시지

**사용 예시**:
```bash
# PX4 비행 데이터 확인
ros2 topic echo /fmu/out/vehicle_status
ros2 topic echo /fmu/out/vehicle_odometry
```

### 2. VIM4 내부 ROS2 토픽 (Phase 3에서 추가)

**목적**: VIM4 내부의 열화상/라이다 데이터를 ROS2 토픽으로 발행

**경로**:
```
VIM4 내부 애플리케이션 (humiro_fire_suppression)
    ↓ (ROS2 Publisher)
ROS2 토픽 (VIM4)
    - /thermal/image
    - /thermal/max_temperature
    - /lidar/points
    - /lidar/front_distance
    - 등등...
```

**특징**:
- **일반 ROS2** 사용 (Micro-ROS 아님)
- **열화상/라이다 데이터**
- VIM4 내부에서만 생성 (PX4와 무관)
- 선택적 활성화 (`-DENABLE_ROS2=ON`)

**설정 위치**:
- `thermal/src/thermal_ros2_publisher.*`
- `lidar/src/lidar_ros2_publisher.*`

**사용 예시**:
```bash
# 열화상/라이다 데이터 확인
ros2 topic echo /thermal/max_temperature
ros2 topic echo /lidar/front_distance
ros2 topic echo /thermal/image --once
```

## 통합 시나리오

### 같은 ROS2 DDS 네트워크에 있을 경우

두 가지 통신 경로가 **같은 ROS2 DDS 도메인**에 있으면, 서로의 토픽을 볼 수 있습니다:

```
ROS2 DDS 네트워크 (도메인 0, 기본)
├── PX4 토픽 (Micro-ROS Agent를 통해)
│   ├── /fmu/out/vehicle_status
│   ├── /fmu/out/vehicle_odometry
│   └── ...
│
└── VIM4 내부 토픽 (Phase 3)
    ├── /thermal/image
    ├── /thermal/max_temperature
    ├── /lidar/points
    └── ...
```

**모든 토픽 확인**:
```bash
# 모든 ROS2 토픽 확인 (PX4 + VIM4 내부)
ros2 topic list

# 출력 예시:
# /fmu/out/vehicle_status          # PX4에서
# /fmu/out/vehicle_odometry        # PX4에서
# /thermal/image                   # VIM4 내부 (아키텍처 리팩토링 Phase 3)
# /thermal/max_temperature         # VIM4 내부 (아키텍처 리팩토링 Phase 3)
# /lidar/front_distance            # VIM4 내부 (아키텍처 리팩토링 Phase 3)
```

### 활용 시나리오

#### 시나리오 1: 비행 데이터 + 열화상/라이다 데이터 통합

```python
# 예시: Python 노드에서 모든 데이터 수집
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # PX4 데이터 구독
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 10)
        
        # VIM4 내부 데이터 구독 (아키텍처 리팩토링 Phase 3)
        self.thermal_temp_sub = self.create_subscription(
            Float32, '/thermal/max_temperature',
            self.thermal_temp_callback, 10)
        
        self.lidar_dist_sub = self.create_subscription(
            Float32, '/lidar/front_distance',
            self.lidar_dist_callback, 10)
    
    def vehicle_status_callback(self, msg):
        # PX4 비행 데이터 처리
        pass
    
    def thermal_temp_callback(self, msg):
        # 열화상 데이터 처리
        pass
    
    def lidar_dist_callback(self, msg):
        # 라이다 데이터 처리
        pass
```

#### 시나리오 2: 원격 모니터링

외부 컴퓨터에서 VIM4의 모든 ROS2 토픽을 확인:

```bash
# 원격 컴퓨터에서 (ROS2 도메인 설정)
export ROS_DOMAIN_ID=0

# VIM4의 모든 토픽 확인
ros2 topic list

# PX4 데이터 확인
ros2 topic echo /fmu/out/vehicle_status

# VIM4 내부 데이터 확인
ros2 topic echo /thermal/max_temperature
ros2 topic echo /lidar/front_distance
```

## 요약

| 구분 | PX4 ↔ VIM4 | VIM4 내부 (Phase 3) |
|------|------------|---------------------|
| **목적** | 비행 제어 데이터 | 열화상/라이다 데이터 |
| **프로토콜** | uxrce-dds (Micro-ROS) | 일반 ROS2 |
| **데이터 타입** | 비행 상태, 명령 | 열화상, 라이다 |
| **필수 여부** | 필수 (비행 제어용) | 선택적 (모니터링용) |
| **토픽 예시** | `/fmu/out/vehicle_status` | `/thermal/max_temperature` |
| **활성화** | Micro-ROS Agent 실행 시 | `application/build`에서 `-DENABLE_ROS2=ON` 빌드 시 |

## 결론

**네, 완전히 다른 통신 경로입니다:**

1. **PX4 ↔ VIM4**: Micro-ROS (uxrce-dds) - 비행 제어 데이터
2. **VIM4 내부**: 일반 ROS2 - 열화상/라이다 데이터 (아키텍처 리팩토링 Phase 3에서 추가)

하지만 **같은 ROS2 DDS 네트워크**에 있으면 서로의 토픽을 볼 수 있어서, 통합 모니터링이나 데이터 수집에 활용할 수 있습니다.

