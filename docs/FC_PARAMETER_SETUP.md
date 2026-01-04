### 📋 기체별 설정 요약

| 항목 | 드론 #1 | 드론 #2 | 드론 #3 |
| --- | --- | --- | --- |
| DRONE\_ID | 1 | 2 | 3 |
| ROS\_NAMESPACE | drone1 | drone2 | drone3 |
| ETH0\_IP | 10.0.0.11 | 10.0.0.21 | 10.0.0.31 |
| FC\_IP | 10.0.0.12 | 10.0.0.22 | 10.0.0.32 |
| WIFI\_IP | 192.168.100.11 | 192.168.100.21 | 192.168.100.31 |

## 7\. PX4 파라미터 설정

QGroundControl에서 FC(Pixhawk)의 파라미터를 설정합니다.

### 🔹 MAV\_SYS\_ID (기체 식별)

| 파라미터 | 드론 #1 | 드론 #2 | 드론 #3 |
| --- | --- | --- | --- |
| **MAV\_SYS\_ID** | 1 | 2 | 3 |

### 🔹 UXRCE\_DDS (ROS2 연결)

| 파라미터 | 드론 #1 | 드론 #2 | 드론 #3 | 설명 |
| --- | --- | --- | --- | --- |
| **UXRCE\_DDS\_AG\_IP** | 167772171 | 167772181 | 167772191 | Agent IP (Decimal) |
| UXRCE\_DDS\_CFG | Ethernet | Ethernet | Ethernet | Serial Config |
| UXRCE\_DDS\_DOM\_ID | 0 | 0 | 0 | Domain ID |
| **UXRCE\_DDS\_KEY** | 1 | 2 | 3 | Session Key |
| UXRCE\_DDS\_PRT | 8888 | 8888 | 8888 | UDP Port |

### 🔹 MAV\_2 (Ethernet MAVLink)

| 파라미터 | 값 | 설명 |
| --- | --- | --- |
| MAV\_2\_BROADCAST | Always broadcast (1) | Heartbeat 브로드캐스트 |
| MAV\_2\_CONFIG | Ethernet | Ethernet 사용 |
| MAV\_2\_MODE | Normal | Offboard 제어용 |
| MAV\_2\_REMOTE\_PRT | 14540 | Remote Port |
| MAV\_2\_UDP\_PRT | 14550 | Network Port |

**📌 IP → Decimal 변환:**  
10.0.0.11 → **167772171**  
10.0.0.21 → **167772181**  
10.0.0.31 → **167772191**  
공식: (10×256³) + (0×256²) + (0×256) + X = 167772160 + X