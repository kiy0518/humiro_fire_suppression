# DronePosition.msg
# 모든 드론이 5Hz로 브로드캐스트하는 위치 정보 (충돌 방지용)

std_msgs/Header header

# 드론 식별
uint8 drone_id

# GPS 좌표
float64 latitude   # 위도 (degrees)
float64 longitude  # 경도 (degrees)

# 고도 (m, MSL)
float32 altitude

# NED 속도 (m/s)
float32 vx  # North 속도
float32 vy  # East 속도
float32 vz  # Down 속도

# 미션 상태
string mission_state

# 안전 데이터 (편대 공유)
float32 battery_remaining   # 0.0~1.0 (-1 if invalid)
uint8 gps_fix_type          # 0=No, 2=2D, 3=3D, 4=DGPS, 5=RTK
uint8 gps_satellites         # 위성 수
