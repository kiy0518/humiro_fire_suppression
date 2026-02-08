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
