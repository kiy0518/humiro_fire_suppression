# LeaderPose.msg
# 리더 드론의 실시간 위치/헤딩 정보 (이동 중 편대 비행용)

std_msgs/Header header

# GPS 좌표
float64 latitude   # 위도 (degrees)
float64 longitude  # 경도 (degrees)

# 고도 (m, MSL 또는 relative)
float32 altitude

# 헤딩 (degrees, 0~360, 북쪽=0)
float32 yaw_deg

# 속도 (m/s)
float32 speed

# NED 좌표 (로컬)
float32 x  # North (m)
float32 y  # East (m)
float32 z  # Down (m, 음수가 위)

# NED 속도
float32 vx  # North 속도 (m/s)
float32 vy  # East 속도 (m/s)
float32 vz  # Down 속도 (m/s)
