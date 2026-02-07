# FollowerStatus.msg
# 팔로워 드론의 상태를 리더에게 보고 (2Hz)

std_msgs/Header header

# 드론 식별
uint8 drone_id

# 현재 위치
float64 latitude
float64 longitude
float32 altitude

# 편대 오프셋 오차 (목표 위치와의 거리, m)
float32 offset_error

# 미션 상태
# IDLE, FOLLOWING, SUPPRESSING, RTL, HOLD, ERROR
string mission_state

# 배터리 잔량 (0~100%)
uint8 battery_percent

# 소화탄 잔량 (0~6)
uint8 ammo_count
