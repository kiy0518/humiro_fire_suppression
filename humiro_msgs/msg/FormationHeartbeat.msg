# FormationHeartbeat.msg
# 리더 생존 확인 및 편대 상태 브로드캐스트 (1Hz)

std_msgs/Header header

# 리더 드론 ID
uint8 leader_id

# 편대 드론 수 (리더 포함)
uint8 drone_count

# 편대 단계
# IDLE, TAKEOFF, NAVIGATE, SUPPRESS, RTL
string formation_phase
