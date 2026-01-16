# LeaderStatus.msg
# 리더 드론의 현재 미션 상태를 팔로워에게 알림

std_msgs/Header header

# 현재 미션 단계
# ARMING, TAKEOFF, WAYPOINT, ARRIVED, DISTANCE_ADJ,
# FIRE_FIGHTING, AIMING, FIRING, RETURNING, LANDING, DISARMED
string phase

# 격발 시 발수 (1~6)
uint8 shot_number

# 미션 진행 중 여부
bool mission_active

# 현재 고도 (m)
float32 altitude

# 진행률 (0~100)
uint8 progress
