# LeaderAimPose.msg
# 리더 드론의 조준 위치 정보 (팔로워 발사 지점 계산용)

std_msgs/Header header

# 리더 현재 위치
float64 latitude      # 리더 위도
float64 longitude     # 리더 경도
float32 altitude      # 리더 고도 (m)

# 리더 헤딩
float32 yaw_deg       # 헤딩 (degrees)

# 목표물까지 거리
float32 distance_to_target  # LiDAR 측정 거리 (m)

# 목표물 위치 (리더가 계산)
float64 target_latitude   # 목표물 위도
float64 target_longitude  # 목표물 경도

# 조준 완료 여부
bool aim_locked

# NED 좌표 (로컬)
float32 x  # North (m)
float32 y  # East (m)
float32 z  # Down (m)
