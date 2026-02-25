# FormationCommand.msg
# 리더가 팔로워에게 보내는 편대 명령 (이벤트)

std_msgs/Header header

# 대상 드론 ID (0=전체)
uint8 target_drone_id

# 명령 코드
# 0=HOLD, 1=FOLLOW, 2=GOTO, 3=RTL, 4=SUPPRESS, 5=SWARM_START, 6=ALIGN_COMPLETE
uint8 command

uint8 CMD_HOLD = 0
uint8 CMD_FOLLOW = 1
uint8 CMD_GOTO = 2
uint8 CMD_RTL = 3
uint8 CMD_SUPPRESS = 4
uint8 CMD_SWARM_START = 5       # 리더→팔로워: 독립 편대 미션 시작 (목표GPS + 접근헤딩)
uint8 CMD_ALIGN_COMPLETE = 6    # 리더→팔로워: 정렬 완료, 진압 위치로 이동

# 목표 좌표 (GOTO/SWARM_START/ALIGN_COMPLETE 공용)
float64 target_latitude
float64 target_longitude

# 편대 오프셋 (cm 단위)
int16 offset_right_cm       # 리더 우측 (+) / 좌측 (-)
int16 offset_behind_cm      # 리더 후방 (+) / 전방 (-)
int16 offset_above_cm       # 리더 상방 (+) / 하방 (-)

# 미션 파라미터 (CMD_FOLLOW/CMD_SWARM_START 시 리더→팔로워 전달)
float32 takeoff_altitude     # 이륙/비행 고도 (m)
float32 flight_speed         # 비행 속도 (m/s)

# Swarm 모드 전용 필드
float32 approach_heading     # 접근 헤딩 (rad) — 팔로워 목적지 계산용 (CMD_SWARM_START)
                             # 리더 헤딩 (rad) — 진압 위치 계산용 (CMD_ALIGN_COMPLETE)
float64 fire_target_latitude   # 추정 화점 GPS 위도 (CMD_ALIGN_COMPLETE)
float64 fire_target_longitude  # 추정 화점 GPS 경도 (CMD_ALIGN_COMPLETE)
