# FormationCommand.msg
# 리더가 팔로워에게 보내는 편대 명령 (이벤트)

std_msgs/Header header

# 대상 드론 ID (0=전체)
uint8 target_drone_id

# 명령 코드
# 0=HOLD, 1=FOLLOW, 2=GOTO, 3=RTL, 4=SUPPRESS
uint8 command

uint8 CMD_HOLD = 0
uint8 CMD_FOLLOW = 1
uint8 CMD_GOTO = 2
uint8 CMD_RTL = 3
uint8 CMD_SUPPRESS = 4

# GOTO 좌표 (command=GOTO일 때 사용)
float64 target_latitude
float64 target_longitude

# 편대 오프셋 (cm 단위)
int16 offset_right_cm       # 리더 우측 (+) / 좌측 (-)
int16 offset_behind_cm      # 리더 후방 (+) / 전방 (-)
int16 offset_above_cm       # 리더 상방 (+) / 하방 (-)

# 미션 파라미터 (CMD_FOLLOW 시 리더→팔로워 전달)
float32 takeoff_altitude     # 이륙/비행 고도 (m)
float32 flight_speed         # 비행 속도 (m/s)
