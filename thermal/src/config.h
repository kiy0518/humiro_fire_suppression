#ifndef CONFIG_H
#define CONFIG_H

// RTSP 설정
#define RTSP_PORT "8554"
#define RTSP_MOUNT_POINT "/stream"

// HTTP 서버 설정
#define ENABLE_HTTP_SERVER true
#define HTTP_PORT 8080

// 카메라 설정
#define THERMAL_CAMERA_KEYWORDS {"purethermal", "thermal", "lepton", "flir"}
#define MAX_THERMAL_KEYWORDS 4

// 출력 해상도
#define OUTPUT_WIDTH 640
#define OUTPUT_HEIGHT 480
#define OUTPUT_FPS 30
#define RGB_TARGET_FPS 30

// 열화상 데이터 처리 설정
#define SCALE 0.75f
#define THERMAL_WIDTH (int)(640 * SCALE)  // 480
#define THERMAL_HEIGHT (int)(480 * SCALE)  // 360
#define THERMAL_DX 0
#define THERMAL_DY -10
#define CUT_PIXELS 30
#define THERMAL_CROPPED_HEIGHT (THERMAL_HEIGHT - CUT_PIXELS)  // 330

// 마커 설정
#define CENTER_X (THERMAL_WIDTH / 2)  // 240
#define CENTER_Y (THERMAL_CROPPED_HEIGHT / 2)  // 165
#define RADIUS_OUTER 100
#define RADIUS_INNER 50

// 인코더 설정
#define BITRATE_KBPS 1500
#define USE_HARDWARE_ENCODER false

// 열화상 오버레이 설정
#define OVERLAY_THERMAL true
#define ALPHA_THERMAL_LAYER 0.3f
#define GRADIENT_BORDER_SIZE 30
#define GRADIENT_CURVE 1.0f

// 미리 계산된 값들
#define RGB_CROP_X (int)(((640 - THERMAL_WIDTH) / 2) + THERMAL_DX)  // 80
#define RGB_CROP_Y (int)((480 - THERMAL_HEIGHT) / 2 + THERMAL_DY < (480 - THERMAL_HEIGHT + CUT_PIXELS) ? \
                         (480 - THERMAL_HEIGHT) / 2 + THERMAL_DY : (480 - THERMAL_HEIGHT + CUT_PIXELS))  // 약 60

// 큐 크기
#define FRAME_QUEUE_SIZE 2
#define RGB_FRAME_QUEUE_SIZE 2
#define WEB_FRAME_QUEUE_SIZE 1

// 로고 설정 (환경 변수 또는 상대 경로 사용)
#define LOGO_PATH "easy_logo_alpha.png"  // thermal/src/ 디렉토리 기준
#define LOGO_WIDTH 120  // 로고 폭 (픽셀) - 원본의 0.5배
#define LOGO_POS_X (OUTPUT_WIDTH - LOGO_WIDTH - 5)  // 오른쪽 상단 (여백 20픽셀)
#define LOGO_POS_Y 5  // 상단 여백

// 정보 표시 배경 투명도
#define INFO_BACKGROUND_ALPHA 0.7f

// USB 카메라 재연결 설정
#define RESET_USB_CAMERAS_ON_START false  // 시작 시 USB 카메라 재연결 여부

// 라이다 오리엔테이션 설정
// 라이다의 0도 방향이 실제 전방과 다를 경우 오프셋 설정 (도 단위)
// 예: 라이다가 90도 회전되어 설치된 경우 90.0f 설정
// 기본값: 0.0f (오프셋 없음, 라이다 0도 = 드론 전방)
#define LIDAR_ORIENTATION_OFFSET 0.0f

// 라이다 디스플레이 모드 설정
// "FULL_360" = 전체 360도 표시
// "FRONT_3DIR" = 정면 중심 3방향 표시 (-90°, 0°, 90°)
// "THREE_POINTS" = 3방향 1포인트씩만 표시
#define LIDAR_DISPLAY_MODE "FULL_360"  // "FULL_360", "FRONT_3DIR", "THREE_POINTS"

// 라이다 표시 설정
#define LIDAR_SHOW_DIRECTION_LINES false  // 방향선 표시 (false = 원형 포인트만)
#define LIDAR_THREE_POINT_TOLERANCE 1.0f  // 3포인트 모드에서 각 방향 허용 범위 (±도)

// ROS2 통합 설정
// ROS2 토픽 발행 기능 활성화 (외부 모니터링/디버깅용)
// 내부 통신은 기존 큐 방식 유지 (성능 보장)
// CMakeLists.txt의 -DENABLE_ROS2 옵션으로 제어됨 (기본값: OFF)

#endif // CONFIG_H
