# TODO: GUI 수정 필요 사항

## custom_message_sender_gui_v2.py 파일 수정 필요

### 문제점
- ARM/DISARM: 14550 포트로 정상 작동 ✅
- 비행 모드 변경: 14550 포트로 전송했지만 작동하지 않음 ❌

### 해결 방안
1. test_px4_mode_change.py로 정확한 PX4 모드 값 찾기
2. 또는 FIRE_SET_MODE 커스텀 메시지로 구현 (ROS2 경유)

### 참조
- arm_handler.cpp::enableOffboardMode() - 작동하는 OFFBOARD 설정 코드
- test_px4_mode_change.py - PX4 모드 테스트 스크립트

작성: 2026-01-05

