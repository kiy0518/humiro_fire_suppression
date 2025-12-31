# 설치 스크립트

## 실행 순서

1. `000-install_all.sh` - 전체 설치 (권장)
2. 또는 개별 설치:
   - `001-install_px4_ros2_complete.sh` - PX4 ROS2 설치
   - `002-install_mavlink_router.sh` - MAVLink Router 설치
   - `003-apply_config.sh` - 설정 적용

## 주의사항

- 모든 스크립트는 `setup_env.sh`를 자동으로 로드합니다
- 환경 변수를 통해 경로를 관리하므로 프로젝트 위치가 변경되어도 동작합니다
