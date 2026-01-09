# 023. 스크립트 device_config.env 통합

**작성일**: 2026-01-10  
**버전**: v1.0

---

## 1. 작업 개요

모든 스크립트가 `device_config.env`를 읽어서 IP, DRONE_ID 등의 설정을 동적으로 처리하도록 수정.

### 1.1 목적
- 하드코딩된 IP 주소, DRONE_ID 제거
- 드론별 설정을 `device_config.env`에서 일괄 관리
- 다른 드론으로 복제 시 `device_config.env`만 수정하면 되도록 개선

---

## 2. 수정된 파일

### 2.1 scripts/install/start_micro_ros_agent_wrapper.sh
**상태**: Deprecated 처리

이 파일은 더 이상 사용되지 않음. `003-apply_config.sh`가 `runtime/start_micro_ros_agent_wrapper.sh`를 자동 생성.

### 2.2 scripts/debug/check_px4_connection.sh
**상태**: device_config.env 로드 추가

스크립트 시작 시 `device_config.env`를 로드하여 동적으로 설정값 사용.

### 2.3 scripts/debug/check_ros2_fc_connection.sh
**상태**: 주석 수정

하드코딩된 예시 주석을 일반적인 설명으로 변경.

---

## 3. 이미 올바르게 구현된 스크립트

다음 스크립트들은 이미 `device_config.env`를 올바르게 사용:

- Install: `000-install_all.sh`, `002-install_mavlink_router.sh`, `003-apply_config.sh`
- Runtime: `start_micro_ros_agent_wrapper.sh`, `humiro_fire_suppression_wrapper.sh`
- Check: `101-check_px4_connection.sh`, `103-verify_installation.sh`
- Debug: `check_px4_uxrce_params.sh`, `check_all_services.sh`

---

## 4. setup_env.sh 구조

프로젝트 루트의 `setup_env.sh`가 모든 환경 변수 정의:

- DEVICE_CONFIG
- MICRO_ROS_WS
- PX4_ROS2_WS
- SCRIPTS_INSTALL, SCRIPTS_CHECK, SCRIPTS_RUNTIME

---

## 5. device_config.env 주요 변수

- DRONE_ID, ROS_NAMESPACE
- ETH0_IP, FC_IP, WIFI_IP
- QGC_UDP_PORT, EXTERNAL_UDP_PORT (신규), FC_MAVLINK_PORT, XRCE_DDS_PORT
- ROS_DOMAIN_ID

---

## 6. 드론 복제 시 절차

1. 프로젝트 복사
2. device_config.env 수정 (DRONE_ID, IP 주소)
3. sudo ./003-apply_config.sh 실행
4. sudo reboot
5. ./scripts/check/103-verify_installation.sh로 검증

---

## 7. 결론

- 모든 스크립트가 device_config.env 사용
- 하드코딩 제거 완료
- 드론별 설정을 한 곳에서 관리
- 복제 및 유지보수 간소화
