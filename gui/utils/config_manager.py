"""
설정 관리 모듈
device_config.env 및 기타 설정 파일 관리
"""

import os
import re
import json
import shutil
from datetime import datetime
from typing import Dict, Optional, Any, List


class ConfigManager:
    """설정 파일 관리자"""

    # 프로그램 파라미터 키 (app_config.env에 저장)
    APP_CONFIG_KEYS = {
        "SUPPRESS_ANGLE", "SUPPRESS_DISTANCE",
        "THERMAL_DX", "THERMAL_DY", "THERMAL_SCALE_X", "THERMAL_SCALE_Y",
        "THERMAL_SCALE", "STATUS_BAR_HEIGHT",
    }

    # DRONE_TYPE ↔ 프로파일 ID 매핑
    DRONE_TYPE_PROFILES = {"F450": "f450", "HEXA1500": "hexa1500"}
    PROFILE_DRONE_TYPES = {v: k for k, v in DRONE_TYPE_PROFILES.items()}

    def __init__(self, project_root: str):
        self.project_root = project_root
        self.config_dir = os.path.join(project_root, "config")
        self.device_config_path = os.path.join(self.config_dir, "device_config.env")
        self.fc_params_dir = os.path.join(self.config_dir, "fc_params")
        self.backup_dir = os.path.join(self.config_dir, "backup")
        self.custom_params_file = os.path.join(self.config_dir, "custom_params.json")

        # 설정 캐시
        self._device_config: Dict[str, str] = {}
        self._app_config: Dict[str, str] = {}
        self._load_device_config()

        # DRONE_ID 기반 app_config 경로 결정
        self.app_config_path = self._resolve_app_config_path()
        self._load_app_config()

        # FC 파라미터 디렉토리 생성
        os.makedirs(self.fc_params_dir, exist_ok=True)
        os.makedirs(self.backup_dir, exist_ok=True)

        # 커스텀 파라미터 파일 초기화
        if not os.path.exists(self.custom_params_file):
            self._init_custom_params_file()

    @staticmethod
    def _load_env_file(path: str) -> Dict[str, str]:
        """env 파일 로드 (공통)"""
        config = {}
        if os.path.exists(path):
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        key, value = line.split("=", 1)
                        value = value.split("#")[0].strip().strip('"\'')
                        config[key.strip()] = value
        return config

    def _load_device_config(self):
        """device_config.env 로드"""
        self._device_config = self._load_env_file(self.device_config_path)

    def _resolve_app_config_path(self) -> str:
        """DRONE_ID 기반 app_config 경로 결정
        app_config{DRONE_ID}.env 우선, 없으면 app_config.env 폴백"""
        drone_id = self._device_config.get("DRONE_ID", "1")
        drone_path = os.path.join(self.config_dir, f"app_config{drone_id}.env")
        if os.path.exists(drone_path):
            return drone_path
        return os.path.join(self.config_dir, "app_config.env")

    def _load_app_config(self):
        """app_config{DRONE_ID}.env 로드 (폴백: app_config.env)"""
        self._app_config = self._load_env_file(self.app_config_path)

    def reload(self):
        """설정 다시 로드"""
        self._load_device_config()
        self.app_config_path = self._resolve_app_config_path()
        self._load_app_config()

    def get(self, key: str, default: str = "") -> str:
        """설정 값 가져오기 (app_config 우선, device_config 폴백)"""
        if key in self.APP_CONFIG_KEYS:
            return self._app_config.get(key, self._device_config.get(key, default))
        return self._device_config.get(key, default)

    def set(self, key: str, value: str):
        """설정 값 변경 (메모리에만, 키에 따라 올바른 config에 저장)"""
        if key in self.APP_CONFIG_KEYS:
            self._app_config[key] = value
        else:
            self._device_config[key] = value

    def get_drone_id(self) -> int:
        """DRONE_ID 가져오기"""
        try:
            return int(self.get("DRONE_ID", "1"))
        except ValueError:
            return 1

    def get_fc_ip(self) -> str:
        """FC IP 주소 가져오기"""
        return self.get("FC_IP", "10.0.0.12")

    def get_eth0_ip(self) -> str:
        """ETH0 IP 주소 가져오기"""
        return self.get("ETH0_IP", "10.0.0.11")

    def get_wifi_ip(self) -> str:
        """WiFi IP 주소 가져오기"""
        return self.get("WIFI_IP", "192.168.100.11")

    def get_all_config(self) -> Dict[str, str]:
        """모든 설정 가져오기 (device + app 통합)"""
        merged = self._device_config.copy()
        merged.update(self._app_config)
        return merged

    def save_device_config(self) -> bool:
        """device_config.env 저장 (앱 파라미터 제외)"""
        try:
            self._create_backup(self.device_config_path)

            # ROS_NAMESPACE와 앱 파라미터 키 제외
            exclude_keys = {"ROS_NAMESPACE"} | self.APP_CONFIG_KEYS
            with open(self.device_config_path, "w") as f:
                f.write("# Humiro Fire Suppression Device Configuration\n")
                f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                for key, value in sorted(self._device_config.items()):
                    if key not in exclude_keys:
                        f.write(f"{key}={value}\n")

            return True
        except Exception as e:
            print(f"설정 저장 실패: {e}")
            return False

    def save_app_config(self) -> bool:
        """app_config{DRONE_ID}.env 저장 (프로그램 파라미터만)"""
        try:
            self._create_backup(self.app_config_path)

            drone_id = self._device_config.get("DRONE_ID", "1")
            filename = os.path.basename(self.app_config_path)
            with open(self.app_config_path, "w") as f:
                f.write(f"# Humiro Fire Suppression Application Parameters - Drone {drone_id}\n")
                f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                for key, value in sorted(self._app_config.items()):
                    f.write(f"{key}={value}\n")

            return True
        except Exception as e:
            print(f"앱 설정 저장 실패: {e}")
            return False

    def save_all_config(self) -> bool:
        """device_config.env + app_config.env 모두 저장"""
        return self.save_device_config() and self.save_app_config()

    def _create_backup(self, file_path: str):
        """파일 백업 생성"""
        if os.path.exists(file_path):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_subdir = os.path.join(self.backup_dir, timestamp)
            os.makedirs(backup_subdir, exist_ok=True)

            filename = os.path.basename(file_path)
            backup_path = os.path.join(backup_subdir, filename)
            shutil.copy2(file_path, backup_path)

    # === 기체별 설정 프리셋 (DRONE_ID 기반 자동 계산) ===

    @staticmethod
    def calculate_config_from_drone_id(drone_id: int, gcs_port_mode: str = "unified") -> Dict[str, str]:
        """DRONE_ID 기반 설정값 자동 계산

        Args:
            drone_id: 드론 번호 (1, 2, 3, ...)
            gcs_port_mode: GCS 포트 모드
                - "unified": 모든 기체가 14550 사용 (QGC 다중 기체 호환, 권장)
                - "separate": 기체별 포트 분리 (1번=14550, 2번=14560, 3번=14570)

        규칙:
        - ETH0_IP: 10.0.0.(드론번호*10+1) → 1번=10.0.0.11, 2번=10.0.0.21, 3번=10.0.0.31
        - FC_IP: 10.0.0.(드론번호*10+2) → 1번=10.0.0.12, 2번=10.0.0.22, 3번=10.0.0.32
        - WIFI_IP: 192.168.100.(드론번호*10+1) → 1번=192.168.100.11, 2번=192.168.100.21, 3번=192.168.100.31
        - ROS_NAMESPACE: drone{드론번호}
        - MAV_SYS_ID: 드론번호
        - MAV_COMP_ID: 191 (고정)
        - EXTERNAL_UDP_PORT: 16001 + (드론번호-1)*10 → 1번=16001, 2번=16011, 3번=16021
        - QGC_UDP_PORT:
            - unified 모드: 14550 (모든 기체 동일)
            - separate 모드: 14550 + (드론번호-1)*10 → 1번=14550, 2번=14560, 3번=14570
        - ROS_DOMAIN_ID: 0 (고정, 모든 기체 동일)
        - ROLE: 1번=Leader, 나머지=Follower
        """
        base_offset = drone_id * 10

        # GCS 포트 계산 (모드에 따라)
        if gcs_port_mode == "unified":
            gcs_port = 14550  # 모든 기체 동일
        else:  # separate (기본값)
            gcs_port = 14550 + (drone_id - 1) * 10  # 1번=14550, 2번=14560, 3번=14570

        # 역할 및 편대 기본값 (DRONE_ID 기반)
        # 1=Leader, 2=Follower_L(좌측), 3=Follower_R(우측)
        role_map = {1: "Leader", 2: "Follower_L", 3: "Follower_R"}
        role = role_map.get(drone_id, f"Follower_L" if drone_id % 2 == 0 else "Follower_R")

        # 편대 오프셋 기본값 (cm)
        # Follower_L: 좌후방 6m (-600, 600)
        # Follower_R: 우후방 6m (+600, 600)
        formation_defaults = {
            "Leader":     {"right": "0", "behind": "0", "above": "0", "sup_dist": "10", "sup_angle": "0"},
            "Follower_L": {"right": "-600", "behind": "600", "above": "0", "sup_dist": "10", "sup_angle": "-30"},
            "Follower_R": {"right": "600", "behind": "600", "above": "0", "sup_dist": "10", "sup_angle": "30"},
        }
        fm = formation_defaults.get(role, formation_defaults["Follower_L"])

        return {
            "device": {
                "DRONE_ID": str(drone_id),
                "ETH0_IP": f"10.0.0.{base_offset + 1}",
                "FC_IP": f"10.0.0.{base_offset + 2}",
                "WIFI_IP": f"192.168.100.{base_offset + 1}",
                "MAV_SYS_ID": str(drone_id),
                "MAV_COMP_ID": "191",
                "EXTERNAL_UDP_PORT": str(16001 + (drone_id - 1) * 10),
                "QGC_UDP_PORT": str(gcs_port),
                "FC_MAVLINK_PORT": "14540",
                "XRCE_DDS_PORT": "8888",
                "ROLE": role,
                "GCS_PORT_MODE": gcs_port_mode,
                "LEADER_NAMESPACE": "drone1",
                "FORMATION_OFFSET_RIGHT": fm["right"],
                "FORMATION_OFFSET_BEHIND": fm["behind"],
                "FORMATION_OFFSET_ABOVE": fm["above"],
            },
            "app": {
                "SUPPRESS_DISTANCE": fm["sup_dist"],
                "SUPPRESS_ANGLE": fm["sup_angle"],
            },
        }

    @staticmethod
    def get_gcs_port_for_drone(drone_id: int, gcs_port_mode: str = "unified") -> int:
        """특정 드론의 GCS 포트 계산

        Args:
            drone_id: 드론 번호
            gcs_port_mode: unified 또는 separate

        Returns:
            GCS 포트 번호
        """
        if gcs_port_mode == "unified":
            return 14550
        return 14550 + (drone_id - 1) * 10

    def get_gcs_port_mode(self) -> str:
        """현재 GCS 포트 모드 가져오기"""
        return self.get("GCS_PORT_MODE", "unified")

    def get_vehicle_preset(self, drone_id: int) -> Dict[str, Dict[str, str]]:
        """기체별 설정 프리셋 반환 (DRONE_ID 기반 자동 계산)

        IP 체계:
        - ETH0 (VIM4): 10.0.0.X1 (X = 드론번호)
        - FC: 10.0.0.X2 (X = 드론번호)
        - WIFI: 192.168.100.X1 (X = 드론번호)

        Returns:
            {"device": {...}, "app": {...}}
        """
        gcs_mode = self.get_gcs_port_mode()
        return self.calculate_config_from_drone_id(drone_id, gcs_mode)

    def apply_vehicle_preset(self, drone_id: int) -> bool:
        """기체 프리셋 적용 (DRONE_ID 기반 자동 계산 값 적용)"""
        preset = self.get_vehicle_preset(drone_id)
        self._device_config.update(preset["device"])

        # DRONE_ID 변경에 따라 app_config 경로 업데이트
        self.app_config_path = self._resolve_app_config_path()
        self._load_app_config()
        self._app_config.update(preset["app"])
        return self.save_device_config() and self.save_app_config()

    def regenerate_config_from_drone_id(
        self,
        drone_id: int,
        preserve_wifi: bool = True,
        gcs_port_mode: str = None
    ) -> bool:
        """DRONE_ID 기반으로 device_config.env 재생성

        Args:
            drone_id: 드론 번호 (1, 2, 3, ...)
            preserve_wifi: True면 기존 WiFi 설정(SSID, 비밀번호, 인터페이스, 게이트웨이 등) 유지
            gcs_port_mode: GCS 포트 모드 ("unified" 또는 "separate"), None이면 기존 설정 유지

        Returns:
            성공 여부
        """
        # 기존 WiFi 설정 및 GCS_PORT_MODE 백업
        wifi_settings = {}
        if preserve_wifi:
            wifi_keys = [
                "WIFI_INTERFACE", "WIFI_GATEWAY", "WIFI_NETMASK",
                "WIFI_SSID_1", "WIFI_PASSWORD_1",
                "WIFI_SSID_2", "WIFI_PASSWORD_2",
                "WIFI_SSID_3", "WIFI_PASSWORD_3",
                "WIFI_SSID_4", "WIFI_PASSWORD_4",
                "WIFI_SSID_5", "WIFI_PASSWORD_5",
                "RTSP_PORT", "STREAM_NAME",
            ]
            for key in wifi_keys:
                if key in self._device_config:
                    wifi_settings[key] = self._device_config[key]

        # GCS 포트 모드 결정 (명시적 지정 > 기존 설정 > 기본값)
        if gcs_port_mode is None:
            gcs_port_mode = self.get_gcs_port_mode()

        # DRONE_ID 기반 자동 계산 값 적용
        calculated = self.calculate_config_from_drone_id(drone_id, gcs_port_mode)
        self._device_config.update(calculated["device"])

        # DRONE_ID 변경에 따라 app_config 경로 업데이트
        self.app_config_path = self._resolve_app_config_path()
        self._load_app_config()
        self._app_config.update(calculated["app"])

        # WiFi 설정 복원
        if preserve_wifi:
            self._device_config.update(wifi_settings)

        return self.save_device_config() and self.save_app_config()

    def get_calculated_preview(self, drone_id: int, gcs_port_mode: str = None) -> Dict[str, str]:
        """DRONE_ID 변경 시 미리보기 (실제 저장 없이 계산 결과 반환, 통합 dict)

        Args:
            drone_id: 드론 번호
            gcs_port_mode: GCS 포트 모드 (None이면 현재 설정 사용)
        """
        if gcs_port_mode is None:
            gcs_port_mode = self.get_gcs_port_mode()
        result = self.calculate_config_from_drone_id(drone_id, gcs_port_mode)
        # 미리보기용: device + app 통합 반환
        merged = result["device"].copy()
        merged.update(result["app"])
        return merged

    # === 드론 네트워크 헬퍼 ===

    @staticmethod
    def ip_to_decimal(ip: str) -> int:
        """IP 주소를 decimal 정수로 변환 (UXRCE_DDS_AG_IP용)"""
        parts = ip.split('.')
        return (int(parts[0]) << 24) + (int(parts[1]) << 16) + (int(parts[2]) << 8) + int(parts[3])

    def get_broadcast_ip(self) -> str:
        """WiFi 브로드캐스트 주소 계산"""
        wifi_ip = self.get_wifi_ip()
        parts = wifi_ip.rsplit('.', 1)
        return f"{parts[0]}.255"

    def get_all_drone_configs(self, max_drones: int = 3) -> Dict[str, Any]:
        """전체 드론(1~max_drones) 네트워크 설정을 동적 생성

        Returns:
            {1: {'ip': '192.168.100.11', 'eth0_ip': '10.0.0.11', 'fc_ip': '10.0.0.12',
                 'name': 'Leader', 'gcs_port': 14550}, ...}
        """
        gcs_mode = self.get_gcs_port_mode()
        # Follower 구분용 라벨
        follower_labels = {2: 'Follower L', 3: 'Follower R'}
        result = {}
        for did in range(1, max_drones + 1):
            cfg = self.calculate_config_from_drone_id(did, gcs_mode)
            dev = cfg["device"]
            name = 'Leader' if did == 1 else follower_labels.get(did, f'Follower {did}')
            result[did] = {
                'ip': dev['WIFI_IP'],
                'eth0_ip': dev['ETH0_IP'],
                'fc_ip': dev['FC_IP'],
                'name': name,
                'gcs_port': int(dev['QGC_UDP_PORT']),
                'mav_sys_id': int(dev['MAV_SYS_ID']),
            }
        return result

    # === 비행 모드 파라미터 프리셋 ===

    def get_indoor_params(self) -> Dict[str, Any]:
        """실내 테스트 모드 파라미터 (PX4 v1.16.0)
        MTF-01 옵티컬플로 센서 설정 포함

        v1.14 이후 변경사항:
        - EKF2_AID_MASK 삭제됨 → 개별 _CTRL 파라미터로 분리
        - EKF2_HGT_MODE 삭제됨 → EKF2_HGT_REF로 변경
        - EKF2_RNG_AID 삭제됨 → EKF2_RNG_CTRL로 통합
        """
        return {
            # === EKF2 센서 융합 제어 (v1.16.0) ===
            "EKF2_GPS_CTRL": 0,  # GPS 융합 비활성화
            "EKF2_HGT_REF": "Range sensor",  # 고도 기준: Range sensor
            "EKF2_OF_CTRL": "Enabled",  # Optical Flow 융합 활성화
            "EKF2_RNG_A_HMAX": 8,  # Range aid 최대 고도 (m)
            "EKF2_RNG_CTRL": "Enabled",  # Range Finder 융합 활성화

            # === 센서 설정 ===
            "GPS_1_CONFIG": 0,  # GPS 포트 비활성화
            "SENS_FLOW_ROT": "No rotation",  # Optical Flow 센서 회전 (기본 방향)

            # === MTF-01 옵티컬플로 센서 (MAVLink) ===
            "MAV_1_CONFIG": "TELEM3",  # MAVLink 포트
            "MAV_1_MODE": "Normal",  # MAVLink 모드
            "SER_TEL3_BAUD": "115200 8N1",  # 시리얼 통신 속도
        }

    def get_outdoor_gps_params(self) -> Dict[str, Any]:
        """야외 GPS 모드 파라미터 (PX4 v1.16.0)

        EKF2_GPS_CTRL 비트마스크:
        - bit 0 (1): Horizontal position (Lon/Lat)
        - bit 1 (2): Vertical position (Altitude)
        - bit 2 (4): 3D Velocity
        - bit 3 (8): Dual antenna heading (Yaw)

        v1.14 이후 변경사항:
        - EKF2_AID_MASK 삭제됨 → EKF2_GPS_CTRL 사용
        - EKF2_HGT_MODE 삭제됨 → EKF2_HGT_REF 사용
        - EKF2_RNG_AID 삭제됨 → EKF2_RNG_CTRL 사용
        """
        return {
            # === EKF2 센서 융합 제어 (v1.16.0) ===
            "EKF2_BARO_CTRL": 1,  # Barometer 융합 활성화
            "EKF2_GPS_CTRL": 7,  # GPS 위치/고도/속도 융합 (1+2+4)
            "EKF2_HGT_REF": "GPS",  # 고도 기준: GPS (Range sensor도 가능)
            "EKF2_OF_CTRL": "Disable",  # Optical Flow 융합 비활성화
            "EKF2_RNG_CTRL": 2,  # Conditional range aiding (조건부 Range 융합)

            # === GPS 파라미터 ===
            "GPS_1_CONFIG": "GPS1",  # GPS 포트 설정
        }

    def get_outdoor_rtk_params(self) -> Dict[str, Any]:
        """야외 RTK-GPS 모드 파라미터 (PX4 v1.16.0)

        야외 GPS 기반 + RTK 프로토콜 설정 추가

        GPS_1_PROTOCOL: 0=Auto, 1=u-blox, 2=MTK, 3=Ashtech/Trimble, 4=Emlid, 5=Femtomes, 6=NMEA
        GPS_UBX_MODE: 0=Default(RTK포함), 1=Heading Rover, 2=Moving Base, 3=Heading 921600, 4=Moving Base 921600, 5=Static Base UART2
        """
        params = self.get_outdoor_gps_params()
        params.update({
            # RTK GPS 프로토콜 설정
            "GPS_1_PROTOCOL": 1,  # 1=u-blox
            "GPS_UBX_MODE": 0,  # 0=Default (RTK 포함)
        })
        return params

    def get_offboard_norc_params(self) -> Dict[str, Any]:
        """RC 없이 OFFBOARD 모드 파라미터 (PX4 v1.16.0)

        조종기 없이 OFFBOARD 모드로만 비행할 때 사용

        COM_RCL_EXCEPT: RC 두절 예외 (비트마스크)
        - bit 0 (1): Mission mode
        - bit 1 (2): Hold mode
        - bit 2 (4): Offboard mode

        COM_RC_IN_MODE: RC 입력 모드
        - 0: RC Transmitter only (RC 필수)
        - 1: RC 선택 (Joystick에서 스틱 입력)
        - 2: Virtual RC by Joystick (가상 RC)
        - 3: RC Scan (RC 연결 전 대기)
        - 4: RC not required (RC 불필요)
        """
        return {
            # RC 없이 OFFBOARD 모드 사용 설정
            "COM_RCL_EXCEPT": 4,  # OFFBOARD 모드에서 RC 두절 페일세이프 예외
            "COM_RC_IN_MODE": 4,  # RC not required (RC 불필요)

            # OFFBOARD 페일세이프 설정
            "COM_OBL_RC_ACT": 4,  # OFFBOARD 두절 시 Land (착륙)
            "COM_OF_LOSS_T": 1.0,  # OFFBOARD 두절 판정 시간 (초)
        }

    def save_fc_params_preset(self, name: str, params: Dict[str, Any]) -> bool:
        """FC 파라미터 프리셋 저장"""
        try:
            filepath = os.path.join(self.fc_params_dir, f"{name}.params")
            with open(filepath, "w") as f:
                f.write(f"# FC Parameters Preset: {name}\n")
                f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                for key, value in sorted(params.items()):
                    f.write(f"{key}\t{value}\n")
            return True
        except Exception as e:
            print(f"파라미터 프리셋 저장 실패: {e}")
            return False

    def load_fc_params_preset(self, name: str) -> Optional[Dict[str, Any]]:
        """FC 파라미터 프리셋 로드"""
        try:
            filepath = os.path.join(self.fc_params_dir, f"{name}.params")
            if not os.path.exists(filepath):
                return None

            params = {}
            with open(filepath, "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#"):
                        parts = line.split("\t")
                        if len(parts) >= 2:
                            key = parts[0]
                            try:
                                value = float(parts[1])
                                if value.is_integer():
                                    value = int(value)
                            except ValueError:
                                value = parts[1]
                            params[key] = value
            return params
        except Exception as e:
            print(f"파라미터 프리셋 로드 실패: {e}")
            return None

    def list_fc_params_presets(self) -> list:
        """저장된 FC 파라미터 프리셋 목록"""
        presets = []
        if os.path.exists(self.fc_params_dir):
            for filename in os.listdir(self.fc_params_dir):
                if filename.endswith(".params"):
                    presets.append(filename[:-7])  # Remove .params extension
        return sorted(presets)

    # === 커스텀 FC 파라미터 관리 ===

    _default_categories = {
        "indoor": {"name": "실내 모드", "params": []},
        "outdoor_gps": {"name": "야외 GPS 모드", "params": []},
        "outdoor_rtk": {"name": "야외 RTK 모드", "params": []},
        "common": {"name": "공통", "params": []}
    }

    def _init_custom_params_file(self):
        """커스텀 파라미터 파일 초기화 (프로파일 구조)"""
        import copy
        default_data = {
            "active_profile": "f450",
            "profiles": {
                "f450": {
                    "name": "F450 테스트 기체",
                    "categories": copy.deepcopy(self._default_categories)
                },
                "hexa1500": {
                    "name": "HEXA1500 소방드론",
                    "categories": copy.deepcopy(self._default_categories)
                }
            },
            "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        with open(self.custom_params_file, "w") as f:
            json.dump(default_data, f, indent=2, ensure_ascii=False)

    def _migrate_to_profiles(self, data: Dict) -> Dict:
        """기존 flat/구버전 구조를 현재 프로파일 구조로 마이그레이션"""
        import copy
        changed = False

        if "profiles" not in data:
            # flat 구조 → 프로파일 구조 마이그레이션
            old_categories = data.get("categories", copy.deepcopy(self._default_categories))
            data = {
                "active_profile": "f450",
                "profiles": {
                    "f450": {"name": "F450 테스트 기체", "categories": old_categories},
                    "hexa1500": {"name": "HEXA1500 소방드론", "categories": copy.deepcopy(self._default_categories)}
                },
                "updated_at": data.get("updated_at", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
            }
            changed = True

        # fire_drone → hexa1500 이름 변경 마이그레이션
        profiles = data.get("profiles", {})
        if "fire_drone" in profiles:
            profiles["hexa1500"] = profiles.pop("fire_drone")
            profiles["hexa1500"]["name"] = "HEXA1500 소방드론"
            if data.get("active_profile") == "fire_drone":
                data["active_profile"] = "hexa1500"
            changed = True

        if changed:
            self.save_custom_params(data)
        return data

    def load_custom_params(self) -> Dict:
        """커스텀 파라미터 전체 로드 (프로파일 구조 자동 마이그레이션)"""
        try:
            with open(self.custom_params_file, "r") as f:
                data = json.load(f)
            return self._migrate_to_profiles(data)
        except Exception as e:
            print(f"커스텀 파라미터 로드 실패: {e}")
            self._init_custom_params_file()
            return self.load_custom_params()

    def save_custom_params(self, data: Dict) -> bool:
        """커스텀 파라미터 저장"""
        try:
            data["updated_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            with open(self.custom_params_file, "w") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"커스텀 파라미터 저장 실패: {e}")
            return False

    def get_active_profile(self) -> str:
        """현재 활성 프로파일 ID 반환 (device_config.env DRONE_TYPE 기준)"""
        drone_type = self._device_config.get("DRONE_TYPE", "F450").strip()
        return self.DRONE_TYPE_PROFILES.get(drone_type, "f450")

    def set_active_profile(self, profile_id: str) -> bool:
        """활성 프로파일 변경 (device_config.env + custom_params.json 동기화)"""
        data = self.load_custom_params()
        if profile_id not in data.get("profiles", {}):
            return False
        # device_config.env에 DRONE_TYPE 저장
        drone_type = self.PROFILE_DRONE_TYPES.get(profile_id)
        if drone_type:
            self._device_config["DRONE_TYPE"] = drone_type
            self.save_device_config()
        # custom_params.json에도 동기화
        data["active_profile"] = profile_id
        return self.save_custom_params(data)

    def get_profiles(self) -> Dict[str, str]:
        """프로파일 목록 반환 {id: name}"""
        data = self.load_custom_params()
        profiles = data.get("profiles", {})
        return {k: v.get("name", k) for k, v in profiles.items()}

    def _get_active_categories(self, data: Dict = None) -> Dict:
        """활성 프로파일의 categories 반환"""
        if data is None:
            data = self.load_custom_params()
        profile_id = self.get_active_profile()
        profiles = data.get("profiles", {})
        if profile_id in profiles:
            return profiles[profile_id].get("categories", {})
        # fallback: 첫 번째 프로파일
        if profiles:
            first = next(iter(profiles.values()))
            return first.get("categories", {})
        return {}

    def get_custom_params_by_category(self, category: str) -> List[Dict]:
        """카테고리별 커스텀 파라미터 목록 반환 (활성 프로파일 기준)"""
        categories = self._get_active_categories()
        if category in categories:
            return categories[category].get("params", [])
        return []

    def add_custom_param(self, category: str, param: Dict) -> bool:
        """커스텀 파라미터 추가 (활성 프로파일에)"""
        data = self.load_custom_params()
        categories = self._get_active_categories(data)

        if category not in categories:
            return False

        if "name" not in param:
            return False

        existing = categories[category].get("params", [])
        for p in existing:
            if p.get("name") == param["name"]:
                p.update(param)
                return self.save_custom_params(data)

        param.setdefault("expected", None)
        param.setdefault("description", "")
        param.setdefault("auto_check", True)

        categories[category]["params"].append(param)
        return self.save_custom_params(data)

    def update_custom_param(self, category: str, param_name: str, param: Dict) -> bool:
        """커스텀 파라미터 업데이트 (활성 프로파일)"""
        data = self.load_custom_params()
        categories = self._get_active_categories(data)

        if category not in categories:
            return False

        params = categories[category].get("params", [])
        for i, p in enumerate(params):
            if p.get("name") == param_name:
                params[i].update(param)
                return self.save_custom_params(data)
        return False

    def delete_custom_param(self, category: str, param_name: str) -> bool:
        """커스텀 파라미터 삭제 (활성 프로파일)"""
        data = self.load_custom_params()
        categories = self._get_active_categories(data)

        if category not in categories:
            return False

        params = categories[category].get("params", [])
        for i, p in enumerate(params):
            if p.get("name") == param_name:
                params.pop(i)
                return self.save_custom_params(data)
        return False

    def get_all_categories(self) -> Dict[str, str]:
        """모든 카테고리 목록 반환 (활성 프로파일)"""
        categories = self._get_active_categories()
        return {k: v.get("name", k) for k, v in categories.items()}

    # === 페일세이프 설정 ===

    def get_failsafe_params(self) -> Dict[str, Any]:
        """페일세이프 권장 설정 파라미터 (PX4 v1.16.0)

        실내/야외 공통 권장 설정

        NAV_RCL_ACT: RC 두절 시 동작
        - 0: 경고만
        - 1: Hold mode
        - 2: RTL (야외 권장)
        - 3: Land (실내 권장)
        - 5: Disarm
        - 6: Terminate

        COM_OBL_RC_ACT: OFFBOARD 두절 시 동작
        - 0: Position mode
        - 1: Altitude mode
        - 2: Manual mode
        - 3: RTL (야외 권장)
        - 4: Land (실내 권장)
        - 5: Hold (Loiter)

        NAV_DLL_ACT: Data Link (통신) 두절 시 동작
        - 0: Disabled (비활성화)
        - 1: Hold mode
        - 2: RTL (야외 권장)
        - 3: Land (실내 권장)
        - 5: Disarm
        - 6: Terminate

        COM_LOW_BAT_ACT: 배터리 저전압 시 동작
        - 0: 경고만
        - 1: Land (실내 권장)
        - 2: RTL (야외 권장)
        - 3: RTL or Land (RTL 가능시 RTL, 아니면 Land)
        """
        return {
            # === RC 페일세이프 ===
            ("NAV_RCL_ACT", "RC 두절 시 동작"): "실내:3(Land) / 야외:2(RTL)",
            ("COM_RC_LOSS_T", "RC 두절 판정 시간"): 0.5,

            # === OFFBOARD 페일세이프 ===
            ("COM_OBL_RC_ACT", "OFFBOARD 두절 시 동작"): "실내:4(Land) / 야외:3(RTL)",
            ("COM_OF_LOSS_T", "OFFBOARD 두절 판정 시간"): 1.0,

            # === Data Link (통신) 페일세이프 ===
            ("NAV_DLL_ACT", "Data Link 두절 시 동작"): "실내:3(Land) / 야외:2(RTL)",
            ("COM_DL_LOSS_T", "Data Link 두절 판정 시간"): 10,

            # === 배터리 페일세이프 ===
            ("COM_LOW_BAT_ACT", "배터리 저전압 시 동작"): "실내:1(Land) / 야외:3(RTL or Land)",
            ("BAT_LOW_THR", "저전압 임계값 (%)"): 15,
            ("BAT_CRIT_THR", "위험 전압 임계값 (%)"): 10,
            ("BAT_EMERGEN_THR", "비상 전압 임계값 (%)"): 5,
        }
