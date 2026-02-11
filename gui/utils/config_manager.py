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

    def __init__(self, project_root: str):
        self.project_root = project_root
        self.config_dir = os.path.join(project_root, "config")
        self.device_config_path = os.path.join(self.config_dir, "device_config.env")
        self.fc_params_dir = os.path.join(self.config_dir, "fc_params")
        self.backup_dir = os.path.join(self.config_dir, "backup")
        self.custom_params_file = os.path.join(self.config_dir, "custom_params.json")

        # 설정 캐시
        self._device_config: Dict[str, str] = {}
        self._load_device_config()

        # FC 파라미터 디렉토리 생성
        os.makedirs(self.fc_params_dir, exist_ok=True)
        os.makedirs(self.backup_dir, exist_ok=True)

        # 커스텀 파라미터 파일 초기화
        if not os.path.exists(self.custom_params_file):
            self._init_custom_params_file()

    def _load_device_config(self):
        """device_config.env 로드"""
        self._device_config = {}
        if os.path.exists(self.device_config_path):
            with open(self.device_config_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        key, value = line.split("=", 1)
                        # 인라인 주석 제거 (# 이후 부분)
                        value = value.split("#")[0].strip().strip('"\'')
                        self._device_config[key.strip()] = value

    def reload(self):
        """설정 다시 로드"""
        self._load_device_config()

    def get(self, key: str, default: str = "") -> str:
        """설정 값 가져오기"""
        return self._device_config.get(key, default)

    def set(self, key: str, value: str):
        """설정 값 변경 (메모리에만)"""
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
        """모든 설정 가져오기"""
        return self._device_config.copy()

    def save_device_config(self) -> bool:
        """device_config.env 저장"""
        try:
            # 백업 생성
            self._create_backup(self.device_config_path)

            # 파일 저장
            # ROS_NAMESPACE는 라우터 페이지 SITL/FC 모드 토글에서만 관리
            # 일반 설정 저장 시 포함되면 FC 모드에서 토픽 불일치 발생
            exclude_keys = {"ROS_NAMESPACE"}
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
    def calculate_config_from_drone_id(drone_id: int, gcs_port_mode: str = "separate") -> Dict[str, str]:
        """DRONE_ID 기반 설정값 자동 계산

        Args:
            drone_id: 드론 번호 (1, 2, 3, ...)
            gcs_port_mode: GCS 포트 모드
                - "unified": 모든 기체가 14550 사용 (브로드캐스트 공유)
                - "separate": 기체별 포트 분리 (1번=14550, 2번=14560, 3번=14570)

        규칙:
        - ETH0_IP: 10.0.0.(드론번호*10+1) → 1번=10.0.0.11, 2번=10.0.0.21, 3번=10.0.0.31
        - FC_IP: 10.0.0.(드론번호*10+2) → 1번=10.0.0.12, 2번=10.0.0.22, 3번=10.0.0.32
        - WIFI_IP: 192.168.100.(드론번호*10+1) → 1번=192.168.100.11, 2번=192.168.100.21, 3번=192.168.100.31
        - ROS_NAMESPACE: drone{드론번호}
        - MAV_SYS_ID: 드론번호
        - MAV_COMP_ID: 191 (고정)
        - EXTERNAL_UDP_PORT: 15001 (고정, 모든 기체 동일 - mavlink-router Application 포트)
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
            "DRONE_ID": str(drone_id),
            "ETH0_IP": f"10.0.0.{base_offset + 1}",
            "FC_IP": f"10.0.0.{base_offset + 2}",
            "WIFI_IP": f"192.168.100.{base_offset + 1}",
            "MAV_SYS_ID": str(drone_id),
            "MAV_COMP_ID": "191",
            "EXTERNAL_UDP_PORT": "15001",
            "ROS_DOMAIN_ID": "0",
            "QGC_UDP_PORT": str(gcs_port),
            "FC_MAVLINK_PORT": "14540",
            "XRCE_DDS_PORT": "8888",
            "ROLE": role,
            "GCS_PORT_MODE": gcs_port_mode,
            "LEADER_NAMESPACE": "drone1",
            "FORMATION_OFFSET_RIGHT": fm["right"],
            "FORMATION_OFFSET_BEHIND": fm["behind"],
            "FORMATION_OFFSET_ABOVE": fm["above"],
            "SUPPRESS_DISTANCE": fm["sup_dist"],
            "SUPPRESS_ANGLE": fm["sup_angle"],
        }

    @staticmethod
    def get_gcs_port_for_drone(drone_id: int, gcs_port_mode: str = "separate") -> int:
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
        return self.get("GCS_PORT_MODE", "separate")

    def get_vehicle_preset(self, drone_id: int) -> Dict[str, str]:
        """기체별 설정 프리셋 반환 (DRONE_ID 기반 자동 계산)

        IP 체계:
        - ETH0 (VIM4): 10.0.0.X1 (X = 드론번호)
        - FC: 10.0.0.X2 (X = 드론번호)
        - WIFI: 192.168.100.X1 (X = 드론번호)
        """
        gcs_mode = self.get_gcs_port_mode()
        return self.calculate_config_from_drone_id(drone_id, gcs_mode)

    def apply_vehicle_preset(self, drone_id: int) -> bool:
        """기체 프리셋 적용 (DRONE_ID 기반 자동 계산 값 적용)"""
        preset = self.get_vehicle_preset(drone_id)
        for key, value in preset.items():
            self._device_config[key] = value
        return self.save_device_config()

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
        self._device_config.update(calculated)

        # WiFi 설정 복원
        if preserve_wifi:
            self._device_config.update(wifi_settings)

        return self.save_device_config()

    def get_calculated_preview(self, drone_id: int, gcs_port_mode: str = None) -> Dict[str, str]:
        """DRONE_ID 변경 시 미리보기 (실제 저장 없이 계산 결과 반환)

        Args:
            drone_id: 드론 번호
            gcs_port_mode: GCS 포트 모드 (None이면 현재 설정 사용)
        """
        if gcs_port_mode is None:
            gcs_port_mode = self.get_gcs_port_mode()
        return self.calculate_config_from_drone_id(drone_id, gcs_port_mode)

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
            name = 'Leader' if did == 1 else follower_labels.get(did, f'Follower {did}')
            result[did] = {
                'ip': cfg['WIFI_IP'],
                'eth0_ip': cfg['ETH0_IP'],
                'fc_ip': cfg['FC_IP'],
                'name': name,
                'gcs_port': int(cfg['QGC_UDP_PORT']),
                'mav_sys_id': int(cfg['MAV_SYS_ID']),
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

    def _init_custom_params_file(self):
        """커스텀 파라미터 파일 초기화"""
        default_data = {
            "categories": {
                "indoor": {
                    "name": "실내 모드",
                    "params": []
                },
                "outdoor_gps": {
                    "name": "야외 GPS 모드",
                    "params": []
                },
                "outdoor_rtk": {
                    "name": "야외 RTK 모드",
                    "params": []
                },
                "common": {
                    "name": "공통",
                    "params": [
                        {
                            "name": "MPC_XY_VEL_MAX",
                            "expected": 5,
                            "description": "최대 수평 속도 (m/s)",
                            "auto_check": True
                        }
                    ]
                }
            },
            "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        with open(self.custom_params_file, "w") as f:
            json.dump(default_data, f, indent=2, ensure_ascii=False)

    def load_custom_params(self) -> Dict:
        """커스텀 파라미터 전체 로드"""
        try:
            with open(self.custom_params_file, "r") as f:
                return json.load(f)
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

    def get_custom_params_by_category(self, category: str) -> List[Dict]:
        """카테고리별 커스텀 파라미터 목록 반환

        Args:
            category: indoor, outdoor_gps, outdoor_rtk, common

        Returns:
            [{"name": "PARAM_NAME", "expected": 123, "description": "설명", "auto_check": True}, ...]
        """
        data = self.load_custom_params()
        categories = data.get("categories", {})
        if category in categories:
            return categories[category].get("params", [])
        return []

    def add_custom_param(self, category: str, param: Dict) -> bool:
        """커스텀 파라미터 추가

        Args:
            category: indoor, outdoor_gps, outdoor_rtk, common
            param: {"name": "PARAM_NAME", "expected": 123, "description": "설명", "auto_check": True}
        """
        data = self.load_custom_params()
        categories = data.get("categories", {})

        if category not in categories:
            return False

        # 필수 필드 검증
        if "name" not in param:
            return False

        # 중복 체크
        existing = categories[category].get("params", [])
        for p in existing:
            if p.get("name") == param["name"]:
                # 이미 존재하면 업데이트
                p.update(param)
                return self.save_custom_params(data)

        # 기본값 설정
        param.setdefault("expected", None)
        param.setdefault("description", "")
        param.setdefault("auto_check", True)

        categories[category]["params"].append(param)
        return self.save_custom_params(data)

    def update_custom_param(self, category: str, param_name: str, param: Dict) -> bool:
        """커스텀 파라미터 업데이트"""
        data = self.load_custom_params()
        categories = data.get("categories", {})

        if category not in categories:
            return False

        params = categories[category].get("params", [])
        for i, p in enumerate(params):
            if p.get("name") == param_name:
                params[i].update(param)
                return self.save_custom_params(data)
        return False

    def delete_custom_param(self, category: str, param_name: str) -> bool:
        """커스텀 파라미터 삭제"""
        data = self.load_custom_params()
        categories = data.get("categories", {})

        if category not in categories:
            return False

        params = categories[category].get("params", [])
        for i, p in enumerate(params):
            if p.get("name") == param_name:
                params.pop(i)
                return self.save_custom_params(data)
        return False

    def get_all_categories(self) -> Dict[str, str]:
        """모든 카테고리 목록 반환"""
        data = self.load_custom_params()
        categories = data.get("categories", {})
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
