"""
설정 관리 모듈
device_config.env 및 기타 설정 파일 관리
"""

import os
import re
import shutil
from datetime import datetime
from typing import Dict, Optional, Any


class ConfigManager:
    """설정 파일 관리자"""

    def __init__(self, project_root: str):
        self.project_root = project_root
        self.config_dir = os.path.join(project_root, "config")
        self.device_config_path = os.path.join(self.config_dir, "device_config.env")
        self.fc_params_dir = os.path.join(self.config_dir, "fc_params")
        self.backup_dir = os.path.join(self.config_dir, "backup")

        # 설정 캐시
        self._device_config: Dict[str, str] = {}
        self._load_device_config()

        # FC 파라미터 디렉토리 생성
        os.makedirs(self.fc_params_dir, exist_ok=True)
        os.makedirs(self.backup_dir, exist_ok=True)

    def _load_device_config(self):
        """device_config.env 로드"""
        self._device_config = {}
        if os.path.exists(self.device_config_path):
            with open(self.device_config_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#") and "=" in line:
                        key, value = line.split("=", 1)
                        self._device_config[key.strip()] = value.strip().strip('"\'')

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
            with open(self.device_config_path, "w") as f:
                f.write("# Humiro Fire Suppression Device Configuration\n")
                f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                for key, value in sorted(self._device_config.items()):
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

    # === 기체별 설정 프리셋 ===

    def get_vehicle_preset(self, drone_id: int) -> Dict[str, str]:
        """기체별 설정 프리셋 반환"""
        presets = {
            1: {  # Leader
                "DRONE_ID": "1",
                "ETH0_IP": "10.0.0.11",
                "FC_IP": "10.0.0.12",
                "WIFI_IP": "192.168.100.11",
                "ROS_NAMESPACE": "drone1",
                "MAV_SYS_ID": "1",
                "ROLE": "Leader",
            },
            2: {  # Follower Left
                "DRONE_ID": "2",
                "ETH0_IP": "10.0.0.21",
                "FC_IP": "10.0.0.22",
                "WIFI_IP": "192.168.100.21",
                "ROS_NAMESPACE": "drone2",
                "MAV_SYS_ID": "2",
                "ROLE": "Follower",
            },
            3: {  # Follower Right
                "DRONE_ID": "3",
                "ETH0_IP": "10.0.0.31",
                "FC_IP": "10.0.0.32",
                "WIFI_IP": "192.168.100.31",
                "ROS_NAMESPACE": "drone3",
                "MAV_SYS_ID": "3",
                "ROLE": "Follower",
            },
        }
        return presets.get(drone_id, presets[1])

    def apply_vehicle_preset(self, drone_id: int) -> bool:
        """기체 프리셋 적용"""
        preset = self.get_vehicle_preset(drone_id)
        for key, value in preset.items():
            self._device_config[key] = value
        return self.save_device_config()

    # === 비행 모드 파라미터 프리셋 ===

    def get_indoor_params(self) -> Dict[str, Any]:
        """실내 테스트 모드 파라미터"""
        return {
            # Position Estimator
            "EKF2_AID_MASK": 26,  # Optical Flow + Range Finder
            "EKF2_HGT_MODE": 2,  # Range Sensor (LiDAR)
            "EKF2_RNG_AID": 1,  # Range Finder 보조 활성화

            # Optical Flow
            "EKF2_OF_CTRL": 1,  # Optical Flow 활성화
            "EKF2_OF_DELAY": 20,  # 지연 (ms)
            "EKF2_OF_QMIN": 1,  # 최소 품질
            "EKF2_OF_N_MIN": 0.15,  # 최소 노이즈
            "EKF2_OF_N_MAX": 0.5,  # 최대 노이즈

            # Range Finder
            "EKF2_RNG_CTRL": 1,  # Range Finder 활성화
            "EKF2_RNG_A_HMAX": 5.0,  # 최대 고도 (m)
            "EKF2_RNG_NOISE": 0.1,  # 노이즈 (m)
            "SENS_EN_RANGEFNDR": 1,  # Range Finder 센서 활성화

            # GPS 비활성화
            "EKF2_GPS_CTRL": 0,  # GPS 비활성화
            "GPS_1_CONFIG": 0,  # GPS 포트 비활성화
        }

    def get_outdoor_gps_params(self) -> Dict[str, Any]:
        """야외 GPS 모드 파라미터"""
        return {
            # Position Estimator
            "EKF2_AID_MASK": 1,  # GPS 사용
            "EKF2_HGT_MODE": 1,  # GPS
            "EKF2_RNG_AID": 0,  # Range Finder 보조 비활성화

            # GPS
            "EKF2_GPS_CTRL": 7,  # GPS 활성화 (lon/lat/alt/vel)
            "GPS_1_CONFIG": 201,  # GPS UART 포트
            "EKF2_GPS_DELAY": 110,  # GPS 지연 (ms)

            # Optical Flow 비활성화
            "EKF2_OF_CTRL": 0,  # Optical Flow 비활성화
        }

    def get_outdoor_rtk_params(self) -> Dict[str, Any]:
        """야외 RTK 모드 파라미터"""
        params = self.get_outdoor_gps_params()
        params.update({
            "GPS_1_PROTOCOL": 1,  # uBlox
            "GPS_UBX_MODE": 2,  # RTK Float/Fixed
        })
        return params

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

    # === 페일세이프 설정 ===

    def get_failsafe_params(self) -> Dict[str, Any]:
        """기본 페일세이프 파라미터"""
        return {
            # OFFBOARD 모드 페일세이프
            "COM_OBL_RC_ACT": 5,  # OFFBOARD 두절 시 → Hold (Loiter)
            "COM_OF_LOSS_T": 1.0,  # OFFBOARD 두절 판정 시간 (초)

            # RC 페일세이프
            "COM_RC_LOSS_T": 0.5,  # RC 두절 판정 시간 (초)
            "NAV_RCL_ACT": 2,  # RC 두절 시 → RTL

            # 데이터링크 페일세이프
            "NAV_DLL_ACT": 0,  # 데이터링크 두절 시 → 동작 없음
            "COM_DL_LOSS_T": 10,  # 데이터링크 두절 판정 시간 (초)

            # 배터리 페일세이프
            "COM_LOW_BAT_ACT": 3,  # 저전압 시 → RTL
            "BAT_LOW_THR": 0.15,  # 15% 이하 경고
            "BAT_CRIT_THR": 0.07,  # 7% 이하 긴급 착륙
            "BAT_EMERGEN_THR": 0.05,  # 5% 이하 즉시 착륙
        }

    def get_indoor_failsafe_params(self) -> Dict[str, Any]:
        """실내 페일세이프 파라미터"""
        params = self.get_failsafe_params()
        params["COM_OBL_RC_ACT"] = 4  # Land (실내에서는 착륙)
        return params

    def get_outdoor_failsafe_params(self) -> Dict[str, Any]:
        """야외 페일세이프 파라미터"""
        params = self.get_failsafe_params()
        params["COM_OBL_RC_ACT"] = 3  # RTL (야외에서는 복귀)
        return params
