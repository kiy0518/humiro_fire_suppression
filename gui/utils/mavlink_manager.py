"""
MAVLink 통신 관리 모듈
FC 파라미터 읽기/쓰기 및 재부팅 명령
"""

import time
import threading
from typing import Dict, Optional, Callable, Any
from pymavlink import mavutil


class MAVLinkManager:
    """MAVLink 통신 관리자"""

    def __init__(self, connection_string: str = "udp:10.0.0.12:14550"):
        self.connection_string = connection_string
        self.connection = None
        self.connected = False
        self._lock = threading.Lock()
        self._params_cache: Dict[str, float] = {}

    def connect(self, timeout: float = 5.0) -> bool:
        """FC에 연결"""
        try:
            with self._lock:
                if self.connection:
                    self.connection.close()

                self.connection = mavutil.mavlink_connection(
                    self.connection_string,
                    source_system=255,
                    source_component=190
                )

                # 하트비트 대기
                msg = self.connection.wait_heartbeat(timeout=timeout)
                if msg:
                    self.connected = True
                    print(f"MAVLink 연결 성공: sysid={self.connection.target_system}")
                    return True
                else:
                    self.connected = False
                    return False

        except Exception as e:
            print(f"MAVLink 연결 실패: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """연결 해제"""
        with self._lock:
            if self.connection:
                self.connection.close()
                self.connection = None
            self.connected = False

    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.connected and self.connection is not None

    def get_param(self, param_name: str, timeout: float = 3.0) -> Optional[float]:
        """파라미터 읽기"""
        if not self.is_connected():
            return None

        try:
            with self._lock:
                # 파라미터 요청
                self.connection.mav.param_request_read_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    param_name.encode('utf-8'),
                    -1  # param_index (-1 = use name)
                )

                # 응답 대기
                start = time.time()
                while time.time() - start < timeout:
                    msg = self.connection.recv_match(
                        type='PARAM_VALUE',
                        blocking=True,
                        timeout=0.5
                    )
                    if msg and msg.param_id == param_name:
                        self._params_cache[param_name] = msg.param_value
                        return msg.param_value

        except Exception as e:
            print(f"파라미터 읽기 실패 ({param_name}): {e}")

        return None

    def set_param(self, param_name: str, value: float, timeout: float = 3.0) -> bool:
        """파라미터 쓰기"""
        if not self.is_connected():
            return False

        try:
            with self._lock:
                # 파라미터 설정
                self.connection.mav.param_set_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    param_name.encode('utf-8'),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )

                # 확인 응답 대기
                start = time.time()
                while time.time() - start < timeout:
                    msg = self.connection.recv_match(
                        type='PARAM_VALUE',
                        blocking=True,
                        timeout=0.5
                    )
                    if msg and msg.param_id == param_name:
                        if abs(msg.param_value - value) < 0.001:
                            self._params_cache[param_name] = value
                            return True
                        else:
                            print(f"파라미터 설정 불일치: {param_name}={msg.param_value} (expected {value})")
                            return False

        except Exception as e:
            print(f"파라미터 쓰기 실패 ({param_name}): {e}")

        return False

    def get_params_batch(self, param_names: list, callback: Optional[Callable] = None) -> Dict[str, Optional[float]]:
        """여러 파라미터 일괄 읽기"""
        results = {}
        total = len(param_names)

        for i, name in enumerate(param_names):
            value = self.get_param(name)
            results[name] = value

            if callback:
                callback(i + 1, total, name, value)

        return results

    def set_params_batch(
        self,
        params: Dict[str, float],
        callback: Optional[Callable] = None
    ) -> Dict[str, bool]:
        """여러 파라미터 일괄 쓰기"""
        results = {}
        total = len(params)

        for i, (name, value) in enumerate(params.items()):
            success = self.set_param(name, value)
            results[name] = success

            if callback:
                callback(i + 1, total, name, success)

            # 파라미터 간 약간의 딜레이
            time.sleep(0.1)

        return results

    def save_params(self, timeout: float = 5.0) -> bool:
        """파라미터를 EEPROM에 저장"""
        if not self.is_connected():
            return False

        try:
            with self._lock:
                # MAV_CMD_PREFLIGHT_STORAGE: 파라미터 저장
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
                    0,  # confirmation
                    1,  # param1: 1 = Write parameters
                    0, 0, 0, 0, 0, 0
                )

                # 응답 대기
                start = time.time()
                while time.time() - start < timeout:
                    msg = self.connection.recv_match(
                        type='COMMAND_ACK',
                        blocking=True,
                        timeout=1.0
                    )
                    if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
                        return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

        except Exception as e:
            print(f"파라미터 저장 실패: {e}")

        return False

    def reboot_fc(self, timeout: float = 5.0) -> bool:
        """FC 재부팅"""
        if not self.is_connected():
            return False

        try:
            with self._lock:
                # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0,  # confirmation
                    1,  # param1: 1 = Reboot autopilot
                    0, 0, 0, 0, 0, 0
                )

                # 응답 대기 (재부팅 시 응답이 안 올 수 있음)
                start = time.time()
                while time.time() - start < timeout:
                    msg = self.connection.recv_match(
                        type='COMMAND_ACK',
                        blocking=True,
                        timeout=1.0
                    )
                    if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            self.connected = False
                            return True
                        else:
                            return False

                # 응답 없어도 재부팅 시도한 것으로 간주
                self.connected = False
                return True

        except Exception as e:
            print(f"FC 재부팅 실패: {e}")

        return False

    def get_cached_param(self, param_name: str) -> Optional[float]:
        """캐시된 파라미터 값 반환"""
        return self._params_cache.get(param_name)

    def clear_cache(self):
        """파라미터 캐시 초기화"""
        self._params_cache.clear()


# 싱글톤 인스턴스
_mavlink_manager: Optional[MAVLinkManager] = None


def get_mavlink_manager(connection_string: str = None) -> MAVLinkManager:
    """MAVLinkManager 싱글톤 인스턴스 반환"""
    global _mavlink_manager

    if _mavlink_manager is None:
        conn_str = connection_string or "udp:10.0.0.12:14550"
        _mavlink_manager = MAVLinkManager(conn_str)

    return _mavlink_manager
