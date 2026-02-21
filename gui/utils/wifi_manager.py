"""
WiFi 네트워크 관리 모듈
NetworkManager를 사용하여 저장된 WiFi 네트워크 관리
"""

import subprocess
import re
import threading
import logging
from typing import List, Dict, Optional, Tuple

logger = logging.getLogger(__name__)


class WiFiManager:
    """WiFi 네트워크 관리자 (NetworkManager 사용)"""

    REVERT_TIMEOUT = 60  # 자동 복귀 대기 시간 (초)

    def __init__(self):
        self.nmcli_path = "/usr/bin/nmcli"
        self._revert_timer: Optional[threading.Timer] = None
        self._revert_uuid: Optional[str] = None
        self._revert_name: Optional[str] = None

    def _run_command(self, cmd: List[str], timeout: int = 10) -> Tuple[bool, str, str]:
        """명령 실행 헬퍼"""
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "명령 시간 초과"
        except Exception as e:
            return False, "", str(e)

    def list_saved_networks(self) -> List[Dict[str, str]]:
        """저장된 WiFi 네트워크 목록 조회

        Returns:
            List of dicts with keys: name, uuid, type, device, active
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "-t", "-f", "NAME,UUID,TYPE,DEVICE", "connection", "show"
        ])

        if not success:
            return []

        networks = []
        for line in stdout.strip().split("\n"):
            if not line:
                continue

            parts = line.split(":")
            if len(parts) >= 3:
                conn_type = parts[2].strip()
                # WiFi 연결만 필터링
                if conn_type in ["802-11-wireless", "wifi"]:
                    networks.append({
                        "name": parts[0].strip(),
                        "uuid": parts[1].strip(),
                        "type": conn_type,
                        "device": parts[3].strip() if len(parts) > 3 else "",
                        "active": bool(parts[3].strip()) if len(parts) > 3 else False
                    })

        return networks

    def get_network_details(self, uuid_or_name: str) -> Optional[Dict[str, str]]:
        """네트워크 상세 정보 조회

        Args:
            uuid_or_name: Connection UUID or name

        Returns:
            Dict with connection details or None
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "show", uuid_or_name
        ])

        if not success:
            return None

        details = {}
        for line in stdout.strip().split("\n"):
            if ":" in line:
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()
                details[key] = value

        return details

    def delete_network(self, uuid_or_name: str) -> Tuple[bool, str]:
        """저장된 네트워크 삭제

        Args:
            uuid_or_name: Connection UUID or name

        Returns:
            (success, message)
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "delete", uuid_or_name
        ])

        if success:
            return True, f"네트워크 '{uuid_or_name}' 삭제 완료"
        else:
            return False, f"삭제 실패: {stderr}"

    def connect_to_network(self, uuid_or_name: str) -> Tuple[bool, str]:
        """네트워크에 연결 (auto-revert 포함)

        연결 후 60초 내에 confirm_connection()을 호출하지 않으면
        이전 네트워크로 자동 복귀합니다.

        Args:
            uuid_or_name: Connection UUID or name

        Returns:
            (success, message)
        """
        # 현재 연결 정보 저장 (복귀용)
        prev_uuid = self._get_active_wifi_uuid()

        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "up", uuid_or_name
        ], timeout=30)

        if success:
            self._start_revert_timer(prev_uuid)
            return True, f"네트워크 '{uuid_or_name}' 연결 완료"
        else:
            return False, f"연결 실패: {stderr}"

    def disconnect_network(self, uuid_or_name: str) -> Tuple[bool, str]:
        """네트워크 연결 해제

        Args:
            uuid_or_name: Connection UUID or name

        Returns:
            (success, message)
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "down", uuid_or_name
        ])

        if success:
            return True, f"네트워크 '{uuid_or_name}' 연결 해제 완료"
        else:
            return False, f"연결 해제 실패: {stderr}"

    def add_network(self, ssid: str, password: str, autoconnect: bool = True) -> Tuple[bool, str]:
        """새 WiFi 네트워크 추가 (추가 후 연결, auto-revert 포함)

        Args:
            ssid: WiFi SSID
            password: WiFi password
            autoconnect: Auto-connect on boot

        Returns:
            (success, message)
        """
        # 현재 연결 정보 저장 (복귀용)
        prev_uuid = self._get_active_wifi_uuid()

        cmd = [
            self.nmcli_path, "device", "wifi", "connect", ssid,
            "password", password
        ]

        if not autoconnect:
            cmd.extend(["autoconnect", "no"])

        success, stdout, stderr = self._run_command(cmd, timeout=30)

        if success:
            self._start_revert_timer(prev_uuid)
            return True, f"네트워크 '{ssid}' 추가 완료"
        else:
            return False, f"추가 실패: {stderr}"

    def modify_network_password(self, uuid_or_name: str, new_password: str) -> Tuple[bool, str]:
        """네트워크 비밀번호 수정

        Args:
            uuid_or_name: Connection UUID or name
            new_password: New WiFi password

        Returns:
            (success, message)
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "modify", uuid_or_name,
            "wifi-sec.psk", new_password
        ])

        if success:
            return True, f"네트워크 '{uuid_or_name}' 비밀번호 변경 완료"
        else:
            return False, f"비밀번호 변경 실패: {stderr}"

    def set_autoconnect(self, uuid_or_name: str, autoconnect: bool) -> Tuple[bool, str]:
        """자동 연결 설정 변경

        Args:
            uuid_or_name: Connection UUID or name
            autoconnect: Enable/disable auto-connect

        Returns:
            (success, message)
        """
        value = "yes" if autoconnect else "no"
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "modify", uuid_or_name,
            "connection.autoconnect", value
        ])

        if success:
            return True, f"자동 연결 {'활성화' if autoconnect else '비활성화'} 완료"
        else:
            return False, f"설정 변경 실패: {stderr}"

    def scan_available_networks(self) -> List[Dict[str, str]]:
        """주변 WiFi 네트워크 스캔

        Returns:
            List of dicts with keys: ssid, signal, security, in_use
        """
        # 스캔 요청
        self._run_command([self.nmcli_path, "device", "wifi", "rescan"])

        # 결과 조회
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE",
            "device", "wifi", "list"
        ])

        if not success:
            return []

        networks = []
        seen_ssids = set()

        for line in stdout.strip().split("\n"):
            if not line:
                continue

            parts = line.split(":")
            if len(parts) >= 3:
                ssid = parts[0].strip()
                if ssid and ssid not in seen_ssids:
                    networks.append({
                        "ssid": ssid,
                        "signal": parts[1].strip(),
                        "security": parts[2].strip(),
                        "in_use": parts[3].strip() == "*" if len(parts) > 3 else False
                    })
                    seen_ssids.add(ssid)

        # 신호 강도로 정렬
        networks.sort(key=lambda x: int(x["signal"]) if x["signal"].isdigit() else 0, reverse=True)

        return networks

    def get_current_connection(self) -> Optional[Dict[str, str]]:
        """현재 활성화된 WiFi 연결 정보

        Returns:
            Dict with connection info or None
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "-t", "-f", "NAME,TYPE,DEVICE",
            "connection", "show", "--active"
        ])

        if not success:
            return None

        for line in stdout.strip().split("\n"):
            if not line:
                continue

            parts = line.split(":")
            if len(parts) >= 3:
                conn_type = parts[1].strip()
                if conn_type in ["802-11-wireless", "wifi"]:
                    device = parts[2].strip()
                    info = {
                        "name": parts[0].strip(),
                        "type": conn_type,
                        "device": device,
                        "signal": 0,
                        "frequency": "",
                        "bitrate": "",
                        "channel": "",
                    }
                    # 신호 강도, 채널 조회
                    sig_ok, sig_out, _ = self._run_command([
                        self.nmcli_path, "-t", "-f",
                        "IN-USE,SSID,SIGNAL,FREQ,RATE,CHAN",
                        "device", "wifi", "list", "ifname", device
                    ])
                    if sig_ok:
                        for wline in sig_out.strip().split("\n"):
                            wparts = wline.split(":")
                            if len(wparts) >= 3 and wparts[0].strip() == "*":
                                try:
                                    info["signal"] = int(wparts[2].strip())
                                except (ValueError, IndexError):
                                    pass
                                if len(wparts) >= 4:
                                    info["frequency"] = wparts[3].strip()
                                if len(wparts) >= 5:
                                    info["bitrate"] = wparts[4].strip()
                                if len(wparts) >= 6:
                                    info["channel"] = wparts[5].strip()
                                break
                    # IP 주소 조회
                    ip_ok, ip_out, _ = self._run_command([
                        self.nmcli_path, "-t", "-f", "IP4.ADDRESS",
                        "device", "show", device
                    ])
                    if ip_ok:
                        for ip_line in ip_out.strip().split("\n"):
                            if ip_line.startswith("IP4.ADDRESS"):
                                addr = ip_line.split(":", 1)[1].strip()
                                # CIDR 제거 (예: 192.168.1.10/24 → 192.168.1.10)
                                info["ip_address"] = addr.split("/")[0] if "/" in addr else addr
                                break
                    return info

        return None

    def get_connection_priority(self, uuid_or_name: str) -> Optional[int]:
        """네트워크 우선순위 조회

        Args:
            uuid_or_name: Connection UUID or name

        Returns:
            Priority value or None
        """
        details = self.get_network_details(uuid_or_name)
        if details and "connection.autoconnect-priority" in details:
            try:
                return int(details["connection.autoconnect-priority"])
            except ValueError:
                pass
        return None

    def set_connection_priority(self, uuid_or_name: str, priority: int) -> Tuple[bool, str]:
        """네트워크 우선순위 설정

        Args:
            uuid_or_name: Connection UUID or name
            priority: Priority value (higher = more priority)

        Returns:
            (success, message)
        """
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "connection", "modify", uuid_or_name,
            "connection.autoconnect-priority", str(priority)
        ])

        if success:
            return True, f"우선순위 {priority}로 설정 완료"
        else:
            return False, f"설정 실패: {stderr}"

    # --- Auto-revert 메커니즘 ---

    def _get_active_wifi_uuid(self) -> Optional[str]:
        """현재 활성 WiFi 연결의 UUID 조회"""
        success, stdout, stderr = self._run_command([
            self.nmcli_path, "-t", "-f", "NAME,UUID,TYPE,DEVICE",
            "connection", "show", "--active"
        ])
        if not success:
            return None

        for line in stdout.strip().split("\n"):
            if not line:
                continue
            parts = line.split(":")
            if len(parts) >= 4:
                conn_type = parts[2].strip()
                if conn_type in ["802-11-wireless", "wifi"] and parts[3].strip():
                    return parts[1].strip()
        return None

    def _start_revert_timer(self, prev_uuid: Optional[str]):
        """이전 네트워크로 복귀하는 타이머 시작"""
        # 기존 타이머 취소
        self._cancel_revert_timer()

        if not prev_uuid:
            return

        # 이전 네트워크 이름 조회
        saved = self.list_saved_networks()
        prev_name = None
        for net in saved:
            if net["uuid"] == prev_uuid:
                prev_name = net["name"]
                break

        self._revert_uuid = prev_uuid
        self._revert_name = prev_name or prev_uuid
        self._revert_timer = threading.Timer(self.REVERT_TIMEOUT, self._do_revert)
        self._revert_timer.daemon = True
        self._revert_timer.start()
        logger.info(f"Auto-revert 타이머 시작: {self.REVERT_TIMEOUT}초 후 '{self._revert_name}'으로 복귀")

    def _cancel_revert_timer(self):
        """복귀 타이머 취소"""
        if self._revert_timer and self._revert_timer.is_alive():
            self._revert_timer.cancel()
        self._revert_timer = None
        self._revert_uuid = None
        self._revert_name = None

    def _do_revert(self):
        """이전 네트워크로 복귀 실행"""
        uuid = self._revert_uuid
        name = self._revert_name
        if uuid:
            logger.info(f"Auto-revert 실행: '{name}'으로 복귀")
            self._run_command([
                self.nmcli_path, "connection", "up", uuid
            ], timeout=30)
        self._revert_timer = None
        self._revert_uuid = None
        self._revert_name = None

    def confirm_connection(self) -> Tuple[bool, str]:
        """현재 연결 확인 (auto-revert 타이머 취소)"""
        if self._revert_timer and self._revert_timer.is_alive():
            self._cancel_revert_timer()
            return True, "연결 확인 완료 — 자동 복귀 취소됨"
        return True, "확인할 대기 중인 연결 없음"

    def is_revert_pending(self) -> bool:
        """auto-revert 대기 중 여부"""
        return self._revert_timer is not None and self._revert_timer.is_alive()

    def get_revert_info(self) -> Optional[Dict[str, str]]:
        """복귀 대기 정보 반환"""
        if self.is_revert_pending():
            return {
                "pending": True,
                "revert_to": self._revert_name or "",
                "timeout": self.REVERT_TIMEOUT
            }
        return {"pending": False}
