"""
시스템 상태 확인 모듈
"""

import os
import subprocess
import socket
import re
from typing import Optional, List, Dict, Tuple


class SystemChecker:
    """시스템 상태 확인"""

    def ping_host(self, host: str, timeout: int = 1) -> bool:
        """호스트 핑 테스트"""
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", str(timeout), host],
                capture_output=True,
                timeout=timeout + 1
            )
            return result.returncode == 0
        except Exception:
            return False

    def get_interface_ip(self, interface: str) -> Optional[str]:
        """네트워크 인터페이스 IP 주소 가져오기"""
        try:
            result = subprocess.run(
                ["ip", "addr", "show", interface],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                # inet 주소 추출
                match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', result.stdout)
                if match:
                    return match.group(1)
        except Exception:
            pass
        return None

    def is_service_running(self, service_name: str) -> bool:
        """systemd 서비스 실행 상태 확인"""
        try:
            result = subprocess.run(
                ["systemctl", "is-active", service_name],
                capture_output=True,
                text=True
            )
            return result.stdout.strip() == "active"
        except Exception:
            return False

    def get_service_status(self, service_name: str) -> str:
        """systemd 서비스 상태 가져오기"""
        try:
            result = subprocess.run(
                ["systemctl", "is-active", service_name],
                capture_output=True,
                text=True
            )
            return result.stdout.strip()
        except Exception:
            return "unknown"

    def get_ros2_topics(self) -> List[str]:
        """ROS2 토픽 목록 가져오기"""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return [t for t in result.stdout.strip().split("\n") if t]
        except Exception:
            pass
        return []

    def get_ros2_topic_hz(self, topic: str, timeout: float = 2.0) -> Optional[float]:
        """ROS2 토픽 주파수 측정"""
        try:
            result = subprocess.run(
                ["ros2", "topic", "hz", topic],
                capture_output=True,
                text=True,
                timeout=timeout
            )
            # average rate: 10.0 Hz 형식에서 추출
            match = re.search(r'average rate:\s+([\d.]+)', result.stdout)
            if match:
                return float(match.group(1))
        except Exception:
            pass
        return None

    def check_ros2_topic_exists(self, topic: str) -> bool:
        """ROS2 토픽 존재 여부 확인"""
        topics = self.get_ros2_topics()
        return topic in topics

    def get_cpu_usage(self) -> float:
        """CPU 사용량 (%)"""
        try:
            import psutil
            return psutil.cpu_percent(interval=0.1)
        except ImportError:
            try:
                result = subprocess.run(
                    ["grep", "cpu ", "/proc/stat"],
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    parts = result.stdout.strip().split()
                    idle = int(parts[4])
                    total = sum(int(x) for x in parts[1:])
                    return 100.0 * (1 - idle / total) if total > 0 else 0.0
            except Exception:
                pass
        return 0.0

    def get_memory_usage(self) -> Tuple[float, float, float]:
        """메모리 사용량 (used_gb, total_gb, percent)"""
        try:
            import psutil
            mem = psutil.virtual_memory()
            return (
                mem.used / (1024 ** 3),
                mem.total / (1024 ** 3),
                mem.percent
            )
        except ImportError:
            try:
                with open("/proc/meminfo", "r") as f:
                    meminfo = {}
                    for line in f:
                        parts = line.split()
                        if len(parts) >= 2:
                            meminfo[parts[0].rstrip(":")] = int(parts[1])

                    total = meminfo.get("MemTotal", 0) / (1024 ** 2)
                    available = meminfo.get("MemAvailable", 0) / (1024 ** 2)
                    used = total - available
                    percent = (used / total * 100) if total > 0 else 0

                    return used, total, percent
            except Exception:
                pass
        return 0.0, 0.0, 0.0

    def get_disk_usage(self, path: str = "/") -> Tuple[float, float, float]:
        """디스크 사용량 (used_gb, total_gb, percent)"""
        try:
            import psutil
            disk = psutil.disk_usage(path)
            return (
                disk.used / (1024 ** 3),
                disk.total / (1024 ** 3),
                disk.percent
            )
        except ImportError:
            try:
                result = subprocess.run(
                    ["df", "-B1", path],
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    lines = result.stdout.strip().split("\n")
                    if len(lines) >= 2:
                        parts = lines[1].split()
                        if len(parts) >= 4:
                            total = int(parts[1]) / (1024 ** 3)
                            used = int(parts[2]) / (1024 ** 3)
                            percent = (used / total * 100) if total > 0 else 0
                            return used, total, percent
            except Exception:
                pass
        return 0.0, 0.0, 0.0

    def check_process_running(self, process_name: str) -> bool:
        """프로세스 실행 여부 확인"""
        try:
            result = subprocess.run(
                ["pgrep", "-f", process_name],
                capture_output=True
            )
            return result.returncode == 0
        except Exception:
            return False

    def get_network_interfaces(self) -> Dict[str, Optional[str]]:
        """네트워크 인터페이스 목록 및 IP"""
        interfaces = {}
        for iface in ["eth0", "wlan0", "lo"]:
            interfaces[iface] = self.get_interface_ip(iface)
        return interfaces

    def run_command(self, command: List[str], timeout: int = 10) -> Tuple[int, str, str]:
        """명령어 실행"""
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return -1, "", "Timeout"
        except Exception as e:
            return -1, "", str(e)

    # === 체크리스트 관련 ===

    def check_fc_connection(self, fc_ip: str) -> Dict[str, any]:
        """FC 연결 상태 확인"""
        result = {
            "ping": self.ping_host(fc_ip),
            "ip": fc_ip,
        }
        return result

    def check_ros2_status(self) -> Dict[str, any]:
        """ROS2 상태 확인"""
        result = {
            "micro_ros_agent": self.is_service_running("micro-ros-agent"),
            "topics": self.get_ros2_topics(),
            "vehicle_status": "/fmu/out/vehicle_status" in self.get_ros2_topics(),
        }
        return result

    def check_network_status(self) -> Dict[str, any]:
        """네트워크 상태 확인"""
        interfaces = self.get_network_interfaces()
        result = {
            "eth0": interfaces.get("eth0"),
            "wlan0": interfaces.get("wlan0"),
            "eth0_up": interfaces.get("eth0") is not None,
            "wlan0_up": interfaces.get("wlan0") is not None,
        }
        return result

    def check_services_status(self) -> Dict[str, bool]:
        """서비스 상태 확인"""
        services = [
            "micro-ros-agent",
            "mavlink-router",
            "humiro-fire-suppression",
            "humiro-thermal-streaming",
        ]
        return {s: self.is_service_running(s) for s in services}
