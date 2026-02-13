#!/usr/bin/env python3
"""
Humiro Fire Suppression - Web GUI
Flask 기반 웹 관리 도구
"""

import os
import sys
import json
import struct
import subprocess
import threading
import time
from datetime import datetime
from flask import Flask, render_template, jsonify, request, Response, stream_with_context, redirect

# 프로젝트 경로 설정
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.config_manager import ConfigManager
from utils.system_checker import SystemChecker
from utils.wifi_manager import WiFiManager

app = Flask(__name__)
app.config['SECRET_KEY'] = 'humiro-fire-suppression-2024'

# 전역 객체
config_manager = ConfigManager(PROJECT_ROOT)
system_checker = SystemChecker()
wifi_manager = WiFiManager()

@app.context_processor
def inject_drone_info():
    """모든 템플릿에 기체 정보 주입"""
    drone_id = config_manager.get_drone_id()
    role = config_manager.get('ROLE', 'Leader' if drone_id == 1 else 'Follower')
    drone_configs = config_manager.get_all_drone_configs()
    return {
        'drone_id': drone_id,
        'drone_role': role,
        'wifi_ip': config_manager.get_wifi_ip(),
        'fc_ip': config_manager.get_fc_ip(),
        'eth0_ip': config_manager.get_eth0_ip(),
        'drone_configs': drone_configs,
    }

# MAVLink CRC 계산 함수
def mavlink_crc16(data, crc=0xFFFF):
    """MAVLink CRC-16-CCITT-FALSE 계산"""
    for byte in data:
        byte = byte ^ (crc & 0xFF)
        byte = byte ^ ((byte << 4) & 0xFF)
        crc = ((crc >> 8) ^ (byte << 8) ^ (byte << 3) ^ (byte >> 4)) & 0xFFFF
    return crc

def calculate_mavlink_crc(message, crc_extra):
    """MAVLink 메시지 CRC 계산 (헤더[1:] + 페이로드 + CRC_EXTRA)"""
    # STX 바이트(첫 바이트)를 제외한 나머지로 CRC 계산
    crc = mavlink_crc16(message[1:], 0xFFFF)
    # CRC_EXTRA 추가
    crc = mavlink_crc16([crc_extra], crc)
    return crc

# 빌드 상태
build_status = {
    "running": False,
    "logs": [],
    "current_target": "",
    "success": None
}

def get_mavlink_tcp_port():
    """mavlink-router 설정에서 TCP 포트 읽기"""
    try:
        with open('/etc/mavlink-router/main.conf', 'r') as f:
            for line in f:
                if line.strip().startswith('TcpServerPort'):
                    return int(line.split('=')[1].strip())
    except:
        pass
    return 5790  # 기본값

# MAVLink 연결 관리자 (스레드 안전)
class MavlinkManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._mav = None
        self._target_sys = None
        self._target_comp = None
        self._last_connect = 0

    def _connect(self):
        """MAVLink 연결 (내부용)"""
        try:
            from pymavlink import mavutil
            if self._mav:
                try:
                    self._mav.close()
                except:
                    pass
            tcp_port = get_mavlink_tcp_port()
            self._mav = mavutil.mavlink_connection(
                f'tcp:127.0.0.1:{tcp_port}',
                source_system=255,
                source_component=190
            )
            # FC의 heartbeat만 대기 (자기 자신 제외)
            start = time.time()
            while time.time() - start < 5:
                msg = self._mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    src_sys = msg.get_srcSystem()
                    # FC heartbeat 찾기 (sys_id != 0 and != 255)
                    if src_sys not in [0, 255]:
                        self._target_sys = src_sys
                        self._target_comp = msg.get_srcComponent()
                        self._last_connect = time.time()
                        # 잔여 메시지 비우기
                        while self._mav.recv_match(blocking=False):
                            pass
                        return True
        except Exception as e:
            print(f"MAVLink 연결 실패: {e}")
        return False

    def read_param(self, param_name):
        """파라미터 읽기 (스레드 안전)"""
        with self._lock:
            # 연결 확인 및 재연결 (30초마다 또는 연결 없을 때)
            if not self._mav or (time.time() - self._last_connect) > 30:
                if not self._connect():
                    return None, "FC 연결 실패"

            try:
                # 잔여 메시지 비우기
                while self._mav.recv_match(blocking=False):
                    pass

                # 파라미터 요청
                self._mav.mav.param_request_read_send(
                    self._target_sys,
                    self._target_comp,
                    param_name.encode('utf-8'),
                    -1
                )

                # 응답 대기 (최대 2초)
                resp = self._mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if resp:
                    raw_value = resp.param_value
                    param_type = resp.param_type

                    # param_type: 1-6=정수, 9=FLOAT
                    # PX4는 정수 파라미터도 float로 전송하므로 비트 재해석 필요
                    # 단, 일부 파라미터(COM_DL_LOSS_T 등)는 type=6이지만 실제 float 값
                    if param_type in [1, 2, 3, 4, 5, 6]:
                        # 정수 타입: float 비트를 정수로 재해석
                        int_value = struct.unpack('I', struct.pack('f', raw_value))[0]

                        # raw_value가 합리적인 float 범위(0.001~100000)이고
                        # int_value가 매우 크면(>10M) → 실제 float 값으로 판단
                        # 예: raw_value=10.0, int_value=1092616192
                        if 0.001 <= abs(raw_value) <= 100000 and int_value > 10000000:
                            value = raw_value
                        else:
                            value = int_value
                            # 부호 처리
                            if param_type == 2 and value > 127:
                                value -= 256
                            elif param_type == 4 and value > 32767:
                                value -= 65536
                            elif param_type == 6 and value > 2147483647:
                                value -= 4294967296
                    else:
                        # FLOAT 타입 (type=9)
                        value = raw_value

                    return value, None
                else:
                    return None, "파라미터 없음"
            except Exception as e:
                # 연결 문제 시 다음 호출에 재연결
                self._mav = None
                return None, str(e)

    def is_armed(self):
        """FC arming 상태 확인. True=armed, False=disarmed, None=확인불가"""
        with self._lock:
            try:
                if not self._mav or (time.time() - self._last_connect) > 30:
                    if not self._connect():
                        return None
                # heartbeat에서 base_mode 확인
                msg = self._mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
                if msg and msg.get_srcSystem() not in [0, 255]:
                    return bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
                return None
            except:
                return None

mavlink_mgr = MavlinkManager()


# ==================== 페이지 라우트 ====================

@app.route('/')
def index():
    """대시보드 페이지"""
    return render_template('index.html', active_tab='dashboard')


@app.route('/config')
def config_page():
    """설정 페이지 → 기체 설정으로 리다이렉트"""
    return redirect('/vehicle-setup')


@app.route('/flight-mode')
def flight_mode_page():
    """비행 모드 페이지"""
    return render_template('flight_mode.html', active_tab='flight-mode')


@app.route('/vehicle-setup')
def vehicle_setup_page():
    """기체 설정 페이지"""
    return render_template('vehicle_setup.html', active_tab='vehicle-setup')


@app.route('/params')
def params_page():
    """FC 파라미터 관리 → 체크리스트로 리다이렉트"""
    return redirect('/checklist')


@app.route('/checklist')
def checklist_page():
    """체크리스트 페이지"""
    return render_template('checklist.html', active_tab='checklist')


@app.route('/build')
def build_page():
    """빌드 페이지"""
    return render_template('build.html', active_tab='build')


@app.route('/terminal')
def terminal_page():
    """웹 터미널 페이지"""
    return render_template('terminal.html', active_tab='terminal')


@app.route('/router')
def router_page():
    """MAVLink 라우터 페이지"""
    return render_template('router.html', active_tab='router')


@app.route('/micro-ros')
def micro_ros_page():
    """micro-ros-agent 테스트 페이지"""
    return render_template('micro_ros.html', active_tab='micro-ros')


@app.route('/camera')
def camera_page():
    """카메라 스트림 페이지"""
    return render_template('camera.html', active_tab='camera')


# ==================== API 라우트 ====================

@app.route('/api/status')
def api_status():
    """시스템 상태 API"""
    fc_ip = config_manager.get_fc_ip()
    drone_id = config_manager.get_drone_id()

    # FC 연결 상태
    fc_connected = system_checker.ping_host(fc_ip)

    # 네트워크 상태
    eth0_ip = system_checker.get_interface_ip("eth0")
    wlan0_ip = system_checker.get_interface_ip("wlan0")

    # ROS2 상태
    ros2_running = system_checker.is_service_running("micro-ros-agent")

    # 서비스 상태
    services = system_checker.check_services_status()

    # 시스템 리소스
    cpu_usage = system_checker.get_cpu_usage()
    mem_used, mem_total, mem_percent = system_checker.get_memory_usage()
    disk_used, disk_total, disk_percent = system_checker.get_disk_usage("/")

    # MAVLink 설정
    wifi_ip = config_manager.get_wifi_ip()
    mav_sys_id = int(config_manager.get("MAV_SYS_ID", str(drone_id)))
    mav_comp_id = int(config_manager.get("MAV_COMP_ID", "191"))
    # 앱 수신 포트 (mavlink-router Application 엔드포인트)
    external_port_str = config_manager.get("EXTERNAL_UDP_PORT", "15001").split('#')[0].strip()
    external_port = int(external_port_str)

    return jsonify({
        "drone_id": drone_id,
        "role": "Leader" if drone_id == 1 else "Follower",
        "fc": {
            "connected": fc_connected,
            "ip": fc_ip
        },
        "network": {
            "eth0": eth0_ip,
            "wlan0": wlan0_ip,
            "wifi_ip": wifi_ip
        },
        "mavlink": {
            "system_id": mav_sys_id,
            "component_id": mav_comp_id,
            "target_ip": wifi_ip,  # MAVLink router에 메시지를 보내기 위해 WiFi IP 사용
            "target_port": external_port
        },
        "ros2": {
            "running": ros2_running
        },
        "services": services,
        "resources": {
            "cpu": round(cpu_usage, 1),
            "memory": {
                "used": round(mem_used, 1),
                "total": round(mem_total, 1),
                "percent": round(mem_percent, 1)
            },
            "disk": {
                "used": round(disk_used, 1),
                "total": round(disk_total, 1),
                "percent": round(disk_percent, 1)
            }
        }
    })


@app.route('/api/system/voltage')
def api_system_voltage():
    """VIM4 입력 전압 조회 API"""
    try:
        # VIM4 SARADC 채널 2가 입력 전압 (분압 비율 적용 필요)
        with open('/sys/devices/platform/fe026000.saradc/iio:device0/in_voltage2_input', 'r') as f:
            raw_value = int(f.read().strip())
        # SARADC 값을 실제 전압으로 변환
        # raw 값 범위: 0-1023 (10-bit ADC), 기준 전압: 1.8V, 분압 비율: 약 6.67
        voltage = (raw_value / 1023.0) * 1.8 * 6.67
        return jsonify({
            "success": True,
            "voltage": round(voltage, 2),
            "raw": raw_value
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        })


@app.route('/api/config', methods=['GET'])
def api_get_config():
    """설정 조회 API"""
    config_manager.reload()
    return jsonify(config_manager.get_all_config())


@app.route('/api/config', methods=['POST'])
def api_set_config():
    """설정 저장 API"""
    data = request.json

    for key, value in data.items():
        config_manager.set(key, str(value))

    if config_manager.save_device_config():
        return jsonify({"success": True, "message": "설정이 저장되었습니다."})
    else:
        return jsonify({"success": False, "message": "설정 저장 실패"}), 500


def update_mavlink_router_gcs_port(qgc_port: int) -> bool:
    """mavlink-router 설정 파일에서 GCS 포트 업데이트

    Args:
        qgc_port: 새로운 QGC UDP 포트 번호

    Returns:
        성공 여부
    """
    config_path = '/etc/mavlink-router/main.conf'

    try:
        with open(config_path, 'r') as f:
            lines = f.readlines()

        # GCS 엔드포인트 섹션에서 Port 값 변경
        new_lines = []
        in_gcs_section = False

        for line in lines:
            stripped = line.strip()

            # GCS 섹션 시작 감지
            if stripped == '[UdpEndpoint GCS]':
                in_gcs_section = True
                new_lines.append(line)
            # 다른 섹션 시작 시 GCS 섹션 종료
            elif stripped.startswith('[') and stripped.endswith(']'):
                in_gcs_section = False
                new_lines.append(line)
            # GCS 섹션 내 Port 라인 수정
            elif in_gcs_section and stripped.startswith('Port'):
                new_lines.append(f'Port = {qgc_port}\n')
            else:
                new_lines.append(line)

        # sudo로 파일 쓰기 (권한 필요)
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.conf') as tmp:
            tmp.writelines(new_lines)
            tmp_path = tmp.name

        # sudo cp로 복사
        result = subprocess.run(
            ['sudo', 'cp', tmp_path, config_path],
            capture_output=True,
            text=True
        )
        os.unlink(tmp_path)

        return result.returncode == 0

    except Exception as e:
        print(f"mavlink-router 설정 업데이트 오류: {e}")
        return False


@app.route('/api/config/regenerate', methods=['POST'])
def api_config_regenerate():
    """DRONE_ID 기반 설정 재생성 API

    GCS_PORT_MODE 옵션 지원:
    - 'unified': 모든 기체가 14550 포트 사용
    - 'separate': 기체별 분리 포트 (1=14550, 2=14560, 3=14570)
    """
    data = request.json or {}
    drone_id = data.get('drone_id', config_manager.get_drone_id())
    gcs_port_mode = data.get('gcs_port_mode', 'separate')

    try:
        success = config_manager.regenerate_config_from_drone_id(
            drone_id=int(drone_id),
            gcs_port_mode=gcs_port_mode
        )

        if success:
            config_manager.reload()

            # mavlink-router 설정도 업데이트
            qgc_port = int(config_manager.get('QGC_UDP_PORT', 14550))
            router_updated = update_mavlink_router_gcs_port(qgc_port)

            return jsonify({
                "success": True,
                "message": f"설정이 재생성되었습니다. (DRONE_ID={drone_id}, GCS_PORT_MODE={gcs_port_mode})",
                "config": config_manager.get_all_config(),
                "router_config_updated": router_updated
            })
        else:
            return jsonify({
                "success": False,
                "error": "설정 저장 실패"
            }), 400
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/api/vehicle-preset/<int:drone_id>')
def api_vehicle_preset(drone_id):
    """기체 프리셋 조회 API"""
    preset = config_manager.get_vehicle_preset(drone_id)
    return jsonify(preset)


@app.route('/api/vehicle-preset/<int:drone_id>', methods=['POST'])
def api_apply_vehicle_preset(drone_id):
    """기체 프리셋 적용 API"""
    preset = config_manager.get_vehicle_preset(drone_id)

    for key, value in preset.items():
        config_manager.set(key, str(value))

    if config_manager.save_device_config():
        return jsonify({"success": True, "message": f"{drone_id}번 기체 설정이 적용되었습니다."})
    else:
        return jsonify({"success": False, "message": "설정 적용 실패"}), 500


@app.route('/api/apply-config', methods=['POST'])
def api_apply_config():
    """003-apply_config.sh 스크립트 실행 API (FC 재부팅 + VIM4 재부팅)"""
    try:
        auto_reboot = request.json.get('auto_reboot', False) if request.is_json else False

        script_path = os.path.join(PROJECT_ROOT, 'scripts', 'install', '003-apply_config.sh')

        if not os.path.exists(script_path):
            return jsonify({"success": False, "message": "003-apply_config.sh 스크립트를 찾을 수 없습니다."}), 404

        # 스크립트 실행 (sudo 필요)
        cmd = ['sudo', script_path]
        if auto_reboot:
            cmd.append('--auto-reboot')

        # 백그라운드에서 실행 (재부팅이 포함되므로)
        if auto_reboot:
            # nohup으로 백그라운드 실행
            subprocess.Popen(
                ['nohup'] + cmd,
                stdout=open('/tmp/apply_config.log', 'w'),
                stderr=subprocess.STDOUT,
                start_new_session=True
            )
            return jsonify({
                "success": True,
                "message": "설정 적용 시작됨. FC 재부팅 후 VIM4가 재부팅됩니다.",
                "log_file": "/tmp/apply_config.log"
            })
        else:
            # 동기 실행 (재부팅 없이)
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=120
            )

            if result.returncode == 0:
                return jsonify({
                    "success": True,
                    "message": "설정이 적용되었습니다.",
                    "output": result.stdout[-2000:] if len(result.stdout) > 2000 else result.stdout
                })
            else:
                return jsonify({
                    "success": False,
                    "message": "스크립트 실행 실패",
                    "error": result.stderr[-1000:] if len(result.stderr) > 1000 else result.stderr
                }), 500

    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "스크립트 실행 시간 초과"}), 500
    except Exception as e:
        return jsonify({"success": False, "message": f"오류: {str(e)}"}), 500


@app.route('/api/fc-params/<mode>')
def api_fc_params(mode):
    """FC 파라미터 조회 API"""
    if mode == "indoor":
        params = config_manager.get_indoor_params()
    elif mode == "outdoor_rtk":
        params = config_manager.get_outdoor_rtk_params()
    elif mode == "offboard_norc":
        params = config_manager.get_offboard_norc_params()
    elif mode == "failsafe":
        raw_params = config_manager.get_failsafe_params()
        # 튜플 키를 분리하여 설명 포함 형식으로 변환
        params = []
        for key, value in raw_params.items():
            param_name, description = key
            params.append({
                "name": param_name,
                "value": value,
                "description": description
            })
        return jsonify(params)
    else:
        params = config_manager.get_outdoor_gps_params()

    return jsonify(params)


@app.route('/api/fc/version')
def api_fc_version():
    """FC 펌웨어 버전 조회 API (MAVLink)"""
    try:
        from pymavlink import mavutil

        # MAVLink 연결 (mavlink-router TCP 포트 사용)
        tcp_port = get_mavlink_tcp_port()
        connection_string = f"tcp:127.0.0.1:{tcp_port}"
        mav = mavutil.mavlink_connection(connection_string, source_system=255, source_component=190)

        # 하트비트 대기 (FC 연결 확인)
        msg = mav.wait_heartbeat(timeout=3)
        if not msg:
            mav.close()
            return jsonify({
                "success": False,
                "message": "FC 연결 실패 (하트비트 없음)"
            })

        # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES 사용 (더 안정적)
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            0,  # confirmation
            1,  # param1: 1 = request capabilities
            0, 0, 0, 0, 0, 0
        )

        # 응답 대기
        version_msg = mav.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
        mav.close()

        if version_msg:
            # 펌웨어 버전 파싱
            fw_version = version_msg.flight_sw_version
            major = (fw_version >> 24) & 0xFF
            minor = (fw_version >> 16) & 0xFF
            patch = (fw_version >> 8) & 0xFF
            version_type = fw_version & 0xFF

            type_names = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "release"}
            type_name = type_names.get(version_type, f"type{version_type}")

            version_str = f"v{major}.{minor}.{patch}"
            if version_type != 255:
                version_str += f"-{type_name}"

            return jsonify({
                "success": True,
                "firmware_version": version_str,
                "board_version": version_msg.board_version,
                "raw": {
                    "flight_sw_version": fw_version,
                    "board_version": version_msg.board_version,
                    "vendor_id": version_msg.vendor_id,
                    "product_id": version_msg.product_id
                }
            })
        else:
            return jsonify({
                "success": False,
                "message": "버전 정보를 받지 못했습니다"
            })

    except ImportError:
        return jsonify({
            "success": False,
            "message": "pymavlink이 설치되지 않았습니다"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        })


@app.route('/api/fc/reboot', methods=['POST'])
def api_fc_reboot():
    """FC 재부팅 API (MAVLink)"""
    try:
        from pymavlink import mavutil

        tcp_port = get_mavlink_tcp_port()
        connection_string = f"tcp:127.0.0.1:{tcp_port}"
        mav = mavutil.mavlink_connection(connection_string, source_system=255, source_component=190)

        # 하트비트 대기
        msg = mav.wait_heartbeat(timeout=3)
        if not msg:
            mav.close()
            return jsonify({
                "success": False,
                "message": "FC 연결 실패"
            })

        # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        # param1: 1 = reboot autopilot
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # confirmation
            1,  # param1: 1 = Reboot autopilot
            0, 0, 0, 0, 0, 0
        )

        # ACK 대기 (재부팅 시 응답이 안 올 수 있음)
        ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        mav.close()

        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return jsonify({
                "success": True,
                "message": "FC 재부팅 명령 전송 완료"
            })
        else:
            # 재부팅 명령은 ACK 없이 바로 재부팅될 수 있음
            return jsonify({
                "success": True,
                "message": "FC 재부팅 명령 전송됨 (응답 없음 - 정상)"
            })

    except ImportError:
        return jsonify({
            "success": False,
            "message": "pymavlink이 설치되지 않았습니다"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        })


@app.route('/api/services')
def api_services():
    """서비스 상태 API"""
    services = [
        "micro-ros-agent",
        "mavlink-router",
        "humiro-fire-suppression",
    ]

    result = []
    for service in services:
        try:
            status_result = subprocess.run(
                ["systemctl", "is-active", service],
                capture_output=True, text=True, timeout=2
            )
            status = status_result.stdout.strip()
        except:
            status = "unknown"

        try:
            pid_result = subprocess.run(
                ["systemctl", "show", service, "--property=MainPID", "--value"],
                capture_output=True, text=True, timeout=2
            )
            pid = pid_result.stdout.strip()
            if pid == "0":
                pid = "-"
        except:
            pid = "-"

        result.append({
            "name": service,
            "status": status,
            "pid": pid
        })

    return jsonify(result)


@app.route('/api/service/<action>/<service>', methods=['POST'])
def api_service_control(action, service):
    """서비스 제어 API"""
    allowed_services = [
        "micro-ros-agent",
        "mavlink-router",
        "humiro-fire-suppression",
    ]

    if service not in allowed_services:
        return jsonify({"success": False, "message": "허용되지 않은 서비스"}), 400

    if action not in ["start", "stop", "restart"]:
        return jsonify({"success": False, "message": "허용되지 않은 액션"}), 400

    try:
        subprocess.run(
            ["sudo", "systemctl", action, service],
            check=True, timeout=10
        )
        return jsonify({"success": True, "message": f"{service} {action} 완료"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@app.route('/api/ros2/topics')
def api_ros2_topics():
    """ROS2 토픽 목록 API"""
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            topics = [t for t in result.stdout.strip().split("\n") if t]
            return jsonify({"success": True, "topics": topics})
        else:
            return jsonify({"success": False, "message": result.stderr})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/ros2/topic/echo', methods=['POST'])
def api_ros2_echo():
    """ROS2 토픽 Echo API"""
    topic = request.json.get("topic")
    if not topic:
        return jsonify({"success": False, "message": "토픽을 지정해주세요"})

    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            capture_output=True, text=True, timeout=5
        )
        return jsonify({
            "success": True,
            "data": result.stdout if result.stdout else "데이터 없음"
        })
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "Timeout: 토픽 데이터 없음"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/build/start', methods=['POST'])
def api_build_start():
    """빌드 시작 API"""
    global build_status

    if build_status["running"]:
        return jsonify({"success": False, "message": "빌드가 이미 실행 중입니다."})

    data = request.json
    targets = data.get("targets", [])
    clean_build = data.get("clean", False)

    if not targets:
        return jsonify({"success": False, "message": "빌드 타겟을 선택해주세요."})

    build_status = {
        "running": True,
        "logs": [f"빌드 시작: {', '.join(targets)}"],
        "current_target": "",
        "success": None
    }

    # 백그라운드에서 빌드 실행
    thread = threading.Thread(target=run_build, args=(targets, clean_build))
    thread.daemon = True
    thread.start()

    return jsonify({"success": True, "message": "빌드가 시작되었습니다."})


@app.route('/api/build/status')
def api_build_status():
    """빌드 상태 API"""
    return jsonify(build_status)


@app.route('/api/checklist/<mode>')
def api_checklist(mode):
    """체크리스트 항목 API"""
    no_rc = request.args.get('no_rc', 'false').lower() == 'true'

    if mode == "indoor":
        items = get_indoor_checklist(no_rc)
    elif mode == "outdoor_gps":
        items = get_outdoor_gps_checklist(no_rc)
    elif mode == "outdoor_rtk":
        items = get_outdoor_rtk_checklist(no_rc)
    elif mode == "postflight":
        items = get_postflight_checklist()
    else:
        items = get_postflight_checklist()

    return jsonify(items)


# ==================== 커스텀 파라미터 관리 API ====================

@app.route('/api/custom-params')
def api_get_custom_params():
    """커스텀 파라미터 전체 목록"""
    data = config_manager.load_custom_params()
    return jsonify(data)


@app.route('/api/custom-params/categories')
def api_get_categories():
    """카테고리 목록"""
    categories = config_manager.get_all_categories()
    return jsonify(categories)


@app.route('/api/custom-params/<category>')
def api_get_custom_params_by_category(category):
    """카테고리별 커스텀 파라미터 목록"""
    params = config_manager.get_custom_params_by_category(category)
    return jsonify(params)


@app.route('/api/custom-params/<category>', methods=['POST'])
def api_add_custom_param(category):
    """커스텀 파라미터 추가/수정

    Request Body:
    {
        "name": "PARAM_NAME",
        "expected": 123,
        "description": "설명",
        "auto_check": true
    }
    """
    param = request.get_json()
    if not param:
        return jsonify({"success": False, "message": "요청 데이터 없음"}), 400

    success = config_manager.add_custom_param(category, param)
    return jsonify({
        "success": success,
        "message": "파라미터 추가됨" if success else "추가 실패"
    })


@app.route('/api/custom-params/<category>/<param_name>', methods=['PUT'])
def api_update_custom_param(category, param_name):
    """커스텀 파라미터 업데이트"""
    param = request.get_json()
    if not param:
        return jsonify({"success": False, "message": "요청 데이터 없음"}), 400

    success = config_manager.update_custom_param(category, param_name, param)
    return jsonify({
        "success": success,
        "message": "파라미터 업데이트됨" if success else "업데이트 실패"
    })


@app.route('/api/custom-params/<category>/<param_name>', methods=['DELETE'])
def api_delete_custom_param(category, param_name):
    """커스텀 파라미터 삭제"""
    success = config_manager.delete_custom_param(category, param_name)
    return jsonify({
        "success": success,
        "message": "파라미터 삭제됨" if success else "삭제 실패"
    })


@app.route('/api/custom-params/read/<param_name>')
def api_read_custom_param(param_name):
    """FC에서 파라미터 값 읽기 (커스텀 파라미터용)"""
    value, error = mavlink_mgr.read_param(param_name)
    if error:
        return jsonify({"success": False, "message": error, "value": None})
    return jsonify({"success": True, "value": value, "message": str(value)})


@app.route('/api/check/fc-ping')
def api_check_fc_ping():
    """FC 핑 체크 API"""
    fc_ip = config_manager.get_fc_ip()
    result = system_checker.ping_host(fc_ip)
    return jsonify({
        "success": result,
        "message": "응답" if result else "응답 없음"
    })


@app.route('/api/drone/ping/<ip>')
def api_drone_ping(ip):
    """드론 미션 컴퓨터 연결 확인 API

    다른 기체의 GUI 서버(5000포트)로 연결 가능한지 확인
    """
    import socket

    # IP 주소 유효성 검증 (보안) - device_config.env 기반 동적 생성
    drone_configs = config_manager.get_all_drone_configs()
    allowed_ips = [d['ip'] for d in drone_configs.values()]
    if ip not in allowed_ips:
        return jsonify({
            "success": False,
            "message": "허용되지 않은 IP 주소"
        })

    # TCP 연결로 GUI 서버 확인 (더 빠르고 정확함)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.5)  # 1.5초 타임아웃
        result = sock.connect_ex((ip, 5000))
        sock.close()

        if result == 0:
            return jsonify({
                "success": True,
                "message": "연결 가능",
                "ip": ip
            })
        else:
            return jsonify({
                "success": False,
                "message": "GUI 서버 응답 없음",
                "ip": ip
            })
    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e),
            "ip": ip
        })


@app.route('/api/router/network-drones')
def api_router_network_drones():
    """네트워크에 연결된 모든 드론 상태 확인 API

    각 드론의 온라인 상태와 GCS 포트 사용 여부를 확인하여
    다이어그램에 표시할 수 있도록 정보 제공
    """
    import socket
    import requests

    # 현재 기체 ID (정수로 변환)
    current_drone_id = int(config_manager.get('DRONE_ID', 1))

    # GCS 포트 모드 확인
    gcs_port_mode = config_manager.get('GCS_PORT_MODE', 'separate')

    # 드론 네트워크 정보 - device_config.env 기반 동적 생성
    all_drone_configs = config_manager.get_all_drone_configs()
    drones = {
        did: {'ip': cfg['ip'], 'name': cfg['name'], 'gcs_port': cfg['gcs_port']}
        for did, cfg in all_drone_configs.items()
    }

    results = []

    for drone_id, info in drones.items():
        drone_status = {
            'id': drone_id,
            'ip': info['ip'],
            'name': info['name'],
            'gcs_port': info['gcs_port'],
            'online': False,
            'is_current': drone_id == current_drone_id,
            'gcs_active': False,
            'fc_connected': False
        }

        # 현재 기체는 항상 온라인
        if drone_id == current_drone_id:
            drone_status['online'] = True
            # 로컬 GCS 포트 상태 확인
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(0.1)
                result = sock.connect_ex(('127.0.0.1', info['gcs_port']))
                sock.close()
                drone_status['gcs_active'] = True  # 로컬이므로 활성 상태
            except:
                pass
        else:
            # 다른 기체 GUI 서버 연결 확인
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                result = sock.connect_ex((info['ip'], 5000))
                sock.close()
                drone_status['online'] = (result == 0)

                # 온라인이면 원격 상태 조회
                if drone_status['online']:
                    try:
                        resp = requests.get(
                            f"http://{info['ip']}:5000/api/status",
                            timeout=1.5
                        )
                        if resp.status_code == 200:
                            remote_status = resp.json()
                            drone_status['fc_connected'] = remote_status.get('fc', {}).get('connected', False)
                            drone_status['gcs_active'] = True
                    except:
                        pass
            except:
                pass

        results.append(drone_status)

    online_drones = [d for d in results if d['online']]

    # GCS 포트 모드에 따른 상태 결정
    current_drone_cfg = all_drone_configs.get(current_drone_id, {})
    if gcs_port_mode == 'unified':
        gcs_shared = len(online_drones) > 1
        current_gcs_port = ConfigManager.get_gcs_port_for_drone(current_drone_id, 'unified')
    else:
        gcs_shared = False
        current_gcs_port = current_drone_cfg.get('gcs_port', ConfigManager.get_gcs_port_for_drone(current_drone_id))

    broadcast_ip = config_manager.get_broadcast_ip()
    return jsonify({
        'success': True,
        'current_drone_id': current_drone_id,
        'gcs_port_mode': gcs_port_mode,
        'drones': results,
        'gcs_conflict': False,
        'gcs_shared': gcs_shared,
        'conflict_details': [],
        'gcs_port': current_gcs_port,
        'gcs_broadcast': f'{broadcast_ip}:{current_gcs_port}'
    })


def parse_mavlink_router_config():
    """mavlink-router 설정 파일 파싱"""
    config_path = '/etc/mavlink-router/main.conf'
    endpoints = []
    general = {}

    try:
        with open(config_path, 'r') as f:
            current_section = None
            current_data = {}

            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                # 섹션 헤더
                if line.startswith('[') and line.endswith(']'):
                    # 이전 섹션 저장
                    if current_section:
                        if current_section == 'General':
                            general = current_data.copy()
                        else:
                            current_data['section'] = current_section
                            endpoints.append(current_data.copy())

                    current_section = line[1:-1]
                    current_data = {}
                elif '=' in line:
                    key, value = line.split('=', 1)
                    current_data[key.strip()] = value.strip()

            # 마지막 섹션 저장
            if current_section:
                if current_section == 'General':
                    general = current_data.copy()
                else:
                    current_data['section'] = current_section
                    endpoints.append(current_data.copy())

    except Exception as e:
        print(f"설정 파일 파싱 오류: {e}")

    return general, endpoints


@app.route('/api/router/config')
def api_router_config():
    """MAVLink 라우터 설정 파일 API"""
    general, endpoints = parse_mavlink_router_config()

    # 엔드포인트 정보 정리
    nodes = []
    for ep in endpoints:
        section = ep.get('section', '')
        # UdpEndpoint NAME 형식에서 이름 추출
        if section.startswith('UdpEndpoint '):
            name = section.replace('UdpEndpoint ', '')
            # GCS는 QGC로 이름 변경하고 양방향 (모든 기체 14550 공유)
            if name == 'GCS':
                node = {
                    'id': 'gcs',  # ID는 유지 (기존 호환성)
                    'name': 'QGC',
                    'type': 'udp',
                    'port': 14550,  # 모든 기체 공통 포트
                    'address': ep.get('Address', ''),
                    'mode': ep.get('Mode', 'Normal'),
                    'direction': 'both'  # 양방향
                }
            else:
                node = {
                    'id': name.lower().replace(' ', '_'),
                    'name': name,
                    'type': 'udp',
                    'port': int(ep.get('Port', 0)),
                    'address': ep.get('Address', ''),
                    'mode': ep.get('Mode', 'Normal'),
                    'direction': 'in' if ep.get('Mode') == 'Server' else 'out'
                }
            nodes.append(node)
        elif section.startswith('TcpEndpoint '):
            name = section.replace('TcpEndpoint ', '')
            node = {
                'id': name.lower().replace(' ', '_'),
                'name': name,
                'type': 'tcp',
                'port': int(ep.get('Port', 0)),
                'address': ep.get('Address', ''),
                'mode': ep.get('Mode', 'Normal'),
                'direction': 'both'
            }
            nodes.append(node)

    # TCP 서버 포트 추가 (General 섹션)
    tcp_port = int(general.get('TcpServerPort', 5790))

    return jsonify({
        'general': general,
        'tcp_server_port': tcp_port,
        'nodes': nodes
    })


@app.route('/api/router/status')
def api_router_status():
    """MAVLink 라우터 상태 API"""
    import socket

    # 서비스 실행 상태
    service_running = system_checker.is_service_running("mavlink-router")

    # 설정 파일에서 엔드포인트 정보 가져오기
    general, endpoints = parse_mavlink_router_config()
    tcp_server_port = int(general.get('TcpServerPort', 5790))

    # ss 명령 한 번만 실행
    try:
        ss_result = subprocess.run(
            ['ss', '-uln'],
            capture_output=True, text=True, timeout=2
        )
        ss_output = ss_result.stdout
    except:
        ss_output = ''

    # 각 포트 상태 확인
    ports = {}

    for ep in endpoints:
        section = ep.get('section', '')
        if section.startswith('UdpEndpoint '):
            name = section.replace('UdpEndpoint ', '')
            port_id = name.lower().replace(' ', '_')
            port_num = int(ep.get('Port', 0))
            mode = ep.get('Mode', 'Normal')

            ports[port_id] = {'connected': False, 'message': '', 'port': port_num}

            if mode == 'Server':
                # Server 모드: 포트 리슨 상태 확인
                if f':{port_num}' in ss_output:
                    ports[port_id]['connected'] = True
                    ports[port_id]['message'] = '수신 대기 중'
            elif port_id == 'pc_sitl':
                # SITL 모드: PC에 ping으로 연결 상태 확인
                address = ep.get('Address', '')
                if address and address != '0.0.0.0':
                    try:
                        ping_result = subprocess.run(
                            ['ping', '-c', '1', '-W', '1', address],
                            capture_output=True, timeout=2
                        )
                        if ping_result.returncode == 0:
                            ports[port_id]['connected'] = True
                            ports[port_id]['message'] = f'{address} 연결됨'
                        else:
                            ports[port_id]['connected'] = False
                            ports[port_id]['message'] = f'{address} 연결 안됨'
                    except:
                        ports[port_id]['connected'] = False
                        ports[port_id]['message'] = f'{address} 확인 실패'
                else:
                    ports[port_id]['connected'] = service_running
                    ports[port_id]['message'] = '송신 중' if service_running else ''
            else:
                # Normal 모드 (브로드캐스트/유니캐스트): 서비스 실행 여부로 판단
                ports[port_id]['connected'] = service_running
                ports[port_id]['message'] = '송신 중' if service_running else ''

    # TCP 서버 상태
    ports['tcp'] = {'connected': False, 'message': '', 'port': tcp_server_port}
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('127.0.0.1', tcp_server_port))
        sock.close()
        ports['tcp']['connected'] = (result == 0)
        ports['tcp']['message'] = '연결 가능' if result == 0 else '연결 불가'
    except:
        pass

    return jsonify({
        "service_running": service_running,
        "ports": ports
    })


@app.route('/api/router/test/<port_id>')
def api_router_test(port_id):
    """개별 포트 연결 테스트 API"""
    import socket

    # 설정 파일에서 포트 정보 가져오기
    general, endpoints = parse_mavlink_router_config()
    tcp_server_port = int(general.get('TcpServerPort', 5790))

    # TCP 서버 테스트
    if port_id == 'tcp':
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex(('127.0.0.1', tcp_server_port))
            sock.close()
            if result == 0:
                return jsonify({'connected': True, 'message': 'TCP 연결 성공'})
            else:
                return jsonify({'connected': False, 'message': 'TCP 연결 실패'})
        except Exception as e:
            return jsonify({'connected': False, 'message': str(e)})

    # UDP 엔드포인트 찾기
    for ep in endpoints:
        section = ep.get('section', '')
        if section.startswith('UdpEndpoint '):
            name = section.replace('UdpEndpoint ', '')
            ep_id = name.lower().replace(' ', '_')

            if ep_id == port_id:
                port_num = int(ep.get('Port', 0))
                mode = ep.get('Mode', 'Normal')

                if mode == 'Server':
                    # Server 모드: 포트 리슨 상태 확인
                    try:
                        result = subprocess.run(
                            ['ss', '-uln'],
                            capture_output=True, text=True, timeout=2
                        )
                        if f':{port_num}' in result.stdout:
                            return jsonify({'connected': True, 'message': 'UDP 포트 활성'})
                        else:
                            return jsonify({'connected': False, 'message': 'UDP 포트 비활성'})
                    except Exception as e:
                        return jsonify({'connected': False, 'message': str(e)})
                else:
                    # Normal 모드: 서비스 실행 여부로 판단
                    if system_checker.is_service_running("mavlink-router"):
                        return jsonify({'connected': True, 'message': '브로드캐스트/유니캐스트 송신 중'})
                    else:
                        return jsonify({'connected': False, 'message': '서비스 중지됨'})

    return jsonify({'connected': False, 'message': '알 수 없는 포트'})


@app.route('/api/router/port-scan')
def api_router_port_scan():
    """네트워크 내 MAVLink 포트 스캔 및 충돌 감지"""
    try:
        import socket

        # 드론 ID 기반 포트 계산
        drone_id = config_manager.get_drone_id()
        port_offset = (drone_id - 1) * 10

        # 스캔할 포트 목록 (드론별 동적 계산)
        qgc_port = 14550 + port_offset
        ros2_port = 14551 + port_offset
        app_port = 15001 + port_offset
        tcp_port = 5790 + port_offset

        ports_to_scan = [14540, qgc_port, ros2_port, 14553, app_port, tcp_port]
        port_names = {
            14540: 'FC',
            qgc_port: 'QGC',
            ros2_port: 'ROS2',
            14553: 'External',
            app_port: 'App',
            tcp_port: 'TCP Server'
        }

        # WiFi 서브넷 확인
        wifi_ip = config_manager.get_wifi_ip()
        if not wifi_ip or wifi_ip == "127.0.0.1":
            return jsonify({
                'success': False,
                'error': 'WiFi IP를 찾을 수 없습니다'
            })

        # IP 범위 계산
        ip_parts = wifi_ip.split('.')
        subnet = '.'.join(ip_parts[:3])

        results = []

        # 로컬 호스트 상세 스캔
        local_ports = {}
        for port in ports_to_scan:
            # UDP 포트 리슨 확인
            udp_result = subprocess.run(
                ['ss', '-uln'],
                capture_output=True, text=True, timeout=2
            )
            # TCP 포트 리슨 확인
            tcp_result = subprocess.run(
                ['ss', '-tln'],
                capture_output=True, text=True, timeout=2
            )

            udp_listening = f':{port}' in udp_result.stdout
            tcp_listening = f':{port}' in tcp_result.stdout

            # 프로세스 확인 (sudo 사용)
            lsof_result = subprocess.run(
                ['sudo', 'lsof', '-i', f':{port}', '-n', '-P'],
                capture_output=True, text=True, timeout=2
            )
            processes = []
            for line in lsof_result.stdout.split('\n')[1:]:  # 헤더 스킵
                if line.strip():
                    parts = line.split()
                    if len(parts) >= 2:
                        proc_name = parts[0]
                        proc_pid = parts[1]
                        # COMMAND 필드에서 실제 명령어 이름 추출
                        processes.append(f"{proc_name} (PID:{proc_pid})")

            local_ports[port] = {
                'name': port_names.get(port, str(port)),
                'udp': udp_listening,
                'tcp': tcp_listening,
                'processes': processes
            }

        results.append({
            'ip': wifi_ip,
            'hostname': socket.gethostname(),
            'ports': local_ports,
            'is_local': True
        })

        return jsonify({
            'success': True,
            'results': results,
            'local_ip': wifi_ip
        })
    except Exception as e:
        import traceback
        return jsonify({
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc()
        })


@app.route('/api/router/messages')
def api_router_messages():
    """실시간 MAVLink 메시지 모니터링 API"""
    try:
        # humiro-fire-suppression 서비스 로그에서 MAVLink 메시지 추출
        result = subprocess.run(
            ['journalctl', '-u', 'humiro-fire-suppression', '-n', '50', '--no-pager', '--since', '5 seconds ago'],
            capture_output=True, text=True, timeout=3
        )

        messages = []
        for line in result.stdout.split('\n'):
            # CustomMessage 로그 파싱
            if '[CustomMessage]' in line and 'UDP 수신' in line:
                # 예: [CustomMessage] [STEP 1] UDP 수신 (epoll): 28 bytes, 첫 바이트: 0xfd, MSG_ID=60000, 포트: 14551
                parts = line.split('MSG_ID=')
                if len(parts) > 1:
                    msg_id_part = parts[1].split(',')[0].strip()
                    port_part = line.split('포트: ')
                    port = port_part[1].split(',')[0].strip() if len(port_part) > 1 else 'unknown'

                    # 메시지 타입 매핑
                    msg_types = {
                        '60000': 'FIRE_MISSION_START',
                        '60001': 'FIRE_AUTO_AIM',
                        '60002': 'FIRE_LAUNCH',
                        '60003': 'FIRE_RETURN',
                        '76': 'COMMAND_LONG',
                        '0': 'HEARTBEAT'
                    }

                    msg_type = msg_types.get(msg_id_part, f'MSG_{msg_id_part}')

                    messages.append({
                        'port': port,
                        'type': msg_type,
                        'description': f'ID={msg_id_part}'
                    })

            # FIRE_MISSION_START 파싱
            elif 'FIRE_MISSION_START 수신' in line:
                messages.append({
                    'port': '14551',  # ROS2 포트에서 수신됨
                    'type': 'FIRE_MISSION_START',
                    'description': '화재 진압 미션 시작 명령'
                })

            # COMMAND_LONG 파싱
            elif 'COMMAND_LONG' in line:
                messages.append({
                    'port': 'unknown',
                    'type': 'COMMAND_LONG',
                    'description': '비행 제어 명령'
                })

        return jsonify({
            'success': True,
            'messages': messages,
            'count': len(messages)
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'messages': []
        })

@app.route('/api/router/port-reset')
def api_router_port_reset():
    """포트 충돌 해결 - MAVLink 라우터 및 관련 서비스 재시작"""
    try:
        steps = []

        # 1. humiro-fire-suppression 서비스 중지
        steps.append('1. humiro-fire-suppression 서비스 중지 중...')
        result = subprocess.run(
            ['sudo', 'systemctl', 'stop', 'humiro-fire-suppression'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            steps.append('   ✓ humiro-fire-suppression 중지 완료')
        else:
            steps.append(f'   ⚠ humiro-fire-suppression 중지 실패: {result.stderr}')

        # 2. mavlink-router 서비스 재시작
        steps.append('2. mavlink-router 서비스 재시작 중...')
        result = subprocess.run(
            ['sudo', 'systemctl', 'restart', 'mavlink-router'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            steps.append('   ✓ mavlink-router 재시작 완료')
        else:
            steps.append(f'   ⚠ mavlink-router 재시작 실패: {result.stderr}')

        # 3. 2초 대기 (라우터 초기화 대기)
        time.sleep(2)
        steps.append('3. 라우터 초기화 대기 (2초)...')

        # 4. humiro-fire-suppression 서비스 시작
        steps.append('4. humiro-fire-suppression 서비스 시작 중...')
        result = subprocess.run(
            ['sudo', 'systemctl', 'start', 'humiro-fire-suppression'],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            steps.append('   ✓ humiro-fire-suppression 시작 완료')
        else:
            steps.append(f'   ⚠ humiro-fire-suppression 시작 실패: {result.stderr}')

        # 5. 포트 상태 확인
        steps.append('5. 포트 상태 확인 중...')
        time.sleep(1)

        # 드론 ID 기반 포트 계산
        drone_id = config_manager.get_drone_id()
        port_offset = (drone_id - 1) * 10
        ros2_port = 14551 + port_offset
        app_port = 15001 + port_offset

        # ROS2 포트 확인 (드론별 동적)
        lsof_result = subprocess.run(
            ['sudo', 'lsof', '-i', f':{ros2_port}', '-n', '-P'],
            capture_output=True, text=True, timeout=3
        )

        if str(ros2_port) in lsof_result.stdout:
            processes = []
            for line in lsof_result.stdout.split('\n')[1:]:
                if line.strip():
                    parts = line.split()
                    if len(parts) > 1:
                        processes.append(parts[0])

            if len(processes) == 1 and 'mavlink' in processes[0].lower():
                steps.append(f'   ✓ {ros2_port} 포트: MAVLink 라우터만 사용 중 (정상)')
            else:
                steps.append(f'   ⚠ {ros2_port} 포트: 여러 프로세스 사용 중 ({", ".join(processes)})')
        else:
            steps.append(f'   ⚠ {ros2_port} 포트: 아무도 사용하지 않음 (MAVLink 라우터 미연결)')

        # Application 포트 확인 (드론별 동적)
        lsof_result = subprocess.run(
            ['sudo', 'lsof', '-i', f':{app_port}', '-n', '-P'],
            capture_output=True, text=True, timeout=3
        )

        if str(app_port) in lsof_result.stdout:
            steps.append(f'   ✓ {app_port} 포트: application_manager 사용 중 (정상)')
        else:
            steps.append(f'   ⚠ {app_port} 포트: 아무도 사용하지 않음')

        steps.append('')
        steps.append('✅ 포트 리셋 완료!')
        steps.append('')
        steps.append('💡 참고: FC와의 통신이 여전히 끊어진 경우:')
        steps.append('   1. QGC를 재시작하세요')
        steps.append('   2. 드론의 FC 재부팅을 시도하세요 (전원 재인가)')

        return jsonify({
            'success': True,
            'steps': steps
        })

    except Exception as e:
        import traceback
        return jsonify({
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc()
        })


# ==================== SITL 모드 API ====================

def get_sitl_mode_status():
    """현재 mavlink-router가 SITL 모드인지 FC 모드인지 확인"""
    config_path = '/etc/mavlink-router/main.conf'

    try:
        with open(config_path, 'r') as f:
            content = f.read()

        # SITL 모드 판별: [UdpEndpoint SITL] 엔드포인트가 있으면 SITL 모드
        has_sitl_endpoint = '[UdpEndpoint SITL]' in content or '[UdpEndpoint PC_SITL]' in content
        has_fc_endpoint = '[UdpEndpoint FC]' in content
        is_sitl_comment = '시뮬레이션 PC IP' in content

        if has_sitl_endpoint or is_sitl_comment:
            # SITL IP 추출 시도 - 주석에서 "시뮬레이션 PC IP: x.x.x.x" 형식 찾기
            sitl_ip = None
            import re
            
            # 방법 1: 주석에서 "시뮬레이션 PC IP: x.x.x.x" 패턴 찾기
            match = re.search(r'시뮬레이션 PC IP[:\s]+(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', content)
            if match:
                sitl_ip = match.group(1)
            else:
                # 방법 2: PC_SITL 엔드포인트의 Address 찾기 (0.0.0.0이 아닌 경우)
                match = re.search(r'\[UdpEndpoint PC_SITL\].*?Address\s*=\s*(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', content, re.DOTALL)
                if match and match.group(1) != '0.0.0.0':
                    sitl_ip = match.group(1)
            
            return {
                'mode': 'sitl',
                'sitl_ip': sitl_ip,
                'description': 'SITL 시뮬레이션 모드'
            }
        elif has_fc_endpoint and not has_sitl_endpoint:
            return {
                'mode': 'fc',
                'sitl_ip': None,
                'description': '실제 FC 연결 모드'
            }
        else:
            return {
                'mode': 'unknown',
                'sitl_ip': None,
                'description': '알 수 없음'
            }
    except Exception as e:
        return {
            'mode': 'error',
            'sitl_ip': None,
            'description': f'설정 파일 읽기 오류: {e}'
        }


@app.route('/api/router/sitl-mode')
def api_router_sitl_mode():
    """현재 SITL/FC 모드 조회"""
    status = get_sitl_mode_status()
    return jsonify({
        'success': True,
        'is_sitl_mode': status['mode'] == 'sitl',
        **status
    })


@app.route('/api/router/sitl-instances')
def api_router_sitl_instances():
    """SITL 모드일 때 모든 SITL 인스턴스(1, 2, 3번) 상태 조회

    PC에서 실행 중인 SITL 인스턴스들의 연결 상태를 확인합니다.
    각 인스턴스는 14540, 14541, 14542 포트를 사용합니다.
    """
    status = get_sitl_mode_status()

    if status['mode'] != 'sitl':
        return jsonify({
            'success': True,
            'is_sitl_mode': False,
            'instances': []
        })

    sitl_ip = status.get('sitl_ip', '192.168.100.4')
    instances = []

    # SITL 인스턴스 1, 2, 3 확인 (포트 14540, 14541, 14542)
    for i in range(1, 4):
        port = 14540 + i - 1
        instance = {
            'id': i,
            'port': port,
            'ip': sitl_ip,
            'connected': False,
            'message': '확인 중...'
        }

        # PC에 ping으로 연결 확인 (SITL IP 도달 가능 여부)
        try:
            ping_result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', sitl_ip],
                capture_output=True, timeout=2
            )
            if ping_result.returncode == 0:
                # ping 성공 - PC 연결됨, 포트 연결 상태는 netcat으로 확인
                try:
                    # UDP 포트 테스트는 어려우므로, TCP 연결 또는 MAVLink 메시지로 확인해야 함
                    # 간단히 ping 성공이면 "PC 연결됨"으로 표시
                    instance['connected'] = True
                    instance['message'] = f'{sitl_ip}:{port} 연결됨'
                except:
                    instance['message'] = f'{sitl_ip} 도달 가능 (포트 미확인)'
            else:
                instance['message'] = f'{sitl_ip} 연결 안됨'
        except Exception as e:
            instance['message'] = f'연결 확인 오류: {str(e)}'

        instances.append(instance)

    return jsonify({
        'success': True,
        'is_sitl_mode': True,
        'sitl_ip': sitl_ip,
        'instances': instances
    })


@app.route('/api/router/sitl-mode', methods=['POST'])
def api_router_set_sitl_mode():
    """SITL/FC 모드 전환

    요청 JSON:
    - mode: 'sitl' 또는 'fc'
    - sitl_ip: SITL 모드일 때 PC IP 주소 (필수)
    """
    data = request.json or {}
    mode = data.get('mode', 'fc')
    sitl_ip = data.get('sitl_ip', '192.168.100.4')

    # DISARM 상태 확인 (armed 상태에서는 모드 전환 불가)
    armed = mavlink_mgr.is_armed()
    if armed is True:
        return jsonify({
            'success': False,
            'error': 'FC가 Armed 상태입니다. Disarm 후 모드를 전환하세요.'
        }), 400

    drone_id = config_manager.get_drone_id()
    qgc_port = int(config_manager.get('QGC_UDP_PORT', str(ConfigManager.get_gcs_port_for_drone(drone_id))))

    # 드론별 포트 계산 (003-apply_config.sh와 동일 공식)
    port_offset = (drone_id - 1) * 10
    external_port = 16001 + port_offset
    application_port = 15001 + port_offset
    ros2_port = 14551 + port_offset
    tcp_port = 5790 + port_offset
    fc_mavlink_port = int(config_manager.get('FC_MAVLINK_PORT', '14540'))
    sitl_port = 18000 + drone_id  # SITL 포트: 18001, 18002, 18003 (socat 포워딩용)

    try:
        if mode == 'sitl':
            # SITL 모드 설정 - Server 모드로 PC의 SITL에서 연결 받음
            config_content = f"""# ==============================================================================
# MAVLink Router 설정 - SITL 시뮬레이션 전용
# 생성: {subprocess.check_output(['date']).decode().strip()}
# Drone ID: {drone_id}
# VIM4 IP: {config_manager.get_wifi_ip()}
# 시뮬레이션 PC IP: {sitl_ip}
# ==============================================================================

[General]
TcpServerPort = {tcp_port}
ReportStats = false
MavlinkDialect = common

# SITL 연결 (시뮬레이션용) - WiFi (192.168.100.x)
# 시뮬레이션 PC에서: mavlink start -u {fc_mavlink_port} -o {sitl_port} -t {config_manager.get_wifi_ip()}
[UdpEndpoint SITL]
Mode = Server
Address = 0.0.0.0
Port = {sitl_port}

# GCS (QGroundControl) - 브로드캐스트
[UdpEndpoint GCS]
Mode = Normal
Address = {config_manager.get_broadcast_ip()}
Port = {qgc_port}

# External (커스텀 메시지 테스트용)
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = {external_port}

# ROS2 노드 연결 (로컬)
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = {ros2_port}

# Application Manager 연결 (로컬)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = {application_port}
"""
        else:
            # FC 모드 설정
            config_content = f"""# Drone #{drone_id} mavlink-router 설정 (자동 생성)
# 생성: {subprocess.check_output(['date']).decode().strip()}
# 모드: FC 연결 (실제 비행)

[General]
TcpServerPort = {tcp_port}
ReportStats = false
MavlinkDialect = common

# FC (PX4) 연결 - 이더넷 (10.0.0.x)
[UdpEndpoint FC]
Mode = Server
Address = 0.0.0.0
Port = {fc_mavlink_port}

# GCS (QGroundControl) 브로드캐스트 - 드론 {drone_id} 전용 포트
[UdpEndpoint GCS]
Mode = Normal
Address = {config_manager.get_broadcast_ip()}
Port = {qgc_port}

# 외부 테스트/디버깅 도구 (SENDER GUI 등) - 드론 {drone_id} 전용 포트
[UdpEndpoint External]
Mode = Server
Address = 0.0.0.0
Port = {external_port}

# ROS2 노드 연결 - 드론 {drone_id} 전용 포트
[UdpEndpoint ROS2]
Mode = Normal
Address = 127.0.0.1
Port = {ros2_port}

# Application Manager 연결 - 드론 {drone_id} 전용 포트 (라우터와 충돌 방지)
[UdpEndpoint Application]
Mode = Normal
Address = 127.0.0.1
Port = {application_port}
"""

        # 백업 생성
        config_path = '/etc/mavlink-router/main.conf'
        backup_cmd = f"sudo cp {config_path} {config_path}.backup.$(date +%Y%m%d_%H%M%S)"
        subprocess.run(backup_cmd, shell=True, capture_output=True)

        # 설정 파일 저장
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.conf') as tmp:
            tmp.write(config_content)
            tmp_path = tmp.name

        # sudo로 복사
        result = subprocess.run(
            ['sudo', 'cp', tmp_path, config_path],
            capture_output=True, text=True
        )
        os.unlink(tmp_path)

        if result.returncode != 0:
            return jsonify({
                'success': False,
                'error': f'설정 파일 저장 실패: {result.stderr}'
            }), 500

        # NOTE: ROS_NAMESPACE는 PX4 uXRCE-DDS와 무관 (px4_ns="" 하드코딩).
        # PX4 토픽은 항상 /fmu/in/..., /fmu/out/... 으로 네임스페이스 없이 동작.
        # ROS_NAMESPACE는 편대비행 inter-drone DDS에서만 사용되며 모드 전환과 무관.

        # DDS 포트(8888) iptables 규칙: SITL↔FC 데이터 소스 격리
        # SITL 모드: eth0(FC)에서 오는 DDS 차단 → SITL PC만 연결
        # FC 모드: wlan0(SITL)에서 오는 DDS 차단 → FC만 연결
        subprocess.run(['sudo', 'modprobe', 'xt_udp'],
                       capture_output=True, timeout=5)  # iptables udp 매칭 모듈 로드
        fc_ip = config_manager.get('FC_IP', '10.0.0.12')
        # 기존 INPUT 규칙 제거 (없어도 무시)
        subprocess.run(['sudo', 'iptables', '-D', 'INPUT', '-i', 'eth0', '-s', fc_ip,
                        '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                       capture_output=True, timeout=5)
        subprocess.run(['sudo', 'iptables', '-D', 'INPUT', '-i', 'wlan0',
                        '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                       capture_output=True, timeout=5)
        # 기존 OUTPUT 규칙 제거 (없어도 무시)
        subprocess.run(['sudo', 'iptables', '-D', 'OUTPUT', '-o', 'eth0', '-d', fc_ip,
                        '-p', 'udp', '-j', 'DROP'],
                       capture_output=True, timeout=5)
        subprocess.run(['sudo', 'iptables', '-D', 'OUTPUT', '-o', 'wlan0',
                        '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                       capture_output=True, timeout=5)

        if mode == 'sitl':
            # SITL 모드: FC(eth0) DDS 완전 차단 (수신 + 송신)
            subprocess.run(['sudo', 'iptables', '-A', 'INPUT', '-i', 'eth0', '-s', fc_ip,
                            '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                           capture_output=True, timeout=5)
            subprocess.run(['sudo', 'iptables', '-A', 'OUTPUT', '-o', 'eth0', '-d', fc_ip,
                            '-p', 'udp', '-j', 'DROP'],
                           capture_output=True, timeout=5)
        else:
            # FC 모드: SITL PC(wlan0) DDS 완전 차단 (수신 + 송신)
            subprocess.run(['sudo', 'iptables', '-A', 'INPUT', '-i', 'wlan0',
                            '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                           capture_output=True, timeout=5)
            subprocess.run(['sudo', 'iptables', '-A', 'OUTPUT', '-o', 'wlan0',
                            '-p', 'udp', '--dport', '8888', '-j', 'DROP'],
                           capture_output=True, timeout=5)

        # iptables 규칙 저장 (재부팅 후 자동 복원용)
        subprocess.run('sudo iptables-save | sudo tee /etc/iptables.rules > /dev/null',
                       shell=True, capture_output=True, timeout=5)

        # ============================================================
        # Phase 1: 모든 서비스 중지 (역순)
        # ============================================================
        # 1a. 애플리케이션 중지 (stale DDS 구독 정리)
        subprocess.run(['sudo', 'systemctl', 'stop', 'humiro-fire-suppression'],
                       capture_output=True, text=True, timeout=10)
        subprocess.run(['pkill', '-f', 'humiro_fire_suppression/application/build/humiro_fire_suppression'],
                       capture_output=True, text=True, timeout=5)

        # 1b. micro-ros-agent 중지 + 강제 종료 (UDP 8888 포트 확실히 해제)
        subprocess.run(['sudo', 'systemctl', 'stop', 'micro-ros-agent'],
                       capture_output=True, text=True, timeout=10)
        subprocess.run(['sudo', 'pkill', '-9', '-f', 'micro_ros_agent'],
                       capture_output=True, text=True, timeout=5)

        # 1c. mavlink-router 중지
        subprocess.run(['sudo', 'systemctl', 'stop', 'mavlink-router'],
                       capture_output=True, text=True, timeout=10)

        # ============================================================
        # Phase 2: 소켓/세션 정리 대기
        # ============================================================
        import time
        time.sleep(2)  # UDP 소켓 해제 + DDS 세션 정리 대기

        # 포트 8888 해제 확인 (최대 5초)
        for _retry in range(5):
            port_check = subprocess.run(
                ['ss', '-uln'], capture_output=True, text=True, timeout=2)
            if ':8888' not in port_check.stdout:
                break
            time.sleep(1)

        # ============================================================
        # Phase 3: 서비스 시작 (의존성 순서)
        # ============================================================
        # 3a. mavlink-router 시작
        mavlink_result = subprocess.run(
            ['sudo', 'systemctl', 'start', 'mavlink-router'],
            capture_output=True, text=True, timeout=10
        )
        if mavlink_result.returncode != 0:
            return jsonify({
                'success': False,
                'error': f'mavlink-router 시작 실패: {mavlink_result.stderr}'
            }), 500

        # 3b. micro-ros-agent 시작
        micro_ros_result = subprocess.run(
            ['sudo', 'systemctl', 'start', 'micro-ros-agent'],
            capture_output=True, text=True, timeout=10
        )
        if micro_ros_result.returncode != 0:
            return jsonify({
                'success': False,
                'error': f'micro-ros-agent 시작 실패: {micro_ros_result.stderr}'
            }), 500

        # 3c. micro-ros-agent 포트 8888 리스닝 확인 (최대 5초)
        agent_ready = False
        for _retry in range(10):
            port_check = subprocess.run(
                ['ss', '-uln'], capture_output=True, text=True, timeout=2)
            if ':8888' in port_check.stdout:
                agent_ready = True
                break
            time.sleep(0.5)

        # ============================================================
        # Phase 3.5: GUI 내부 전역 상태 초기화
        # ============================================================
        # MAVLink 연결 초기화 (이전 FC/SITL 세션 정리)
        with mavlink_mgr._lock:
            if mavlink_mgr._mav:
                try:
                    mavlink_mgr._mav.close()
                except:
                    pass
            mavlink_mgr._mav = None
            mavlink_mgr._target_sys = None
            mavlink_mgr._target_comp = None
            mavlink_mgr._last_connect = 0

        # ROS2 토픽 스트리밍 프로세스 종료
        global _topic_echo_process
        with _topic_echo_lock:
            if _topic_echo_process and _topic_echo_process.poll() is None:
                _topic_echo_process.terminate()
                try:
                    _topic_echo_process.wait(timeout=2)
                except:
                    _topic_echo_process.kill()
            _topic_echo_process = None

        # 설정 캐시 리로드
        config_manager.reload()

        # 3d. 애플리케이션 재시작
        app_result = subprocess.run(
            ['sudo', 'systemctl', 'start', 'humiro-fire-suppression'],
            capture_output=True, text=True, timeout=10
        )

        # ============================================================
        # Phase 4: 결과 반환
        # ============================================================
        mode_desc = 'SITL 시뮬레이션' if mode == 'sitl' else '실제 FC 연결'
        return jsonify({
            'success': True,
            'mode': mode,
            'message': f'{mode_desc} 모드로 전환되었습니다.',
            'sitl_ip': sitl_ip if mode == 'sitl' else None,
            'agent_ready': agent_ready,
            'app_restarted': app_result.returncode == 0
        })

    except Exception as e:
        import traceback
        return jsonify({
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc()
        }), 500


# ==================== micro-ros-agent API ====================

@app.route('/api/micro-ros/status')
def api_micro_ros_status():
    """micro-ros-agent 종합 상태 API"""
    # 서비스 상태
    service_running = system_checker.is_service_running("micro-ros-agent")

    # UDP 포트 8888 리슨 상태 확인
    port_listening = False
    try:
        result = subprocess.run(
            ['ss', '-uln'],
            capture_output=True, text=True, timeout=2
        )
        port_listening = ':8888' in result.stdout
    except:
        pass

    # ROS2 토픽 목록
    topics = system_checker.get_ros2_topics()

    # FC 관련 토픽 필터링
    fc_topics = [t for t in topics if 'fmu' in t.lower() or 'px4' in t.lower()]

    return jsonify({
        "service_running": service_running,
        "port_listening": port_listening,
        "dds_connected": len(fc_topics) > 0,
        "topic_count": len(topics),
        "fc_topic_count": len(fc_topics)
    })


@app.route('/api/micro-ros/topics')
def api_micro_ros_topics():
    """ROS2 토픽 목록 API"""
    topics = system_checker.get_ros2_topics()

    # 토픽 분류
    categorized = {
        'fc': [],      # FC(PX4) 관련
        'sensor': [],  # 센서 관련
        'control': [], # 제어 관련
        'other': []    # 기타
    }

    for topic in topics:
        topic_lower = topic.lower()
        if 'fmu' in topic_lower or 'px4' in topic_lower:
            categorized['fc'].append(topic)
        elif any(s in topic_lower for s in ['sensor', 'imu', 'gps', 'baro', 'mag', 'flow', 'range', 'lidar']):
            categorized['sensor'].append(topic)
        elif any(s in topic_lower for s in ['control', 'cmd', 'setpoint', 'attitude', 'position', 'velocity']):
            categorized['control'].append(topic)
        else:
            categorized['other'].append(topic)

    return jsonify({
        "total": len(topics),
        "topics": topics,
        "categorized": categorized
    })


@app.route('/api/micro-ros/topic/<path:topic_name>')
def api_micro_ros_topic_info(topic_name):
    """특정 토픽 정보 API"""
    topic_name = '/' + topic_name if not topic_name.startswith('/') else topic_name

    # ROS2 환경 소스 명령
    ros2_source = "source /opt/ros/humble/setup.bash && "

    # 토픽 타입 확인
    try:
        result = subprocess.run(
            ['bash', '-c', f'{ros2_source}ros2 topic info {topic_name}'],
            capture_output=True, text=True, timeout=5,
            env={**os.environ, 'ROS_DOMAIN_ID': '0'}
        )
        info = result.stdout.strip()

        # Hz 측정은 터미널에서 직접 실행하도록 변경
        # 웹에서는 토픽 정보만 표시

        return jsonify({
            "success": True,
            "topic": topic_name,
            "info": info
        })
    except subprocess.TimeoutExpired:
        return jsonify({
            "success": False,
            "topic": topic_name,
            "message": "토픽 정보 조회 시간 초과"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "topic": topic_name,
            "message": str(e)
        })


# 현재 실행 중인 토픽 echo 프로세스
_topic_echo_process = None
_topic_echo_lock = threading.Lock()


@app.route('/api/micro-ros/topic-stream/<path:topic_name>')
def api_micro_ros_topic_stream(topic_name):
    """토픽 데이터 스트리밍 API (SSE)"""
    global _topic_echo_process

    topic_name = '/' + topic_name if not topic_name.startswith('/') else topic_name
    # px4_msgs를 사용하려면 px4_ros2_ws도 source 필요
    ros2_source = "source /opt/ros/humble/setup.bash && source /home/khadas/humiro_fire_suppression/workspaces/px4_ros2_ws/install/setup.bash 2>/dev/null && "

    def generate():
        global _topic_echo_process

        # 이전 프로세스 종료
        with _topic_echo_lock:
            if _topic_echo_process and _topic_echo_process.poll() is None:
                _topic_echo_process.terminate()
                try:
                    _topic_echo_process.wait(timeout=2)
                except:
                    _topic_echo_process.kill()

        # 새 프로세스 시작
        try:
            proc = subprocess.Popen(
                ['bash', '-c', f'{ros2_source}ros2 topic echo {topic_name}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env={**os.environ, 'ROS_DOMAIN_ID': '0', 'PYTHONUNBUFFERED': '1'}
            )
            with _topic_echo_lock:
                _topic_echo_process = proc

            # 시작 메시지
            yield f"data: === ros2 topic echo {topic_name} ===\n\n"

            # 출력 스트리밍 (1개 메시지만)
            msg_count = 0
            max_messages = 1
            while proc.poll() is None and msg_count < max_messages:
                line = proc.stdout.readline()
                if line:
                    clean_line = line.rstrip('\n\r')
                    yield f"data: {clean_line}\n\n"
                    # '---' 구분자로 메시지 카운트
                    if clean_line == '---':
                        msg_count += 1

            # 종료 메시지
            yield f"data: === {msg_count}개 메시지 출력 완료 ===\n\n"
            proc.terminate()

        except GeneratorExit:
            # 클라이언트 연결 종료
            with _topic_echo_lock:
                if _topic_echo_process and _topic_echo_process.poll() is None:
                    _topic_echo_process.terminate()
        except Exception as e:
            yield f"data: [오류] {str(e)}\n\n"
        finally:
            with _topic_echo_lock:
                if _topic_echo_process and _topic_echo_process.poll() is None:
                    _topic_echo_process.terminate()

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no'
        }
    )


@app.route('/api/micro-ros/topic-stop')
def api_micro_ros_topic_stop():
    """토픽 스트리밍 중지 API"""
    global _topic_echo_process

    with _topic_echo_lock:
        if _topic_echo_process and _topic_echo_process.poll() is None:
            _topic_echo_process.terminate()
            try:
                _topic_echo_process.wait(timeout=2)
            except:
                _topic_echo_process.kill()
            _topic_echo_process = None
            return jsonify({"success": True, "message": "스트리밍 중지됨"})

    return jsonify({"success": True, "message": "실행 중인 스트리밍 없음"})


@app.route('/api/micro-ros/logs')
def api_micro_ros_logs():
    """micro-ros-agent 서비스 로그 API"""
    lines = request.args.get('lines', '50')

    try:
        result = subprocess.run(
            ['journalctl', '-u', 'micro-ros-agent', '-n', lines, '--no-pager'],
            capture_output=True, text=True, timeout=5
        )
        logs = result.stdout.strip()

        return jsonify({
            "success": True,
            "logs": logs
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "logs": "",
            "message": str(e)
        })


@app.route('/api/micro-ros/connection-test')
def api_micro_ros_connection_test():
    """FC ↔ Agent ↔ ROS2 연결 진단 API"""
    results = {
        "service": {"status": False, "message": ""},
        "port": {"status": False, "message": ""},
        "dds": {"status": False, "message": ""},
        "topics": {"status": False, "message": ""}
    }

    # 1. 서비스 상태
    if system_checker.is_service_running("micro-ros-agent"):
        results["service"] = {"status": True, "message": "micro-ros-agent 실행 중"}
    else:
        results["service"] = {"status": False, "message": "micro-ros-agent 중지됨"}
        return jsonify({"success": False, "results": results, "message": "서비스가 실행되지 않음"})

    # 2. UDP 포트 확인
    try:
        ss_result = subprocess.run(['ss', '-uln'], capture_output=True, text=True, timeout=2)
        if ':8888' in ss_result.stdout:
            results["port"] = {"status": True, "message": "UDP 8888 포트 수신 대기 중"}
        else:
            results["port"] = {"status": False, "message": "UDP 8888 포트가 열리지 않음"}
    except:
        results["port"] = {"status": False, "message": "포트 확인 실패"}

    # 3. ROS2 토픽 확인
    topics = system_checker.get_ros2_topics()
    if len(topics) > 0:
        results["topics"] = {"status": True, "message": f"{len(topics)}개 토픽 발견"}
    else:
        results["topics"] = {"status": False, "message": "ROS2 토픽 없음"}

    # 4. FC 연결 확인 (fmu 토픽 존재 여부)
    fc_topics = [t for t in topics if 'fmu' in t.lower()]
    if len(fc_topics) > 0:
        results["dds"] = {"status": True, "message": f"FC 연결됨 ({len(fc_topics)}개 fmu 토픽)"}
    else:
        results["dds"] = {"status": False, "message": "FC 토픽 없음 (DDS 연결 실패)"}

    # 전체 성공 여부
    all_success = all(r["status"] for r in results.values())

    return jsonify({
        "success": all_success,
        "results": results,
        "message": "모든 연결 정상" if all_success else "일부 연결 실패"
    })


@app.route('/api/check/service/<service>')
def api_check_service(service):
    """서비스 체크 API"""
    result = system_checker.is_service_running(service)
    return jsonify({
        "success": result,
        "message": "실행 중" if result else "중지"
    })


@app.route('/api/check/ros2-topics')
def api_check_ros2_topics():
    """ROS2 토픽 체크 API"""
    topics = system_checker.get_ros2_topics()
    return jsonify({
        "success": len(topics) > 0,
        "message": f"{len(topics)}개",
        "count": len(topics)
    })


@app.route('/api/check/fc-param/<param_name>')
def api_check_fc_param(param_name):
    """FC 파라미터 자동 확인 API (MAVLink)

    쿼리 파라미터:
    - expected: 예상 값 (optional, 지정 시 값 비교)
    """
    expected = request.args.get('expected', None)

    # 전역 MAVLink 매니저 사용
    value, error = mavlink_mgr.read_param(param_name)

    if error:
        return jsonify({
            "success": False,
            "message": error
        })

    result = {
        "success": True,
        "param_name": param_name,
        "value": value
    }

    # 예상값과 비교
    if expected is not None:
        try:
            expected_val = float(expected)

            # 정수 비교
            if isinstance(value, int):
                expected_val = int(expected_val)
                match = (value == expected_val)
                display_val = value
                display_expected = expected_val
            else:
                # float 비교 - 부동소수점 오차 허용 (상대 오차 0.1% 또는 절대 오차 0.001)
                if expected_val == 0:
                    match = abs(value) < 0.001
                else:
                    match = abs(value - expected_val) / abs(expected_val) < 0.001 or abs(value - expected_val) < 0.001
                # 표시용 값 (소수점 3자리로 반올림)
                display_val = round(value, 3)
                display_expected = round(expected_val, 3)

            if match:
                result["match"] = True
                result["message"] = f"✓ {display_val}"
            else:
                result["match"] = False
                result["message"] = f"✗ {display_val} (예상: {display_expected})"
                result["success"] = False
        except ValueError:
            result["match"] = str(value) == expected
            result["message"] = f"값: {value}"
    else:
        # 표시만 할 때도 float는 반올림
        if isinstance(value, float):
            result["message"] = f"{round(value, 3)}"
        else:
            result["message"] = f"{value}"

    return jsonify(result)


# ==================== WiFi 네트워크 관리 API ====================

@app.route('/wifi')
def wifi_page():
    """WiFi 네트워크 관리 페이지"""
    return render_template('wifi.html', active_tab='wifi')


@app.route('/api/wifi/saved')
def api_wifi_saved():
    """저장된 WiFi 네트워크 목록"""
    networks = wifi_manager.list_saved_networks()
    return jsonify({
        "success": True,
        "networks": networks
    })


@app.route('/api/wifi/scan')
def api_wifi_scan():
    """주변 WiFi 네트워크 스캔"""
    networks = wifi_manager.scan_available_networks()
    return jsonify({
        "success": True,
        "networks": networks
    })


@app.route('/api/wifi/current')
def api_wifi_current():
    """현재 연결된 WiFi"""
    current = wifi_manager.get_current_connection()
    return jsonify({
        "success": current is not None,
        "connection": current
    })


@app.route('/api/wifi/delete/<uuid>', methods=['POST'])
def api_wifi_delete(uuid):
    """WiFi 네트워크 삭제"""
    success, message = wifi_manager.delete_network(uuid)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/connect/<uuid>', methods=['POST'])
def api_wifi_connect(uuid):
    """WiFi 네트워크 연결"""
    success, message = wifi_manager.connect_to_network(uuid)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/disconnect/<uuid>', methods=['POST'])
def api_wifi_disconnect(uuid):
    """WiFi 네트워크 연결 해제"""
    success, message = wifi_manager.disconnect_network(uuid)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/add', methods=['POST'])
def api_wifi_add():
    """새 WiFi 네트워크 추가"""
    data = request.json
    ssid = data.get('ssid', '')
    password = data.get('password', '')
    autoconnect = data.get('autoconnect', True)

    if not ssid:
        return jsonify({
            "success": False,
            "message": "SSID를 입력하세요"
        })

    success, message = wifi_manager.add_network(ssid, password, autoconnect)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/modify-password', methods=['POST'])
def api_wifi_modify_password():
    """WiFi 비밀번호 변경"""
    data = request.json
    uuid = data.get('uuid', '')
    password = data.get('password', '')

    if not uuid or not password:
        return jsonify({
            "success": False,
            "message": "UUID와 비밀번호를 입력하세요"
        })

    success, message = wifi_manager.modify_network_password(uuid, password)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/autoconnect', methods=['POST'])
def api_wifi_autoconnect():
    """자동 연결 설정 변경"""
    data = request.json
    uuid = data.get('uuid', '')
    autoconnect = data.get('autoconnect', True)

    if not uuid:
        return jsonify({
            "success": False,
            "message": "UUID를 입력하세요"
        })

    success, message = wifi_manager.set_autoconnect(uuid, autoconnect)
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/wifi/priority', methods=['POST'])
def api_wifi_priority():
    """네트워크 우선순위 설정"""
    data = request.json
    uuid = data.get('uuid', '')
    priority = data.get('priority', 0)

    if not uuid:
        return jsonify({
            "success": False,
            "message": "UUID를 입력하세요"
        })

    success, message = wifi_manager.set_connection_priority(uuid, priority)
    return jsonify({
        "success": success,
        "message": message
    })


# ==================== 헬퍼 함수 ====================

def run_build(targets: list, clean_build: bool):
    """빌드 실행 (백그라운드)"""
    global build_status

    build_order = [
        "lidar", "thermal", "targeting", "osd",
        "streaming", "ros2", "navigation", "application"
    ]

    targets_to_build = [t for t in build_order if t in targets]

    try:
        for target in targets_to_build:
            build_status["current_target"] = target
            build_status["logs"].append(f"\n{'='*50}")
            build_status["logs"].append(f"빌드 시작: {target}")

            target_dir = os.path.join(PROJECT_ROOT, target)

            if not os.path.exists(target_dir):
                build_status["logs"].append(f"디렉토리 없음: {target_dir}")
                continue

            build_dir = os.path.join(target_dir, "build")

            # Clean build
            if clean_build and os.path.exists(build_dir):
                build_status["logs"].append("클린 빌드: build 폴더 삭제 중...")
                subprocess.run(["rm", "-rf", build_dir], check=False)

            os.makedirs(build_dir, exist_ok=True)

            # CMake
            build_status["logs"].append("CMake 실행 중...")
            cmake_result = subprocess.run(
                ["cmake", ".."], cwd=build_dir,
                capture_output=True, text=True
            )
            if cmake_result.returncode != 0:
                build_status["logs"].append(f"CMake 실패: {cmake_result.stderr}")
                build_status["running"] = False
                build_status["success"] = False
                return

            # Make
            build_status["logs"].append("Make 실행 중...")
            make_result = subprocess.run(
                ["make", "-j4"], cwd=build_dir,
                capture_output=True, text=True
            )
            if make_result.returncode != 0:
                build_status["logs"].append(f"Make 실패: {make_result.stderr}")
                build_status["running"] = False
                build_status["success"] = False
                return

            build_status["logs"].append(f"✅ {target} 빌드 완료")

        build_status["logs"].append("\n✅ 모든 빌드 완료!")
        build_status["success"] = True

    except Exception as e:
        build_status["logs"].append(f"오류: {e}")
        build_status["success"] = False

    finally:
        build_status["running"] = False
        build_status["current_target"] = ""


def get_vehicle_params_checklist(drone_id: int):
    """기체별 FC 파라미터 체크리스트 (uXRCE-DDS, MAVLink 설정)"""
    # device_config.env 기반 동적 생성
    cfg = ConfigManager.calculate_config_from_drone_id(drone_id)
    eth0_ip = cfg['ETH0_IP']
    uxrce_ip_decimal = ConfigManager.ip_to_decimal(eth0_ip)
    mav_sys_id = int(cfg['MAV_SYS_ID'])
    role = cfg['ROLE']

    return [
        {"id": "vp0", "text": f"[기체 {drone_id}번 - {role}]", "auto": False},
        {"id": "vp1", "text": f"UXRCE_DDS_AG_IP = {uxrce_ip_decimal} (Agent IP: {eth0_ip})", "auto": True, "check": f"fc-param/UXRCE_DDS_AG_IP?expected={uxrce_ip_decimal}"},
        {"id": "vp2", "text": "UXRCE_DDS_CFG = 1000 (Ethernet)", "auto": True, "check": "fc-param/UXRCE_DDS_CFG?expected=1000"},
        {"id": "vp3", "text": "UXRCE_DDS_DOM_ID = 0 (Domain ID)", "auto": True, "check": "fc-param/UXRCE_DDS_DOM_ID?expected=0"},
        {"id": "vp4", "text": f"UXRCE_DDS_KEY = {drone_id} (Session Key)", "auto": True, "check": f"fc-param/UXRCE_DDS_KEY?expected={drone_id}"},
        {"id": "vp5", "text": "UXRCE_DDS_PRT = 8888 (UDP Port)", "auto": True, "check": "fc-param/UXRCE_DDS_PRT?expected=8888"},
        {"id": "vp6", "text": f"MAV_SYS_ID = {mav_sys_id} (MAVLink System ID)", "auto": True, "check": f"fc-param/MAV_SYS_ID?expected={mav_sys_id}"},
        {"id": "vp7", "text": "MAV_2_BROADCAST = 1 (Always broadcast)", "auto": True, "check": "fc-param/MAV_2_BROADCAST?expected=1"},
        {"id": "vp8", "text": "MAV_2_CONFIG = 1000 (Ethernet)", "auto": True, "check": "fc-param/MAV_2_CONFIG?expected=1000"},
        {"id": "vp9", "text": "MAV_2_MODE = 0 (Normal)", "auto": True, "check": "fc-param/MAV_2_MODE?expected=0"},
        {"id": "vp10", "text": "MAV_2_RADIO_CTL = 0 (Disabled)", "auto": True, "check": "fc-param/MAV_2_RADIO_CTL?expected=0"},
        {"id": "vp11", "text": "MAV_2_RATE = 100000 B/s (MAVLink sending rate)", "auto": True, "check": "fc-param/MAV_2_RATE?expected=100000"},
        {"id": "vp12", "text": "MAV_2_REMOTE_PRT = 14540 (Remote Port)", "auto": True, "check": "fc-param/MAV_2_REMOTE_PRT?expected=14540"},
        {"id": "vp13", "text": "MAV_2_UDP_PRT = 14550 (UDP Port)", "auto": True, "check": "fc-param/MAV_2_UDP_PRT?expected=14550"},
    ]


def get_failsafe_params_checklist(indoor=True):
    """페일세이프 파라미터 체크리스트 (공통)

    Args:
        indoor: True=실내 모드 (Land 권장), False=야외 모드 (RTL 권장)
    """
    # 실내/야외별 권장 값 (PX4 v1.16.0)
    # NAV_RCL_ACT: 2=RTL, 3=Land
    # COM_OBL_RC_ACT: 3=RTL, 4=Land
    # NAV_DLL_ACT: 2=RTL, 3=Land
    # COM_LOW_BAT_ACT: 2=Land, 3=Return then Land
    nav_rcl_act = 3 if indoor else 2  # 실내:Land, 야외:RTL
    com_obl_rc_act = 4 if indoor else 3  # 실내:Land, 야외:RTL
    nav_dll_act = 3 if indoor else 2  # 실내:Land, 야외:RTL
    com_low_bat_act = 2 if indoor else 3  # 실내:Land, 야외:Return then Land

    return [
        {"id": "fs1", "text": f"NAV_RCL_ACT = {nav_rcl_act} (RC 두절: {'Land' if indoor else 'RTL'})", "auto": True, "check": f"fc-param/NAV_RCL_ACT?expected={nav_rcl_act}"},
        {"id": "fs2", "text": "COM_RC_LOSS_T = 0.5 (RC 두절 판정 시간)", "auto": True, "check": "fc-param/COM_RC_LOSS_T?expected=0.5"},
        {"id": "fs3", "text": f"COM_OBL_RC_ACT = {com_obl_rc_act} (OFFBOARD 두절: {'Land' if indoor else 'RTL'})", "auto": True, "check": f"fc-param/COM_OBL_RC_ACT?expected={com_obl_rc_act}"},
        {"id": "fs4", "text": "COM_OF_LOSS_T = 1.0 (OFFBOARD 두절 판정 시간)", "auto": True, "check": "fc-param/COM_OF_LOSS_T?expected=1.0"},
        {"id": "fs5", "text": f"NAV_DLL_ACT = {nav_dll_act} (Data Link 두절: {'Land' if indoor else 'RTL'})", "auto": True, "check": f"fc-param/NAV_DLL_ACT?expected={nav_dll_act}"},
        {"id": "fs6", "text": "COM_DL_LOSS_T = 10 (Data Link 두절 판정 시간)", "auto": True, "check": "fc-param/COM_DL_LOSS_T?expected=10"},
        {"id": "fs7", "text": f"COM_LOW_BAT_ACT = {com_low_bat_act} (배터리 저전압: {'Land' if indoor else 'Return then Land'})", "auto": True, "check": f"fc-param/COM_LOW_BAT_ACT?expected={com_low_bat_act}"},
        {"id": "fs8", "text": "BAT_LOW_THR = 0.15 (저전압 임계값 15%)", "auto": True, "check": "fc-param/BAT_LOW_THR?expected=0.15"},
        {"id": "fs9", "text": "BAT_CRIT_THR = 0.1 (위험 전압 임계값 10%)", "auto": True, "check": "fc-param/BAT_CRIT_THR?expected=0.1"},
        {"id": "fs10", "text": "BAT_EMERGEN_THR = 0.05 (비상 전압 임계값 5%)", "auto": True, "check": "fc-param/BAT_EMERGEN_THR?expected=0.05"},
    ]


def get_custom_params_checklist(category: str):
    """커스텀 파라미터를 체크리스트 형식으로 변환"""
    # 해당 카테고리 + common 파라미터 가져오기
    params = config_manager.get_custom_params_by_category(category)
    common_params = config_manager.get_custom_params_by_category('common') if category != 'common' else []

    all_params = params + common_params
    if not all_params:
        return []

    items = []
    for i, param in enumerate(all_params):
        item_id = f"custom_{category}_{i+1}"

        # 텍스트 생성
        text = param['name']
        if param.get('expected') is not None:
            text += f" = {param['expected']}"
        if param.get('description'):
            text += f" ({param['description']})"

        item = {
            "id": item_id,
            "text": text,
            "auto": param.get('auto_check', True) and param.get('expected') is not None,
        }

        # 자동 확인이 가능하면 check 추가
        if item["auto"]:
            item["check"] = f"fc-param/{param['name']}?expected={param['expected']}"

        items.append(item)

    return items


def get_norc_params_checklist():
    """RC 없이 OFFBOARD 모드 파라미터 체크리스트"""
    return [
        {"id": "norc1", "text": "COM_RCL_EXCEPT = 4 (RC 두절 예외: OFFBOARD)", "auto": True, "check": "fc-param/COM_RCL_EXCEPT?expected=4"},
        {"id": "norc2", "text": "COM_RC_IN_MODE = 4 (RC 입력: 불필요)", "auto": True, "check": "fc-param/COM_RC_IN_MODE?expected=4"},
        {"id": "norc3", "text": "COM_OBL_RC_ACT = 4 (OFFBOARD 두절: Land)", "auto": True, "check": "fc-param/COM_OBL_RC_ACT?expected=4"},
        {"id": "norc4", "text": "COM_OF_LOSS_T = 1.0 (OFFBOARD 두절 판정 시간)", "auto": True, "check": "fc-param/COM_OF_LOSS_T?expected=1.0"},
    ]


def get_indoor_checklist(no_rc=False):
    """실내 테스트 체크리스트 (Optical Flow + LiDAR)"""
    drone_id = config_manager.get_drone_id()

    checklist = [
        {"section": "하드웨어 확인", "items": [
            {"id": "hw1", "text": "배터리 충전 상태 확인 (70% 이상)", "auto": False},
            {"id": "hw2", "text": "프로펠러 장착 및 상태 확인", "auto": False},
            {"id": "hw3", "text": "Optical Flow 센서 연결 확인 (MTF-01)", "auto": False},
            {"id": "hw4", "text": "LiDAR 센서 연결 확인", "auto": False},
            {"id": "hw5", "text": "FC 전원 LED 확인", "auto": False},
        ]},
        {"section": "소프트웨어 확인", "items": [
            {"id": "sw1", "text": "FC 연결 확인 (ping)", "auto": True, "check": "fc-ping"},
            {"id": "sw2", "text": "micro-ros-agent 서비스 실행", "auto": True, "check": "service/micro-ros-agent"},
            {"id": "sw3", "text": "mavlink-router 서비스 실행", "auto": True, "check": "service/mavlink-router"},
            {"id": "sw4", "text": "ROS2 토픽 수신 확인", "auto": True, "check": "ros2-topics"},
        ]},
        {"section": "기체별 FC 파라미터 (uXRCE-DDS, MAVLink)", "items": get_vehicle_params_checklist(drone_id)},
        {"section": "FC 파라미터 확인 (실내 모드)", "items": [
            {"id": "param1", "text": "EKF2_GPS_CTRL = 0 (GPS 비활성화)", "auto": True, "check": "fc-param/EKF2_GPS_CTRL?expected=0"},
            {"id": "param2", "text": "EKF2_HGT_REF = 2 (Range sensor)", "auto": True, "check": "fc-param/EKF2_HGT_REF?expected=2"},
            {"id": "param3", "text": "EKF2_OF_CTRL = 1 (Optical Flow 활성화)", "auto": True, "check": "fc-param/EKF2_OF_CTRL?expected=1"},
            {"id": "param4", "text": "EKF2_RNG_CTRL = 2 (Range Finder 활성화)", "auto": True, "check": "fc-param/EKF2_RNG_CTRL?expected=2"},
            {"id": "param5", "text": "EKF2_RNG_A_HMAX = 8m (Range aid 최대 고도)", "auto": True, "check": "fc-param/EKF2_RNG_A_HMAX?expected=8"},
            {"id": "param6", "text": "GPS_1_CONFIG = 0 (GPS 포트 비활성화)", "auto": True, "check": "fc-param/GPS_1_CONFIG?expected=0"},
            {"id": "param7", "text": "SENS_FLOW_ROT = 0 (No rotation)", "auto": True, "check": "fc-param/SENS_FLOW_ROT?expected=0"},
            {"id": "param8", "text": "MAV_1_CONFIG = 103 (TELEM3)", "auto": True, "check": "fc-param/MAV_1_CONFIG?expected=103"},
            {"id": "param9", "text": "MAV_1_MODE = 0 (Normal)", "auto": True, "check": "fc-param/MAV_1_MODE?expected=0"},
            {"id": "param10", "text": "SER_TEL3_BAUD = 115200", "auto": True, "check": "fc-param/SER_TEL3_BAUD?expected=115200"},
        ]},
        {"section": "페일세이프 파라미터 확인 (실내)", "items": get_failsafe_params_checklist(indoor=True)},
        {"section": "비행 환경 확인", "items": [
            {"id": "env1", "text": "비행 공간 확보 (3m x 3m 이상)", "auto": False},
            {"id": "env2", "text": "바닥 텍스처 충분 (Optical Flow용)", "auto": False},
            {"id": "env3", "text": "조명 충분 (Optical Flow용)", "auto": False},
            {"id": "env4", "text": "장애물 제거", "auto": False},
            {"id": "env5", "text": "안전 거리 확보 (벽면에서 1m 이상)", "auto": False},
        ]},
    ]

    # 커스텀 파라미터 섹션 추가
    custom_items = get_custom_params_checklist('indoor')
    if custom_items:
        checklist.append({"section": "사용자 추가 파라미터", "items": custom_items})

    if no_rc:
        checklist.append({"section": "RC 없이 OFFBOARD 파라미터", "items": get_norc_params_checklist()})

    return checklist


def get_outdoor_gps_checklist(no_rc=False):
    """야외 GPS 비행 체크리스트"""
    drone_id = config_manager.get_drone_id()

    checklist = [
        {"section": "하드웨어 확인", "items": [
            {"id": "hw1", "text": "배터리 충전 상태 확인 (80% 이상)", "auto": False},
            {"id": "hw2", "text": "프로펠러 장착 및 상태 확인", "auto": False},
            {"id": "hw3", "text": "GPS 안테나 연결 확인", "auto": False},
            {"id": "hw4", "text": "조종기 연결 확인", "auto": False},
        ]},
        {"section": "GPS 확인", "items": [
            {"id": "gps1", "text": "GPS 위성 수 확인 (6개 이상)", "auto": False},
            {"id": "gps2", "text": "HDOP 확인 (2.0 이하)", "auto": False},
            {"id": "gps3", "text": "GPS Fix 타입 확인 (3D Fix)", "auto": False},
        ]},
        {"section": "소프트웨어 확인", "items": [
            {"id": "sw1", "text": "FC 연결 확인 (ping)", "auto": True, "check": "fc-ping"},
            {"id": "sw2", "text": "micro-ros-agent 서비스 실행", "auto": True, "check": "service/micro-ros-agent"},
            {"id": "sw3", "text": "mavlink-router 서비스 실행", "auto": True, "check": "service/mavlink-router"},
        ]},
        {"section": "기체별 FC 파라미터 (uXRCE-DDS, MAVLink)", "items": get_vehicle_params_checklist(drone_id)},
        {"section": "FC 파라미터 확인 (야외 GPS 모드)", "items": [
            {"id": "param1", "text": "EKF2_BARO_CTRL = 1 (Barometer 활성화)", "auto": True, "check": "fc-param/EKF2_BARO_CTRL?expected=1"},
            {"id": "param2", "text": "EKF2_GPS_CTRL = 7 (GPS 위치/고도/속도 융합)", "auto": True, "check": "fc-param/EKF2_GPS_CTRL?expected=7"},
            {"id": "param3", "text": "EKF2_HGT_REF = 1 (GPS)", "auto": True, "check": "fc-param/EKF2_HGT_REF?expected=1"},
            {"id": "param4", "text": "EKF2_OF_CTRL = 0 (Optical Flow 비활성화)", "auto": True, "check": "fc-param/EKF2_OF_CTRL?expected=0"},
            {"id": "param5", "text": "EKF2_RNG_CTRL (0=Disable, 1=Conditional 권장, 2=Enabled)", "auto": False},  # 야외: 선택 가능
            {"id": "param6", "text": "GPS_1_CONFIG = 201 (GPS1)", "auto": True, "check": "fc-param/GPS_1_CONFIG?expected=201"},
        ]},
        {"section": "페일세이프 파라미터 확인 (야외)", "items": get_failsafe_params_checklist(indoor=False)},
        {"section": "안전 확인", "items": [
            {"id": "safe1", "text": "비행 금지 구역 확인", "auto": False},
            {"id": "safe2", "text": "지오펜스 설정 확인", "auto": False},
            {"id": "safe3", "text": "복귀 지점(Home) 설정 확인", "auto": False},
            {"id": "safe4", "text": "기상 조건 확인 (풍속 8m/s 이하)", "auto": False},
        ]},
    ]

    # 커스텀 파라미터 섹션 추가
    custom_items = get_custom_params_checklist('outdoor_gps')
    if custom_items:
        checklist.append({"section": "사용자 추가 파라미터", "items": custom_items})

    if no_rc:
        checklist.append({"section": "RC 없이 OFFBOARD 파라미터", "items": get_norc_params_checklist()})

    return checklist


def get_outdoor_rtk_checklist(no_rc=False):
    """야외 RTK GPS 비행 체크리스트"""
    drone_id = config_manager.get_drone_id()

    checklist = [
        {"section": "하드웨어 확인", "items": [
            {"id": "hw1", "text": "배터리 충전 상태 확인 (80% 이상)", "auto": False},
            {"id": "hw2", "text": "프로펠러 장착 및 상태 확인", "auto": False},
            {"id": "hw3", "text": "RTK GPS 안테나 연결 확인", "auto": False},
            {"id": "hw4", "text": "조종기 연결 확인", "auto": False},
            {"id": "hw5", "text": "RTK 베이스스테이션/NTRIP 연결 확인", "auto": False},
        ]},
        {"section": "RTK GPS 확인", "items": [
            {"id": "rtk1", "text": "RTK Fix 상태 확인 (Fixed)", "auto": False},
            {"id": "rtk2", "text": "GPS 위성 수 확인 (8개 이상 권장)", "auto": False},
            {"id": "rtk3", "text": "HDOP 확인 (1.0 이하 권장)", "auto": False},
            {"id": "rtk4", "text": "베이스스테이션 거리 확인 (10km 이내)", "auto": False},
        ]},
        {"section": "소프트웨어 확인", "items": [
            {"id": "sw1", "text": "FC 연결 확인 (ping)", "auto": True, "check": "fc-ping"},
            {"id": "sw2", "text": "micro-ros-agent 서비스 실행", "auto": True, "check": "service/micro-ros-agent"},
            {"id": "sw3", "text": "mavlink-router 서비스 실행", "auto": True, "check": "service/mavlink-router"},
        ]},
        {"section": "기체별 FC 파라미터 (uXRCE-DDS, MAVLink)", "items": get_vehicle_params_checklist(drone_id)},
        {"section": "FC 파라미터 확인 (야외 RTK 모드)", "items": [
            {"id": "param1", "text": "EKF2_BARO_CTRL = 1 (Barometer 활성화)", "auto": True, "check": "fc-param/EKF2_BARO_CTRL?expected=1"},
            {"id": "param2", "text": "EKF2_GPS_CTRL = 7 (GPS 위치/고도/속도 융합)", "auto": True, "check": "fc-param/EKF2_GPS_CTRL?expected=7"},
            {"id": "param3", "text": "EKF2_HGT_REF = 1 (GPS)", "auto": True, "check": "fc-param/EKF2_HGT_REF?expected=1"},
            {"id": "param4", "text": "EKF2_OF_CTRL = 0 (Optical Flow 비활성화)", "auto": True, "check": "fc-param/EKF2_OF_CTRL?expected=0"},
            {"id": "param5", "text": "EKF2_RNG_CTRL (0=Disable, 1=Conditional 권장, 2=Enabled)", "auto": False},  # 야외: 선택 가능
            {"id": "param6", "text": "GPS_1_CONFIG = 201 (GPS1)", "auto": True, "check": "fc-param/GPS_1_CONFIG?expected=201"},
            {"id": "param7", "text": "GPS_1_PROTOCOL = 1 (u-blox)", "auto": True, "check": "fc-param/GPS_1_PROTOCOL?expected=1"},
            {"id": "param8", "text": "GPS_UBX_MODE = 0 (Default, RTK 포함)", "auto": True, "check": "fc-param/GPS_UBX_MODE?expected=0"},
        ]},
        {"section": "페일세이프 파라미터 확인 (야외)", "items": get_failsafe_params_checklist(indoor=False)},
        {"section": "안전 확인", "items": [
            {"id": "safe1", "text": "비행 금지 구역 확인", "auto": False},
            {"id": "safe2", "text": "지오펜스 설정 확인", "auto": False},
            {"id": "safe3", "text": "복귀 지점(Home) 설정 확인", "auto": False},
            {"id": "safe4", "text": "기상 조건 확인 (풍속 8m/s 이하)", "auto": False},
        ]},
    ]

    # 커스텀 파라미터 섹션 추가
    custom_items = get_custom_params_checklist('outdoor_rtk')
    if custom_items:
        checklist.append({"section": "사용자 추가 파라미터", "items": custom_items})

    if no_rc:
        checklist.append({"section": "RC 없이 OFFBOARD 파라미터", "items": get_norc_params_checklist()})

    return checklist


def get_postflight_checklist():
    """비행 후 체크리스트"""
    return [
        {"section": "기체 점검", "items": [
            {"id": "post1", "text": "프로펠러 손상 확인", "auto": False},
            {"id": "post2", "text": "모터 상태 확인", "auto": False},
            {"id": "post3", "text": "배터리 상태 확인", "auto": False},
            {"id": "post4", "text": "프레임 손상 확인", "auto": False},
        ]},
        {"section": "데이터 관리", "items": [
            {"id": "data1", "text": "비행 로그 저장", "auto": False},
            {"id": "data2", "text": "비행 영상 백업", "auto": False},
            {"id": "data3", "text": "이상 사항 기록", "auto": False},
        ]},
        {"section": "정리", "items": [
            {"id": "clean1", "text": "배터리 분리 및 보관", "auto": False},
            {"id": "clean2", "text": "기체 보관", "auto": False},
            {"id": "clean3", "text": "장비 정리", "auto": False},
        ]},
    ]


# ==================== MAVLink 송신 API ====================

@app.route('/mavlink-sender')
def mavlink_sender_page():
    """MAVLink 송신 페이지"""
    return render_template('mavlink_sender.html', active_tab='mavlink-sender')


@app.route('/api/mavlink/test-connection', methods=['POST'])
def api_mavlink_test_connection():
    """MAVLink 연결 테스트"""
    try:
        data = request.json
        target_ip = data.get('target_ip', config_manager.get_fc_ip())
        target_port = int(data.get('target_port', config_manager.get('EXTERNAL_UDP_PORT', '15001')))

        # UDP 소켓으로 간단한 연결 테스트
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1)

        # HEARTBEAT 메시지 전송 시도
        test_data = b'\xfd\x09\x00\x00\x00\x01\xbf\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x04'
        sock.sendto(test_data, (target_ip, target_port))
        sock.close()

        return jsonify({"success": True, "message": "연결 테스트 완료"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/mavlink/send-standard', methods=['POST'])
def api_mavlink_send_standard():
    """표준 MAVLink 메시지 전송"""
    try:
        import socket
        from pymavlink import mavutil

        data = request.json
        target_ip = data.get('target_ip')
        target_port = int(data.get('target_port'))
        system_id = int(data.get('system_id', config_manager.get('MAV_SYS_ID', '1')))
        component_id = int(data.get('component_id', config_manager.get('MAV_COMP_ID', '191')))
        message_type = data.get('message_type')
        params = data.get('params', {})

        # MAVLink 연결 (UDP)
        connection_string = f'udpout:{target_ip}:{target_port}'
        mav = mavutil.mavlink_connection(
            connection_string,
            source_system=system_id,
            source_component=component_id
        )

        # 메시지 타입별 전송
        # 간편 비행 제어 명령어 (COMMAND_LONG로 변환)
        import time

        print(f"[DEBUG] Sending {message_type} to {target_ip}:{target_port}, sys={system_id}, comp={component_id}")

        # Target FC: MAV_SYS_ID는 DRONE_ID와 동일
        target_system = int(config_manager.get('MAV_SYS_ID', '1'))
        target_component = 1

        if message_type == 'ARM':
            # MAV_CMD_COMPONENT_ARM_DISARM = 400
            force = float(params.get('force', 0))
            # 여러 번 전송 (안정성 확보)
            for i in range(3):
                mav.mav.command_long_send(
                    target_system,     # FC system ID
                    target_component,  # FC component ID
                    400,  # MAV_CMD_COMPONENT_ARM_DISARM
                    0,    # confirmation
                    1,    # param1: 1=arm
                    force,  # param2: force (21196 to override safety checks)
                    0, 0, 0, 0, 0
                )
                print(f"[DEBUG] Sent ARM command to target {target_system}/{target_component} (attempt {i+1}/3)")
                time.sleep(0.1)  # 100ms 간격
        elif message_type == 'DISARM':
            # MAV_CMD_COMPONENT_ARM_DISARM = 400
            force = float(params.get('force', 0))
            # 여러 번 전송 (안정성 확보)
            for i in range(3):
                mav.mav.command_long_send(
                    target_system,     # FC system ID
                    target_component,  # FC component ID
                    400,  # MAV_CMD_COMPONENT_ARM_DISARM
                    0,    # confirmation
                    0,    # param1: 0=disarm
                    force,  # param2: force (21196 to override safety checks)
                    0, 0, 0, 0, 0
                )
                print(f"[DEBUG] Sent DISARM command to target {target_system}/{target_component} (attempt {i+1}/3)")
                time.sleep(0.1)  # 100ms 간격
        elif message_type == 'TAKEOFF':
            # MAV_CMD_NAV_TAKEOFF = 22
            altitude = float(params.get('altitude', 2.5))
            # 여러 번 전송 (안정성 확보)
            for i in range(3):
                mav.mav.command_long_send(
                    target_system,     # FC system ID
                    target_component,  # FC component ID
                    22,   # MAV_CMD_NAV_TAKEOFF
                    0,    # confirmation
                    0, 0, 0, 0,  # param1-4: unused
                    0, 0,        # param5-6: unused
                    altitude     # param7: altitude
                )
                print(f"[DEBUG] Sent TAKEOFF command (attempt {i+1}/3)")
                time.sleep(0.1)  # 100ms 간격
        elif message_type == 'LAND':
            # MAV_CMD_NAV_LAND = 21
            # 여러 번 전송 (안정성 확보)
            for i in range(3):
                mav.mav.command_long_send(
                    target_system,     # FC system ID
                    target_component,  # FC component ID
                    21,   # MAV_CMD_NAV_LAND
                    0,    # confirmation
                    0, 0, 0, 0, 0, 0, 0  # all params unused
                )
                print(f"[DEBUG] Sent LAND command (attempt {i+1}/3)")
                time.sleep(0.1)  # 100ms 간격
        elif message_type == 'HEARTBEAT':
            mav.mav.heartbeat_send(
                int(params.get('type', 2)),
                int(params.get('autopilot', 3)),
                int(params.get('base_mode', 0)),
                int(params.get('custom_mode', 0)),
                int(params.get('system_status', 4)),
                3  # mavlink_version
            )
        elif message_type == 'COMMAND_LONG':
            mav.mav.command_long_send(
                target_system,     # FC system ID
                target_component,  # FC component ID
                int(params.get('command', 400)),
                int(params.get('confirmation', 0)),
                float(params.get('param1', 0)),
                float(params.get('param2', 0)),
                float(params.get('param3', 0)),
                float(params.get('param4', 0)),
                float(params.get('param5', 0)),
                float(params.get('param6', 0)),
                float(params.get('param7', 0))
            )
        elif message_type == 'SET_MODE':
            mav.mav.set_mode_send(
                target_system,     # FC system ID
                int(params.get('base_mode', 1)),
                int(params.get('custom_mode', 0))
            )
        elif message_type == 'PARAM_SET':
            param_id = params.get('param_id', 'PARAM_NAME')
            # 16자로 제한
            param_id_bytes = param_id[:16].ljust(16, '\x00').encode('utf-8')
            mav.mav.param_set_send(
                system_id,
                component_id,
                param_id_bytes,
                float(params.get('param_value', 0)),
                int(params.get('param_type', 1))
            )
        elif message_type == 'MISSION_ITEM':
            mav.mav.mission_item_send(
                system_id,
                component_id,
                int(params.get('seq', 0)),
                int(params.get('frame', 0)),
                int(params.get('command', 16)),
                int(params.get('current', 0)),
                int(params.get('autocontinue', 1)),
                float(params.get('param1', 0)),
                float(params.get('param2', 0)),
                float(params.get('param3', 0)),
                float(params.get('param4', 0)),
                float(params.get('x', 0)),
                float(params.get('y', 0)),
                float(params.get('z', 0))
            )

        mav.close()
        return jsonify({"success": True, "message": f"{message_type} 전송 완료"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/mavlink/send-custom', methods=['POST'])
def api_mavlink_send_custom():
    """커스텀 MAVLink 메시지 전송"""
    try:
        import socket

        data = request.json
        target_ip = data.get('target_ip')
        target_port = int(data.get('target_port'))
        system_id = int(data.get('system_id', config_manager.get('MAV_SYS_ID', '1')))
        component_id = int(data.get('component_id', config_manager.get('MAV_COMP_ID', '191')))
        message_id = int(data.get('message_id'))
        payload_hex = data.get('payload_hex', '').strip()

        # Hex 문자열을 바이트로 변환
        payload_bytes = bytes.fromhex(payload_hex.replace(' ', ''))

        # MAVLink v2 헤더 생성
        # STX(0xFD) | LEN | INCOMPAT | COMPAT | SEQ | SYSID | COMPID | MSGID(3 bytes) | PAYLOAD | CHECKSUM(2 bytes)
        stx = 0xFD
        payload_len = len(payload_bytes)
        incompat_flags = 0
        compat_flags = 0
        seq = 0

        # 메시지 ID (24bit, little endian)
        msgid_bytes = message_id.to_bytes(3, 'little')

        # 헤더 + Payload
        header = struct.pack('<BBBBBBB', stx, payload_len, incompat_flags, compat_flags, seq, system_id, component_id)
        message = header + msgid_bytes + payload_bytes

        # CRC_EXTRA 값 (메시지 ID별 고유값)
        # 60000: 미션 스타트, 60001: 자동조준, 60002: 발사, 60003: 복귀
        crc_extra_map = {
            60000: 100,  # FIRE_MISSION_START
            60001: 101,  # AUTO_AIM
            60002: 102,  # FIRE_COMMAND
            60003: 103,  # RETURN_TO_LAUNCH
        }
        crc_extra = crc_extra_map.get(message_id, 0)

        # MAVLink CRC 계산
        crc_value = calculate_mavlink_crc(message, crc_extra)
        crc = struct.pack('<H', crc_value)  # little endian 16-bit
        message += crc

        # UDP로 전송
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message, (target_ip, target_port))
        sock.close()

        return jsonify({"success": True, "message": f"커스텀 메시지 전송 완료 ({len(payload_bytes)} bytes)"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/mavlink/send-fire', methods=['POST'])
def api_mavlink_send_fire():
    """화재 진압 커스텀 메시지 전송"""
    try:
        import socket

        data = request.json
        target_ip = data.get('target_ip')
        target_port = int(data.get('target_port'))
        system_id = int(data.get('system_id', config_manager.get('MAV_SYS_ID', '1')))
        component_id = int(data.get('component_id', config_manager.get('MAV_COMP_ID', '191')))
        message_type = data.get('message_type')
        params = data.get('params', {})

        # 메시지 ID 매핑
        # 60000: 미션 스타트, 60001: 자동조준, 60002: 발사, 60003: 복귀
        message_ids = {
            'FIRE_MISSION_START': 60000,
            'FIRE_AUTO_AIM': 60001,
            'FIRE_LAUNCH': 60002,
            'FIRE_RETURN': 60003
        }

        message_id = message_ids.get(message_type, 0)

        # 메시지별 Payload 생성
        payload = b''
        if message_type == 'FIRE_MISSION_START':
            # Wire Format 순서 (4바이트 먼저, 1바이트 마지막)
            # int32_t target_lat, int32_t target_lon, float target_alt,
            # float takeoff_speed, float flight_speed,
            # uint8_t target_system, uint8_t target_component,
            # uint8_t auto_fire, uint8_t max_projectiles
            payload = struct.pack('<iifffBBBB',
                int(params.get('target_lat', 0)),
                int(params.get('target_lon', 0)),
                float(params.get('target_alt', 10.0)),
                float(params.get('takeoff_speed', 3.0)),
                float(params.get('flight_speed', 5.0)),
                int(params.get('target_system', 1)),
                int(params.get('target_component', 191)),
                int(params.get('auto_fire', 0)),
                int(params.get('max_projectiles', 1))
            )
        elif message_type == 'FIRE_AUTO_AIM':
            # 60001 자동조준 - target_system, target_component만 포함
            payload = struct.pack('<BB',
                int(params.get('target_system', 1)),
                int(params.get('target_component', 191))
            )
        elif message_type == 'FIRE_LAUNCH':
            # 60002 발사 - target_system, target_component만 포함
            payload = struct.pack('<BB',
                int(params.get('target_system', 1)),
                int(params.get('target_component', 191))
            )
        elif message_type == 'FIRE_RETURN':
            # 60003 복귀 - target_system, target_component만 포함
            payload = struct.pack('<BB',
                int(params.get('target_system', 1)),
                int(params.get('target_component', 191))
            )

        # MAVLink v2 헤더 생성
        stx = 0xFD
        payload_len = len(payload)
        incompat_flags = 0
        compat_flags = 0
        seq = 0

        # 메시지 ID (24bit, little endian)
        msgid_bytes = message_id.to_bytes(3, 'little')

        # 헤더 + Payload
        header = struct.pack('<BBBBBBB', stx, payload_len, incompat_flags, compat_flags, seq, system_id, component_id)
        message = header + msgid_bytes + payload

        # CRC_EXTRA 값 (메시지 ID별 고유값)
        # 60000: 미션 스타트, 60001: 자동조준, 60002: 발사, 60003: 복귀
        crc_extra_map = {
            60000: 100,  # FIRE_MISSION_START
            60001: 101,  # AUTO_AIM
            60002: 102,  # FIRE_COMMAND
            60003: 103,  # RETURN_TO_LAUNCH
        }
        crc_extra = crc_extra_map.get(message_id, 0)

        # MAVLink CRC 계산
        crc_value = calculate_mavlink_crc(message, crc_extra)
        crc = struct.pack('<H', crc_value)  # little endian 16-bit
        message += crc

        # UDP로 전송 (1회)
        print(f"[DEBUG] Sending {message_type} to {target_ip}:{target_port}, payload length: {len(payload)}, message length: {len(message)}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sent_bytes = sock.sendto(message, (target_ip, target_port))
            print(f"[DEBUG] Sent {sent_bytes} bytes")
            sock.close()
            return jsonify({"success": True, "message": f"{message_type} 전송 완료 ({len(payload)} bytes)"})
        except Exception as e:
            print(f"[DEBUG] Send failed: {e}")
            sock.close()
            return jsonify({"success": False, "message": f"{message_type} 전송 실패: {e}"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


## ========== 오프보드 설정 페이지 ==========

OFFBOARD_CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "offboard_config.json")

@app.route('/offboard-settings')
def offboard_settings_page():
    """오프보드 모드 설정 페이지"""
    return render_template('offboard_settings.html', active_tab='offboard-settings')


@app.route('/api/offboard-config', methods=['GET'])
def api_get_offboard_config():
    """오프보드 설정 읽기"""
    try:
        if os.path.exists(OFFBOARD_CONFIG_PATH):
            with open(OFFBOARD_CONFIG_PATH, 'r') as f:
                data = json.load(f)
            return jsonify({"success": True, "config": data})
        else:
            return jsonify({"success": True, "config": {"target_altitude": 10.0, "updated_at": ""}})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/offboard-config', methods=['POST'])
def api_set_offboard_config():
    """오프보드 설정 저장"""
    try:
        data = request.json

        # mission_mode 변경 시 DISARM 상태 확인
        if 'mission_mode' in data:
            # 현재 설정과 비교
            current_mode = None
            if os.path.exists(OFFBOARD_CONFIG_PATH):
                with open(OFFBOARD_CONFIG_PATH, 'r') as f:
                    current = json.load(f)
                    current_mode = current.get('mission_mode')
            if current_mode and current_mode != data['mission_mode']:
                armed = mavlink_mgr.is_armed()
                if armed is True:
                    return jsonify({
                        "success": False,
                        "message": "FC가 Armed 상태입니다. Disarm 후 비행 모드를 변경하세요."
                    })

        from datetime import datetime
        data["updated_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        os.makedirs(os.path.dirname(OFFBOARD_CONFIG_PATH), exist_ok=True)
        with open(OFFBOARD_CONFIG_PATH, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        return jsonify({"success": True, "message": "오프보드 설정이 저장되었습니다."})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


## ========== 편대 비행 설정 API ==========

@app.route('/api/formation-config', methods=['GET'])
def api_get_formation_config():
    """편대 비행 설정 읽기 (device_config.env에서)"""
    try:
        return jsonify({
            "success": True,
            "config": {
                "role": config_manager.get("ROLE", "Leader"),
                "drone_id": config_manager.get_drone_id(),
                "leader_namespace": config_manager.get("LEADER_NAMESPACE", "drone1"),
                "offset_right": int(config_manager.get("FORMATION_OFFSET_RIGHT", "0")),
                "offset_behind": int(config_manager.get("FORMATION_OFFSET_BEHIND", "0")),
                "offset_above": int(config_manager.get("FORMATION_OFFSET_ABOVE", "0")),
                "suppress_distance": int(config_manager.get("SUPPRESS_DISTANCE", "30")),
                "suppress_angle": int(config_manager.get("SUPPRESS_ANGLE", "0")),
            }
        })
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/formation-config', methods=['POST'])
def api_set_formation_config():
    """편대 비행 설정 저장 (device_config.env에)"""
    try:
        data = request.json
        role = data.get("role", "Leader")
        config_manager.set("ROLE", role)
        config_manager.set("LEADER_NAMESPACE", data.get("leader_namespace", "drone1"))
        # 역할에 따라 ROS_NAMESPACE 자동 설정
        ros_ns = data.get("ros_namespace", "")
        if ros_ns:
            config_manager.set("ROS_NAMESPACE", ros_ns)
        config_manager.set("FORMATION_OFFSET_RIGHT", str(int(data.get("offset_right", 0))))
        config_manager.set("FORMATION_OFFSET_BEHIND", str(int(data.get("offset_behind", 0))))
        config_manager.set("FORMATION_OFFSET_ABOVE", str(int(data.get("offset_above", 0))))
        config_manager.set("SUPPRESS_DISTANCE", str(int(data.get("suppress_distance", 30))))
        config_manager.set("SUPPRESS_ANGLE", str(int(data.get("suppress_angle", 0))))

        if not config_manager.save_device_config():
            return jsonify({"success": False, "message": "설정 저장 실패"})

        # Armed 상태 확인 → DISARMED일 때만 자동 재시작
        armed = mavlink_mgr.is_armed()
        if armed is True:
            return jsonify({"success": True, "message": "편대 설정이 저장되었습니다. 현재 Armed 상태이므로 재시작하지 않습니다. DISARM 후 수동 재시작 필요."})

        # DISARMED → 애플리케이션 자동 재시작
        import subprocess
        restart_result = subprocess.run(
            ['sudo', 'systemctl', 'restart', 'humiro-fire-suppression'],
            capture_output=True, text=True, timeout=10
        )
        if restart_result.returncode == 0:
            return jsonify({"success": True, "message": "편대 설정이 저장되었습니다. 애플리케이션을 재시작했습니다."})
        else:
            return jsonify({"success": True, "message": f"편대 설정이 저장되었습니다. 자동 재시작 실패: {restart_result.stderr}. 수동 재시작 필요."})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


if __name__ == '__main__':
    print("=" * 50)
    print("Humiro Fire Suppression - Web GUI")
    print("=" * 50)
    print(f"프로젝트 경로: {PROJECT_ROOT}")
    print(f"드론 ID: {config_manager.get_drone_id()}")
    print("")
    print("웹 GUI 시작: http://0.0.0.0:5000")
    print("=" * 50)

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
