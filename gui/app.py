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
from flask import Flask, render_template, jsonify, request, Response, stream_with_context

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

# 빌드 상태
build_status = {
    "running": False,
    "logs": [],
    "current_target": "",
    "success": None
}

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
            self._mav = mavutil.mavlink_connection(
                'tcp:127.0.0.1:5790',
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

mavlink_mgr = MavlinkManager()


# ==================== 페이지 라우트 ====================

@app.route('/')
def index():
    """대시보드 페이지"""
    return render_template('index.html', active_tab='dashboard')


@app.route('/config')
def config_page():
    """설정 페이지"""
    return render_template('config.html', active_tab='config')


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
    """FC 파라미터 관리 페이지"""
    return render_template('params.html', active_tab='params')


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

    return jsonify({
        "drone_id": drone_id,
        "role": "Leader" if drone_id == 1 else "Follower",
        "fc": {
            "connected": fc_connected,
            "ip": fc_ip
        },
        "network": {
            "eth0": eth0_ip,
            "wlan0": wlan0_ip
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
        connection_string = "tcp:127.0.0.1:5790"
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

        connection_string = "tcp:127.0.0.1:5790"
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

    # IP 주소 유효성 검증 (보안)
    allowed_ips = ['192.168.100.11', '192.168.100.21', '192.168.100.31']
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
    # 기체별 설정값
    presets = {
        1: {  # Leader
            "UXRCE_DDS_AG_IP": 167772171,  # 10.0.0.11 → decimal
            "ETH0_IP": "10.0.0.11",
            "MAV_SYS_ID": 1,
        },
        2: {  # Follower Left
            "UXRCE_DDS_AG_IP": 167772181,  # 10.0.0.21 → decimal
            "ETH0_IP": "10.0.0.21",
            "MAV_SYS_ID": 2,
        },
        3: {  # Follower Right
            "UXRCE_DDS_AG_IP": 167772191,  # 10.0.0.31 → decimal
            "ETH0_IP": "10.0.0.31",
            "MAV_SYS_ID": 3,
        },
    }

    preset = presets.get(drone_id, presets[1])
    role = "Leader" if drone_id == 1 else "Follower"

    return [
        {"id": "vp0", "text": f"[기체 {drone_id}번 - {role}]", "auto": False},
        {"id": "vp1", "text": f"UXRCE_DDS_AG_IP = {preset['UXRCE_DDS_AG_IP']} (Agent IP: {preset['ETH0_IP']})", "auto": True, "check": f"fc-param/UXRCE_DDS_AG_IP?expected={preset['UXRCE_DDS_AG_IP']}"},
        {"id": "vp2", "text": "UXRCE_DDS_CFG = 1000 (Ethernet)", "auto": True, "check": "fc-param/UXRCE_DDS_CFG?expected=1000"},
        {"id": "vp3", "text": "UXRCE_DDS_DOM_ID = 0 (Domain ID)", "auto": True, "check": "fc-param/UXRCE_DDS_DOM_ID?expected=0"},
        {"id": "vp4", "text": "UXRCE_DDS_KEY = 1 (Session Key)", "auto": True, "check": "fc-param/UXRCE_DDS_KEY?expected=1"},
        {"id": "vp5", "text": "UXRCE_DDS_PRT = 8888 (UDP Port)", "auto": True, "check": "fc-param/UXRCE_DDS_PRT?expected=8888"},
        {"id": "vp6", "text": f"MAV_SYS_ID = {preset['MAV_SYS_ID']} (MAVLink System ID)", "auto": True, "check": f"fc-param/MAV_SYS_ID?expected={preset['MAV_SYS_ID']}"},
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
    # 실내/야외별 권장 값 (PX4 v1.15.0)
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
