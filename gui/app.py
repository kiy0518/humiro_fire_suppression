#!/usr/bin/env python3
"""
Humiro Fire Suppression - Web GUI
Flask 기반 웹 관리 도구
"""

import os
import sys
import json
import subprocess
import threading
from datetime import datetime
from flask import Flask, render_template, jsonify, request

# 프로젝트 경로 설정
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.config_manager import ConfigManager
from utils.system_checker import SystemChecker

app = Flask(__name__)
app.config['SECRET_KEY'] = 'humiro-fire-suppression-2024'

# 전역 객체
config_manager = ConfigManager(PROJECT_ROOT)
system_checker = SystemChecker()

# 빌드 상태
build_status = {
    "running": False,
    "logs": [],
    "current_target": "",
    "success": None
}


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


@app.route('/checklist')
def checklist_page():
    """체크리스트 페이지"""
    return render_template('checklist.html', active_tab='checklist')


@app.route('/build')
def build_page():
    """빌드 페이지"""
    return render_template('build.html', active_tab='build')


@app.route('/monitor')
def monitor_page():
    """모니터 페이지"""
    return render_template('monitor.html', active_tab='monitor')


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
    if mode == "indoor":
        items = get_indoor_checklist()
    elif mode == "outdoor":
        items = get_outdoor_checklist()
    else:
        items = get_postflight_checklist()

    return jsonify(items)


@app.route('/api/check/fc-ping')
def api_check_fc_ping():
    """FC 핑 체크 API"""
    fc_ip = config_manager.get_fc_ip()
    result = system_checker.ping_host(fc_ip)
    return jsonify({
        "success": result,
        "message": "응답" if result else "응답 없음"
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


def get_indoor_checklist():
    """실내 테스트 체크리스트"""
    return [
        {"section": "하드웨어 확인", "items": [
            {"id": "hw1", "text": "배터리 충전 상태 확인 (70% 이상)", "auto": False},
            {"id": "hw2", "text": "프로펠러 장착 및 상태 확인", "auto": False},
            {"id": "hw3", "text": "Optical Flow 센서 연결 확인", "auto": False},
            {"id": "hw4", "text": "LiDAR 센서 연결 확인", "auto": False},
            {"id": "hw5", "text": "FC 전원 LED 확인", "auto": False},
        ]},
        {"section": "소프트웨어 확인", "items": [
            {"id": "sw1", "text": "FC 연결 확인 (ping)", "auto": True, "check": "fc-ping"},
            {"id": "sw2", "text": "micro-ros-agent 서비스 실행", "auto": True, "check": "service/micro-ros-agent"},
            {"id": "sw3", "text": "mavlink-router 서비스 실행", "auto": True, "check": "service/mavlink-router"},
            {"id": "sw4", "text": "ROS2 토픽 수신 확인", "auto": True, "check": "ros2-topics"},
        ]},
        {"section": "비행 환경 확인", "items": [
            {"id": "env1", "text": "비행 공간 확보 (3m x 3m 이상)", "auto": False},
            {"id": "env2", "text": "바닥 텍스처 충분 (Optical Flow용)", "auto": False},
            {"id": "env3", "text": "조명 충분 (Optical Flow용)", "auto": False},
            {"id": "env4", "text": "장애물 제거", "auto": False},
            {"id": "env5", "text": "안전 거리 확보 (벽면에서 1m 이상)", "auto": False},
        ]},
    ]


def get_outdoor_checklist():
    """야외 비행 체크리스트"""
    return [
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
        {"section": "안전 확인", "items": [
            {"id": "safe1", "text": "비행 금지 구역 확인", "auto": False},
            {"id": "safe2", "text": "지오펜스 설정 확인", "auto": False},
            {"id": "safe3", "text": "복귀 지점(Home) 설정 확인", "auto": False},
            {"id": "safe4", "text": "페일세이프 설정 확인", "auto": False},
            {"id": "safe5", "text": "기상 조건 확인 (풍속 8m/s 이하)", "auto": False},
        ]},
    ]


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
