#!/usr/bin/env python3
"""
PX4 모드 변경 테스트 프로그램
실제 작동하는 OFFBOARD 설정 방식을 참조

사용법:
    python3 test_px4_mode_change.py [mode_name]
    
사용 가능한 모드:
    manual, altitude, position, mission, hold, rtl, land, acro, stabilized, offboard
"""

import socket
import struct
import sys
import time

# MAVLink 2.0
MAVLINK_MAGIC = 0xFD
MAV_CMD_DO_SET_MODE = 176
COMMAND_LONG_MSG_ID = 76

# PX4 모드 정의 (Main mode + Sub mode)
PX4_MODES = {
    # Main modes (간단 방식 - OFFBOARD처럼)
    'manual': 1,
    'altitude': 2,
    'position': 3,
    'acro': 5,
    'offboard': 6,
    'stabilized': 7,
    'rattitude': 8,
    
    # AUTO sub modes (Full custom mode)
    'mission': (4 << 16) | 4,    # AUTO.MISSION
    'hold': (4 << 16) | 3,       # AUTO.LOITER
    'rtl': (4 << 16) | 5,        # AUTO.RTL
    'land': (4 << 16) | 6,       # AUTO.LAND
    'takeoff': (4 << 16) | 2,    # AUTO.TAKEOFF
}

def calculate_crc16(data, initial_crc=0xFFFF):
    """MAVLink CRC-16 계산"""
    crc = initial_crc
    if isinstance(data, bytes):
        data = list(data)
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def send_set_mode(mode_name, target_ip="192.168.100.11", target_port=14550):
    """PX4 모드 변경 명령 전송"""
    
    mode_name = mode_name.lower()
    
    if mode_name not in PX4_MODES:
        print(f"오류: 알 수 없는 모드 '{mode_name}'")
        print(f"사용 가능: {', '.join(PX4_MODES.keys())}")
        return False
    
    custom_mode = PX4_MODES[mode_name]
    
    print(f"\n=== PX4 모드 변경: {mode_name.upper()} ===")
    print(f"대상: {target_ip}:{target_port}")
    print(f"Custom mode 값: {custom_mode} (0x{custom_mode:08X})")
    
    # Socket 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # COMMAND_LONG 페이로드
    payload = struct.pack(
        '<BBHB7f',
        1,    # target_system (FC)
        1,    # target_component (FC)
        MAV_CMD_DO_SET_MODE,  # command (176)
        0,    # confirmation
        1.0,  # param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        float(custom_mode),  # param2: custom_mode
        0.0, 0.0, 0.0, 0.0, 0.0  # param3-7: unused
    )
    
    # MAVLink 2.0 헤더
    seq = 0
    system_id = 255  # GCS
    component_id = 190  # MAV_COMP_ID_MISSIONPLANNER
    payload_len = len(payload)
    
    msg_id_low = COMMAND_LONG_MSG_ID & 0xFF
    msg_id_mid = (COMMAND_LONG_MSG_ID >> 8) & 0xFF
    msg_id_high = (COMMAND_LONG_MSG_ID >> 16) & 0xFF
    
    header = struct.pack(
        '<BBBBBBBBBB',
        MAVLINK_MAGIC,
        payload_len,
        0,  # incompat_flags
        0,  # compat_flags
        seq,
        system_id,
        component_id,
        msg_id_low,
        msg_id_mid,
        msg_id_high
    )
    
    # CRC 계산 (CRC_EXTRA = 152 for COMMAND_LONG)
    message_without_crc = header[1:] + payload
    crc = calculate_crc16(message_without_crc)
    crc = calculate_crc16(bytes([152]), initial_crc=crc)  # CRC_EXTRA
    
    # 전체 메시지
    message = header + payload + struct.pack('<H', crc)
    
    # 여러 번 전송 (OFFBOARD 설정 방식 참조)
    print(f"\n모드 변경 명령 전송 중...")
    success_count = 0
    for i in range(5):
        try:
            sock.sendto(message, (target_ip, target_port))
            success_count += 1
            print(f"  [{i+1}/5] 전송 완료")
            time.sleep(0.1)
        except Exception as e:
            print(f"  [{i+1}/5] 전송 실패: {e}")
    
    sock.close()
    
    print(f"\n✓ 모드 변경 명령 전송 완료: {mode_name.upper()}")
    print(f"  - 전송 성공: {success_count}/5")
    print(f"  - 메시지: COMMAND_LONG (ID: {COMMAND_LONG_MSG_ID})")
    print(f"  - 명령: MAV_CMD_DO_SET_MODE ({MAV_CMD_DO_SET_MODE})")
    print(f"  - Param1: 1.0 (CUSTOM_MODE_ENABLED)")
    print(f"  - Param2: {custom_mode} (0x{custom_mode:08X})")
    print(f"\n주의: FC에서 모드 변경 가능 조건을 확인하세요.")
    print(f"       (예: ARM 상태, GPS 고정, 센서 상태 등)")
    
    return True

if __name__ == "__main__":
    print("==========================================")
    print("   PX4 모드 변경 테스트 (MAVLink 2.0)")
    print("==========================================")
    
    if len(sys.argv) < 2:
        print("\n사용법: python3 test_px4_mode_change.py [mode_name]\n")
        print("사용 가능한 모드:")
        print("  - manual       : 수동 모드")
        print("  - altitude     : 고도 제어")
        print("  - position     : 위치 제어")
        print("  - mission      : 자동 미션")
        print("  - hold         : 정지/배회")
        print("  - rtl          : 귀환")
        print("  - land         : 착륙")
        print("  - takeoff      : 이륙")
        print("  - acro         : 곡예 모드")
        print("  - stabilized   : 안정화 모드")
        print("  - offboard     : 외부 제어")
        print("\n예시:")
        print("  python3 test_px4_mode_change.py offboard")
        print("  python3 test_px4_mode_change.py position")
        print("  python3 test_px4_mode_change.py mission")
        sys.exit(1)
    
    mode_name = sys.argv[1]
    send_set_mode(mode_name)
