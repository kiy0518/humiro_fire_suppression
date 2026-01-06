#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
커스텀 메시지 송신 GUI 프로그램 (MAVLink 2.0)
외부 PC에서 실행하여 15000 포트로 MAVLink 2.0 커스텀 메시지를 전송합니다.

요구사항:
    pip install pymavlink

사용법:
    python3 custom_message_sender_gui_v2.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, font
import socket
import struct
from datetime import datetime
import time
import sys


class CustomMessageSenderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Custom Message Sender (MAVLink 2.0 - 15000 Test Port)")
        self.root.geometry("800x700")

        # ttk 스타일 설정
        style = ttk.Style()
        style.theme_use('clam')
        
        # 한글 폰트 설정
        self.korean_font = self.get_korean_font()
        if self.korean_font:
            try:
                style.configure('.', font=self.korean_font)
                style.configure('TLabel', font=self.korean_font)
                style.configure('TLabelFrame', font=self.korean_font)
                style.configure('TLabelframe.Label', font=self.korean_font)
            except:
                pass
        
        # UTF-8 출력 설정
        if sys.stdout.encoding != 'utf-8':
            import io
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
        if sys.stderr.encoding != 'utf-8':
            import io
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

        # UDP 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # MAVLink 2.0 시퀀스 번호
        self.sequence_number = 0

        self.create_widgets()
        if self.korean_font:
            self.log(f"폰트 설정: {self.korean_font[0]}")
        else:
            self.log("한글 폰트를 찾을 수 없습니다.")
        self.log("프로그램 시작 (MAVLink 2.0)")
    
    def get_korean_font(self):
        """한글을 지원하는 폰트 찾기"""
        try:
            available_fonts = [f.lower() for f in font.families()]
            korean_fonts = [
                'nanumgothic',
                'nanumbarungothic',
                'noto sans cjk kr',
                'noto sans kr',
                'malgun gothic',
                'gulim',
                'dotum',
                'batang',
                'unifont',
                'dejavu sans'
            ]
            
            for font_name in korean_fonts:
                for available_font in font.families():
                    if font_name in available_font.lower():
                        return (available_font, 9)
            return None
        except Exception as e:
            print(f"폰트 찾기 오류: {e}")
            return None

    def create_widgets(self):
        # 상단 연결 설정
        conn_frame = ttk.LabelFrame(self.root, text="연결 설정", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="대상 IP:").grid(row=0, column=0, sticky="w")
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.insert(0, "192.168.100.11")
        self.ip_entry.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="포트:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.insert(0, "14550")
        self.port_entry.grid(row=0, column=3, padx=5)

        # 시스템 ID / 컴포넌트 ID
        ttk.Label(conn_frame, text="System ID (송신자):").grid(row=1, column=0, sticky="w", pady=5)
        self.system_id_entry = ttk.Entry(conn_frame, width=10)
        self.system_id_entry.insert(0, "1")
        self.system_id_entry.grid(row=1, column=1, padx=5, sticky="w")

        ttk.Label(conn_frame, text="Component ID (송신자):").grid(row=1, column=2, sticky="w", padx=(20, 0))
        self.component_id_entry = ttk.Entry(conn_frame, width=10)
        self.component_id_entry.insert(0, "1")
        self.component_id_entry.grid(row=1, column=3, padx=5, sticky="w")
        
        ttk.Label(conn_frame, text="Target System (FC):").grid(row=2, column=0, sticky="w", pady=5)
        self.target_system_entry = ttk.Entry(conn_frame, width=10)
        self.target_system_entry.insert(0, "1")
        self.target_system_entry.grid(row=2, column=1, padx=5, sticky="w")

        ttk.Label(conn_frame, text="Target Component (FC):").grid(row=2, column=2, sticky="w", padx=(20, 0))
        self.target_component_entry = ttk.Entry(conn_frame, width=10)
        self.target_component_entry.insert(0, "1")
        self.target_component_entry.grid(row=2, column=3, padx=5, sticky="w")

        # 메시지 전송 섹션
        msg_frame = ttk.LabelFrame(self.root, text="메시지 전송", padding=10)
        msg_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # FIRE_MISSION_START
        self.create_mission_start_section(msg_frame)

        # FIRE_MISSION_STATUS
        self.create_mission_status_section(msg_frame)

        # FIRE_SUPPRESSION_RESULT
        self.create_suppression_result_section(msg_frame)

        # FIRE_LAUNCH_CONTROL
        self.create_launch_control_section(msg_frame)

        # ARMING (표준 MAVLink 메시지)
        self.create_arming_section(msg_frame)

        # SET_MODE (비행 모드 설정)
        self.create_set_mode_section(msg_frame)

        # 로그 영역
        log_frame = ttk.LabelFrame(self.root, text="로그", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        font_config = self.korean_font if self.korean_font else ('TkDefaultFont', 9)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, state='disabled', font=font_config)
        self.log_text.pack(fill="both", expand=True)

    def create_mission_start_section(self, parent):
        frame = ttk.LabelFrame(parent, text="1. FIRE_MISSION_START (미션 시작)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="위도(°):").pack(side="left")
        self.lat_entry = ttk.Entry(row_frame, width=15)
        self.lat_entry.insert(0, "37.1234567")
        self.lat_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="경도(°):").pack(side="left")
        self.lon_entry = ttk.Entry(row_frame, width=15)
        self.lon_entry.insert(0, "127.1234567")
        self.lon_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="고도(m):").pack(side="left")
        self.alt_entry = ttk.Entry(row_frame, width=10)
        self.alt_entry.insert(0, "10.0")
        self.alt_entry.pack(side="left", padx=5)

        self.auto_fire_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(row_frame, text="자동 발사", variable=self.auto_fire_var).pack(side="left", padx=10)

        ttk.Button(row_frame, text="전송", command=self.send_mission_start).pack(side="right", padx=5)

    def create_mission_status_section(self, parent):
        frame = ttk.LabelFrame(parent, text="2. FIRE_MISSION_STATUS (미션 상태)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="Phase:").pack(side="left")
        self.phase_combo = ttk.Combobox(row_frame, width=20, state="readonly")
        self.phase_combo['values'] = [
            '0: IDLE',
            '1: NAVIGATING',
            '2: SCANNING',
            '3: READY_TO_FIRE',
            '4: SUPPRESSING',
            '5: VERIFYING',
            '6: COMPLETE'
        ]
        self.phase_combo.current(1)
        self.phase_combo.pack(side="left", padx=5)

        ttk.Label(row_frame, text="진행률(%):").pack(side="left")
        self.progress_entry = ttk.Entry(row_frame, width=10)
        self.progress_entry.insert(0, "50")
        self.progress_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="거리(m):").pack(side="left")
        self.distance_entry = ttk.Entry(row_frame, width=10)
        self.distance_entry.insert(0, "25.5")
        self.distance_entry.pack(side="left", padx=5)

        ttk.Button(row_frame, text="전송", command=self.send_mission_status).pack(side="right", padx=5)

    def create_suppression_result_section(self, parent):
        frame = ttk.LabelFrame(parent, text="3. FIRE_SUPPRESSION_RESULT (소화 결과)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="발사 번호:").pack(side="left")
        self.shot_entry = ttk.Entry(row_frame, width=10)
        self.shot_entry.insert(0, "1")
        self.shot_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="발사 전 온도(°C):").pack(side="left")
        self.temp_before_entry = ttk.Entry(row_frame, width=10)
        self.temp_before_entry.insert(0, "90.0")
        self.temp_before_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="발사 후 온도(°C):").pack(side="left")
        self.temp_after_entry = ttk.Entry(row_frame, width=10)
        self.temp_after_entry.insert(0, "30.0")
        self.temp_after_entry.pack(side="left", padx=5)

        self.success_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(row_frame, text="성공", variable=self.success_var).pack(side="left", padx=10)

        ttk.Button(row_frame, text="전송", command=self.send_suppression_result).pack(side="right", padx=5)

    def create_launch_control_section(self, parent):
        frame = ttk.LabelFrame(parent, text="4. FIRE_LAUNCH_CONTROL (발사 제어)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Button(row_frame, text="확인 (CONFIRM)",
                  command=lambda: self.send_launch_control(0)).pack(side="left", padx=5)
        ttk.Button(row_frame, text="중단 (ABORT)",
                  command=lambda: self.send_launch_control(1)).pack(side="left", padx=5)
        ttk.Button(row_frame, text="상태 요청 (REQUEST_STATUS)",
                  command=lambda: self.send_launch_control(2)).pack(side="left", padx=5)

    def create_arming_section(self, parent):
        frame = ttk.LabelFrame(parent, text="5. ARMING (무장/해제)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Button(row_frame, text="ARM (무장)",
                  command=lambda: self.send_arming(1)).pack(side="left", padx=5)
        ttk.Button(row_frame, text="DISARM (해제)",
                  command=lambda: self.send_arming(0)).pack(side="left", padx=5)

    def create_set_mode_section(self, parent):
        frame = ttk.LabelFrame(parent, text="6. SET_MODE (비행 모드 설정 - PX4)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="모드:").pack(side="left")
        self.mode_combo = ttk.Combobox(row_frame, width=30, state="readonly")
        # PX4 custom_mode 값: (main_mode << 16) | (sub_mode << 8)
        self.mode_combo['values'] = [
            'Manual (65536)',           # 1 << 16
            'Altitude (131072)',          # 2 << 16
            'Position (196608)',          # 3 << 16
            'Mission (262400)',         # (4 << 16) | (4 << 8)

        ]
        self.mode_combo.current(0)  # MANUAL
        self.mode_combo.pack(side="left", padx=5)

        # 메시지 타입 선택
        ttk.Label(row_frame, text="방법:").pack(side="left", padx=(10, 0))
        self.mode_method_combo = ttk.Combobox(row_frame, width=15, state="readonly")
        self.mode_method_combo['values'] = ['COMMAND_LONG (권장)', 'SET_MODE']
        self.mode_method_combo.current(0)
        self.mode_method_combo.pack(side="left", padx=5)

        ttk.Button(row_frame, text="모드 설정", command=self.send_set_mode).pack(side="right", padx=5)

    def calculate_crc16(self, data, initial_crc=0xFFFF):
        """MAVLink 2.0 CRC-16/MCRF4XX 계산"""
        crc = initial_crc
        # data가 리스트나 바이트 시퀀스일 수 있음
        if isinstance(data, bytes):
            data = list(data)
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return crc

    def send_mavlink2_message(self, msg_id, payload):
        """MAVLink 2.0 메시지 패킷 생성 및 전송"""
        # MAVLink 2.0 헤더 (10 bytes)
        MAVLINK_MAGIC = 0xFD
        payload_len = len(payload)
        system_id = int(self.system_id_entry.get())
        component_id = int(self.component_id_entry.get())
        
        # 메시지 ID 분리 (24-bit)
        msg_id_low = msg_id & 0xFF
        msg_id_mid = (msg_id >> 8) & 0xFF
        msg_id_high = (msg_id >> 16) & 0xFF
        
        # 헤더 생성 (MAVLink 2.0: 10 bytes)
        # 구조: magic(1) + len(1) + incompat_flags(1) + compat_flags(1) + 
        #       seq(1) + sysid(1) + compid(1) + msgid(1) + msgid_ext[2](2) = 10 bytes
        header = struct.pack(
            '<BBBBBBBBBB',
            MAVLINK_MAGIC,      # magic (0xFD) - 1 byte
            payload_len,        # payload length - 1 byte
            0,                  # incompat_flags - 1 byte
            0,                  # compat_flags - 1 byte
            self.sequence_number & 0xFF,  # sequence number - 1 byte
            system_id,          # system ID - 1 byte
            component_id,       # component ID - 1 byte
            msg_id_low,         # message ID (lower 8 bits) - 1 byte
            msg_id_mid,         # message ID (bits 8-15) - 1 byte
            msg_id_high         # message ID (bits 16-23) - 1 byte
        )
        
        # 시퀀스 번호 증가
        self.sequence_number = (self.sequence_number + 1) % 256
        
        # CRC_EXTRA 값 (메시지 ID별로 다름)
        crc_extra_map = {
            12900: 100,  # FIRE_MISSION_START
            12901: 101,  # FIRE_MISSION_STATUS
            12902: 102,  # FIRE_LAUNCH_CONTROL
            12903: 103,  # FIRE_SUPPRESSION_RESULT
            76: 19,      # COMMAND_LONG (표준 MAVLink 메시지)
            11: 89       # SET_MODE (표준 MAVLink 메시지)
        }
        crc_extra = crc_extra_map.get(msg_id, 0)
        
        # 헤더 + 페이로드 (CRC 계산용, magic 제외)
        # header[1:]은 bytes 객체, payload도 bytes 객체
        message_without_crc = header[1:] + payload  # magic 제외하고 CRC 계산
        
        # CRC 계산 (헤더 + 페이로드)
        crc = self.calculate_crc16(message_without_crc)
        
        # CRC_EXTRA 추가 (bytes로 변환하여 추가)
        crc_extra_bytes = bytes([crc_extra])
        crc = self.calculate_crc16(crc_extra_bytes, initial_crc=crc)
        
        # 전체 메시지: 헤더 + 페이로드 + CRC
        message = header + payload + struct.pack('<H', crc)
        
        # UDP 전송
        target_ip = self.ip_entry.get()
        target_port = int(self.port_entry.get())
        
        self.sock.sendto(message, (target_ip, target_port))
        
        return len(message)

    def send_mission_start(self):
        try:
            lat = int(float(self.lat_entry.get()) * 1e7)
            lon = int(float(self.lon_entry.get()) * 1e7)
            alt = float(self.alt_entry.get())
            auto_fire = 1 if self.auto_fire_var.get() else 0
            system_id = int(self.system_id_entry.get())
            component_id = int(self.component_id_entry.get())

            # Message ID: 12900
            msg_id = 12900

            # 메시지 페이로드 (struct FireMissionStart)
            payload = struct.pack(
                '<BBiifBB',  # Little endian
                system_id,          # target_system
                component_id,       # target_component
                lat,                # target_lat (int32_t)
                lon,                # target_lon (int32_t)
                alt,                # target_alt (float)
                auto_fire,          # auto_fire (uint8_t)
                5                   # max_projectiles (uint8_t)
            )

            self.send_mavlink2_message(msg_id, payload)
            self.log(f"✓ FIRE_MISSION_START 전송: {lat/1e7}°, {lon/1e7}°, {alt}m, Auto={auto_fire}")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}")

    def send_mission_status(self):
        try:
            phase = int(self.phase_combo.get().split(':')[0])
            progress = int(self.progress_entry.get())
            distance = float(self.distance_entry.get())

            # Message ID: 12901
            msg_id = 12901

            # 메시지 페이로드 (struct FireMissionStatus)
            status_text = "Flying to target".encode('utf-8')
            status_text_padded = status_text[:49].ljust(50, b'\0')  # 50 bytes로 패딩
            
            payload = struct.pack(
                '<BBBfh50s',  # phase, progress, remaining, distance, temp, status_text
                phase,                    # phase (uint8_t)
                progress,                 # progress (uint8_t)
                8,                        # remaining_projectiles (uint8_t)
                distance,                 # distance_to_target (float)
                850,                      # thermal_max_temp (int16_t, 85.0°C * 10)
                status_text_padded        # status_text (char[50])
            )

            self.send_mavlink2_message(msg_id, payload)
            self.log(f"✓ FIRE_MISSION_STATUS 전송: Phase={phase}, Progress={progress}%, Distance={distance}m")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}")

    def send_suppression_result(self):
        try:
            shot = int(self.shot_entry.get())
            temp_before = int(float(self.temp_before_entry.get()) * 10)
            temp_after = int(float(self.temp_after_entry.get()) * 10)
            success = 1 if self.success_var.get() else 0

            # Message ID: 12903
            msg_id = 12903

            # 메시지 페이로드 (struct FireSuppressionResult)
            payload = struct.pack(
                '<BhhB',  # shot_number, temp_before, temp_after, success
                shot,              # shot_number (uint8_t)
                temp_before,       # temp_before (int16_t)
                temp_after,        # temp_after (int16_t)
                success            # success (uint8_t)
            )

            self.send_mavlink2_message(msg_id, payload)
            self.log(f"✓ FIRE_SUPPRESSION_RESULT 전송: Shot={shot}, {temp_before/10}°C → {temp_after/10}°C, Success={success}")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}")

    def send_launch_control(self, command):
        try:
            system_id = int(self.system_id_entry.get())
            component_id = int(self.component_id_entry.get())

            # Message ID: 12902
            msg_id = 12902

            # 메시지 페이로드 (struct FireLaunchControl)
            payload = struct.pack(
                '<BBB',  # target_system, target_component, command
                system_id,      # target_system (uint8_t)
                component_id,   # target_component (uint8_t)
                command         # command (uint8_t)
            )

            cmd_names = {0: "CONFIRM", 1: "ABORT", 2: "REQUEST_STATUS"}
            self.send_mavlink2_message(msg_id, payload)
            self.log(f"✓ FIRE_LAUNCH_CONTROL 전송: {cmd_names.get(command, 'UNKNOWN')}")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}")

    def send_arming(self, arm_value):
        """MAVLink COMMAND_LONG 메시지로 ARM/DISARM 전송"""
        try:
            # 디버그: 받은 arm_value 값 확인
            self.log(f"[DEBUG] send_arming 호출: arm_value={arm_value} (type: {type(arm_value)})")
            
            # 송신자 ID (헤더에 사용)
            system_id = int(self.system_id_entry.get())
            component_id = int(self.component_id_entry.get())
            
            # 수신자 ID (FC의 시스템/컴포넌트 ID, 페이로드에 사용)
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            
            # COMMAND_LONG 메시지 ID: 76
            msg_id = 76
            # MAV_CMD_COMPONENT_ARM_DISARM: 400
            command = 400
            
            # param1 값 확인 및 변환
            param1_value = float(arm_value)
            self.log(f"[DEBUG] param1 값: {param1_value} (1.0=ARM, 0.0=DISARM)")
            
            # COMMAND_LONG 페이로드 구조
            # target_system, target_component, command, confirmation, 
            # param1, param2, param3, param4, param5, param6, param7
            payload = struct.pack(
                '<BBHB7f',
                target_system,      # target_system (FC의 시스템 ID)
                target_component,   # target_component (FC의 컴포넌트 ID)
                command,            # command (MAV_CMD_COMPONENT_ARM_DISARM = 400)
                0,                  # confirmation
                param1_value,      # param1: 1.0 = ARM, 0.0 = DISARM
                0.0,                # param2: 0 = force disarming (not used for ARM)
                0.0,                # param3: unused
                0.0,                # param4: unused
                0.0,                # param5: unused
                0.0,                # param6: unused
                0.0                 # param7: unused
            )
            
            self.send_mavlink2_message(msg_id, payload)
            action = "ARM" if arm_value == 1 else "DISARM"
            self.log(f"✓ COMMAND_LONG ({action}) 전송: target_system={target_system}, target_component={target_component}, MAV_CMD_COMPONENT_ARM_DISARM={command}, param1={param1_value}")
            
        except Exception as e:
            self.log(f"✗ ARM/DISARM 전송 실패: {e}")

    def send_set_mode(self):
        """PX4 비행 모드 설정 (COMMAND_LONG 또는 SET_MODE)"""
        try:
            # 수신자 ID (FC의 시스템/컴포넌트 ID)
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())

            # 선택된 모드 값 추출
            mode_str = self.mode_combo.get()
            custom_mode = int(mode_str.split('(')[1].split(')')[0])  # 괄호 안의 숫자 추출
            mode_name = mode_str.split('(')[0].strip()

            # base_mode: PX4에서는 MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 사용
            base_mode = 1  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

            # 선택된 방법 확인
            method = self.mode_method_combo.get()

            if 'COMMAND_LONG' in method:
                # COMMAND_LONG (MSG_ID: 76) 사용 - PX4 권장 방법
                msg_id = 76
                command = 176  # MAV_CMD_DO_SET_MODE

                # COMMAND_LONG 페이로드 구조
                # target_system, target_component, command, confirmation, param1-7 (7개 float)
                payload = struct.pack(
                    '<BBHBfffffff',
                    target_system,       # target_system - uint8_t
                    target_component,    # target_component - uint8_t
                    command,             # command (MAV_CMD_DO_SET_MODE = 176) - uint16_t
                    0,                   # confirmation - uint8_t
                    float(base_mode),    # param1: base_mode - float
                    float(custom_mode),  # param2: custom_mode - float
                    0.0, 0.0, 0.0, 0.0, 0.0  # param3-7 - float
                )

                self.send_mavlink2_message(msg_id, payload)
                self.log(f"✓ COMMAND_LONG (DO_SET_MODE) 전송: target={target_system}, base_mode={base_mode}, custom_mode={custom_mode} ({mode_name})")

            else:
                # SET_MODE (MSG_ID: 11) 사용 - 구형 방법
                msg_id = 11

                # SET_MODE 페이로드 구조
                # target_system (uint8_t), base_mode (uint8_t), custom_mode (uint32_t)
                payload = struct.pack(
                    '<BBL',
                    target_system,      # target_system - uint8_t
                    base_mode,          # base_mode - uint8_t
                    custom_mode         # custom_mode - uint32_t
                )

                self.send_mavlink2_message(msg_id, payload)
                self.log(f"✓ SET_MODE 전송: target={target_system}, base_mode={base_mode}, custom_mode={custom_mode} ({mode_name})")

        except Exception as e:
            self.log(f"✗ 모드 설정 전송 실패: {e}")

    def log(self, message):
        """로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state='disabled')

    def on_closing(self):
        """프로그램 종료"""
        self.sock.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CustomMessageSenderGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

