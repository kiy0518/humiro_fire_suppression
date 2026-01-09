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
        self.root.geometry("900x900")  # 창 크기 확대 (새로운 섹션 추가로 인해)

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
        # 초기 기체 설정 적용
        self.on_drone_selection_changed()
    
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

        # 기체 선택
        ttk.Label(conn_frame, text="기체 선택:").grid(row=0, column=0, sticky="w")
        self.drone_selection = ttk.Combobox(conn_frame, width=15, state="readonly")
        self.drone_selection['values'] = ("기체 1", "기체 2", "기체 3", "수동 설정")
        self.drone_selection.current(0)  # 기본값: 기체 1
        self.drone_selection.grid(row=0, column=1, padx=5, sticky="w")
        self.drone_selection.bind("<<ComboboxSelected>>", self.on_drone_selection_changed)

        ttk.Label(conn_frame, text="대상 IP:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.insert(0, "192.168.100.11")
        self.ip_entry.grid(row=0, column=3, padx=5)

        ttk.Label(conn_frame, text="포트:").grid(row=0, column=4, sticky="w", padx=(20, 0))
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.insert(0, "14550")
        self.port_entry.grid(row=0, column=5, padx=5)

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
        
        # TAKEOFF (이륙)
        self.create_takeoff_section(msg_frame)
        
        # LAND (착륙)
        self.create_land_section(msg_frame)
        
        # RTL (출발지 복귀)
        self.create_rtl_section(msg_frame)
        
        # 위치 이동 (SET_POSITION_TARGET)
        self.create_position_target_section(msg_frame)

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
    
    def create_takeoff_section(self, parent):
        """TAKEOFF 섹션 생성"""
        frame = ttk.LabelFrame(parent, text="7. TAKEOFF (이륙) - 표준 메시지", padding=5)
        frame.pack(fill="x", pady=5)
        
        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")
        
        ttk.Label(row_frame, text="고도(m):").pack(side="left")
        self.takeoff_alt_entry = ttk.Entry(row_frame, width=10)
        self.takeoff_alt_entry.insert(0, "10.0")
        self.takeoff_alt_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="피치(°):").pack(side="left", padx=(10, 0))
        self.takeoff_pitch_entry = ttk.Entry(row_frame, width=10)
        self.takeoff_pitch_entry.insert(0, "0.0")
        self.takeoff_pitch_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="Yaw(°):").pack(side="left", padx=(10, 0))
        self.takeoff_yaw_entry = ttk.Entry(row_frame, width=10)
        self.takeoff_yaw_entry.insert(0, "0.0")
        self.takeoff_yaw_entry.pack(side="left", padx=5)
        
        ttk.Button(row_frame, text="이륙", command=self.send_takeoff).pack(side="right", padx=5)
        
        info_label = ttk.Label(row_frame, text="(MAVLink 라우터가 자동으로 FC로 전달)", 
                              font=('TkDefaultFont', 7), foreground='gray')
        info_label.pack(side="right", padx=(5, 0))
    
    def create_land_section(self, parent):
        """LAND 섹션 생성"""
        frame = ttk.LabelFrame(parent, text="8. LAND (착륙) - 표준 메시지", padding=5)
        frame.pack(fill="x", pady=5)
        
        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")
        
        ttk.Label(row_frame, text="위도(°):").pack(side="left")
        self.land_lat_entry = ttk.Entry(row_frame, width=15)
        self.land_lat_entry.insert(0, "0.0")  # 현재 위치에 착륙
        self.land_lat_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="경도(°):").pack(side="left")
        self.land_lon_entry = ttk.Entry(row_frame, width=15)
        self.land_lon_entry.insert(0, "0.0")  # 현재 위치에 착륙
        self.land_lon_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="고도(m):").pack(side="left", padx=(10, 0))
        self.land_alt_entry = ttk.Entry(row_frame, width=10)
        self.land_alt_entry.insert(0, "0.0")
        self.land_alt_entry.pack(side="left", padx=5)
        
        ttk.Button(row_frame, text="착륙", command=self.send_land).pack(side="right", padx=5)
        
        info_label = ttk.Label(row_frame, text="(MAVLink 라우터가 자동으로 FC로 전달)", 
                              font=('TkDefaultFont', 7), foreground='gray')
        info_label.pack(side="right", padx=(5, 0))
    
    def create_rtl_section(self, parent):
        """RTL 섹션 생성"""
        frame = ttk.LabelFrame(parent, text="9. RTL (출발지 복귀) - 표준 메시지", padding=5)
        frame.pack(fill="x", pady=5)
        
        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")
        
        ttk.Button(row_frame, text="RTL (Return to Launch)", 
                  command=self.send_rtl).pack(side="left", padx=5)
        
        info_label = ttk.Label(row_frame, text="(MAVLink 라우터가 자동으로 FC로 전달)", 
                              font=('TkDefaultFont', 7), foreground='gray')
        info_label.pack(side="left", padx=(10, 0))
    
    def create_position_target_section(self, parent):
        """위치 이동 섹션 생성"""
        frame = ttk.LabelFrame(parent, text="10. 위치 이동 (SET_POSITION_TARGET) - 표준 메시지", padding=5)
        frame.pack(fill="x", pady=5)
        
        row_frame1 = ttk.Frame(frame)
        row_frame1.pack(fill="x", pady=2)
        
        ttk.Label(row_frame1, text="위도(°):").pack(side="left")
        self.pos_lat_entry = ttk.Entry(row_frame1, width=15)
        self.pos_lat_entry.insert(0, "37.1234567")
        self.pos_lat_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame1, text="경도(°):").pack(side="left")
        self.pos_lon_entry = ttk.Entry(row_frame1, width=15)
        self.pos_lon_entry.insert(0, "127.1234567")
        self.pos_lon_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame1, text="고도(m):").pack(side="left", padx=(10, 0))
        self.pos_alt_entry = ttk.Entry(row_frame1, width=10)
        self.pos_alt_entry.insert(0, "10.0")
        self.pos_alt_entry.pack(side="left", padx=5)
        
        row_frame2 = ttk.Frame(frame)
        row_frame2.pack(fill="x", pady=2)
        
        ttk.Label(row_frame2, text="속도(m/s):").pack(side="left")
        self.pos_vx_entry = ttk.Entry(row_frame2, width=10)
        self.pos_vx_entry.insert(0, "0.0")
        self.pos_vx_entry.pack(side="left", padx=5)
        
        ttk.Label(row_frame2, text="타입:").pack(side="left", padx=(10, 0))
        self.pos_type_combo = ttk.Combobox(row_frame2, width=20, state="readonly")
        self.pos_type_combo['values'] = [
            'POSITION (0)',
            'VELOCITY (1)',
            'ACCELERATION (2)',
            'FORCE (3)',
            'YAW (4)',
            'YAW_RATE (5)'
        ]
        self.pos_type_combo.current(0)
        self.pos_type_combo.pack(side="left", padx=5)
        
        ttk.Button(row_frame2, text="이동", command=self.send_position_target).pack(side="right", padx=5)
        
        info_label = ttk.Label(row_frame2, text="(MAVLink 라우터가 자동으로 FC로 전달)", 
                              font=('TkDefaultFont', 7), foreground='gray')
        info_label.pack(side="right", padx=(5, 0))

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
        
        # CRC_EXTRA 값 (메시지 ID별로 다름) - MAVLink 공식 값
        crc_extra_map = {
            0: 50,       # HEARTBEAT (표준 MAVLink 메시지) - 공식 값
            11: 89,      # SET_MODE (표준 MAVLink 메시지) - 공식 값
            76: 152,     # COMMAND_LONG (표준 MAVLink 메시지) - 공식 값
            84: 143,     # SET_POSITION_TARGET_LOCAL_NED (표준 MAVLink 메시지) - 공식 값
            86: 5,       # SET_POSITION_TARGET_GLOBAL_INT (표준 MAVLink 메시지) - 공식 값
            50000: 100,  # FIRE_MISSION_START (커스텀)
            50001: 101,  # FIRE_MISSION_STATUS (커스텀)
            50002: 102,  # FIRE_LAUNCH_CONTROL (커스텀)
            50003: 103   # FIRE_SUPPRESSION_RESULT (커스텀)
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

            # Message ID: 50000
            msg_id = 50000

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

            # Message ID: 50001
            msg_id = 50001

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

            # Message ID: 50003
            msg_id = 50003

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

            # Message ID: 50002
            msg_id = 50002

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
            
            # 여러 번 전송 (FC가 확실히 받도록)
            action = "ARM" if arm_value == 1 else "DISARM"
            success_count = 0
            for i in range(3):  # 3번 전송
                sent = self.send_mavlink2_message(msg_id, payload)
                if sent > 0:
                    success_count += 1
                time.sleep(0.1)  # 100ms 간격
            
            if success_count > 0:
                self.log(f"✓ COMMAND_LONG ({action}) 전송: {success_count}/3 성공")
                self.log(f"  → target_system={target_system}, target_component={target_component}, command={command}, param1={param1_value}")
                self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
                if success_count < 3:
                    self.log(f"  ⚠ 일부 전송 실패: {3-success_count}개")
            else:
                self.log(f"✗ COMMAND_LONG ({action}) 전송 실패")
                self.log(f"  → 네트워크 연결 및 MAVLink 라우터 상태 확인 필요")
            
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

                # 중요: FC가 이전 명령을 처리할 시간을 주기 위해 초기 지연 추가
                # ARMING 명령 후 비행모드 변경 명령이 성공하는 이유는 이 지연 때문일 수 있음
                time.sleep(0.2)  # 200ms 초기 지연 (FC 준비 시간 확보)
                
                # 여러 번 전송 (PX4는 모드 변경 명령을 여러 번 받아야 안정적으로 처리됨)
                # QGC처럼 지속적으로 전송하여 성공률 향상
                success_count = 0
                send_count = 20  # 10회 → 20회로 증가 (QGC처럼 지속적 전송)
                for i in range(send_count):
                    sent = self.send_mavlink2_message(msg_id, payload)
                    if sent > 0:
                        success_count += 1
                    time.sleep(0.1)  # 150ms → 100ms 간격으로 단축 (더 빠른 전송)
                
                if success_count > 0:
                    self.log(f"✓ COMMAND_LONG (DO_SET_MODE) 전송: {success_count}/{send_count} 성공")
                    self.log(f"  → target={target_system}, base_mode={base_mode}, custom_mode={custom_mode} ({mode_name})")
                    self.log(f"  → main_mode={custom_mode >> 16}, MAVLink 라우터가 자동으로 FC로 전달")
                else:
                    self.log(f"✗ COMMAND_LONG (DO_SET_MODE) 전송 실패 (모든 {send_count}회 시도 실패)")

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

                # 중요: FC가 이전 명령을 처리할 시간을 주기 위해 초기 지연 추가
                # ARMING 명령 후 비행모드 변경 명령이 성공하는 이유는 이 지연 때문일 수 있음
                time.sleep(0.2)  # 200ms 초기 지연 (FC 준비 시간 확보)
                
                # 여러 번 전송 (PX4는 모드 변경 명령을 여러 번 받아야 안정적으로 처리됨)
                # QGC처럼 지속적으로 전송하여 성공률 향상
                success_count = 0
                send_count = 20  # 10회 → 20회로 증가 (QGC처럼 지속적 전송)
                for i in range(send_count):
                    sent = self.send_mavlink2_message(msg_id, payload)
                    if sent > 0:
                        success_count += 1
                    time.sleep(0.1)  # 150ms → 100ms 간격으로 단축 (더 빠른 전송)
                
                if success_count > 0:
                    self.log(f"✓ SET_MODE 전송: {success_count}/{send_count} 성공")
                    self.log(f"  → target={target_system}, base_mode={base_mode}, custom_mode={custom_mode} ({mode_name})")
                    self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
                else:
                    self.log(f"✗ SET_MODE 전송 실패")

        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except Exception as e:
            self.log(f"✗ 모드 설정 전송 실패: {e}")
    
    def send_takeoff(self):
        """TAKEOFF 명령 전송 (COMMAND_LONG)"""
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            altitude = float(self.takeoff_alt_entry.get())
            pitch = float(self.takeoff_pitch_entry.get())
            yaw = float(self.takeoff_yaw_entry.get())
            
            # COMMAND_LONG 메시지 ID: 76
            msg_id = 76
            command = 22  # MAV_CMD_NAV_TAKEOFF
            
            # COMMAND_LONG 페이로드
            # param1: 최소 피치 (pitch)
            # param2: 빈 값
            # param3: 빈 값
            # param4: Yaw (방향)
            # param5: 위도 (0 = 현재 위치)
            # param6: 경도 (0 = 현재 위치)
            # param7: 고도 (altitude)
            payload = struct.pack(
                '<BBHB7f',
                target_system,
                target_component,
                command,
                0,  # confirmation
                pitch,      # param1: 최소 피치
                0.0,        # param2: 빈 값
                0.0,        # param3: 빈 값
                yaw,        # param4: Yaw
                0.0,        # param5: 위도 (0 = 현재 위치)
                0.0,        # param6: 경도 (0 = 현재 위치)
                altitude    # param7: 고도
            )
            
            # 여러 번 전송
            success_count = 0
            for i in range(3):
                sent = self.send_mavlink2_message(msg_id, payload)
                if sent > 0:
                    success_count += 1
                time.sleep(0.1)
            
            if success_count > 0:
                self.log(f"✓ TAKEOFF 전송: {success_count}/3 성공")
                self.log(f"  → 고도={altitude}m, 피치={pitch}°, Yaw={yaw}°")
                self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
            else:
                self.log(f"✗ TAKEOFF 전송 실패")
                
        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except Exception as e:
            self.log(f"✗ TAKEOFF 전송 실패: {e}")
    
    def send_land(self):
        """LAND 명령 전송 (COMMAND_LONG)"""
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            lat = float(self.land_lat_entry.get())
            lon = float(self.land_lon_entry.get())
            alt = float(self.land_alt_entry.get())
            
            # COMMAND_LONG 메시지 ID: 76
            msg_id = 76
            command = 21  # MAV_CMD_NAV_LAND
            
            # COMMAND_LONG 페이로드
            # param1: 빈 값
            # param2: 빈 값
            # param3: 빈 값
            # param4: Yaw (방향, 0 = 현재 방향 유지)
            # param5: 위도 (0 = 현재 위치)
            # param6: 경도 (0 = 현재 위치)
            # param7: 고도 (0 = 현재 고도)
            payload = struct.pack(
                '<BBHB7f',
                target_system,
                target_component,
                command,
                0,  # confirmation
                0.0,        # param1: 빈 값
                0.0,        # param2: 빈 값
                0.0,        # param3: 빈 값
                0.0,        # param4: Yaw (0 = 현재 방향 유지)
                lat,        # param5: 위도 (0 = 현재 위치)
                lon,        # param6: 경도 (0 = 현재 위치)
                alt         # param7: 고도 (0 = 현재 고도)
            )
            
            # 여러 번 전송
            success_count = 0
            for i in range(3):
                sent = self.send_mavlink2_message(msg_id, payload)
                if sent > 0:
                    success_count += 1
                time.sleep(0.1)
            
            if success_count > 0:
                if lat == 0.0 and lon == 0.0:
                    self.log(f"✓ LAND 전송: {success_count}/3 성공 (현재 위치에 착륙)")
                else:
                    self.log(f"✓ LAND 전송: {success_count}/3 성공")
                    self.log(f"  → 위치: {lat}°, {lon}°, 고도={alt}m")
                self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
            else:
                self.log(f"✗ LAND 전송 실패")
                
        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except Exception as e:
            self.log(f"✗ LAND 전송 실패: {e}")
    
    def send_rtl(self):
        """RTL (Return to Launch) 명령 전송 (COMMAND_LONG)"""
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            
            # COMMAND_LONG 메시지 ID: 76
            msg_id = 76
            command = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
            
            # COMMAND_LONG 페이로드
            # RTL은 파라미터가 모두 0
            payload = struct.pack(
                '<BBHB7f',
                target_system,
                target_component,
                command,
                0,  # confirmation
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # param1-7 모두 0
            )
            
            # 여러 번 전송
            success_count = 0
            for i in range(3):
                sent = self.send_mavlink2_message(msg_id, payload)
                if sent > 0:
                    success_count += 1
                time.sleep(0.1)
            
            if success_count > 0:
                self.log(f"✓ RTL (Return to Launch) 전송: {success_count}/3 성공")
                self.log(f"  → 출발지로 복귀")
                self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
            else:
                self.log(f"✗ RTL 전송 실패")
                
        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except Exception as e:
            self.log(f"✗ RTL 전송 실패: {e}")
    
    def send_position_target(self):
        """SET_POSITION_TARGET_GLOBAL_INT 메시지 전송"""
        try:
            target_system = int(self.target_system_entry.get())
            lat = float(self.pos_lat_entry.get())
            lon = float(self.pos_lon_entry.get())
            alt = float(self.pos_alt_entry.get())
            vx = float(self.pos_vx_entry.get())
            pos_type = int(self.pos_type_combo.get().split('(')[1].split(')')[0])
            
            # SET_POSITION_TARGET_GLOBAL_INT 메시지 ID: 86
            msg_id = 86
            
            # 좌표 변환 (degrees * 1e7)
            lat_int = int(lat * 1e7)
            lon_int = int(lon * 1e7)
            alt_float = alt  # 고도는 float (m, MSL)
            
            # SET_POSITION_TARGET_GLOBAL_INT 페이로드 구조
            # time_boot_ms(4) + target_system(1) + target_component(1) + 
            # coordinate_frame(1) + type_mask(2) + lat(4) + lon(4) + alt(4) + 
            # vx(4) + vy(4) + vz(4) + afx(4) + afy(4) + afz(4) + yaw(4) + yaw_rate(4) = 51 bytes
            payload = struct.pack(
                '<IBBBIiifffffffff',
                0,              # time_boot_ms (uint32_t) - 0 = 즉시
                target_system,  # target_system (uint8_t)
                0,              # target_component (uint8_t) - 0 = 모든 컴포넌트
                6,              # coordinate_frame (uint8_t) - MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
                pos_type,       # type_mask (uint16_t) - 위치/속도/가속도 타입
                lat_int,        # lat (int32_t) - degrees * 1e7
                lon_int,        # lon (int32_t) - degrees * 1e7
                alt_float,      # alt (float) - 고도 (m, MSL)
                vx,             # vx (float) - X 속도 (m/s)
                0.0,            # vy (float) - Y 속도 (m/s)
                0.0,            # vz (float) - Z 속도 (m/s)
                0.0,            # afx (float) - X 가속도 (m/s²)
                0.0,            # afy (float) - Y 가속도 (m/s²)
                0.0,            # afz (float) - Z 가속도 (m/s²)
                0.0,            # yaw (float) - Yaw (rad)
                0.0             # yaw_rate (float) - Yaw rate (rad/s)
            )
            
            # 여러 번 전송
            success_count = 0
            for i in range(3):
                sent = self.send_mavlink2_message(msg_id, payload)
                if sent > 0:
                    success_count += 1
                time.sleep(0.1)
            
            if success_count > 0:
                self.log(f"✓ SET_POSITION_TARGET_GLOBAL_INT 전송: {success_count}/3 성공")
                self.log(f"  → 위치: {lat}°, {lon}°, 고도={alt}m, 속도={vx}m/s")
                self.log(f"  → MAVLink 라우터가 자동으로 FC로 전달 (VIM4 메인 프로그램 불필요)")
            else:
                self.log(f"✗ 위치 이동 전송 실패")
                
        except ValueError as e:
            self.log(f"✗ 입력값 오류: {e}")
        except Exception as e:
            self.log(f"✗ 위치 이동 전송 실패: {e}")

    def on_drone_selection_changed(self, event=None):
        """기체 선택이 변경될 때 IP와 Target System 자동 설정"""
        selection = self.drone_selection.get()
        
        if selection == "기체 1":
            self.ip_entry.delete(0, tk.END)
            self.ip_entry.insert(0, "192.168.100.11")
            self.target_system_entry.delete(0, tk.END)
            self.target_system_entry.insert(0, "1")
            self.log("기체 1 선택: IP=192.168.100.11, Target System=1")
        elif selection == "기체 2":
            self.ip_entry.delete(0, tk.END)
            self.ip_entry.insert(0, "192.168.100.21")
            self.target_system_entry.delete(0, tk.END)
            self.target_system_entry.insert(0, "2")
            self.log("기체 2 선택: IP=192.168.100.21, Target System=2")
        elif selection == "기체 3":
            self.ip_entry.delete(0, tk.END)
            self.ip_entry.insert(0, "192.168.100.31")
            self.target_system_entry.delete(0, tk.END)
            self.target_system_entry.insert(0, "3")
            self.log("기체 3 선택: IP=192.168.100.31, Target System=3")
        elif selection == "수동 설정":
            # 수동 설정 선택 시에는 현재 값 유지
            self.log("수동 설정 모드: IP와 Target System을 직접 입력하세요")
    
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



