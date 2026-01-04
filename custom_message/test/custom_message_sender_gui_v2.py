#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
커스텀 메시지 송신 GUI 프로그램 (MAVLink 2.0)
외부 PC에서 실행하여 설정된 포트로 MAVLink 메시지를 전송합니다.

포트 설정:
- 14550: MAVLink router → FC (ARM, 비행모드 등)
- 15000: 커스텀 메시지 테스트용

요구사항: Python 3.x (추가 패키지 불필요)

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
        self.root.title("커스텀 메시지 송신 GUI (MAVLink 2.0)")
        self.root.geometry("900x800")

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

        # UDP 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # MAVLink 2.0 시퀀스 번호
        self.sequence_number = 0

        self.create_widgets()
        self.log("프로그램 시작 (MAVLink 2.0)")
    
    def get_korean_font(self):
        """한글을 지원하는 폰트 찾기"""
        try:
            available_fonts = [f.lower() for f in font.families()]
            korean_fonts = [
                'nanumgothic', 'nanumbarungothic', 'noto sans cjk kr',
                'noto sans kr', 'malgun gothic', 'gulim', 'dotum',
                'batang', 'unifont', 'dejavu sans'
            ]
            
            for font_name in korean_fonts:
                for available_font in font.families():
                    if font_name in available_font.lower():
                        return (available_font, 9)
            return None
        except Exception as e:
            return None

    def create_widgets(self):
        # 상단 연결 설정
        conn_frame = ttk.LabelFrame(self.root, text="📡 연결 설정", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        # 첫 번째 행: IP와 포트
        row1 = ttk.Frame(conn_frame)
        row1.pack(fill="x", pady=2)

        ttk.Label(row1, text="대상 IP:").grid(row=0, column=0, sticky="w", padx=5)
        self.ip_entry = ttk.Entry(row1, width=20)
        self.ip_entry.insert(0, "192.168.100.11")
        self.ip_entry.grid(row=0, column=1, padx=5)

        ttk.Label(row1, text="포트:").grid(row=0, column=2, sticky="w", padx=(20, 5))
        self.port_entry = ttk.Entry(row1, width=10)
        self.port_entry.insert(0, "14550")
        self.port_entry.grid(row=0, column=3, padx=5)

        # 포트 프리셋 버튼
        ttk.Button(row1, text="14550 (FC)", 
                  command=lambda: self.set_port(14550)).grid(row=0, column=4, padx=2)
        ttk.Button(row1, text="15000 (Test)", 
                  command=lambda: self.set_port(15000)).grid(row=0, column=5, padx=2)

        # 두 번째 행: System/Component ID
        row2 = ttk.Frame(conn_frame)
        row2.pack(fill="x", pady=2)

        ttk.Label(row2, text="System ID (송신자):").grid(row=0, column=0, sticky="w", padx=5)
        self.system_id_entry = ttk.Entry(row2, width=10)
        self.system_id_entry.insert(0, "255")
        self.system_id_entry.grid(row=0, column=1, padx=5, sticky="w")

        ttk.Label(row2, text="Component ID (송신자):").grid(row=0, column=2, sticky="w", padx=(20, 5))
        self.component_id_entry = ttk.Entry(row2, width=10)
        self.component_id_entry.insert(0, "190")
        self.component_id_entry.grid(row=0, column=3, padx=5, sticky="w")

        # 세 번째 행: Target System/Component ID
        row3 = ttk.Frame(conn_frame)
        row3.pack(fill="x", pady=2)
        
        ttk.Label(row3, text="Target System (FC):").grid(row=0, column=0, sticky="w", padx=5)
        self.target_system_entry = ttk.Entry(row3, width=10)
        self.target_system_entry.insert(0, "1")
        self.target_system_entry.grid(row=0, column=1, padx=5, sticky="w")

        ttk.Label(row3, text="Target Component (FC):").grid(row=0, column=2, sticky="w", padx=(20, 5))
        self.target_component_entry = ttk.Entry(row3, width=10)
        self.target_component_entry.insert(0, "1")
        self.target_component_entry.grid(row=0, column=3, padx=5, sticky="w")

        # 메시지 전송 섹션
        msg_frame = ttk.LabelFrame(self.root, text="메시지 전송", padding=10)
        msg_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # PX4 제어 메시지 (우선 배치)
        self.create_arming_section(msg_frame)
        self.create_set_mode_section(msg_frame)

        # 커스텀 메시지
        self.create_mission_start_section(msg_frame)
        self.create_mission_status_section(msg_frame)
        self.create_suppression_result_section(msg_frame)
        self.create_launch_control_section(msg_frame)

        # 로그 영역
        log_frame = ttk.LabelFrame(self.root, text="📝 로그", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        font_config = self.korean_font if self.korean_font else ('TkDefaultFont', 9)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, state='disabled', font=font_config)
        self.log_text.pack(fill="both", expand=True)

        # 로그 색상 태그
        self.log_text.tag_config('info', foreground='blue')
        self.log_text.tag_config('success', foreground='green')
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('warning', foreground='orange')

    def set_port(self, port):
        """포트 설정"""
        self.port_entry.delete(0, tk.END)
        self.port_entry.insert(0, str(port))
        port_name = "FC (MAVLink router)" if port == 14550 else "Test (커스텀)"
        self.log(f"포트 변경: {port} ({port_name})", "info")

    def create_arming_section(self, parent):
        frame = ttk.LabelFrame(parent, text="1️⃣ ARMING (무장/해제) - 14550 포트 권장", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Button(row_frame, text="🔓 ARM (무장)",
                  command=lambda: self.send_arming(1), width=20).pack(side="left", padx=5)
        ttk.Button(row_frame, text="🔒 DISARM (해제)",
                  command=lambda: self.send_arming(0), width=20).pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="현재 포트:").pack(side="left", padx=(20, 5))
        self.arm_port_label = ttk.Label(row_frame, text="", foreground="blue")
        self.arm_port_label.pack(side="left")

    def create_set_mode_section(self, parent):
        frame = ttk.LabelFrame(parent, text="2️⃣ 비행 모드 설정 (PX4) - 14550 포트 권장", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="모드:").pack(side="left", padx=5)
        self.mode_combo = ttk.Combobox(row_frame, width=25, state="readonly")
        # PX4 Custom Mode 값
        self.mode_combo['values'] = [
            '1: MANUAL (수동)',
            '2: ALTCTL (고도 제어)',
            '3: POSCTL (위치 제어)',
            '4: AUTO.MISSION (자동 미션)',
            '5: AUTO.LOITER (자동 배회)',
            '6: AUTO.RTL (귀환)',
            '7: ACRO (곡예)',
            '8: OFFBOARD (외부 제어)',
            '9: STABILIZED (안정화)',
            '10: RATTITUDE',
            '11: AUTO.TAKEOFF (자동 이륙)',
            '12: AUTO.LAND (자동 착륙)',
            '13: AUTO.FOLLOW_TARGET (추적)',
            '14: AUTO.PRECLAND (정밀 착륙)'
        ]
        self.mode_combo.current(0)
        self.mode_combo.pack(side="left", padx=5)

        ttk.Button(row_frame, text="✈️ 모드 설정", 
                  command=self.send_set_mode, width=15).pack(side="left", padx=5)
        
        ttk.Label(row_frame, text="현재 포트:").pack(side="left", padx=(20, 5))
        self.mode_port_label = ttk.Label(row_frame, text="", foreground="blue")
        self.mode_port_label.pack(side="left")

    def create_mission_start_section(self, parent):
        frame = ttk.LabelFrame(parent, text="3️⃣ FIRE_MISSION_START (미션 시작) - 15000 포트 권장", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="위도(°):").pack(side="left")
        self.lat_entry = ttk.Entry(row_frame, width=15)
        self.lat_entry.insert(0, "37.5665")
        self.lat_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="경도(°):").pack(side="left")
        self.lon_entry = ttk.Entry(row_frame, width=15)
        self.lon_entry.insert(0, "126.9780")
        self.lon_entry.pack(side="left", padx=5)

        ttk.Label(row_frame, text="고도(m):").pack(side="left")
        self.alt_entry = ttk.Entry(row_frame, width=10)
        self.alt_entry.insert(0, "10.0")
        self.alt_entry.pack(side="left", padx=5)

        self.auto_fire_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(row_frame, text="자동 발사", variable=self.auto_fire_var).pack(side="left", padx=10)

        ttk.Button(row_frame, text="🚀 전송", command=self.send_mission_start).pack(side="right", padx=5)

    def create_mission_status_section(self, parent):
        frame = ttk.LabelFrame(parent, text="4️⃣ FIRE_MISSION_STATUS (미션 상태)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Label(row_frame, text="Phase:").pack(side="left")
        self.phase_combo = ttk.Combobox(row_frame, width=20, state="readonly")
        self.phase_combo['values'] = [
            '0: IDLE', '1: NAVIGATING', '2: SCANNING',
            '3: READY_TO_FIRE', '4: SUPPRESSING', '5: VERIFYING', '6: COMPLETE'
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

        ttk.Button(row_frame, text="📊 전송", command=self.send_mission_status).pack(side="right", padx=5)

    def create_suppression_result_section(self, parent):
        frame = ttk.LabelFrame(parent, text="5️⃣ FIRE_SUPPRESSION_RESULT (소화 결과)", padding=5)
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

        ttk.Button(row_frame, text="🎯 전송", command=self.send_suppression_result).pack(side="right", padx=5)

    def create_launch_control_section(self, parent):
        frame = ttk.LabelFrame(parent, text="6️⃣ FIRE_LAUNCH_CONTROL (발사 제어)", padding=5)
        frame.pack(fill="x", pady=5)

        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x")

        ttk.Button(row_frame, text="✅ 확인 (CONFIRM)",
                  command=lambda: self.send_launch_control(0), width=20).pack(side="left", padx=5)
        ttk.Button(row_frame, text="❌ 중단 (ABORT)",
                  command=lambda: self.send_launch_control(1), width=20).pack(side="left", padx=5)
        ttk.Button(row_frame, text="❓ 상태 요청",
                  command=lambda: self.send_launch_control(2), width=20).pack(side="left", padx=5)

    def calculate_crc16(self, data, initial_crc=0xFFFF):
        """MAVLink 2.0 CRC-16/MCRF4XX 계산"""
        crc = initial_crc
        if isinstance(data, bytes):
            data = list(data)
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return crc

    def send_mavlink2_message(self, msg_id, payload):
        """MAVLink 2.0 메시지 패킷 생성 및 전송"""
        MAVLINK_MAGIC = 0xFD
        payload_len = len(payload)
        system_id = int(self.system_id_entry.get())
        component_id = int(self.component_id_entry.get())
        
        msg_id_low = msg_id & 0xFF
        msg_id_mid = (msg_id >> 8) & 0xFF
        msg_id_high = (msg_id >> 16) & 0xFF
        
        header = struct.pack(
            '<BBBBBBBBBB',
            MAVLINK_MAGIC,
            payload_len,
            0,  # incompat_flags
            0,  # compat_flags
            self.sequence_number & 0xFF,
            system_id,
            component_id,
            msg_id_low,
            msg_id_mid,
            msg_id_high
        )
        
        self.sequence_number = (self.sequence_number + 1) % 256
        
        # CRC_EXTRA 값
        crc_extra_map = {
            12900: 100, 12901: 101, 12902: 102, 12903: 103,
            76: 152,  # COMMAND_LONG
            11: 89    # SET_MODE
        }
        crc_extra = crc_extra_map.get(msg_id, 0)
        
        message_without_crc = header[1:] + payload
        crc = self.calculate_crc16(message_without_crc)
        crc = self.calculate_crc16(bytes([crc_extra]), initial_crc=crc)
        
        message = header + payload + struct.pack('<H', crc)
        
        # GUI에서 설정된 포트로 전송
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
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())

            payload = struct.pack('<BBiifBB',
                target_system, target_component,
                lat, lon, alt, auto_fire, 5
            )

            self.send_mavlink2_message(12900, payload)
            port = self.port_entry.get()
            self.log(f"✓ FIRE_MISSION_START 전송 (포트:{port}): {lat/1e7}°, {lon/1e7}°, {alt}m", "success")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}", "error")

    def send_mission_status(self):
        try:
            phase = int(self.phase_combo.get().split(':')[0])
            progress = int(self.progress_entry.get())
            distance = float(self.distance_entry.get())

            status_text = "Flying to target".encode('utf-8')
            status_text_padded = status_text[:49].ljust(50, b'\0')
            
            payload = struct.pack('<BBBfh50s',
                phase, progress, 8, distance, 850, status_text_padded
            )

            self.send_mavlink2_message(12901, payload)
            port = self.port_entry.get()
            self.log(f"✓ FIRE_MISSION_STATUS 전송 (포트:{port}): Phase={phase}, {progress}%", "success")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}", "error")

    def send_suppression_result(self):
        try:
            shot = int(self.shot_entry.get())
            temp_before = int(float(self.temp_before_entry.get()) * 10)
            temp_after = int(float(self.temp_after_entry.get()) * 10)
            success = 1 if self.success_var.get() else 0

            payload = struct.pack('<BhhB', shot, temp_before, temp_after, success)

            self.send_mavlink2_message(12903, payload)
            port = self.port_entry.get()
            self.log(f"✓ FIRE_SUPPRESSION_RESULT 전송 (포트:{port}): Shot={shot}", "success")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}", "error")

    def send_launch_control(self, command):
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())

            payload = struct.pack('<BBB', target_system, target_component, command)

            cmd_names = {0: "CONFIRM", 1: "ABORT", 2: "REQUEST_STATUS"}
            self.send_mavlink2_message(12902, payload)
            port = self.port_entry.get()
            self.log(f"✓ FIRE_LAUNCH_CONTROL 전송 (포트:{port}): {cmd_names.get(command)}", "success")

        except Exception as e:
            self.log(f"✗ 전송 실패: {e}", "error")

    def send_arming(self, arm_value):
        """MAVLink COMMAND_LONG로 ARM/DISARM 전송"""
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            
            # COMMAND_LONG (76), MAV_CMD_COMPONENT_ARM_DISARM (400)
            payload = struct.pack('<BBHB7f',
                target_system, target_component,
                400,  # MAV_CMD_COMPONENT_ARM_DISARM
                0,    # confirmation
                float(arm_value),  # param1: 1.0=ARM, 0.0=DISARM
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            )
            
            self.send_mavlink2_message(76, payload)
            action = "ARM" if arm_value == 1 else "DISARM"
            port = self.port_entry.get()
            self.log(f"✓ {action} 전송 (포트:{port}) → FC(sys:{target_system}, comp:{target_component})", "success")
            self.arm_port_label.config(text=f"{port}")
            
        except Exception as e:
            self.log(f"✗ ARM/DISARM 전송 실패: {e}", "error")

    def send_set_mode(self):
        """MAVLink COMMAND_LONG로 비행 모드 설정 (PX4)"""
        try:
            target_system = int(self.target_system_entry.get())
            target_component = int(self.target_component_entry.get())
            
            mode_str = self.mode_combo.get()
            custom_mode = int(mode_str.split(':')[0])
            
            # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            base_mode = 1
            
            # COMMAND_LONG (76), MAV_CMD_DO_SET_MODE (176)
            payload = struct.pack('<BBHB7f',
                target_system, target_component,
                176,  # MAV_CMD_DO_SET_MODE
                0,    # confirmation
                float(base_mode),   # param1: base_mode
                float(custom_mode), # param2: custom_mode
                0.0, 0.0, 0.0, 0.0, 0.0
            )
            
            self.send_mavlink2_message(76, payload)
            mode_name = mode_str.split(':')[1].split('(')[0].strip()
            port = self.port_entry.get()
            self.log(f"✓ 비행모드 설정 (포트:{port}): {mode_name} (custom_mode={custom_mode}) → FC(sys:{target_system})", "success")
            self.mode_port_label.config(text=f"{port}")
            
        except Exception as e:
            self.log(f"✗ SET_MODE 전송 실패: {e}", "error")

    def log(self, message, tag='info'):
        """로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
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
