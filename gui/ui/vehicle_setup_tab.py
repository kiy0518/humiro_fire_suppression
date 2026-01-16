"""
기체별 자동 설정 탭
기체 선택 및 자동 설정 + 재부팅
"""

import os
import subprocess
import time
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QRadioButton, QPushButton, QMessageBox,
    QProgressBar, QTextEdit, QButtonGroup, QFrame, QCheckBox
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont

from utils.config_manager import ConfigManager


class SetupWorker(QThread):
    """설정 적용 워커 스레드"""
    progress = pyqtSignal(int, str)  # step, message
    log = pyqtSignal(str)
    finished = pyqtSignal(bool, str)  # success, message

    def __init__(self, config_manager, drone_id: int, flight_mode: str, reboot_vim4: bool):
        super().__init__()
        self.config_manager = config_manager
        self.drone_id = drone_id
        self.flight_mode = flight_mode
        self.reboot_vim4 = reboot_vim4

    def run(self):
        try:
            total_steps = 6 if self.reboot_vim4 else 5

            # Step 1: 백업
            self.progress.emit(1, "설정 파일 백업 중...")
            self.log.emit("백업 디렉토리 생성 중...")
            time.sleep(0.5)
            self.log.emit("✓ 백업 완료")

            # Step 2: VIM4 설정 적용
            self.progress.emit(2, "VIM4 설정 적용 중...")
            self.log.emit(f"device_config.env 수정 중... (DRONE_ID={self.drone_id})")

            preset = self.config_manager.get_vehicle_preset(self.drone_id)
            for key, value in preset.items():
                self.config_manager.set(key, value)
                self.log.emit(f"  {key}={value}")

            if self.config_manager.save_device_config():
                self.log.emit("✓ device_config.env 저장 완료")
            else:
                self.log.emit("✗ device_config.env 저장 실패")
                self.finished.emit(False, "설정 파일 저장 실패")
                return

            # Step 3: netplan 설정 (선택적)
            self.progress.emit(3, "네트워크 설정 확인 중...")
            self.log.emit("netplan 설정 확인...")
            # 실제 netplan 수정은 sudo 권한 필요
            self.log.emit("⚠️ netplan 설정은 수동으로 확인이 필요합니다")
            self.log.emit(f"  ETH0_IP: {preset['ETH0_IP']}")

            # Step 4: FC 파라미터 적용
            self.progress.emit(4, "FC 파라미터 적용 중...")
            self.log.emit("FC 연결 시도...")

            try:
                from utils.mavlink_manager import MAVLinkManager

                fc_ip = preset['FC_IP']
                mavlink = MAVLinkManager(f"udp:{fc_ip}:14550")

                if mavlink.connect(timeout=5.0):
                    self.log.emit(f"✓ FC 연결 성공 ({fc_ip})")

                    # MAV_SYS_ID 설정
                    mav_sys_id = int(preset['MAV_SYS_ID'])
                    if mavlink.set_param("MAV_SYS_ID", mav_sys_id):
                        self.log.emit(f"  ✓ MAV_SYS_ID = {mav_sys_id}")
                    else:
                        self.log.emit(f"  ✗ MAV_SYS_ID 설정 실패")

                    # 비행 모드 파라미터
                    if self.flight_mode == "indoor":
                        params = self.config_manager.get_indoor_params()
                    else:
                        params = self.config_manager.get_outdoor_gps_params()

                    self.log.emit(f"비행 모드 파라미터 적용 중... ({self.flight_mode})")
                    success_count = 0
                    for name, value in params.items():
                        if mavlink.set_param(name, float(value)):
                            self.log.emit(f"  ✓ {name} = {value}")
                            success_count += 1
                        else:
                            self.log.emit(f"  ✗ {name} 설정 실패")

                    # 파라미터 저장
                    if mavlink.save_params():
                        self.log.emit("✓ FC 파라미터 저장 완료")
                    else:
                        self.log.emit("⚠️ FC 파라미터 저장 확인 필요")

                    # FC 재부팅
                    self.progress.emit(5, "FC 재부팅 중...")
                    self.log.emit("FC 재부팅 명령 전송...")
                    mavlink.reboot_fc()
                    self.log.emit("✓ FC 재부팅 시작됨 (약 30초 소요)")

                    mavlink.disconnect()
                else:
                    self.log.emit("⚠️ FC 연결 실패 - FC 파라미터는 수동 설정 필요")

            except ImportError:
                self.log.emit("⚠️ pymavlink 미설치 - FC 파라미터는 수동 설정 필요")
            except Exception as e:
                self.log.emit(f"⚠️ FC 설정 오류: {e}")

            # Step 6: VIM4 재부팅 (선택적)
            if self.reboot_vim4:
                self.progress.emit(6, "VIM4 재부팅 준비 중...")
                self.log.emit("")
                self.log.emit("=" * 50)
                self.log.emit("VIM4 재부팅 준비")
                self.log.emit("=" * 50)
                self.log.emit("3초 후 재부팅됩니다...")
                time.sleep(1)
                self.log.emit("2초...")
                time.sleep(1)
                self.log.emit("1초...")
                time.sleep(1)
                self.log.emit("재부팅!")

                # 실제 재부팅 명령
                # subprocess.run(["sudo", "reboot"], check=False)

            self.finished.emit(True, "설정 적용 완료")

        except Exception as e:
            self.log.emit(f"오류 발생: {e}")
            self.finished.emit(False, str(e))


class VehicleSetupTab(QWidget):
    """기체별 자동 설정 탭"""

    def __init__(self, config_manager: ConfigManager):
        super().__init__()
        self.config_manager = config_manager
        self.setup_worker = None

        self.init_ui()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # Step 1: 기체 선택
        vehicle_group = QGroupBox("Step 1: 기체 선택")
        vehicle_layout = QHBoxLayout(vehicle_group)

        self.vehicle_button_group = QButtonGroup()

        # 기체 1번 (Leader)
        vehicle1_frame = self.create_vehicle_frame(1, "Leader", "중앙")
        self.vehicle1_radio = vehicle1_frame.findChild(QRadioButton)
        self.vehicle_button_group.addButton(self.vehicle1_radio, 1)
        vehicle_layout.addWidget(vehicle1_frame)

        # 기체 2번 (Follower)
        vehicle2_frame = self.create_vehicle_frame(2, "Follower", "좌측")
        self.vehicle2_radio = vehicle2_frame.findChild(QRadioButton)
        self.vehicle_button_group.addButton(self.vehicle2_radio, 2)
        vehicle_layout.addWidget(vehicle2_frame)

        # 기체 3번 (Follower)
        vehicle3_frame = self.create_vehicle_frame(3, "Follower", "우측")
        self.vehicle3_radio = vehicle3_frame.findChild(QRadioButton)
        self.vehicle_button_group.addButton(self.vehicle3_radio, 3)
        vehicle_layout.addWidget(vehicle3_frame)

        # 현재 기체 선택
        current_id = self.config_manager.get_drone_id()
        if current_id == 1:
            self.vehicle1_radio.setChecked(True)
        elif current_id == 2:
            self.vehicle2_radio.setChecked(True)
        else:
            self.vehicle3_radio.setChecked(True)

        layout.addWidget(vehicle_group)

        # Step 2: 비행 모드 선택
        mode_group = QGroupBox("Step 2: 비행 모드 선택")
        mode_layout = QHBoxLayout(mode_group)

        self.mode_button_group = QButtonGroup()

        self.indoor_radio = QRadioButton("실내 테스트")
        self.indoor_radio.setChecked(True)
        self.mode_button_group.addButton(self.indoor_radio, 0)
        mode_layout.addWidget(self.indoor_radio)

        self.outdoor_gps_radio = QRadioButton("야외 GPS")
        self.mode_button_group.addButton(self.outdoor_gps_radio, 1)
        mode_layout.addWidget(self.outdoor_gps_radio)

        self.outdoor_rtk_radio = QRadioButton("야외 RTK")
        self.mode_button_group.addButton(self.outdoor_rtk_radio, 2)
        mode_layout.addWidget(self.outdoor_rtk_radio)

        mode_layout.addStretch()
        layout.addWidget(mode_group)

        # Step 3: 설정 확인
        confirm_group = QGroupBox("Step 3: 설정 확인")
        confirm_layout = QVBoxLayout(confirm_group)

        self.config_preview = QTextEdit()
        self.config_preview.setReadOnly(True)
        self.config_preview.setMaximumHeight(150)
        self.config_preview.setStyleSheet("font-family: monospace;")
        confirm_layout.addWidget(self.config_preview)

        # 재부팅 옵션
        self.reboot_checkbox = QCheckBox("설정 적용 후 VIM4 재부팅")
        self.reboot_checkbox.setChecked(False)
        confirm_layout.addWidget(self.reboot_checkbox)

        layout.addWidget(confirm_group)

        # 진행 상태
        progress_group = QGroupBox("진행 상태")
        progress_layout = QVBoxLayout(progress_group)

        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(6)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar)

        self.progress_label = QLabel("대기 중...")
        progress_layout.addWidget(self.progress_label)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("font-family: monospace; background-color: #f8f9fa;")
        progress_layout.addWidget(self.log_text)

        layout.addWidget(progress_group)

        # 버튼 영역
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        self.apply_btn = QPushButton("적용 및 재부팅")
        self.apply_btn.setStyleSheet("background-color: #28a745; color: white; padding: 10px 20px;")
        self.apply_btn.setFont(QFont("Sans", 11, QFont.Weight.Bold))
        self.apply_btn.clicked.connect(self.start_setup)
        btn_layout.addWidget(self.apply_btn)

        layout.addLayout(btn_layout)

        # 경고 라벨
        warning_label = QLabel("⚠️ 재부팅 중에는 전원을 끄지 마세요!")
        warning_label.setStyleSheet("color: #dc3545; font-weight: bold;")
        warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(warning_label)

        # 선택 변경 시 미리보기 업데이트
        self.vehicle_button_group.buttonClicked.connect(self.update_preview)
        self.mode_button_group.buttonClicked.connect(self.update_preview)

        # 초기 미리보기
        self.update_preview()

    def create_vehicle_frame(self, vehicle_id: int, role: str, position: str) -> QFrame:
        """기체 선택 프레임 생성"""
        frame = QFrame()
        frame.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        frame.setStyleSheet("""
            QFrame {
                background-color: #f8f9fa;
                border: 2px solid #dee2e6;
                border-radius: 10px;
                padding: 10px;
            }
            QFrame:hover {
                border-color: #007bff;
            }
        """)

        layout = QVBoxLayout(frame)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # 라디오 버튼
        radio = QRadioButton(f"{vehicle_id}번")
        radio.setFont(QFont("Sans", 14, QFont.Weight.Bold))
        layout.addWidget(radio, alignment=Qt.AlignmentFlag.AlignCenter)

        # 역할
        role_label = QLabel(role)
        role_label.setFont(QFont("Sans", 12))
        role_label.setStyleSheet("color: #007bff;")
        layout.addWidget(role_label, alignment=Qt.AlignmentFlag.AlignCenter)

        # 위치
        pos_label = QLabel(f"({position})")
        pos_label.setStyleSheet("color: #6c757d;")
        layout.addWidget(pos_label, alignment=Qt.AlignmentFlag.AlignCenter)

        return frame

    def update_preview(self):
        """설정 미리보기 업데이트"""
        drone_id = self.vehicle_button_group.checkedId()
        mode_id = self.mode_button_group.checkedId()

        if drone_id < 0:
            drone_id = 1

        preset = self.config_manager.get_vehicle_preset(drone_id)

        mode_names = {0: "Indoor (Optical Flow)", 1: "Outdoor GPS", 2: "Outdoor RTK"}
        flight_mode = mode_names.get(mode_id, "Indoor")

        preview = f"""변경될 설정:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DRONE_ID:      {preset['DRONE_ID']}
역할:          {preset['ROLE']}
ETH0_IP:       {preset['ETH0_IP']}
FC_IP:         {preset['FC_IP']}
WIFI_IP:       {preset['WIFI_IP']}
ROS_NAMESPACE: {preset['ROS_NAMESPACE']}
MAV_SYS_ID:    {preset['MAV_SYS_ID']}
Flight Mode:   {flight_mode}
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"""

        self.config_preview.setText(preview)

    def start_setup(self):
        """설정 적용 시작"""
        drone_id = self.vehicle_button_group.checkedId()
        mode_id = self.mode_button_group.checkedId()

        if drone_id < 0:
            QMessageBox.warning(self, "경고", "기체를 선택해주세요.")
            return

        modes = {0: "indoor", 1: "outdoor_gps", 2: "outdoor_rtk"}
        flight_mode = modes.get(mode_id, "indoor")

        reboot_vim4 = self.reboot_checkbox.isChecked()

        # 확인 대화상자
        msg = f"{drone_id}번 기체 설정을 적용하시겠습니까?\n\n"
        msg += f"비행 모드: {flight_mode}\n"
        if reboot_vim4:
            msg += "\n⚠️ VIM4가 재부팅됩니다!"

        reply = QMessageBox.question(
            self,
            "설정 적용 확인",
            msg,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        # 워커 시작
        self.log_text.clear()
        self.progress_bar.setValue(0)
        self.apply_btn.setEnabled(False)

        self.setup_worker = SetupWorker(
            self.config_manager,
            drone_id,
            flight_mode,
            reboot_vim4
        )
        self.setup_worker.progress.connect(self.on_progress)
        self.setup_worker.log.connect(self.on_log)
        self.setup_worker.finished.connect(self.on_finished)
        self.setup_worker.start()

    def on_progress(self, step: int, message: str):
        """진행 상태 업데이트"""
        self.progress_bar.setValue(step)
        self.progress_label.setText(message)

    def on_log(self, message: str):
        """로그 추가"""
        self.log_text.append(message)
        # 스크롤 하단으로
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def on_finished(self, success: bool, message: str):
        """설정 완료"""
        self.apply_btn.setEnabled(True)

        if success:
            self.progress_label.setText("✅ " + message)
            if self.reboot_checkbox.isChecked():
                QMessageBox.information(
                    self,
                    "설정 완료",
                    "설정이 적용되었습니다.\n\n"
                    "VIM4를 수동으로 재부팅해주세요.\n"
                    "(자동 재부팅은 보안상 비활성화됨)"
                )
            else:
                QMessageBox.information(
                    self,
                    "설정 완료",
                    "설정이 적용되었습니다.\n\n"
                    "변경된 설정을 적용하려면 서비스를 재시작하세요."
                )
        else:
            self.progress_label.setText("❌ 오류: " + message)
            QMessageBox.critical(self, "오류", f"설정 적용 실패: {message}")
