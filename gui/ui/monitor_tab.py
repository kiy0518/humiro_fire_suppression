"""
모니터 탭
ROS2 토픽 모니터링 및 서비스 관리
"""

import subprocess
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QPushButton, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QTextEdit, QComboBox, QSplitter, QFrame
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont


class TopicMonitorWorker(QThread):
    """토픽 모니터링 워커"""
    data_received = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, topic: str):
        super().__init__()
        self.topic = topic
        self.running = True
        self.process = None

    def run(self):
        try:
            self.process = subprocess.Popen(
                ["ros2", "topic", "echo", self.topic, "--once"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            stdout, stderr = self.process.communicate(timeout=5)
            if stdout:
                self.data_received.emit(stdout)
            if stderr:
                self.error.emit(stderr)

        except subprocess.TimeoutExpired:
            if self.process:
                self.process.kill()
            self.error.emit("Timeout: 토픽 데이터 없음")
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self.running = False
        if self.process:
            self.process.kill()


class MonitorTab(QWidget):
    """모니터 탭"""

    def __init__(self):
        super().__init__()
        self.topic_worker = None
        self.init_ui()
        self.setup_timer()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 스플리터로 상하 분할
        splitter = QSplitter(Qt.Orientation.Vertical)

        # 상단: 서비스 상태
        top_widget = QWidget()
        top_layout = QVBoxLayout(top_widget)

        # 서비스 상태 섹션
        service_group = QGroupBox("서비스 상태")
        service_layout = QVBoxLayout(service_group)

        # 서비스 테이블
        self.service_table = QTableWidget()
        self.service_table.setColumnCount(4)
        self.service_table.setHorizontalHeaderLabels(["서비스", "상태", "PID", "액션"])
        self.service_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.service_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.service_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.service_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        self.service_table.setMaximumHeight(200)
        service_layout.addWidget(self.service_table)

        # 서비스 버튼
        service_btn_layout = QHBoxLayout()
        refresh_btn = QPushButton("새로고침")
        refresh_btn.clicked.connect(self.refresh_services)
        service_btn_layout.addWidget(refresh_btn)

        restart_all_btn = QPushButton("모두 재시작")
        restart_all_btn.clicked.connect(self.restart_all_services)
        service_btn_layout.addWidget(restart_all_btn)

        service_btn_layout.addStretch()
        service_layout.addLayout(service_btn_layout)

        top_layout.addWidget(service_group)
        splitter.addWidget(top_widget)

        # 하단: ROS2 토픽 모니터
        bottom_widget = QWidget()
        bottom_layout = QVBoxLayout(bottom_widget)

        topic_group = QGroupBox("ROS2 토픽 모니터")
        topic_layout = QVBoxLayout(topic_group)

        # 토픽 선택
        topic_select_layout = QHBoxLayout()
        topic_select_layout.addWidget(QLabel("토픽:"))

        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.addItems([
            "/fmu/out/vehicle_status",
            "/fmu/out/battery_status",
            "/fmu/out/vehicle_local_position",
            "/fmu/out/vehicle_global_position",
            "/fmu/out/vehicle_attitude",
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
        ])
        topic_select_layout.addWidget(self.topic_combo)

        refresh_topics_btn = QPushButton("토픽 목록 갱신")
        refresh_topics_btn.clicked.connect(self.refresh_topic_list)
        topic_select_layout.addWidget(refresh_topics_btn)

        topic_layout.addLayout(topic_select_layout)

        # 토픽 정보
        topic_info_layout = QHBoxLayout()

        self.echo_btn = QPushButton("Echo (1회)")
        self.echo_btn.clicked.connect(self.echo_topic)
        topic_info_layout.addWidget(self.echo_btn)

        self.hz_btn = QPushButton("Hz 측정")
        self.hz_btn.clicked.connect(self.measure_hz)
        topic_info_layout.addWidget(self.hz_btn)

        self.info_btn = QPushButton("토픽 정보")
        self.info_btn.clicked.connect(self.show_topic_info)
        topic_info_layout.addWidget(self.info_btn)

        topic_info_layout.addStretch()
        topic_layout.addLayout(topic_info_layout)

        # 토픽 데이터 출력
        self.topic_output = QTextEdit()
        self.topic_output.setReadOnly(True)
        self.topic_output.setStyleSheet("font-family: monospace; background-color: #f8f9fa;")
        topic_layout.addWidget(self.topic_output)

        bottom_layout.addWidget(topic_group)
        splitter.addWidget(bottom_widget)

        layout.addWidget(splitter)

        # 초기 데이터 로드
        QTimer.singleShot(100, self.refresh_services)

    def setup_timer(self):
        """자동 갱신 타이머"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_services)
        self.refresh_timer.start(10000)  # 10초마다

    def refresh_services(self):
        """서비스 상태 갱신"""
        services = [
            "micro-ros-agent",
            "mavlink-router",
            "humiro-fire-suppression",
            "humiro-thermal-streaming",
        ]

        self.service_table.setRowCount(len(services))

        for row, service in enumerate(services):
            # 서비스 이름
            self.service_table.setItem(row, 0, QTableWidgetItem(service))

            # 상태 확인
            try:
                result = subprocess.run(
                    ["systemctl", "is-active", service],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                status = result.stdout.strip()
            except:
                status = "unknown"

            status_item = QTableWidgetItem(status)
            if status == "active":
                status_item.setForeground(Qt.GlobalColor.darkGreen)
            elif status == "inactive":
                status_item.setForeground(Qt.GlobalColor.darkGray)
            else:
                status_item.setForeground(Qt.GlobalColor.red)
            self.service_table.setItem(row, 1, status_item)

            # PID
            try:
                result = subprocess.run(
                    ["systemctl", "show", service, "--property=MainPID", "--value"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                pid = result.stdout.strip()
                if pid == "0":
                    pid = "-"
            except:
                pid = "-"
            self.service_table.setItem(row, 2, QTableWidgetItem(pid))

            # 액션 버튼
            action_widget = QWidget()
            action_layout = QHBoxLayout(action_widget)
            action_layout.setContentsMargins(2, 2, 2, 2)

            if status == "active":
                stop_btn = QPushButton("중지")
                stop_btn.setFixedWidth(50)
                stop_btn.clicked.connect(lambda checked, s=service: self.stop_service(s))
                action_layout.addWidget(stop_btn)

                restart_btn = QPushButton("재시작")
                restart_btn.setFixedWidth(50)
                restart_btn.clicked.connect(lambda checked, s=service: self.restart_service(s))
                action_layout.addWidget(restart_btn)
            else:
                start_btn = QPushButton("시작")
                start_btn.setFixedWidth(50)
                start_btn.clicked.connect(lambda checked, s=service: self.start_service(s))
                action_layout.addWidget(start_btn)

            self.service_table.setCellWidget(row, 3, action_widget)

    def start_service(self, service: str):
        """서비스 시작"""
        try:
            subprocess.run(["sudo", "systemctl", "start", service], check=True, timeout=10)
            QMessageBox.information(self, "성공", f"{service} 서비스가 시작되었습니다.")
            self.refresh_services()
        except Exception as e:
            QMessageBox.critical(self, "오류", f"서비스 시작 실패: {e}")

    def stop_service(self, service: str):
        """서비스 중지"""
        try:
            subprocess.run(["sudo", "systemctl", "stop", service], check=True, timeout=10)
            QMessageBox.information(self, "성공", f"{service} 서비스가 중지되었습니다.")
            self.refresh_services()
        except Exception as e:
            QMessageBox.critical(self, "오류", f"서비스 중지 실패: {e}")

    def restart_service(self, service: str):
        """서비스 재시작"""
        try:
            subprocess.run(["sudo", "systemctl", "restart", service], check=True, timeout=10)
            QMessageBox.information(self, "성공", f"{service} 서비스가 재시작되었습니다.")
            self.refresh_services()
        except Exception as e:
            QMessageBox.critical(self, "오류", f"서비스 재시작 실패: {e}")

    def restart_all_services(self):
        """모든 서비스 재시작"""
        reply = QMessageBox.question(
            self,
            "확인",
            "모든 서비스를 재시작하시겠습니까?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            services = ["micro-ros-agent", "mavlink-router"]
            for service in services:
                try:
                    subprocess.run(["sudo", "systemctl", "restart", service], check=True, timeout=10)
                except:
                    pass
            self.refresh_services()
            QMessageBox.information(self, "완료", "서비스가 재시작되었습니다.")

    def refresh_topic_list(self):
        """토픽 목록 갱신"""
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                topics = [t for t in result.stdout.strip().split("\n") if t]
                self.topic_combo.clear()
                self.topic_combo.addItems(topics)
                self.topic_output.append(f"토픽 목록 갱신됨: {len(topics)}개")
            else:
                self.topic_output.append(f"토픽 목록 갱신 실패: {result.stderr}")

        except Exception as e:
            self.topic_output.append(f"오류: {e}")

    def echo_topic(self):
        """토픽 Echo"""
        topic = self.topic_combo.currentText()
        if not topic:
            return

        self.topic_output.append(f"\n{'='*40}")
        self.topic_output.append(f"Echo: {topic}")
        self.topic_output.append(f"{'='*40}")

        self.echo_btn.setEnabled(False)

        self.topic_worker = TopicMonitorWorker(topic)
        self.topic_worker.data_received.connect(self.on_topic_data)
        self.topic_worker.error.connect(self.on_topic_error)
        self.topic_worker.finished.connect(lambda: self.echo_btn.setEnabled(True))
        self.topic_worker.start()

    def on_topic_data(self, data: str):
        """토픽 데이터 수신"""
        self.topic_output.append(data)

    def on_topic_error(self, error: str):
        """토픽 오류"""
        self.topic_output.append(f"오류: {error}")

    def measure_hz(self):
        """토픽 주파수 측정"""
        topic = self.topic_combo.currentText()
        if not topic:
            return

        self.topic_output.append(f"\n주파수 측정 중: {topic} (3초간)")

        try:
            result = subprocess.run(
                ["ros2", "topic", "hz", topic],
                capture_output=True,
                text=True,
                timeout=4
            )
            self.topic_output.append(result.stdout if result.stdout else "데이터 없음")
        except subprocess.TimeoutExpired:
            self.topic_output.append("측정 완료")
        except Exception as e:
            self.topic_output.append(f"오류: {e}")

    def show_topic_info(self):
        """토픽 정보 표시"""
        topic = self.topic_combo.currentText()
        if not topic:
            return

        self.topic_output.append(f"\n{'='*40}")
        self.topic_output.append(f"토픽 정보: {topic}")
        self.topic_output.append(f"{'='*40}")

        try:
            result = subprocess.run(
                ["ros2", "topic", "info", topic, "-v"],
                capture_output=True,
                text=True,
                timeout=5
            )
            self.topic_output.append(result.stdout if result.stdout else "정보 없음")
        except Exception as e:
            self.topic_output.append(f"오류: {e}")
