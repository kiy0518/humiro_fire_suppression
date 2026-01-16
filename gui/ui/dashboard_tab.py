"""
대시보드 탭
시스템 상태 개요 표시
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QProgressBar, QPushButton, QFrame
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

from utils.config_manager import ConfigManager
from utils.system_checker import SystemChecker


class StatusCard(QFrame):
    """상태 카드 위젯"""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.setStyleSheet("""
            StatusCard {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 8px;
                padding: 10px;
            }
        """)

        layout = QVBoxLayout(self)

        # 타이틀
        self.title_label = QLabel(title)
        self.title_label.setFont(QFont("Sans", 10, QFont.Weight.Bold))
        layout.addWidget(self.title_label)

        # 상태 레이블
        self.status_label = QLabel("확인 중...")
        self.status_label.setFont(QFont("Sans", 12))
        layout.addWidget(self.status_label)

        # 상세 레이블
        self.detail_label = QLabel("")
        self.detail_label.setFont(QFont("Sans", 9))
        self.detail_label.setStyleSheet("color: #6c757d;")
        layout.addWidget(self.detail_label)

    def set_status(self, status: str, color: str = "black"):
        """상태 설정"""
        self.status_label.setText(status)
        self.status_label.setStyleSheet(f"color: {color};")

    def set_detail(self, detail: str):
        """상세 정보 설정"""
        self.detail_label.setText(detail)


class DashboardTab(QWidget):
    """대시보드 탭"""

    def __init__(self, config_manager: ConfigManager, system_checker: SystemChecker):
        super().__init__()
        self.config_manager = config_manager
        self.system_checker = system_checker

        self.init_ui()
        self.setup_timer()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 기체 정보 섹션
        vehicle_group = QGroupBox("기체 정보")
        vehicle_layout = QHBoxLayout(vehicle_group)

        drone_id = self.config_manager.get_drone_id()
        role = "Leader" if drone_id == 1 else "Follower"

        self.vehicle_info = QLabel(f"기체 번호: {drone_id}번 | 역할: {role}")
        self.vehicle_info.setFont(QFont("Sans", 14, QFont.Weight.Bold))
        vehicle_layout.addWidget(self.vehicle_info)

        vehicle_layout.addStretch()

        # 빠른 전환 버튼
        for i in range(1, 4):
            btn = QPushButton(f"{i}번 기체")
            btn.setFixedWidth(80)
            btn.clicked.connect(lambda checked, x=i: self.quick_switch_vehicle(x))
            vehicle_layout.addWidget(btn)

        layout.addWidget(vehicle_group)

        # 상태 카드 그리드
        cards_layout = QGridLayout()

        # FC 연결 상태
        self.fc_card = StatusCard("FC 연결")
        cards_layout.addWidget(self.fc_card, 0, 0)

        # 네트워크 상태
        self.network_card = StatusCard("네트워크")
        cards_layout.addWidget(self.network_card, 0, 1)

        # ROS2 상태
        self.ros2_card = StatusCard("ROS2")
        cards_layout.addWidget(self.ros2_card, 0, 2)

        # 서비스 상태
        self.service_card = StatusCard("서비스")
        cards_layout.addWidget(self.service_card, 1, 0)

        # CPU 사용량
        self.cpu_card = StatusCard("CPU")
        cards_layout.addWidget(self.cpu_card, 1, 1)

        # 메모리 사용량
        self.memory_card = StatusCard("메모리")
        cards_layout.addWidget(self.memory_card, 1, 2)

        layout.addLayout(cards_layout)

        # 시스템 리소스 섹션
        resource_group = QGroupBox("시스템 리소스")
        resource_layout = QVBoxLayout(resource_group)

        # CPU 프로그레스 바
        cpu_layout = QHBoxLayout()
        cpu_layout.addWidget(QLabel("CPU:"))
        self.cpu_progress = QProgressBar()
        self.cpu_progress.setMaximum(100)
        cpu_layout.addWidget(self.cpu_progress)
        self.cpu_percent_label = QLabel("0%")
        cpu_layout.addWidget(self.cpu_percent_label)
        resource_layout.addLayout(cpu_layout)

        # 메모리 프로그레스 바
        mem_layout = QHBoxLayout()
        mem_layout.addWidget(QLabel("RAM:"))
        self.mem_progress = QProgressBar()
        self.mem_progress.setMaximum(100)
        mem_layout.addWidget(self.mem_progress)
        self.mem_percent_label = QLabel("0%")
        mem_layout.addWidget(self.mem_percent_label)
        resource_layout.addLayout(mem_layout)

        # 디스크 프로그레스 바
        disk_layout = QHBoxLayout()
        disk_layout.addWidget(QLabel("Disk:"))
        self.disk_progress = QProgressBar()
        self.disk_progress.setMaximum(100)
        disk_layout.addWidget(self.disk_progress)
        self.disk_percent_label = QLabel("0%")
        disk_layout.addWidget(self.disk_percent_label)
        resource_layout.addLayout(disk_layout)

        layout.addWidget(resource_group)

        # 새로고침 버튼
        refresh_layout = QHBoxLayout()
        refresh_layout.addStretch()
        refresh_btn = QPushButton("새로고침")
        refresh_btn.clicked.connect(self.refresh)
        refresh_layout.addWidget(refresh_btn)
        layout.addLayout(refresh_layout)

        layout.addStretch()

    def setup_timer(self):
        """자동 업데이트 타이머"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.refresh)
        self.update_timer.start(10000)  # 10초마다

        # 초기 업데이트
        QTimer.singleShot(100, self.refresh)

    def refresh(self):
        """상태 새로고침"""
        # FC 연결 상태
        fc_ip = self.config_manager.get_fc_ip()
        if self.system_checker.ping_host(fc_ip):
            self.fc_card.set_status("✅ 연결됨", "green")
            self.fc_card.set_detail(f"IP: {fc_ip}")
        else:
            self.fc_card.set_status("❌ 연결 안됨", "red")
            self.fc_card.set_detail(f"IP: {fc_ip}")

        # 네트워크 상태
        eth0_ip = self.system_checker.get_interface_ip("eth0")
        wlan0_ip = self.system_checker.get_interface_ip("wlan0")
        if eth0_ip:
            self.network_card.set_status("✅ 정상", "green")
            detail = f"eth0: {eth0_ip}"
            if wlan0_ip:
                detail += f"\nwlan0: {wlan0_ip}"
            self.network_card.set_detail(detail)
        else:
            self.network_card.set_status("⚠️ eth0 없음", "orange")
            self.network_card.set_detail("")

        # ROS2 상태
        if self.system_checker.is_service_running("micro-ros-agent"):
            self.ros2_card.set_status("✅ 실행 중", "green")
            topics = self.system_checker.get_ros2_topics()
            self.ros2_card.set_detail(f"토픽 수: {len(topics)}")
        else:
            self.ros2_card.set_status("⚠️ 중지됨", "orange")
            self.ros2_card.set_detail("")

        # 서비스 상태
        services = self.system_checker.check_services_status()
        running = sum(1 for v in services.values() if v)
        total = len(services)
        if running == total:
            self.service_card.set_status(f"✅ {running}/{total}", "green")
        elif running > 0:
            self.service_card.set_status(f"⚠️ {running}/{total}", "orange")
        else:
            self.service_card.set_status(f"❌ {running}/{total}", "red")
        self.service_card.set_detail("\n".join(f"{'✓' if v else '✗'} {k}" for k, v in services.items()))

        # CPU 사용량
        cpu_usage = self.system_checker.get_cpu_usage()
        self.cpu_progress.setValue(int(cpu_usage))
        self.cpu_percent_label.setText(f"{cpu_usage:.1f}%")
        if cpu_usage > 80:
            self.cpu_card.set_status(f"⚠️ {cpu_usage:.1f}%", "orange")
        else:
            self.cpu_card.set_status(f"✅ {cpu_usage:.1f}%", "green")

        # 메모리 사용량
        used, total, percent = self.system_checker.get_memory_usage()
        self.mem_progress.setValue(int(percent))
        self.mem_percent_label.setText(f"{percent:.1f}%")
        self.memory_card.set_status(f"{used:.1f}/{total:.1f} GB", "green" if percent < 80 else "orange")
        self.memory_card.set_detail(f"{percent:.1f}%")

        # 디스크 사용량
        used, total, percent = self.system_checker.get_disk_usage("/")
        self.disk_progress.setValue(int(percent))
        self.disk_percent_label.setText(f"{percent:.1f}%")

    def quick_switch_vehicle(self, drone_id: int):
        """빠른 기체 전환"""
        from PyQt6.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            "기체 전환",
            f"{drone_id}번 기체로 전환하시겠습니까?\n\n"
            "설정이 변경되며 재부팅이 필요합니다.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            # VehicleSetupTab으로 전환 필요
            QMessageBox.information(
                self,
                "안내",
                "기체 설정 탭에서 설정을 완료해주세요."
            )
