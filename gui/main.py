#!/usr/bin/env python3
"""
Humiro Fire Suppression - GUI Management Tool
메인 진입점
"""

import sys
import os

# 프로젝트 루트 경로 설정
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QStatusBar, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QAction

from ui.dashboard_tab import DashboardTab
from ui.config_tab import ConfigTab
from ui.flight_mode_tab import FlightModeTab
from ui.vehicle_setup_tab import VehicleSetupTab
from ui.checklist_tab import ChecklistTab
from ui.build_tab import BuildTab
from ui.monitor_tab import MonitorTab
from ui.progress_tab import ProgressTab

from utils.config_manager import ConfigManager
from utils.system_checker import SystemChecker


class MainWindow(QMainWindow):
    """메인 윈도우"""

    def __init__(self):
        super().__init__()
        self.config_manager = ConfigManager(PROJECT_ROOT)
        self.system_checker = SystemChecker()

        self.init_ui()
        self.setup_status_bar()
        self.setup_timer()

    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Humiro Fire Suppression - Management Tool")
        self.setMinimumSize(1200, 800)

        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 메인 레이아웃
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # 헤더
        header = self.create_header()
        main_layout.addWidget(header)

        # 탭 위젯
        self.tab_widget = QTabWidget()
        self.tab_widget.setFont(QFont("Sans", 10))

        # 각 탭 추가
        self.dashboard_tab = DashboardTab(self.config_manager, self.system_checker)
        self.config_tab = ConfigTab(self.config_manager)
        self.flight_mode_tab = FlightModeTab(self.config_manager)
        self.vehicle_setup_tab = VehicleSetupTab(self.config_manager)
        self.checklist_tab = ChecklistTab(self.config_manager, self.system_checker)
        self.build_tab = BuildTab(PROJECT_ROOT)
        self.monitor_tab = MonitorTab()
        self.progress_tab = ProgressTab(PROJECT_ROOT)

        self.tab_widget.addTab(self.dashboard_tab, "📊 대시보드")
        self.tab_widget.addTab(self.config_tab, "⚙️ 설정")
        self.tab_widget.addTab(self.flight_mode_tab, "🛩️ 비행 모드")
        self.tab_widget.addTab(self.vehicle_setup_tab, "🚁 기체 설정")
        self.tab_widget.addTab(self.checklist_tab, "✅ 체크리스트")
        self.tab_widget.addTab(self.build_tab, "🔨 빌드")
        self.tab_widget.addTab(self.monitor_tab, "📡 모니터")
        self.tab_widget.addTab(self.progress_tab, "📈 진행률")

        main_layout.addWidget(self.tab_widget)

        # 메뉴바 설정
        self.setup_menu_bar()

    def create_header(self) -> QWidget:
        """헤더 위젯 생성"""
        header = QWidget()
        header.setStyleSheet("background-color: #2c3e50; border-radius: 5px;")
        header.setFixedHeight(60)

        layout = QHBoxLayout(header)

        # 타이틀
        title = QLabel("🔥 Humiro Fire Suppression System")
        title.setFont(QFont("Sans", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        layout.addStretch()

        # 현재 기체 정보
        drone_id = self.config_manager.get_drone_id()
        role = "Leader" if drone_id == 1 else "Follower"

        self.drone_info_label = QLabel(f"기체: {drone_id}번 ({role})")
        self.drone_info_label.setFont(QFont("Sans", 12))
        self.drone_info_label.setStyleSheet("color: #3498db;")
        layout.addWidget(self.drone_info_label)

        return header

    def setup_menu_bar(self):
        """메뉴바 설정"""
        menubar = self.menuBar()

        # 파일 메뉴
        file_menu = menubar.addMenu("파일")

        reload_action = QAction("설정 다시 로드", self)
        reload_action.triggered.connect(self.reload_config)
        file_menu.addAction(reload_action)

        file_menu.addSeparator()

        exit_action = QAction("종료", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 도구 메뉴
        tools_menu = menubar.addMenu("도구")

        refresh_status_action = QAction("상태 새로고침", self)
        refresh_status_action.triggered.connect(self.refresh_all_status)
        tools_menu.addAction(refresh_status_action)

        # 도움말 메뉴
        help_menu = menubar.addMenu("도움말")

        about_action = QAction("정보", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def setup_status_bar(self):
        """상태바 설정"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # FC 연결 상태
        self.fc_status_label = QLabel("FC: 확인 중...")
        self.status_bar.addWidget(self.fc_status_label)

        # 구분선
        self.status_bar.addWidget(QLabel(" | "))

        # 네트워크 상태
        self.network_status_label = QLabel("Network: 확인 중...")
        self.status_bar.addWidget(self.network_status_label)

        # 구분선
        self.status_bar.addWidget(QLabel(" | "))

        # ROS2 상태
        self.ros2_status_label = QLabel("ROS2: 확인 중...")
        self.status_bar.addWidget(self.ros2_status_label)

    def setup_timer(self):
        """타이머 설정 (주기적 상태 업데이트)"""
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(5000)  # 5초마다 업데이트

        # 초기 상태 업데이트
        QTimer.singleShot(100, self.update_status)

    def update_status(self):
        """상태 업데이트"""
        # FC 연결 확인
        fc_ip = self.config_manager.get_fc_ip()
        fc_connected = self.system_checker.ping_host(fc_ip)
        if fc_connected:
            self.fc_status_label.setText(f"FC: ✅ 연결됨 ({fc_ip})")
            self.fc_status_label.setStyleSheet("color: green;")
        else:
            self.fc_status_label.setText(f"FC: ❌ 연결 안됨 ({fc_ip})")
            self.fc_status_label.setStyleSheet("color: red;")

        # 네트워크 상태
        eth0_ip = self.system_checker.get_interface_ip("eth0")
        if eth0_ip:
            self.network_status_label.setText(f"Network: ✅ eth0={eth0_ip}")
            self.network_status_label.setStyleSheet("color: green;")
        else:
            self.network_status_label.setText("Network: ⚠️ eth0 없음")
            self.network_status_label.setStyleSheet("color: orange;")

        # ROS2 상태
        ros2_running = self.system_checker.is_service_running("micro-ros-agent")
        if ros2_running:
            self.ros2_status_label.setText("ROS2: ✅ micro-ros-agent 실행 중")
            self.ros2_status_label.setStyleSheet("color: green;")
        else:
            self.ros2_status_label.setText("ROS2: ⚠️ micro-ros-agent 중지")
            self.ros2_status_label.setStyleSheet("color: orange;")

    def reload_config(self):
        """설정 다시 로드"""
        self.config_manager.reload()
        self.update_drone_info()
        self.status_bar.showMessage("설정을 다시 로드했습니다.", 3000)

    def update_drone_info(self):
        """드론 정보 업데이트"""
        drone_id = self.config_manager.get_drone_id()
        role = "Leader" if drone_id == 1 else "Follower"
        self.drone_info_label.setText(f"기체: {drone_id}번 ({role})")

    def refresh_all_status(self):
        """모든 상태 새로고침"""
        self.update_status()
        self.dashboard_tab.refresh()
        self.status_bar.showMessage("상태를 새로고침했습니다.", 3000)

    def show_about(self):
        """정보 대화상자"""
        QMessageBox.about(
            self,
            "정보",
            "Humiro Fire Suppression System\n"
            "GUI Management Tool\n\n"
            "Version: 1.0.0\n"
            "작성일: 2026-01-16"
        )

    def closeEvent(self, event):
        """종료 이벤트"""
        reply = QMessageBox.question(
            self,
            "종료 확인",
            "프로그램을 종료하시겠습니까?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            event.accept()
        else:
            event.ignore()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)

    # 애플리케이션 스타일
    app.setStyle("Fusion")

    # 다크 테마 설정 (선택적)
    # palette = QPalette()
    # palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
    # app.setPalette(palette)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
