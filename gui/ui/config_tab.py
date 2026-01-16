"""
설정 관리 탭
device_config.env 및 기타 설정 편집
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QLineEdit, QPushButton, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QComboBox,
    QScrollArea, QFrame
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from utils.config_manager import ConfigManager


class ConfigTab(QWidget):
    """설정 관리 탭"""

    def __init__(self, config_manager: ConfigManager):
        super().__init__()
        self.config_manager = config_manager
        self.init_ui()
        self.load_config()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 스크롤 영역
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)

        # 기본 설정 섹션
        basic_group = QGroupBox("기본 설정")
        basic_layout = QGridLayout(basic_group)

        # DRONE_ID
        basic_layout.addWidget(QLabel("드론 ID:"), 0, 0)
        self.drone_id_combo = QComboBox()
        self.drone_id_combo.addItems(["1 (Leader)", "2 (Follower)", "3 (Follower)"])
        basic_layout.addWidget(self.drone_id_combo, 0, 1)

        # ETH0_IP
        basic_layout.addWidget(QLabel("ETH0 IP:"), 1, 0)
        self.eth0_ip_edit = QLineEdit()
        self.eth0_ip_edit.setPlaceholderText("10.0.0.11")
        basic_layout.addWidget(self.eth0_ip_edit, 1, 1)

        # FC_IP
        basic_layout.addWidget(QLabel("FC IP:"), 2, 0)
        self.fc_ip_edit = QLineEdit()
        self.fc_ip_edit.setPlaceholderText("10.0.0.12")
        basic_layout.addWidget(self.fc_ip_edit, 2, 1)

        # WIFI_IP
        basic_layout.addWidget(QLabel("WiFi IP:"), 3, 0)
        self.wifi_ip_edit = QLineEdit()
        self.wifi_ip_edit.setPlaceholderText("192.168.100.11")
        basic_layout.addWidget(self.wifi_ip_edit, 3, 1)

        scroll_layout.addWidget(basic_group)

        # ROS2 설정 섹션
        ros2_group = QGroupBox("ROS2 설정")
        ros2_layout = QGridLayout(ros2_group)

        # ROS_NAMESPACE
        ros2_layout.addWidget(QLabel("ROS Namespace:"), 0, 0)
        self.ros_namespace_edit = QLineEdit()
        self.ros_namespace_edit.setPlaceholderText("drone1")
        ros2_layout.addWidget(self.ros_namespace_edit, 0, 1)

        # ROS_DOMAIN_ID
        ros2_layout.addWidget(QLabel("ROS Domain ID:"), 1, 0)
        self.ros_domain_edit = QLineEdit()
        self.ros_domain_edit.setPlaceholderText("0")
        ros2_layout.addWidget(self.ros_domain_edit, 1, 1)

        scroll_layout.addWidget(ros2_group)

        # MAVLink 설정 섹션
        mavlink_group = QGroupBox("MAVLink 설정")
        mavlink_layout = QGridLayout(mavlink_group)

        # MAV_SYS_ID
        mavlink_layout.addWidget(QLabel("MAV System ID:"), 0, 0)
        self.mav_sysid_edit = QLineEdit()
        self.mav_sysid_edit.setPlaceholderText("1")
        mavlink_layout.addWidget(self.mav_sysid_edit, 0, 1)

        # MAV_COMP_ID
        mavlink_layout.addWidget(QLabel("MAV Component ID:"), 1, 0)
        self.mav_compid_edit = QLineEdit()
        self.mav_compid_edit.setPlaceholderText("1")
        mavlink_layout.addWidget(self.mav_compid_edit, 1, 1)

        scroll_layout.addWidget(mavlink_group)

        # 스트리밍 설정 섹션
        streaming_group = QGroupBox("스트리밍 설정")
        streaming_layout = QGridLayout(streaming_group)

        # RTSP_PORT
        streaming_layout.addWidget(QLabel("RTSP 포트:"), 0, 0)
        self.rtsp_port_edit = QLineEdit()
        self.rtsp_port_edit.setPlaceholderText("8554")
        streaming_layout.addWidget(self.rtsp_port_edit, 0, 1)

        # STREAM_NAME
        streaming_layout.addWidget(QLabel("스트림 이름:"), 1, 0)
        self.stream_name_edit = QLineEdit()
        self.stream_name_edit.setPlaceholderText("humiro1")
        streaming_layout.addWidget(self.stream_name_edit, 1, 1)

        scroll_layout.addWidget(streaming_group)

        # 모든 설정 테이블
        all_config_group = QGroupBox("모든 설정 (device_config.env)")
        all_config_layout = QVBoxLayout(all_config_group)

        self.config_table = QTableWidget()
        self.config_table.setColumnCount(2)
        self.config_table.setHorizontalHeaderLabels(["키", "값"])
        self.config_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.config_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        all_config_layout.addWidget(self.config_table)

        # 테이블 버튼
        table_btn_layout = QHBoxLayout()
        add_row_btn = QPushButton("행 추가")
        add_row_btn.clicked.connect(self.add_config_row)
        table_btn_layout.addWidget(add_row_btn)

        remove_row_btn = QPushButton("행 삭제")
        remove_row_btn.clicked.connect(self.remove_config_row)
        table_btn_layout.addWidget(remove_row_btn)

        table_btn_layout.addStretch()
        all_config_layout.addLayout(table_btn_layout)

        scroll_layout.addWidget(all_config_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        # 버튼 영역
        btn_layout = QHBoxLayout()

        reload_btn = QPushButton("다시 로드")
        reload_btn.clicked.connect(self.load_config)
        btn_layout.addWidget(reload_btn)

        btn_layout.addStretch()

        apply_btn = QPushButton("적용")
        apply_btn.clicked.connect(self.apply_config)
        btn_layout.addWidget(apply_btn)

        save_btn = QPushButton("저장")
        save_btn.setStyleSheet("background-color: #28a745; color: white;")
        save_btn.clicked.connect(self.save_config)
        btn_layout.addWidget(save_btn)

        layout.addLayout(btn_layout)

    def load_config(self):
        """설정 로드"""
        self.config_manager.reload()
        config = self.config_manager.get_all_config()

        # 기본 필드 설정
        drone_id = self.config_manager.get_drone_id()
        self.drone_id_combo.setCurrentIndex(drone_id - 1)

        self.eth0_ip_edit.setText(config.get("ETH0_IP", ""))
        self.fc_ip_edit.setText(config.get("FC_IP", ""))
        self.wifi_ip_edit.setText(config.get("WIFI_IP", ""))
        self.ros_namespace_edit.setText(config.get("ROS_NAMESPACE", ""))
        self.ros_domain_edit.setText(config.get("ROS_DOMAIN_ID", "0"))
        self.mav_sysid_edit.setText(config.get("MAV_SYS_ID", "1"))
        self.mav_compid_edit.setText(config.get("MAV_COMP_ID", "1"))
        self.rtsp_port_edit.setText(config.get("RTSP_PORT", "8554"))
        self.stream_name_edit.setText(config.get("STREAM_NAME", ""))

        # 테이블 업데이트
        self.config_table.setRowCount(len(config))
        for row, (key, value) in enumerate(sorted(config.items())):
            self.config_table.setItem(row, 0, QTableWidgetItem(key))
            self.config_table.setItem(row, 1, QTableWidgetItem(value))

    def apply_config(self):
        """UI 값을 ConfigManager에 적용 (저장하지 않음)"""
        # 드론 ID
        drone_id = self.drone_id_combo.currentIndex() + 1
        self.config_manager.set("DRONE_ID", str(drone_id))

        # 기본 설정
        self.config_manager.set("ETH0_IP", self.eth0_ip_edit.text())
        self.config_manager.set("FC_IP", self.fc_ip_edit.text())
        self.config_manager.set("WIFI_IP", self.wifi_ip_edit.text())

        # ROS2 설정
        self.config_manager.set("ROS_NAMESPACE", self.ros_namespace_edit.text())
        self.config_manager.set("ROS_DOMAIN_ID", self.ros_domain_edit.text())

        # MAVLink 설정
        self.config_manager.set("MAV_SYS_ID", self.mav_sysid_edit.text())
        self.config_manager.set("MAV_COMP_ID", self.mav_compid_edit.text())

        # 스트리밍 설정
        self.config_manager.set("RTSP_PORT", self.rtsp_port_edit.text())
        self.config_manager.set("STREAM_NAME", self.stream_name_edit.text())

        # 테이블 값 적용
        for row in range(self.config_table.rowCount()):
            key_item = self.config_table.item(row, 0)
            value_item = self.config_table.item(row, 1)
            if key_item and value_item:
                key = key_item.text().strip()
                value = value_item.text().strip()
                if key:
                    self.config_manager.set(key, value)

        QMessageBox.information(self, "적용", "설정이 적용되었습니다.\n저장하려면 '저장' 버튼을 누르세요.")

    def save_config(self):
        """설정 저장"""
        self.apply_config()

        reply = QMessageBox.question(
            self,
            "저장 확인",
            "설정을 저장하시겠습니까?\n\n변경된 설정은 서비스 재시작 후 적용됩니다.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            if self.config_manager.save_device_config():
                QMessageBox.information(self, "저장 완료", "설정이 저장되었습니다.")
            else:
                QMessageBox.critical(self, "저장 실패", "설정 저장에 실패했습니다.")

    def add_config_row(self):
        """테이블에 행 추가"""
        row = self.config_table.rowCount()
        self.config_table.insertRow(row)
        self.config_table.setItem(row, 0, QTableWidgetItem(""))
        self.config_table.setItem(row, 1, QTableWidgetItem(""))

    def remove_config_row(self):
        """선택된 행 삭제"""
        current_row = self.config_table.currentRow()
        if current_row >= 0:
            self.config_table.removeRow(current_row)
