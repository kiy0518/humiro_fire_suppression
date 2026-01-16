"""
체크리스트 탭
비행 전/후 체크리스트
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QCheckBox, QPushButton, QMessageBox,
    QScrollArea, QFrame, QComboBox, QProgressBar
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

from utils.config_manager import ConfigManager
from utils.system_checker import SystemChecker


class ChecklistItem(QFrame):
    """체크리스트 항목 위젯"""

    def __init__(self, text: str, auto_check_func=None, parent=None):
        super().__init__(parent)
        self.auto_check_func = auto_check_func

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)

        # 체크박스
        self.checkbox = QCheckBox(text)
        layout.addWidget(self.checkbox)

        layout.addStretch()

        # 상태 라벨
        self.status_label = QLabel("")
        self.status_label.setMinimumWidth(100)
        layout.addWidget(self.status_label)

        # 자동 확인 버튼 (auto_check_func가 있는 경우)
        if auto_check_func:
            self.auto_btn = QPushButton("확인")
            self.auto_btn.setFixedWidth(60)
            self.auto_btn.clicked.connect(self.run_auto_check)
            layout.addWidget(self.auto_btn)

    def run_auto_check(self):
        """자동 확인 실행"""
        if self.auto_check_func:
            result, message = self.auto_check_func()
            self.checkbox.setChecked(result)
            self.status_label.setText(message)
            if result:
                self.status_label.setStyleSheet("color: green;")
            else:
                self.status_label.setStyleSheet("color: red;")

    def is_checked(self) -> bool:
        """체크 상태 반환"""
        return self.checkbox.isChecked()

    def set_checked(self, checked: bool):
        """체크 상태 설정"""
        self.checkbox.setChecked(checked)

    def set_status(self, message: str, success: bool):
        """상태 설정"""
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"color: {'green' if success else 'red'};")


class ChecklistTab(QWidget):
    """체크리스트 탭"""

    def __init__(self, config_manager: ConfigManager, system_checker: SystemChecker):
        super().__init__()
        self.config_manager = config_manager
        self.system_checker = system_checker

        self.checklist_items = []

        self.init_ui()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 모드 선택
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("체크리스트 유형:"))

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["실내 테스트 비행 전", "야외 비행 전", "비행 후"])
        self.mode_combo.currentIndexChanged.connect(self.load_checklist)
        mode_layout.addWidget(self.mode_combo)

        mode_layout.addStretch()

        # 자동 확인 버튼
        auto_all_btn = QPushButton("전체 자동 확인")
        auto_all_btn.clicked.connect(self.run_all_auto_checks)
        mode_layout.addWidget(auto_all_btn)

        # 모두 체크/해제 버튼
        check_all_btn = QPushButton("모두 체크")
        check_all_btn.clicked.connect(lambda: self.set_all_checked(True))
        mode_layout.addWidget(check_all_btn)

        uncheck_all_btn = QPushButton("모두 해제")
        uncheck_all_btn.clicked.connect(lambda: self.set_all_checked(False))
        mode_layout.addWidget(uncheck_all_btn)

        layout.addLayout(mode_layout)

        # 진행률 바
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("완료율:"))
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(100)
        progress_layout.addWidget(self.progress_bar)
        self.progress_label = QLabel("0/0")
        progress_layout.addWidget(self.progress_label)
        layout.addLayout(progress_layout)

        # 스크롤 영역
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.scroll_widget = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_widget)
        scroll.setWidget(self.scroll_widget)
        layout.addWidget(scroll)

        # 하단 버튼
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        save_btn = QPushButton("체크리스트 저장")
        save_btn.clicked.connect(self.save_checklist)
        btn_layout.addWidget(save_btn)

        layout.addLayout(btn_layout)

        # 초기 체크리스트 로드
        self.load_checklist()

    def load_checklist(self):
        """체크리스트 로드"""
        # 기존 항목 제거
        for item in self.checklist_items:
            item.deleteLater()
        self.checklist_items.clear()

        # 레이아웃의 기존 위젯 제거
        while self.scroll_layout.count():
            child = self.scroll_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        mode = self.mode_combo.currentIndex()

        if mode == 0:  # 실내 테스트 비행 전
            self.create_indoor_preflight_checklist()
        elif mode == 1:  # 야외 비행 전
            self.create_outdoor_preflight_checklist()
        else:  # 비행 후
            self.create_postflight_checklist()

        self.scroll_layout.addStretch()
        self.update_progress()

    def create_indoor_preflight_checklist(self):
        """실내 테스트 비행 전 체크리스트"""
        # 하드웨어 섹션
        hw_group = QGroupBox("하드웨어 확인")
        hw_layout = QVBoxLayout(hw_group)

        hw_items = [
            ("배터리 충전 상태 확인 (70% 이상)", None),
            ("프로펠러 장착 및 상태 확인", None),
            ("Optical Flow 센서 연결 확인", None),
            ("LiDAR 센서 연결 확인", None),
            ("FC 전원 LED 확인", None),
        ]

        for text, func in hw_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            hw_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(hw_group)

        # 소프트웨어 섹션
        sw_group = QGroupBox("소프트웨어 확인")
        sw_layout = QVBoxLayout(sw_group)

        fc_ip = self.config_manager.get_fc_ip()

        sw_items = [
            ("FC 연결 확인 (ping)", lambda: self._check_ping(fc_ip)),
            ("micro-ros-agent 서비스 실행", lambda: self._check_service("micro-ros-agent")),
            ("mavlink-router 서비스 실행", lambda: self._check_service("mavlink-router")),
            ("ROS2 토픽 수신 확인", lambda: self._check_ros2_topics()),
            ("VehicleStatus 토픽 확인", lambda: self._check_vehicle_status_topic()),
        ]

        for text, func in sw_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            sw_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(sw_group)

        # 비행 환경 섹션
        env_group = QGroupBox("비행 환경 확인")
        env_layout = QVBoxLayout(env_group)

        env_items = [
            ("비행 공간 확보 (3m x 3m 이상)", None),
            ("바닥 텍스처 충분 (Optical Flow용)", None),
            ("조명 충분 (Optical Flow용)", None),
            ("장애물 제거", None),
            ("안전 거리 확보 (벽면에서 1m 이상)", None),
        ]

        for text, func in env_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            env_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(env_group)

        # FC 파라미터 섹션
        param_group = QGroupBox("FC 파라미터 확인")
        param_layout = QVBoxLayout(param_group)

        param_items = [
            ("GPS 비활성화 확인 (EKF2_GPS_CTRL=0)", None),
            ("Optical Flow 활성화 확인 (EKF2_OF_CTRL=1)", None),
            ("Range Finder 활성화 확인 (EKF2_RNG_CTRL=1)", None),
            ("EKF2 상태 확인 (position valid)", None),
        ]

        for text, func in param_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            param_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(param_group)

    def create_outdoor_preflight_checklist(self):
        """야외 비행 전 체크리스트"""
        # 하드웨어 섹션
        hw_group = QGroupBox("하드웨어 확인")
        hw_layout = QVBoxLayout(hw_group)

        hw_items = [
            ("배터리 충전 상태 확인 (80% 이상)", None),
            ("프로펠러 장착 및 상태 확인", None),
            ("GPS 안테나 연결 확인", None),
            ("FC 전원 LED 확인", None),
            ("조종기 연결 확인", None),
        ]

        for text, func in hw_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            hw_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(hw_group)

        # GPS 섹션
        gps_group = QGroupBox("GPS 확인")
        gps_layout = QVBoxLayout(gps_group)

        gps_items = [
            ("GPS 위성 수 확인 (6개 이상)", None),
            ("HDOP 확인 (2.0 이하)", None),
            ("GPS Fix 타입 확인 (3D Fix)", None),
            ("RTK 상태 확인 (해당 시)", None),
        ]

        for text, func in gps_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            gps_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(gps_group)

        # 소프트웨어 섹션
        sw_group = QGroupBox("소프트웨어 확인")
        sw_layout = QVBoxLayout(sw_group)

        fc_ip = self.config_manager.get_fc_ip()

        sw_items = [
            ("FC 연결 확인 (ping)", lambda: self._check_ping(fc_ip)),
            ("micro-ros-agent 서비스 실행", lambda: self._check_service("micro-ros-agent")),
            ("mavlink-router 서비스 실행", lambda: self._check_service("mavlink-router")),
            ("QGC 연결 확인", None),
        ]

        for text, func in sw_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            sw_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(sw_group)

        # 안전 섹션
        safety_group = QGroupBox("안전 확인")
        safety_layout = QVBoxLayout(safety_group)

        safety_items = [
            ("비행 금지 구역 확인", None),
            ("지오펜스 설정 확인", None),
            ("복귀 지점(Home) 설정 확인", None),
            ("페일세이프 설정 확인", None),
            ("주변 사람 및 장애물 확인", None),
            ("기상 조건 확인 (풍속 8m/s 이하)", None),
        ]

        for text, func in safety_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            safety_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(safety_group)

    def create_postflight_checklist(self):
        """비행 후 체크리스트"""
        # 기체 점검 섹션
        inspect_group = QGroupBox("기체 점검")
        inspect_layout = QVBoxLayout(inspect_group)

        inspect_items = [
            ("프로펠러 손상 확인", None),
            ("모터 상태 확인", None),
            ("배터리 상태 확인", None),
            ("프레임 손상 확인", None),
            ("센서 상태 확인", None),
        ]

        for text, func in inspect_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            inspect_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(inspect_group)

        # 데이터 섹션
        data_group = QGroupBox("데이터 관리")
        data_layout = QVBoxLayout(data_group)

        data_items = [
            ("비행 로그 저장", None),
            ("비행 영상 백업", None),
            ("이상 사항 기록", None),
        ]

        for text, func in data_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            data_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(data_group)

        # 정리 섹션
        cleanup_group = QGroupBox("정리")
        cleanup_layout = QVBoxLayout(cleanup_group)

        cleanup_items = [
            ("배터리 분리 및 보관", None),
            ("기체 보관", None),
            ("장비 정리", None),
        ]

        for text, func in cleanup_items:
            item = ChecklistItem(text, func)
            item.checkbox.stateChanged.connect(self.update_progress)
            cleanup_layout.addWidget(item)
            self.checklist_items.append(item)

        self.scroll_layout.addWidget(cleanup_group)

    # === 자동 확인 함수들 ===

    def _check_ping(self, host: str):
        """핑 체크"""
        result = self.system_checker.ping_host(host)
        return result, "✓ 응답" if result else "✗ 응답 없음"

    def _check_service(self, service: str):
        """서비스 체크"""
        result = self.system_checker.is_service_running(service)
        return result, "✓ 실행 중" if result else "✗ 중지"

    def _check_ros2_topics(self):
        """ROS2 토픽 체크"""
        topics = self.system_checker.get_ros2_topics()
        count = len(topics)
        return count > 0, f"✓ {count}개" if count > 0 else "✗ 토픽 없음"

    def _check_vehicle_status_topic(self):
        """VehicleStatus 토픽 체크"""
        topics = self.system_checker.get_ros2_topics()
        exists = "/fmu/out/vehicle_status" in topics
        return exists, "✓ 존재" if exists else "✗ 없음"

    # === 유틸리티 함수들 ===

    def run_all_auto_checks(self):
        """모든 자동 확인 실행"""
        for item in self.checklist_items:
            if hasattr(item, 'auto_btn'):
                item.run_auto_check()

    def set_all_checked(self, checked: bool):
        """모두 체크/해제"""
        for item in self.checklist_items:
            item.set_checked(checked)

    def update_progress(self):
        """진행률 업데이트"""
        total = len(self.checklist_items)
        checked = sum(1 for item in self.checklist_items if item.is_checked())

        if total > 0:
            percent = int(checked / total * 100)
            self.progress_bar.setValue(percent)
            self.progress_label.setText(f"{checked}/{total}")
        else:
            self.progress_bar.setValue(0)
            self.progress_label.setText("0/0")

    def save_checklist(self):
        """체크리스트 저장"""
        # TODO: 파일로 저장
        QMessageBox.information(self, "저장", "체크리스트가 저장되었습니다.")
