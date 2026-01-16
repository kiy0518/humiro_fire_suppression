"""
비행 모드 관리 탭
실내/야외 모드 전환 및 FC 파라미터 관리
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QGroupBox, QRadioButton, QPushButton, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QButtonGroup,
    QProgressDialog, QTextEdit, QComboBox, QFrame
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QFont

from utils.config_manager import ConfigManager


class ParamWorker(QThread):
    """파라미터 읽기/쓰기 워커 스레드"""
    progress = pyqtSignal(int, int, str, object)  # current, total, name, value/success
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, mavlink_manager, operation: str, params: dict = None):
        super().__init__()
        self.mavlink = mavlink_manager
        self.operation = operation  # 'read' or 'write'
        self.params = params or {}

    def run(self):
        try:
            if not self.mavlink.connect():
                self.error.emit("FC 연결 실패")
                return

            if self.operation == 'read':
                results = self.mavlink.get_params_batch(
                    list(self.params.keys()),
                    callback=lambda c, t, n, v: self.progress.emit(c, t, n, v)
                )
            else:  # write
                results = self.mavlink.set_params_batch(
                    self.params,
                    callback=lambda c, t, n, s: self.progress.emit(c, t, n, s)
                )

            self.finished.emit(results)

        except Exception as e:
            self.error.emit(str(e))
        finally:
            self.mavlink.disconnect()


class FlightModeTab(QWidget):
    """비행 모드 관리 탭"""

    def __init__(self, config_manager: ConfigManager):
        super().__init__()
        self.config_manager = config_manager
        self.current_mode = "indoor"  # indoor, outdoor_gps, outdoor_rtk
        self.fc_params = {}  # FC에서 읽은 현재 파라미터

        self.init_ui()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 비행 모드 선택 섹션
        mode_group = QGroupBox("비행 모드 선택")
        mode_layout = QVBoxLayout(mode_group)

        # 라디오 버튼 그룹
        self.mode_button_group = QButtonGroup()

        modes_layout = QHBoxLayout()

        # 실내 테스트 모드
        self.indoor_radio = QRadioButton("실내 테스트")
        self.indoor_radio.setChecked(True)
        self.mode_button_group.addButton(self.indoor_radio, 0)
        modes_layout.addWidget(self.indoor_radio)

        # 야외 GPS 모드
        self.outdoor_gps_radio = QRadioButton("야외 GPS")
        self.mode_button_group.addButton(self.outdoor_gps_radio, 1)
        modes_layout.addWidget(self.outdoor_gps_radio)

        # 야외 RTK 모드
        self.outdoor_rtk_radio = QRadioButton("야외 RTK")
        self.mode_button_group.addButton(self.outdoor_rtk_radio, 2)
        modes_layout.addWidget(self.outdoor_rtk_radio)

        modes_layout.addStretch()
        mode_layout.addLayout(modes_layout)

        # 모드 설명
        self.mode_description = QLabel(
            "실내 테스트 모드 - Optical Flow + LiDAR 기반 위치 제어\n"
            "GPS 없이 실내에서 호버링 및 위치 제어 테스트 가능"
        )
        self.mode_description.setStyleSheet("color: #6c757d; padding: 10px;")
        mode_layout.addWidget(self.mode_description)

        # 라디오 버튼 변경 시 설명 업데이트
        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)

        layout.addWidget(mode_group)

        # 파라미터 비교 섹션
        params_group = QGroupBox("파라미터 비교")
        params_layout = QVBoxLayout(params_group)

        # 버튼 영역
        btn_layout = QHBoxLayout()
        read_btn = QPushButton("FC에서 읽기")
        read_btn.clicked.connect(self.read_fc_params)
        btn_layout.addWidget(read_btn)

        btn_layout.addStretch()

        self.show_all_combo = QComboBox()
        self.show_all_combo.addItems(["변경 필요 항목만", "모든 항목"])
        self.show_all_combo.currentIndexChanged.connect(self.update_params_table)
        btn_layout.addWidget(self.show_all_combo)

        params_layout.addLayout(btn_layout)

        # 파라미터 테이블
        self.params_table = QTableWidget()
        self.params_table.setColumnCount(4)
        self.params_table.setHorizontalHeaderLabels(["파라미터", "현재 값", "목표 값", "상태"])
        self.params_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.params_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.params_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.params_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        params_layout.addWidget(self.params_table)

        layout.addWidget(params_group)

        # 페일세이프 설정 섹션
        failsafe_group = QGroupBox("페일세이프 설정")
        failsafe_layout = QGridLayout(failsafe_group)

        # OFFBOARD 두절 시 동작
        failsafe_layout.addWidget(QLabel("OFFBOARD 두절 시:"), 0, 0)
        self.offboard_action_combo = QComboBox()
        self.offboard_action_combo.addItems([
            "0 - Position 모드",
            "1 - Altitude 모드",
            "2 - Manual 모드",
            "3 - RTL (복귀)",
            "4 - Land (착륙)",
            "5 - Hold (Loiter)"
        ])
        self.offboard_action_combo.setCurrentIndex(5)  # Hold
        failsafe_layout.addWidget(self.offboard_action_combo, 0, 1)

        # RC 두절 시 동작
        failsafe_layout.addWidget(QLabel("RC 두절 시:"), 1, 0)
        self.rc_action_combo = QComboBox()
        self.rc_action_combo.addItems([
            "0 - 무시",
            "1 - Hold",
            "2 - RTL",
            "3 - Land"
        ])
        self.rc_action_combo.setCurrentIndex(2)  # RTL
        failsafe_layout.addWidget(self.rc_action_combo, 1, 1)

        # 배터리 저전압 동작
        failsafe_layout.addWidget(QLabel("배터리 저전압 시:"), 2, 0)
        self.battery_action_combo = QComboBox()
        self.battery_action_combo.addItems([
            "0 - 경고만",
            "1 - Land",
            "2 - RTL (착륙 없음)",
            "3 - RTL (착륙)"
        ])
        self.battery_action_combo.setCurrentIndex(3)  # RTL with land
        failsafe_layout.addWidget(self.battery_action_combo, 2, 1)

        # 권장 설정 라벨
        recommend_label = QLabel(
            "💡 권장 설정:\n"
            "- 실내 테스트: OFFBOARD 두절 → Land (착륙)\n"
            "- 야외 비행: OFFBOARD 두절 → RTL (복귀)"
        )
        recommend_label.setStyleSheet("color: #17a2b8; padding: 10px;")
        failsafe_layout.addWidget(recommend_label, 3, 0, 1, 2)

        layout.addWidget(failsafe_group)

        # 하단 버튼 영역
        bottom_layout = QHBoxLayout()

        # 프리셋 저장
        save_preset_btn = QPushButton("프리셋으로 저장")
        save_preset_btn.clicked.connect(self.save_as_preset)
        bottom_layout.addWidget(save_preset_btn)

        bottom_layout.addStretch()

        # 파라미터 적용
        apply_btn = QPushButton("파라미터 적용")
        apply_btn.setStyleSheet("background-color: #28a745; color: white;")
        apply_btn.clicked.connect(self.apply_params)
        bottom_layout.addWidget(apply_btn)

        layout.addLayout(bottom_layout)

        # 경고 라벨
        warning_label = QLabel("⚠️ 파라미터 변경 후 FC 재부팅이 필요합니다")
        warning_label.setStyleSheet("color: #dc3545; font-weight: bold;")
        warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(warning_label)

        # 초기 테이블 업데이트
        self.update_params_table()

    def on_mode_changed(self, button):
        """모드 변경 시"""
        descriptions = {
            0: "실내 테스트 모드 - Optical Flow + LiDAR 기반 위치 제어\nGPS 없이 실내에서 호버링 및 위치 제어 테스트 가능",
            1: "야외 GPS 모드 - GPS 기반 위치 제어\n위성 신호를 이용한 야외 비행",
            2: "야외 RTK 모드 - RTK GPS 기반 정밀 위치 제어\n센티미터 단위 정밀도 (RTK 베이스스테이션 필요)"
        }

        mode_id = self.mode_button_group.id(button)
        self.mode_description.setText(descriptions.get(mode_id, ""))

        modes = {0: "indoor", 1: "outdoor_gps", 2: "outdoor_rtk"}
        self.current_mode = modes.get(mode_id, "indoor")

        # 페일세이프 추천 설정
        if mode_id == 0:  # 실내
            self.offboard_action_combo.setCurrentIndex(4)  # Land
        else:  # 야외
            self.offboard_action_combo.setCurrentIndex(3)  # RTL

        self.update_params_table()

    def get_target_params(self) -> dict:
        """현재 선택된 모드의 목표 파라미터 반환"""
        if self.current_mode == "indoor":
            params = self.config_manager.get_indoor_params()
        elif self.current_mode == "outdoor_rtk":
            params = self.config_manager.get_outdoor_rtk_params()
        else:
            params = self.config_manager.get_outdoor_gps_params()

        # 페일세이프 파라미터 추가
        params["COM_OBL_RC_ACT"] = self.offboard_action_combo.currentIndex()
        params["NAV_RCL_ACT"] = self.rc_action_combo.currentIndex()
        params["COM_LOW_BAT_ACT"] = self.battery_action_combo.currentIndex()

        return params

    def update_params_table(self):
        """파라미터 테이블 업데이트"""
        target_params = self.get_target_params()
        show_all = self.show_all_combo.currentIndex() == 1

        # 표시할 항목 필터링
        rows = []
        for name, target in target_params.items():
            current = self.fc_params.get(name)

            if current is not None:
                # 값 비교 (소수점 고려)
                if isinstance(target, float):
                    match = abs(current - target) < 0.001
                else:
                    match = current == target
                status = "✅ 일치" if match else "⚠️ 불일치"
            else:
                match = False
                status = "❓ 미확인"

            if show_all or not match:
                rows.append((name, current, target, status, match))

        # 테이블 업데이트
        self.params_table.setRowCount(len(rows))
        for row, (name, current, target, status, match) in enumerate(rows):
            self.params_table.setItem(row, 0, QTableWidgetItem(name))

            current_item = QTableWidgetItem(str(current) if current is not None else "-")
            self.params_table.setItem(row, 1, current_item)

            target_item = QTableWidgetItem(str(target))
            self.params_table.setItem(row, 2, target_item)

            status_item = QTableWidgetItem(status)
            if match:
                status_item.setForeground(Qt.GlobalColor.darkGreen)
            else:
                status_item.setForeground(Qt.GlobalColor.red)
            self.params_table.setItem(row, 3, status_item)

    def read_fc_params(self):
        """FC에서 파라미터 읽기"""
        try:
            from utils.mavlink_manager import get_mavlink_manager

            fc_ip = self.config_manager.get_fc_ip()
            mavlink = get_mavlink_manager(f"udp:{fc_ip}:14550")

            target_params = self.get_target_params()

            # 프로그레스 다이얼로그
            progress = QProgressDialog("FC에서 파라미터 읽는 중...", "취소", 0, len(target_params), self)
            progress.setWindowModality(Qt.WindowModality.WindowModal)
            progress.show()

            # 연결
            if not mavlink.connect():
                QMessageBox.critical(self, "오류", "FC 연결 실패")
                return

            # 파라미터 읽기
            self.fc_params = {}
            for i, name in enumerate(target_params.keys()):
                if progress.wasCanceled():
                    break

                progress.setValue(i)
                progress.setLabelText(f"읽는 중: {name}")

                value = mavlink.get_param(name)
                if value is not None:
                    self.fc_params[name] = value

            progress.setValue(len(target_params))
            mavlink.disconnect()

            self.update_params_table()
            QMessageBox.information(self, "완료", f"{len(self.fc_params)}개 파라미터를 읽었습니다.")

        except ImportError:
            QMessageBox.warning(self, "경고", "pymavlink가 설치되지 않았습니다.\npip install pymavlink")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"파라미터 읽기 실패: {e}")

    def apply_params(self):
        """파라미터 FC에 적용"""
        target_params = self.get_target_params()

        # 변경이 필요한 파라미터만 필터링
        params_to_write = {}
        for name, target in target_params.items():
            current = self.fc_params.get(name)
            if current is None:
                params_to_write[name] = target
            elif isinstance(target, float):
                if abs(current - target) >= 0.001:
                    params_to_write[name] = target
            elif current != target:
                params_to_write[name] = target

        if not params_to_write:
            QMessageBox.information(self, "알림", "변경할 파라미터가 없습니다.")
            return

        # 확인
        reply = QMessageBox.question(
            self,
            "파라미터 적용",
            f"{len(params_to_write)}개 파라미터를 변경하시겠습니까?\n\n"
            "변경 후 FC 재부팅이 필요합니다.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            from utils.mavlink_manager import get_mavlink_manager

            fc_ip = self.config_manager.get_fc_ip()
            mavlink = get_mavlink_manager(f"udp:{fc_ip}:14550")

            # 프로그레스 다이얼로그
            progress = QProgressDialog("파라미터 적용 중...", "취소", 0, len(params_to_write) + 1, self)
            progress.setWindowModality(Qt.WindowModality.WindowModal)
            progress.show()

            # 연결
            if not mavlink.connect():
                QMessageBox.critical(self, "오류", "FC 연결 실패")
                return

            # 파라미터 쓰기
            success_count = 0
            for i, (name, value) in enumerate(params_to_write.items()):
                if progress.wasCanceled():
                    break

                progress.setValue(i)
                progress.setLabelText(f"적용 중: {name} = {value}")

                if mavlink.set_param(name, float(value)):
                    self.fc_params[name] = value
                    success_count += 1

            # 파라미터 저장
            progress.setLabelText("파라미터 저장 중...")
            progress.setValue(len(params_to_write))
            mavlink.save_params()

            progress.setValue(len(params_to_write) + 1)
            mavlink.disconnect()

            self.update_params_table()

            if success_count == len(params_to_write):
                QMessageBox.information(
                    self,
                    "완료",
                    f"모든 파라미터({success_count}개)가 적용되었습니다.\n\n"
                    "FC를 재부팅해주세요."
                )
            else:
                QMessageBox.warning(
                    self,
                    "부분 완료",
                    f"{success_count}/{len(params_to_write)}개 파라미터가 적용되었습니다."
                )

        except ImportError:
            QMessageBox.warning(self, "경고", "pymavlink가 설치되지 않았습니다.\npip install pymavlink")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"파라미터 적용 실패: {e}")

    def save_as_preset(self):
        """현재 설정을 프리셋으로 저장"""
        from PyQt6.QtWidgets import QInputDialog

        name, ok = QInputDialog.getText(
            self,
            "프리셋 저장",
            "프리셋 이름을 입력하세요:",
            text=f"{self.current_mode}_custom"
        )

        if ok and name:
            params = self.get_target_params()
            if self.config_manager.save_fc_params_preset(name, params):
                QMessageBox.information(self, "저장 완료", f"프리셋 '{name}'이(가) 저장되었습니다.")
            else:
                QMessageBox.critical(self, "저장 실패", "프리셋 저장에 실패했습니다.")
