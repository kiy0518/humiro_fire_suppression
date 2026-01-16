"""
빌드 탭
원클릭 빌드 및 로그 뷰어
"""

import os
import subprocess
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QMessageBox,
    QTextEdit, QCheckBox, QComboBox, QProgressBar
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QProcess
from PyQt6.QtGui import QFont, QTextCursor


class BuildWorker(QThread):
    """빌드 워커 스레드"""
    output = pyqtSignal(str)
    finished = pyqtSignal(bool, str)

    def __init__(self, project_root: str, targets: list, clean_build: bool):
        super().__init__()
        self.project_root = project_root
        self.targets = targets
        self.clean_build = clean_build

    def run(self):
        try:
            # 빌드 순서 정의
            build_order = [
                "lidar",
                "thermal",
                "targeting",
                "osd",
                "streaming",
                "ros2",
                "navigation",
                "application",
            ]

            # 타겟 필터링
            targets_to_build = [t for t in build_order if t in self.targets]

            for target in targets_to_build:
                self.output.emit(f"\n{'='*60}")
                self.output.emit(f"빌드 시작: {target}")
                self.output.emit(f"{'='*60}\n")

                target_dir = os.path.join(self.project_root, target)

                if not os.path.exists(target_dir):
                    self.output.emit(f"⚠️ 디렉토리 없음: {target_dir}")
                    continue

                build_dir = os.path.join(target_dir, "build")

                # clean build 옵션
                if self.clean_build and os.path.exists(build_dir):
                    self.output.emit(f"클린 빌드: {build_dir} 삭제 중...")
                    subprocess.run(["rm", "-rf", build_dir], check=False)

                # build 디렉토리 생성
                os.makedirs(build_dir, exist_ok=True)

                # cmake
                self.output.emit("CMake 실행 중...")
                cmake_result = subprocess.run(
                    ["cmake", ".."],
                    cwd=build_dir,
                    capture_output=True,
                    text=True
                )
                self.output.emit(cmake_result.stdout)
                if cmake_result.returncode != 0:
                    self.output.emit(f"❌ CMake 실패:\n{cmake_result.stderr}")
                    self.finished.emit(False, f"{target} CMake 실패")
                    return

                # make
                self.output.emit("Make 실행 중...")
                make_result = subprocess.run(
                    ["make", "-j4"],
                    cwd=build_dir,
                    capture_output=True,
                    text=True
                )
                self.output.emit(make_result.stdout)
                if make_result.returncode != 0:
                    self.output.emit(f"❌ Make 실패:\n{make_result.stderr}")
                    self.finished.emit(False, f"{target} Make 실패")
                    return

                self.output.emit(f"✅ {target} 빌드 완료")

            self.finished.emit(True, "모든 빌드 완료")

        except Exception as e:
            self.output.emit(f"오류: {e}")
            self.finished.emit(False, str(e))


class BuildTab(QWidget):
    """빌드 탭"""

    def __init__(self, project_root: str):
        super().__init__()
        self.project_root = project_root
        self.build_worker = None

        self.init_ui()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 빌드 타겟 선택
        target_group = QGroupBox("빌드 타겟 선택")
        target_layout = QVBoxLayout(target_group)

        # 체크박스들
        self.target_checkboxes = {}
        targets = [
            ("lidar", "LiDAR 라이브러리"),
            ("thermal", "Thermal 라이브러리"),
            ("targeting", "Targeting 라이브러리"),
            ("osd", "OSD 라이브러리"),
            ("streaming", "Streaming 라이브러리"),
            ("ros2", "ROS2 메시지"),
            ("navigation", "Navigation"),
            ("application", "Application (메인)"),
        ]

        targets_grid = QHBoxLayout()
        for i, (key, label) in enumerate(targets):
            cb = QCheckBox(label)
            cb.setChecked(True)
            self.target_checkboxes[key] = cb
            targets_grid.addWidget(cb)
            if (i + 1) % 4 == 0:
                target_layout.addLayout(targets_grid)
                targets_grid = QHBoxLayout()

        if targets_grid.count() > 0:
            target_layout.addLayout(targets_grid)

        # 전체 선택/해제 버튼
        select_layout = QHBoxLayout()
        select_all_btn = QPushButton("전체 선택")
        select_all_btn.clicked.connect(lambda: self.set_all_targets(True))
        select_layout.addWidget(select_all_btn)

        deselect_all_btn = QPushButton("전체 해제")
        deselect_all_btn.clicked.connect(lambda: self.set_all_targets(False))
        select_layout.addWidget(deselect_all_btn)

        select_layout.addStretch()
        target_layout.addLayout(select_layout)

        layout.addWidget(target_group)

        # 빌드 옵션
        option_layout = QHBoxLayout()

        self.clean_build_cb = QCheckBox("클린 빌드 (build 폴더 삭제 후 빌드)")
        option_layout.addWidget(self.clean_build_cb)

        option_layout.addStretch()
        layout.addLayout(option_layout)

        # 빌드 버튼
        btn_layout = QHBoxLayout()

        self.build_btn = QPushButton("빌드 시작")
        self.build_btn.setStyleSheet("background-color: #007bff; color: white; padding: 10px 20px;")
        self.build_btn.setFont(QFont("Sans", 11, QFont.Weight.Bold))
        self.build_btn.clicked.connect(self.start_build)
        btn_layout.addWidget(self.build_btn)

        self.stop_btn = QPushButton("중지")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_build)
        btn_layout.addWidget(self.stop_btn)

        btn_layout.addStretch()

        # 빠른 빌드 버튼들
        quick_app_btn = QPushButton("Application만 빌드")
        quick_app_btn.clicked.connect(lambda: self.quick_build(["application"]))
        btn_layout.addWidget(quick_app_btn)

        quick_nav_btn = QPushButton("Navigation만 빌드")
        quick_nav_btn.clicked.connect(lambda: self.quick_build(["navigation"]))
        btn_layout.addWidget(quick_nav_btn)

        layout.addLayout(btn_layout)

        # 빌드 로그
        log_group = QGroupBox("빌드 로그")
        log_layout = QVBoxLayout(log_group)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("font-family: monospace; background-color: #1e1e1e; color: #d4d4d4;")
        log_layout.addWidget(self.log_text)

        # 로그 버튼
        log_btn_layout = QHBoxLayout()
        clear_log_btn = QPushButton("로그 지우기")
        clear_log_btn.clicked.connect(self.log_text.clear)
        log_btn_layout.addWidget(clear_log_btn)

        save_log_btn = QPushButton("로그 저장")
        save_log_btn.clicked.connect(self.save_log)
        log_btn_layout.addWidget(save_log_btn)

        log_btn_layout.addStretch()
        log_layout.addLayout(log_btn_layout)

        layout.addWidget(log_group)

    def set_all_targets(self, checked: bool):
        """모든 타겟 선택/해제"""
        for cb in self.target_checkboxes.values():
            cb.setChecked(checked)

    def get_selected_targets(self) -> list:
        """선택된 타겟 목록 반환"""
        return [key for key, cb in self.target_checkboxes.items() if cb.isChecked()]

    def start_build(self):
        """빌드 시작"""
        targets = self.get_selected_targets()
        if not targets:
            QMessageBox.warning(self, "경고", "빌드할 타겟을 선택해주세요.")
            return

        self.log_text.clear()
        self.log_text.append(f"빌드 시작: {', '.join(targets)}")
        self.log_text.append(f"프로젝트 경로: {self.project_root}")
        self.log_text.append(f"클린 빌드: {'예' if self.clean_build_cb.isChecked() else '아니오'}")
        self.log_text.append("")

        self.build_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.build_worker = BuildWorker(
            self.project_root,
            targets,
            self.clean_build_cb.isChecked()
        )
        self.build_worker.output.connect(self.on_build_output)
        self.build_worker.finished.connect(self.on_build_finished)
        self.build_worker.start()

    def stop_build(self):
        """빌드 중지"""
        if self.build_worker and self.build_worker.isRunning():
            self.build_worker.terminate()
            self.log_text.append("\n⚠️ 빌드가 중지되었습니다.")

        self.build_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def quick_build(self, targets: list):
        """빠른 빌드"""
        for key, cb in self.target_checkboxes.items():
            cb.setChecked(key in targets)
        self.start_build()

    def on_build_output(self, text: str):
        """빌드 출력 처리"""
        self.log_text.append(text)
        # 스크롤 하단으로
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.log_text.setTextCursor(cursor)

    def on_build_finished(self, success: bool, message: str):
        """빌드 완료"""
        self.build_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        if success:
            self.log_text.append(f"\n✅ {message}")
            QMessageBox.information(self, "빌드 완료", message)
        else:
            self.log_text.append(f"\n❌ {message}")
            QMessageBox.critical(self, "빌드 실패", message)

    def save_log(self):
        """로그 저장"""
        from PyQt6.QtWidgets import QFileDialog
        from datetime import datetime

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "로그 저장",
            f"build_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt)"
        )

        if filename:
            with open(filename, "w") as f:
                f.write(self.log_text.toPlainText())
            QMessageBox.information(self, "저장 완료", f"로그가 저장되었습니다:\n{filename}")
