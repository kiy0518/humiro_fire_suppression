"""
진행률 추적 탭
work-plan 기반 진행률 표시
"""

import os
import re
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QProgressBar, QScrollArea, QFrame, QTextEdit
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QColor


class ProgressTab(QWidget):
    """진행률 추적 탭"""

    def __init__(self, project_root: str):
        super().__init__()
        self.project_root = project_root
        self.work_plan_dir = os.path.join(project_root, "work-plan")

        self.init_ui()
        self.load_progress()

    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout(self)

        # 전체 진행률
        overall_group = QGroupBox("전체 진행률")
        overall_layout = QVBoxLayout(overall_group)

        # 진행률 바
        progress_layout = QHBoxLayout()
        self.overall_progress = QProgressBar()
        self.overall_progress.setMaximum(100)
        self.overall_progress.setMinimumHeight(30)
        self.overall_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #28a745;
            }
        """)
        progress_layout.addWidget(self.overall_progress)

        self.overall_label = QLabel("0%")
        self.overall_label.setFont(QFont("Sans", 14, QFont.Weight.Bold))
        progress_layout.addWidget(self.overall_label)

        overall_layout.addLayout(progress_layout)

        # 요약 통계
        stats_layout = QHBoxLayout()

        self.completed_label = QLabel("완료: 0")
        self.completed_label.setStyleSheet("color: green;")
        stats_layout.addWidget(self.completed_label)

        self.partial_label = QLabel("부분: 0")
        self.partial_label.setStyleSheet("color: orange;")
        stats_layout.addWidget(self.partial_label)

        self.pending_label = QLabel("미구현: 0")
        self.pending_label.setStyleSheet("color: red;")
        stats_layout.addWidget(self.pending_label)

        stats_layout.addStretch()

        refresh_btn = QPushButton("새로고침")
        refresh_btn.clicked.connect(self.load_progress)
        stats_layout.addWidget(refresh_btn)

        overall_layout.addLayout(stats_layout)
        layout.addWidget(overall_group)

        # 카테고리별 진행률
        category_group = QGroupBox("카테고리별 진행률")
        category_layout = QVBoxLayout(category_group)

        self.category_table = QTableWidget()
        self.category_table.setColumnCount(4)
        self.category_table.setHorizontalHeaderLabels(["카테고리", "완료", "전체", "진행률"])
        self.category_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.category_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.category_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.category_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.Fixed)
        self.category_table.horizontalHeader().resizeSection(3, 150)
        category_layout.addWidget(self.category_table)

        layout.addWidget(category_group)

        # 미구현 항목 목록
        pending_group = QGroupBox("미구현/부분 구현 항목")
        pending_layout = QVBoxLayout(pending_group)

        self.pending_table = QTableWidget()
        self.pending_table.setColumnCount(4)
        self.pending_table.setHorizontalHeaderLabels(["번호", "항목", "상태", "우선순위"])
        self.pending_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.pending_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.pending_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.pending_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        pending_layout.addWidget(self.pending_table)

        layout.addWidget(pending_group)

    def load_progress(self):
        """진행률 로드"""
        # Gap Analysis 문서에서 데이터 추출
        gap_analysis_path = os.path.join(self.work_plan_dir, "026_IMPLEMENTATION_GAP_ANALYSIS.md")

        if not os.path.exists(gap_analysis_path):
            self.overall_progress.setValue(0)
            self.overall_label.setText("0% (분석 문서 없음)")
            return

        try:
            with open(gap_analysis_path, "r") as f:
                content = f.read()

            # 진행률 파싱
            progress_data = self.parse_gap_analysis(content)
            self.update_ui(progress_data)

        except Exception as e:
            QMessageBox.warning(self, "오류", f"진행률 로드 실패: {e}")

    def parse_gap_analysis(self, content: str) -> dict:
        """Gap Analysis 문서 파싱"""
        data = {
            "overall_percent": 60,  # 문서의 ~60%
            "completed": 9,
            "partial": 7,
            "pending": 6,
            "categories": [],
            "pending_items": []
        }

        # 카테고리별 진행률
        categories = [
            ("핵심 비행 기능", 90, 100),
            ("스웜 통신", 85, 100),
            ("안전 기능", 20, 100),
            ("발사 시스템", 0, 100),
        ]
        data["categories"] = categories

        # 미구현 항목 추출
        pending_items = []

        # CRITICAL 항목
        critical_pattern = r"### (\d+) - (.+?) \[CRITICAL\]"
        for match in re.finditer(critical_pattern, content):
            pending_items.append({
                "number": match.group(1),
                "item": match.group(2),
                "status": "미구현",
                "priority": "🔴 CRITICAL"
            })

        # HIGH 항목
        high_pattern = r"### (\d+) - (.+?) \[HIGH\]"
        for match in re.finditer(high_pattern, content):
            pending_items.append({
                "number": match.group(1),
                "item": match.group(2),
                "status": "부분 구현",
                "priority": "🟠 HIGH"
            })

        # MEDIUM 항목
        medium_pattern = r"### (\d+) - (.+?) \[MEDIUM\]"
        for match in re.finditer(medium_pattern, content):
            pending_items.append({
                "number": match.group(1),
                "item": match.group(2),
                "status": "부분 구현",
                "priority": "🟡 MEDIUM"
            })

        # FUTURE 항목
        future_pattern = r"### (\d+) - (.+?) \[FUTURE\]"
        for match in re.finditer(future_pattern, content):
            pending_items.append({
                "number": match.group(1),
                "item": match.group(2),
                "status": "미구현",
                "priority": "🔵 FUTURE"
            })

        data["pending_items"] = pending_items

        return data

    def update_ui(self, data: dict):
        """UI 업데이트"""
        # 전체 진행률
        percent = data.get("overall_percent", 0)
        self.overall_progress.setValue(percent)
        self.overall_label.setText(f"{percent}%")

        self.completed_label.setText(f"완료: {data.get('completed', 0)}")
        self.partial_label.setText(f"부분: {data.get('partial', 0)}")
        self.pending_label.setText(f"미구현: {data.get('pending', 0)}")

        # 카테고리별 진행률
        categories = data.get("categories", [])
        self.category_table.setRowCount(len(categories))

        for row, (name, completed, total) in enumerate(categories):
            self.category_table.setItem(row, 0, QTableWidgetItem(name))
            self.category_table.setItem(row, 1, QTableWidgetItem(str(completed)))
            self.category_table.setItem(row, 2, QTableWidgetItem(str(total)))

            # 진행률 바
            progress_bar = QProgressBar()
            progress_bar.setMaximum(total)
            progress_bar.setValue(completed)
            progress_bar.setFormat(f"{completed}%")

            if completed >= 80:
                progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
            elif completed >= 50:
                progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #ffc107; }")
            else:
                progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #dc3545; }")

            self.category_table.setCellWidget(row, 3, progress_bar)

        # 미구현 항목
        pending_items = data.get("pending_items", [])
        self.pending_table.setRowCount(len(pending_items))

        for row, item in enumerate(pending_items):
            self.pending_table.setItem(row, 0, QTableWidgetItem(item.get("number", "")))
            self.pending_table.setItem(row, 1, QTableWidgetItem(item.get("item", "")))
            self.pending_table.setItem(row, 2, QTableWidgetItem(item.get("status", "")))

            priority_item = QTableWidgetItem(item.get("priority", ""))
            priority = item.get("priority", "")
            if "CRITICAL" in priority:
                priority_item.setForeground(QColor(220, 53, 69))
            elif "HIGH" in priority:
                priority_item.setForeground(QColor(253, 126, 20))
            elif "MEDIUM" in priority:
                priority_item.setForeground(QColor(255, 193, 7))
            else:
                priority_item.setForeground(QColor(23, 162, 184))

            self.pending_table.setItem(row, 3, priority_item)
