import sys
from PyQt6.QtWidgets import (QApplication, QWidget, QGridLayout, QVBoxLayout, 
                             QHBoxLayout, QLabel, QFrame, QTextEdit)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

class RobotDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 전체 메인 레이아웃 (수직)
        main_layout = QVBoxLayout()
        
        # --- 상단 영역 (Cam A, Cam B, Map) ---
        top_layout = QHBoxLayout()
        
        self.cam_a = self.create_display_box("Cam_A")
        self.cam_b = self.create_display_box("Cam_B")
        self.map_view = self.create_display_box("지도 (map)")
        
        top_layout.addWidget(self.cam_a)
        top_layout.addWidget(self.cam_b)
        top_layout.addWidget(self.map_view)
        
        # --- 하단 영역 (로봇 상태) ---
        bottom_layout = QVBoxLayout()
        
        status_label = QLabel("로봇 상태")
        status_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        
        # 상태 메시지가 출력될 영역
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        self.status_display.setPlaceholderText("~ Human tracking\n~ Root driving")
        self.status_display.setStyleSheet("background-color: #f0f0f0; border-radius: 5px;")
        
        bottom_layout.addWidget(status_label)
        bottom_layout.addWidget(self.status_display)
        
        # 메인 레이아웃에 상/하단 추가 (비율 조절 가능)
        main_layout.addLayout(top_layout, stretch=2)
        main_layout.addLayout(bottom_layout, stretch=1)
        
        self.setLayout(main_layout)
        self.setWindowTitle('Robot Monitoring UI')
        self.resize(1000, 600)

    def create_display_box(self, text):
        """카메라 및 지도를 위한 검은색 박스 생성 함수"""
        label = QLabel(text)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet("""
            background-color: #2c3e50; 
            color: white; 
            border: 2px solid #34495e;
            border-radius: 10px;
            font-size: 16px;
            font-weight: bold;
        """)
        label.setMinimumHeight(300)
        return label

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = RobotDashboard()
    ex.show()
    sys.exit(app.exec())