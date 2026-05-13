# GUI.py
import os
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import sys
import math
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import OccupancyGrid, Path
# [변경] Odometry 제거 → PoseWithCovarianceStamped 추가
# [삭제] from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # [추가] AMCL pose 타입
from std_msgs.msg import String, Int32MultiArray

from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap


class SignalBridge(QObject):
    cam_a_signal   = pyqtSignal(np.ndarray)
    cam_b_signal   = pyqtSignal(np.ndarray)
    map_signal     = pyqtSignal(object)
    # [변경] odom_signal → amcl_signal 로 교체
    # [삭제] odom_signal = pyqtSignal(object)
    amcl_signal    = pyqtSignal(object)  # [추가] /amcl_pose 신호
    mode_signal    = pyqtSignal(str)
    gesture_signal = pyqtSignal(list)
    scan_signal    = pyqtSignal(object)
    goal_signal    = pyqtSignal(object)
    path_signal    = pyqtSignal(object)


class ROSUINode(Node):
    def __init__(self, bridge: SignalBridge):
        super().__init__('robot_ui_node')
        self.bridge = bridge

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # [변경] AMCL도 transient_local QoS 사용 (노드 재시작 시 마지막 위치 즉시 수신)
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(CompressedImage,           'camA_display',   self._camA_cb,    10)
        self.create_subscription(CompressedImage,           'camB_display',   self._camB_cb,    10)
        self.create_subscription(OccupancyGrid,             '/map',           self._map_cb,     map_qos)
        # [변경] /odom(Odometry) 구독 제거 → /amcl_pose(PoseWithCovarianceStamped) 구독으로 교체
        # [삭제] self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',     self._amcl_cb,    amcl_qos)  # [추가]
        self.create_subscription(String,                    'current_mode',   self._mode_cb,    10)
        self.create_subscription(Int32MultiArray,           'gesture_data',   self._gesture_cb, 10)
        self.create_subscription(LaserScan,                 '/scan',          self._scan_cb,    10)
        self.create_subscription(PoseStamped,               'auto_drive_goal',self._goal_cb,    10)
        self.create_subscription(Path,                      'global_path',    self._path_cb,    10)

    def _camA_cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None: self.bridge.cam_a_signal.emit(frame)

    def _camB_cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None: self.bridge.cam_b_signal.emit(frame)

    def _map_cb(self, msg):     self.bridge.map_signal.emit(msg)
    # [변경] _odom_cb 제거 → _amcl_cb 추가
    # [삭제] def _odom_cb(self, msg): self.bridge.odom_signal.emit(msg)
    def _amcl_cb(self, msg):    self.bridge.amcl_signal.emit(msg)  # [추가]
    def _mode_cb(self, msg):    self.bridge.mode_signal.emit(msg.data)
    def _scan_cb(self, msg):    self.bridge.scan_signal.emit(msg)
    def _goal_cb(self, msg):    self.bridge.goal_signal.emit(msg)
    def _path_cb(self, msg):    self.bridge.path_signal.emit(msg)
    def _gesture_cb(self, msg):
        if len(msg.data) >= 2:
            self.bridge.gesture_signal.emit(list(msg.data))


class RobotDashboard(QWidget):
    CLASS_NAMES = {
        0: ("Palm",   "손바닥", "#FFA726"),
        1: ("Back",   "손등",   "#AB47BC"),
        2: ("Finger", "손가락", "#26A69A"),
        3: ("Fist",   "주먹",   "#EF5350"),
        4: ("Human",  "사람",   "#42A5F5"),
    }
    MODE_META = {
        "waiting":    ("WAITING",    "#90A4AE", "●"),
        "auto_drive": ("AUTO DRIVE", "#FB8C00", "●"),
        "gesture":    ("GESTURE",    "#66BB6A", "●"),
        "follow":     ("FOLLOW",     "#29B6F6", "●"),
    }

    def __init__(self, bridge: SignalBridge):
        super().__init__()
        self.bridge = bridge
        self._map_base    = None
        self._map_img     = None
        # [변경] _latest_odom → _latest_amcl 로 교체
        # [삭제] self._latest_odom = None
        self._latest_amcl = None  # [추가] AMCL pose 캐시
        self._latest_scan = None
        self._latest_goal = None
        self._latest_path = None
        self._init_ui()
        self._connect_signals()

        self._map_timer = QTimer(self)
        self._map_timer.timeout.connect(self._render_map)
        self._map_timer.start(200)

    def _init_ui(self):
        self.setWindowTitle('HANDSOME Dashboard v2.0')
        self.resize(1300, 800)
        self.setStyleSheet("QWidget { background-color: #050505; color: #E0E0E0; font-family: 'Segoe UI', sans-serif; }")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(25, 25, 25, 25)
        layout.setSpacing(20)

        # 헤더
        header = QHBoxLayout()
        title_container = QVBoxLayout()
        title = QLabel("ROBOT CONTROL SYSTEM")
        title.setStyleSheet("font-size: 24px; font-weight: 800; color: #FFFFFF; letter-spacing: 1px;")
        subtitle = QLabel("REAL-TIME MONITORING DASHBOARD")
        subtitle.setStyleSheet("font-size: 11px; color: #606060; font-weight: 600;")
        title_container.addWidget(title)
        title_container.addWidget(subtitle)

        self.mode_badge = QLabel("● WAITING")
        self.mode_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mode_badge.setFixedSize(160, 45)
        self.mode_badge.setStyleSheet("background-color: #1A1A1A; border: 1px solid #333333; border-radius: 22px; color: #90A4AE; font-weight: 800; font-size: 14px;")

        header.addLayout(title_container)
        header.addStretch()
        header.addWidget(self.mode_badge)
        layout.addLayout(header)

        # 카메라/맵 패널
        cam_row = QHBoxLayout()
        cam_row.setSpacing(15)
        self.cam_a_label = self._make_panel("CAMERA A", "Main Feed")
        self.cam_b_label = self._make_panel("CAMERA B", "Gesture Recognition")
        self.map_label   = self._make_panel("LOCAL MAP", "Navigation Grid")
        cam_row.addWidget(self.cam_a_label)
        cam_row.addWidget(self.cam_b_label)
        cam_row.addWidget(self.map_label)
        layout.addLayout(cam_row, stretch=5)

        # 하단 카드
        status_row = QHBoxLayout()
        status_row.setSpacing(15)
        self.detect_card = self._make_info_card("DETECTION ANALYTICS")
        self.info_card   = self._make_info_card("ROBOT SYSTEM LOG")
        status_row.addWidget(self.detect_card, stretch=1)
        status_row.addWidget(self.info_card, stretch=1)
        layout.addLayout(status_row, stretch=2)

    def _make_panel(self, title, sub):
        frame = QFrame()
        frame.setStyleSheet("background-color: #111111; border-radius: 15px; border: 1px solid #1A1A1A;")
        vbox = QVBoxLayout(frame)
        vbox.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel()
        lbl.setObjectName("display")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setText(f"<div style='text-align:center;'>"
                    f"<span style='font-size:16px; font-weight:bold; color:#444;'>{title}</span><br>"
                    f"<span style='font-size:10px; color:#333;'>{sub}</span></div>")
        lbl.setStyleSheet("border: none; background-color: transparent;")
        vbox.addWidget(lbl)
        return frame

    def _make_info_card(self, title):
        card = QFrame()
        card.setStyleSheet("background-color: #0F0F0F; border: 1px solid #1A1A1A; border-radius: 15px;")
        vbox = QVBoxLayout(card)
        vbox.setContentsMargins(20, 15, 20, 15)
        t = QLabel(title)
        t.setStyleSheet("color: #404040; font-size: 10px; font-weight: 800; letter-spacing: 1.5px;")
        v = QLabel("NO DATA STREAMING")
        v.setObjectName("value")
        v.setWordWrap(True)
        v.setStyleSheet("color: #808080; font-size: 15px; font-weight: 500; border: none;")
        vbox.addWidget(t)
        vbox.addWidget(v)
        vbox.addStretch()
        return card

    def _connect_signals(self):
        self.bridge.cam_a_signal.connect(self._update_cam_a)
        self.bridge.cam_b_signal.connect(self._update_cam_b)
        self.bridge.map_signal.connect(self._cache_map)
        # [변경] odom_signal → amcl_signal 연결로 교체
        # [삭제] self.bridge.odom_signal.connect(self._cache_odom)
        self.bridge.amcl_signal.connect(self._cache_amcl)  # [추가]
        self.bridge.mode_signal.connect(self._update_mode)
        self.bridge.gesture_signal.connect(self._update_gesture)
        self.bridge.scan_signal.connect(self._cache_scan)
        self.bridge.goal_signal.connect(self._cache_goal)
        self.bridge.path_signal.connect(self._cache_path)

    def _update_cam_a(self, frame):
        lbl = self.cam_a_label.findChild(QLabel, "display")
        if lbl: lbl.setPixmap(self._to_pix(frame, lbl.size()))

    def _update_cam_b(self, frame):
        lbl = self.cam_b_label.findChild(QLabel, "display")
        if lbl: lbl.setPixmap(self._to_pix(frame, lbl.size()))

    def _to_pix(self, frame, size):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qi  = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1]*3, QImage.Format.Format_RGB888)
        return QPixmap.fromImage(qi).scaled(size, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)

    def _cache_map(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        img  = np.zeros((h, w), dtype=np.uint8)
        img[data ==   0] = 255
        img[data == 100] = 50
        img[data ==  -1] = 25
        self._map_img  = cv2.cvtColor(cv2.flip(img, 0), cv2.COLOR_GRAY2BGR)
        self._map_base = msg

    # [변경] _cache_odom 제거 → _cache_amcl 추가
    # [삭제] def _cache_odom(self, msg): self._latest_odom = msg
    def _cache_amcl(self, msg): self._latest_amcl = msg  # [추가] PoseWithCovarianceStamped 캐시
    def _cache_scan(self, msg): self._latest_scan = msg
    def _cache_goal(self, msg): self._latest_goal = msg
    def _cache_path(self, msg): self._latest_path = msg

    def _world_to_canvas(self, wx, wy):
        res = self._map_base.info.resolution
        ox  = self._map_base.info.origin.position.x
        oy  = self._map_base.info.origin.position.y
        h   = self._map_base.info.height
        w   = self._map_base.info.width
        mx  = int((wx - ox) / res)
        my  = h - int((wy - oy) / res)
        mx  = max(0, min(mx, w - 1))
        my  = max(0, min(my, h - 1))
        return mx, my

    def _render_map(self):
        if self._map_img is None or self._map_base is None: return
        canvas = self._map_img.copy()

        # [변경] LiDAR 포인트 시각화: odom → AMCL pose 기준으로 교체
        # AMCL이 map 프레임 기준이므로 LiDAR 포인트 좌표가 지도와 정확히 일치
        if self._latest_amcl and self._latest_scan:
            rx  = self._latest_amcl.pose.pose.position.x    # [변경] amcl에서 위치 추출
            ry  = self._latest_amcl.pose.pose.position.y
            qz  = self._latest_amcl.pose.pose.orientation.z
            qw  = self._latest_amcl.pose.pose.orientation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

            scan  = self._latest_scan
            angle = scan.angle_min
            for r in scan.ranges:
                if math.isfinite(r) and r <= 1.0 and r > 0.05:
                    lx = rx + r * math.cos(yaw + angle)
                    ly = ry + r * math.sin(yaw + angle)
                    px, py = self._world_to_canvas(lx, ly)
                    cv2.circle(canvas, (px, py), 2, (0, 220, 220), -1)
                angle += scan.angle_increment

        # 경로 선 (초록색)
        if self._latest_path and len(self._latest_path.poses) > 1:
            pts = []
            for ps in self._latest_path.poses:
                px, py = self._world_to_canvas(ps.pose.position.x, ps.pose.position.y)
                pts.append((px, py))
            for i in range(len(pts) - 1):
                cv2.line(canvas, pts[i], pts[i+1], (0, 200, 0), 2)

        # 목적지 (빨간 X 표시)
        if self._latest_goal:
            gx, gy = self._world_to_canvas(
                self._latest_goal.pose.position.x,
                self._latest_goal.pose.position.y)
            cv2.drawMarker(canvas, (gx, gy), (0, 0, 255), cv2.MARKER_CROSS, 5, 1)
            cv2.circle(canvas, (gx, gy), 10, (0, 0, 255), 2)

        # [변경] 로봇 위치 표시: odom → AMCL pose 기준으로 교체
        if self._latest_amcl:
            rx  = self._latest_amcl.pose.pose.position.x    # [변경] amcl에서 위치 추출
            ry  = self._latest_amcl.pose.pose.position.y
            qz  = self._latest_amcl.pose.pose.orientation.z
            qw  = self._latest_amcl.pose.pose.orientation.w
            yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

            mx, my     = self._world_to_canvas(rx, ry)
            arrow_len  = 15
            ax = int(mx + arrow_len * math.cos(yaw))
            ay = int(my - arrow_len * math.sin(yaw))
            ay = max(0, min(ay, canvas.shape[0] - 1))
            ax = max(0, min(ax, canvas.shape[1] - 1))

            cv2.circle(canvas, (mx, my), 3, (255, 80, 80), -1)
            cv2.arrowedLine(canvas, (mx, my), (ax, ay), (255, 255, 255), 1, tipLength=0.2)

        lbl = self.map_label.findChild(QLabel, "display")
        if lbl:
            lbl.setPixmap(self._to_pix(canvas, lbl.size()))

    def _update_mode(self, mode):
        label, color, icon = self.MODE_META.get(mode, (mode.upper(), "#FFFFFF", "●"))
        self.mode_badge.setText(f"{icon} {label}")
        self.mode_badge.setStyleSheet(f"background-color: {color}15; border: 1px solid {color}50; border-radius: 22px; color: {color}; font-weight: 800; font-size: 14px;")
        self._set_v(self.info_card, f"<span style='color:{color}; font-size:20px;'><b>{label}</b></span><br><span style='color:#505050;'>Current operation mode status is active.</span>")

    def _update_gesture(self, data):
        en, ko, color = self.CLASS_NAMES.get(data[0], (f"ID{data[0]}", "?", "#FFFFFF"))
        self._set_v(self.detect_card, f"<span style='color:{color}; font-size:22px;'><b>{en}</b></span><br><span style='color:#505050;'>{ko} 감지됨 (X: {data[1]}px)</span>")

    def _set_v(self, card, html):
        v = card.findChild(QLabel, "value")
        if v: v.setText(html)


def main():
    cv2.setNumThreads(0)
    rclpy.init()
    app    = QApplication(sys.argv)
    bridge = SignalBridge()
    node   = ROSUINode(bridge)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    db = RobotDashboard(bridge)
    db.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()