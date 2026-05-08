#!/usr/bin/env python3
"""
URJC DEEPRACER — visualiza /image_raw y /cmd_vel en tiempo real.
Requisitos:
    pip install PyQt6 pyqtgraph numpy opencv-python --break-system-packages
    sudo apt install ros-humble-cv-bridge  (o pip install cv_bridge si usas venv)
"""

import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu

CV_BRIDGE_OK = False   # evitamos cv_bridge: incompatible con numpy 2.x en Humble

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QFrame, QSizePolicy, QGridLayout
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor, QPainter, QPen, QBrush, QLinearGradient

import pyqtgraph as pg

# ── Pyqtgraph dark theme ────────────────────────────────────────────────────
pg.setConfigOption("background", "#0d1117")
pg.setConfigOption("foreground", "#c9d1d9")


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 node (runs in a background thread)
# ══════════════════════════════════════════════════════════════════════════════

class Signals(QObject):
    image_received    = pyqtSignal(np.ndarray)
    image2_received   = pyqtSignal(np.ndarray)
    cmd_vel_received = pyqtSignal(float, float)        # linear.x, angular.z
    imu_received     = pyqtSignal(float, float, float, # angular_velocity x y z
                                  float, float, float)  # linear_acceleration x y z
    topic_status     = pyqtSignal(str, bool)           # topic name, alive


class DashboardNode(Node):
    def __init__(self, signals: Signals):
        super().__init__("ros2_dashboard")
        self.signals = signals
        self.bridge  = None
        self._last_img_t   = 0.0
        self._last_img2_t   = 0.0
        self._last_vel_t   = 0.0
        self._last_imu_t   = 0.0

        self.create_subscription(Image, "/camera/image_raw", self._cb_image,  10)
        self.create_subscription(Image, "/coral/image_annotated", self._cb_image2, 10)
        self.create_subscription(Twist, "/cmd_vel",          self._cb_vel,   10)
        self.create_subscription(Imu,   "/imu/mpu6050",      self._cb_imu,   10)

        # Watchdog: emit topic status every second
        self.create_timer(1.0, self._watchdog)

    def _cb_image(self, msg: Image):
        self._last_img_t = time.time()
        try:
            frame = self._decode_image(msg)
            self.signals.image_received.emit(frame)
        except Exception as e:
            self.get_logger().warning(f"image decode error: {e}")

    def _cb_image2(self, msg: Image):
        self._last_img2_t = time.time()
        try:
            frame = self._decode_image(msg)
            self.signals.image2_received.emit(frame)
        except Exception as e:
            self.get_logger().warning(f"image2 decode error: {e}")

    def _decode_image(self, msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            frame = arr.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            return arr.reshape((msg.height, msg.width, 3))
        elif msg.encoding in ("mono8", "8UC1"):
            frame = arr.reshape((msg.height, msg.width))
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            return arr.reshape((msg.height, msg.width, 3))

    def _cb_vel(self, msg: Twist):
        self._last_vel_t = time.time()
        self.signals.cmd_vel_received.emit(msg.linear.x, msg.angular.z)

    def _cb_imu(self, msg: Imu):
        self._last_imu_t = time.time()
        self.signals.imu_received.emit(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )

    def _watchdog(self):
        now = time.time()
        self.signals.topic_status.emit("/camera/image_raw", (now - self._last_img_t) < 3.0)
        self.signals.topic_status.emit("/coral/image_annotated", (now - self._last_img2_t) < 3.0)
        self.signals.topic_status.emit("/cmd_vel",          (now - self._last_vel_t) < 3.0)
        self.signals.topic_status.emit("/imu/mpu6050",      (now - self._last_imu_t) < 3.0)


def ros_spin(node):
    rclpy.spin(node)


# ══════════════════════════════════════════════════════════════════════════════
# UI helpers
# ══════════════════════════════════════════════════════════════════════════════

STYLE = """
QMainWindow, QWidget {
    background: #0d1117;
    color: #c9d1d9;
    font-family: 'JetBrains Mono', 'Fira Code', 'Courier New', monospace;
}
QLabel#title {
    font-size: 20px;
    font-weight: bold;
    color: #58a6ff;
    letter-spacing: 3px;
    padding: 8px 0 4px 0;
}
QLabel#section {
    font-size: 11px;
    color: #8b949e;
    letter-spacing: 2px;
    text-transform: uppercase;
    padding: 4px 0 2px 0;
}
QFrame#panel {
    background: #161b22;
    border: 1px solid #21262d;
    border-radius: 10px;
}
QLabel#stat_val {
    font-size: 32px;
    font-weight: bold;
    color: #58a6ff;
}
QLabel#stat_unit {
    font-size: 11px;
    color: #8b949e;
}
QLabel#dot_on  { color: #3fb950; font-size: 13px; }
QLabel#dot_off { color: #f85149; font-size: 13px; }
"""


def panel(parent=None) -> QFrame:
    f = QFrame(parent)
    f.setObjectName("panel")
    return f


def section_label(text: str) -> QLabel:
    l = QLabel(text.upper())
    l.setObjectName("section")
    return l


# ══════════════════════════════════════════════════════════════════════════════
# Velocity gauge widget (arc painter)
# ══════════════════════════════════════════════════════════════════════════════

class GaugeWidget(QWidget):
    def __init__(self, label="", max_val=2.0, color="#58a6ff", parent=None):
        super().__init__(parent)
        self.label   = label
        self.max_val = max_val
        self.color   = QColor(color)
        self._value  = 0.0
        self.setMinimumSize(140, 140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_value(self, v: float):
        self._value = max(-self.max_val, min(self.max_val, v))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 16

        # Background circle: 360°
        bg = QColor("#21262d")
        p.setPen(QPen(bg, 10, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawArc(cx - r, cy - r, 2*r, 2*r, 0, 360*16)

        # Map value: 0 → 0°, max_val → 360° (full circle)
        frac = self._value / self.max_val          # -1..1
        span = int(frac * 360 * 16)                # full 360° at max
        start_angle = 90 * 16                      # top (12 o'clock)
        color = self.color if self._value >= 0 else QColor("#f85149")
        p.setPen(QPen(color, 10, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawArc(cx - r, cy - r, 2*r, 2*r, start_angle, -span)
        # Center text: value
        p.setPen(QPen(QColor("#c9d1d9")))
        p.setFont(QFont("JetBrains Mono", 18, QFont.Weight.Bold))
        p.drawText(0, cy - 16, w, 32, Qt.AlignmentFlag.AlignCenter, f"{self._value:+.2f}")
        p.setFont(QFont("JetBrains Mono", 9))
        p.setPen(QPen(QColor("#8b949e")))
        p.drawText(0, cy + 10, w, 20, Qt.AlignmentFlag.AlignCenter, self.label)
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
# Main window
# ══════════════════════════════════════════════════════════════════════════════

HISTORY = 200   # puntos en las gráficas

class MainWindow(QMainWindow):
    def __init__(self, signals: Signals):
        super().__init__()
        self.signals = signals
        self.setWindowTitle("URJC DEEPRACER")
        self.resize(1280, 760)
        self.setStyleSheet(STYLE)

        self._lin_buf  = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._ang_buf  = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._fps_buf  = deque(maxlen=30)
        self._last_img = 0.0
        self._topic_labels: dict[str, QLabel] = {}

        # IMU buffers — gyro (rad/s) y accel (m/s²)
        self._gx_buf = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._gy_buf = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._gz_buf = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._ax_buf = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._ay_buf = deque([0.0] * HISTORY, maxlen=HISTORY)
        self._az_buf = deque([0.0] * HISTORY, maxlen=HISTORY)

        self._build_ui()
        self._connect_signals()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(16, 12, 16, 12)
        main.setSpacing(10)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("URJC DEEPRACER")
        title.setObjectName("title")
        hdr.addWidget(title)
        hdr.addStretch()
        self._fps_label = QLabel("-- fps")
        self._fps_label.setObjectName("stat_unit")
        hdr.addWidget(self._fps_label)
        # Topic status dots
        for topic in ["/camera/image_raw", "/coral/image_annotated", "/cmd_vel", "/imu/mpu6050"]:
            dot = QLabel("●  " + topic)
            dot.setObjectName("dot_off")
            self._topic_labels[topic] = dot
            hdr.addSpacing(12)
            hdr.addWidget(dot)
        main.addLayout(hdr)

        # Body: left col (cam + cmd_vel) | right col (IMU)
        body = QHBoxLayout()
        body.setSpacing(10)
        main.addLayout(body, stretch=1)

        # ── Left column ───────────────────────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(10)
        body.addLayout(left, stretch=3)

        # Camera panel
        # Two camera panels side by side
        cams_row = QHBoxLayout()
        cams_row.setSpacing(8)

        cam_panel = panel()
        cam_layout = QVBoxLayout(cam_panel)
        cam_layout.setContentsMargins(10, 10, 10, 10)
        cam_layout.addWidget(section_label("camera  /camera/image_raw"))
        self._cam_label = QLabel()
        self._cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._cam_label.setMinimumSize(200, 200)
        self._cam_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._cam_label.setStyleSheet("background:#0d1117; border-radius:6px;")
        self._cam_label.setText("Waiting image...")
        cam_layout.addWidget(self._cam_label, stretch=1)
        cams_row.addWidget(cam_panel, stretch=1)

        cam2_panel = panel()
        cam2_layout = QVBoxLayout(cam2_panel)
        cam2_layout.setContentsMargins(10, 10, 10, 10)
        cam2_layout.addWidget(section_label("filtered  /coral/image_annotated"))
        self._cam2_label = QLabel()
        self._cam2_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._cam2_label.setMinimumSize(200, 200)
        self._cam2_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._cam2_label.setStyleSheet("background:#0d1117; border-radius:6px;")
        self._cam2_label.setText("Waiting filtered image...")
        cam2_layout.addWidget(self._cam2_label, stretch=1)
        cams_row.addWidget(cam2_panel, stretch=1)

        left.addLayout(cams_row, stretch=3)

        # cmd_vel panel (gauges + plots + stats)
        vel_panel = panel()
        vp_layout = QVBoxLayout(vel_panel)
        vp_layout.setContentsMargins(10, 8, 10, 8)
        vp_layout.addWidget(section_label("cmd_vel  /cmd_vel"))
        gauges_row = QHBoxLayout()
        self._gauge_lin = GaugeWidget("linear.x  m/s",   max_val=1, color="#58a6ff")
        self._gauge_ang = GaugeWidget("angular.z  rad/s", max_val=1, color="#3fb950")
        self._gauge_lin.setMaximumHeight(130)
        self._gauge_ang.setMaximumHeight(130)
        gauges_row.addWidget(self._gauge_lin)
        gauges_row.addWidget(self._gauge_ang)
        vp_layout.addLayout(gauges_row)

        self._plot_lin = pg.PlotWidget()
        self._plot_ang = pg.PlotWidget()
        for pw, color in [(self._plot_lin, "#58a6ff"), (self._plot_ang, "#3fb950")]:
            pw.setBackground("#0d1117")
            pw.getPlotItem().getAxis("bottom").setStyle(showValues=False)
            pw.setMaximumHeight(80)
            pw.showGrid(x=False, y=True, alpha=0.15)
            pw.setMouseEnabled(x=False, y=False)
            pw.setMenuEnabled(False)
        self._curve_lin = self._plot_lin.plot(pen=pg.mkPen("#58a6ff", width=2))
        self._curve_ang = self._plot_ang.plot(pen=pg.mkPen("#3fb950", width=2))
        self._plot_lin.setYRange(-0.6, 0.6, padding=0.05)
        self._plot_ang.setYRange(-0.6, 0.6, padding=0.05)
        self._plot_lin.enableAutoRange(axis='y', enable=False)
        self._plot_ang.enableAutoRange(axis='y', enable=False)
        self._plot_lin.addLine(y=0, pen=pg.mkPen("#4a5568", width=1, style=Qt.PenStyle.DashLine))
        self._plot_ang.addLine(y=0, pen=pg.mkPen("#4a5568", width=1, style=Qt.PenStyle.DashLine))
        vp_layout.addWidget(self._plot_lin)
        vp_layout.addWidget(self._plot_ang)

        left.addWidget(vel_panel, stretch=2)

        # ── Right column — IMU ────────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(10)
        body.addLayout(right, stretch=2)

        imu_panel = panel()
        ip_layout = QVBoxLayout(imu_panel)
        ip_layout.setContentsMargins(10, 10, 10, 10)
        ip_layout.addWidget(section_label("imu  /imu/mpu6050"))

        # 6 numeric stats: gx gy gz | ax ay az
        imu_stats = QGridLayout()
        imu_stats.setSpacing(4)
        imu_attrs = [
            ("gx", "ω x  rad/s", "#c084fc"),
            ("gy", "ω y  rad/s", "#a78bfa"),
            ("gz", "ω z  rad/s", "#818cf8"),
            ("ax", "a x  m/s²",  "#fb923c"),
            ("ay", "a y  m/s²",  "#f97316"),
            ("az", "a z  m/s²",  "#ea580c"),
        ]
        for i, (attr, lbl, color) in enumerate(imu_attrs):
            row, col = divmod(i, 3)
            box = QFrame()
            box.setStyleSheet("background:#0d1117; border-radius:6px;")
            bl = QVBoxLayout(box)
            bl.setContentsMargins(6, 4, 6, 4)
            bl.setSpacing(1)
            vl = QLabel("0.000")
            vl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            vl.setStyleSheet(f"font-size:18px; font-weight:bold; color:{color};")
            ul = QLabel(lbl)
            ul.setAlignment(Qt.AlignmentFlag.AlignCenter)
            ul.setObjectName("stat_unit")
            bl.addWidget(vl)
            bl.addWidget(ul)
            imu_stats.addWidget(box, row, col)
            setattr(self, f"_imu_{attr}", vl)
        ip_layout.addLayout(imu_stats)

        # Gyro plot (3 curves: x y z)
        ip_layout.addWidget(section_label("giroscopio  rad/s"))
        self._plot_gyro = pg.PlotWidget()
        self._plot_gyro.setBackground("#0d1117")
        self._plot_gyro.getPlotItem().getAxis("bottom").setStyle(showValues=False)
        self._plot_gyro.showGrid(x=False, y=True, alpha=0.15)
        self._plot_gyro.setMouseEnabled(x=False, y=False)
        self._plot_gyro.setMenuEnabled(False)
        self._plot_gyro.addLegend(offset=(5, 5))
        self._curve_gx = self._plot_gyro.plot(pen=pg.mkPen("#c084fc", width=1.2), name="x")
        self._curve_gy = self._plot_gyro.plot(pen=pg.mkPen("#a78bfa", width=1.2), name="y")
        self._curve_gz = self._plot_gyro.plot(pen=pg.mkPen("#818cf8", width=1.2), name="z")
        ip_layout.addWidget(self._plot_gyro)

        # Accel plot (3 curves: x y z)
        ip_layout.addWidget(section_label("acelerómetro  m/s²"))
        self._plot_accel = pg.PlotWidget()
        self._plot_accel.setBackground("#0d1117")
        self._plot_accel.getPlotItem().getAxis("bottom").setStyle(showValues=False)
        self._plot_accel.showGrid(x=False, y=True, alpha=0.15)
        self._plot_accel.setMouseEnabled(x=False, y=False)
        self._plot_accel.setMenuEnabled(False)
        self._plot_accel.addLegend(offset=(5, 5))
        self._curve_ax = self._plot_accel.plot(pen=pg.mkPen("#fb923c", width=1.2), name="x")
        self._curve_ay = self._plot_accel.plot(pen=pg.mkPen("#f97316", width=1.2), name="y")
        self._curve_az = self._plot_accel.plot(pen=pg.mkPen("#ea580c", width=1.2), name="z")
        ip_layout.addWidget(self._plot_accel)

        right.addWidget(imu_panel)

        # Refresh timer for plots / fps
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(50)   # 20 Hz UI refresh

    # ── Signals ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self.signals.image_received.connect(self._on_image)
        self.signals.image2_received.connect(self._on_image2)
        self.signals.cmd_vel_received.connect(self._on_cmd_vel)
        self.signals.imu_received.connect(self._on_imu)
        self.signals.topic_status.connect(self._on_topic_status)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_image(self, frame: np.ndarray):
        now = time.time()
        if self._last_img > 0:
            self._fps_buf.append(1.0 / max(now - self._last_img, 1e-6))
        self._last_img = now

        h, w = frame.shape[:2]
        lw = max(self._cam_label.width(), 100)
        lh = max(self._cam_label.height(), 100)
        scale = min(lw / w, lh / h)
        nw, nh = int(w * scale), int(h * scale)
        frame_rgb = cv2.cvtColor(cv2.resize(frame, (nw, nh)), cv2.COLOR_BGR2RGB)
        qimg = QImage(frame_rgb.data, nw, nh, nw * 3, QImage.Format.Format_RGB888)
        self._cam_label.setPixmap(QPixmap.fromImage(qimg))

    def _on_image2(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        lw = max(self._cam2_label.width(), 100)
        lh = max(self._cam2_label.height(), 100)
        scale = min(lw / w, lh / h)
        nw, nh = int(w * scale), int(h * scale)
        frame_rgb = cv2.cvtColor(cv2.resize(frame, (nw, nh)), cv2.COLOR_BGR2RGB)
        qimg = QImage(frame_rgb.data, nw, nh, nw * 3, QImage.Format.Format_RGB888)
        self._cam2_label.setPixmap(QPixmap.fromImage(qimg))

    def _on_cmd_vel(self, lin: float, ang: float):
        self._lin_buf.append(lin)
        self._ang_buf.append(ang)
        self._gauge_lin.set_value(lin)
        self._gauge_ang.set_value(ang)

    def _on_imu(self, gx: float, gy: float, gz: float,
                ax: float, ay: float, az: float):
        self._gx_buf.append(gx); self._gy_buf.append(gy); self._gz_buf.append(gz)
        self._ax_buf.append(ax); self._ay_buf.append(ay); self._az_buf.append(az)
        self._imu_gx.setText(f"{gx:+.3f}")
        self._imu_gy.setText(f"{gy:+.3f}")
        self._imu_gz.setText(f"{gz:+.3f}")
        self._imu_ax.setText(f"{ax:+.3f}")
        self._imu_ay.setText(f"{ay:+.3f}")
        self._imu_az.setText(f"{az:+.3f}")

    def _on_topic_status(self, topic: str, alive: bool):
        lbl = self._topic_labels.get(topic)
        if lbl:
            lbl.setObjectName("dot_on" if alive else "dot_off")
            lbl.setStyleSheet("color: #3fb950;" if alive else "color: #f85149;")

    def _refresh_plots(self):
        x = list(range(HISTORY))
        self._curve_lin.setData(x, list(self._lin_buf))
        self._curve_ang.setData(x, list(self._ang_buf))
        self._curve_gx.setData(x, list(self._gx_buf))
        self._curve_gy.setData(x, list(self._gy_buf))
        self._curve_gz.setData(x, list(self._gz_buf))
        self._curve_ax.setData(x, list(self._ax_buf))
        self._curve_ay.setData(x, list(self._ay_buf))
        self._curve_az.setData(x, list(self._az_buf))

        if self._fps_buf:
            fps = sum(self._fps_buf) / len(self._fps_buf)
            self._fps_label.setText(f"{fps:.1f} fps")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    signals = Signals()

    app = QApplication(sys.argv)
    app.setApplicationName("URJC DEEPRACER")

    win = MainWindow(signals)
    win.showMaximized()

    node   = DashboardNode(signals)
    thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
