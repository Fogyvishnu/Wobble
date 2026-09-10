#!/usr/bin/env python3
"""
Project Wobble: Unified Robot Remote Controller
Author: Antigravity Agent for Project Wobble
Description:
    Interactive remote control interface running alongside Gazebo Harmonic.
    Supports WASD for omnidirectional movement, O/P for squat and stand postures,
    Space for emergency braking, with live balance telemetry display.
    Includes a responsive PyQt5 GUI with key-hold detection and automatic fallback
    to interactive terminal CLI mode.
"""

import os
import sys
import math
import time
import argparse
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

# Check if GUI is supported
HAS_PYQT = False
try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QPushButton, QLabel, QSlider, QFrame, QProgressBar
    )
    from PyQt5.QtGui import QFont, QColor, QPalette, QKeyEvent
    HAS_PYQT = True
except ImportError:
    HAS_PYQT = False


class RemoteControlRosNode(Node):
    """ROS 2 Node handling communication with Wobble balance controller."""

    def __init__(self):
        super().__init__('wobble_remote_control')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_squat_pub = self.create_publisher(Float64, '/cmd_squat', 10)

        # Telemetry Subscriber
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray,
            '/wobble_telemetry',
            self.telemetry_callback,
            10
        )

        # Telemetry state
        self.pitch_deg = 0.0
        self.measured_vel = 0.0
        self.commanded_vel = 0.0
        self.squat_angle = 0.0
        self.is_upright = True
        self.last_telemetry_time = self.get_clock().now()

        # Motion targets
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        # Motion tuning limits
        self.max_linear_vel = 0.40   # m/s
        self.max_angular_vel = 0.85  # rad/s
        self.linear_accel = 0.80     # m/s^2 ramping
        self.angular_accel = 2.00    # rad/s^2 ramping

        # Posture angles
        self.squat_target_angle = -0.42  # rad (~24 deg athletic squat)
        self.stand_target_angle = 0.00   # rad (upright)
        self.current_posture_cmd = 0.00

        # Periodic timer for smooth velocity publication (50 Hz)
        self.timer_period = 0.02
        self.pub_timer = self.create_timer(self.timer_period, self.publish_motion_loop)

    def telemetry_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 7:
            self.pitch_deg = msg.data[0]
            self.measured_vel = msg.data[1]
            self.commanded_vel = msg.data[2]
            self.squat_angle = msg.data[5]
            self.is_upright = (msg.data[6] > 0.5)
            self.last_telemetry_time = self.get_clock().now()

    def set_motion_intent(self, forward: float, turn: float):
        """Set normalized directional intent: forward in [-1, 1], turn in [-1, 1]."""
        self.target_linear_vel = forward * self.max_linear_vel
        self.target_angular_vel = turn * self.max_angular_vel

    def emergency_stop(self):
        """Zero out all velocity commands instantly."""
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def command_squat(self):
        """Command the robot to squat."""
        self.current_posture_cmd = self.squat_target_angle
        msg = Float64()
        msg.data = float(self.squat_target_angle)
        self.cmd_squat_pub.publish(msg)
        self.get_logger().info(f"Posture command: SQUAT ({self.squat_target_angle:.2f} rad)")

    def command_stand(self):
        """Command the robot to stand upright."""
        self.current_posture_cmd = self.stand_target_angle
        msg = Float64()
        msg.data = float(self.stand_target_angle)
        self.cmd_squat_pub.publish(msg)
        self.get_logger().info(f"Posture command: STAND ({self.stand_target_angle:.2f} rad)")

    def set_custom_squat_angle(self, angle_rad: float):
        """Command an arbitrary squat angle."""
        clamped = max(-1.50, min(1.50, angle_rad))
        self.current_posture_cmd = clamped
        msg = Float64()
        msg.data = float(clamped)
        self.cmd_squat_pub.publish(msg)

    def publish_motion_loop(self):
        """Ramps velocities smoothly and publishes Twist commands."""
        # Slew-rate acceleration ramping
        dt = self.timer_period

        # Linear velocity ramp
        d_lin = self.target_linear_vel - self.current_linear_vel
        max_d_lin = self.linear_accel * dt
        if abs(d_lin) > max_d_lin:
            self.current_linear_vel += math.copysign(max_d_lin, d_lin)
        else:
            self.current_linear_vel = self.target_linear_vel

        # Angular velocity ramp
        d_ang = self.target_angular_vel - self.current_angular_vel
        max_d_ang = self.angular_accel * dt
        if abs(d_ang) > max_d_ang:
            self.current_angular_vel += math.copysign(max_d_ang, d_ang)
        else:
            self.current_angular_vel = self.target_angular_vel

        # Publish Twist command
        twist = Twist()
        twist.linear.x = float(self.current_linear_vel)
        twist.angular.z = float(self.current_angular_vel)
        self.cmd_vel_pub.publish(twist)


if HAS_PYQT:
    class WobbleRemoteGUI(QMainWindow):
        """Modern cybernetic PyQt5 remote control interface."""

        def __init__(self, ros_node: RemoteControlRosNode):
            super().__init__()
            self.node = ros_node

            # Key tracking set to support multiple simultaneous keys (e.g. W + A)
            self.active_keys = set()

            self.init_ui()

            # Qt timer to process ROS2 messages and update UI at 50 Hz
            self.ros_timer = QTimer(self)
            self.ros_timer.timeout.connect(self.update_tick)
            self.ros_timer.start(20)

        def init_ui(self):
            self.setWindowTitle("Wobble Remote Controller")
            self.setFixedSize(440, 620)
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #11111b;
                }
                QFrame#card {
                    background-color: #1e1e2e;
                    border: 1px solid #313244;
                    border-radius: 10px;
                }
                QLabel {
                    color: #cdd6f4;
                    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
                }
                QPushButton {
                    background-color: #313244;
                    color: #cdd6f4;
                    font-size: 15px;
                    font-weight: bold;
                    border: 1px solid #45475a;
                    border-radius: 8px;
                    padding: 8px;
                }
                QPushButton:hover {
                    background-color: #45475a;
                    border: 1px solid #89b4fa;
                }
                QPushButton:pressed, QPushButton[active="true"] {
                    background-color: #89b4fa;
                    color: #11111b;
                    border: 1px solid #b4befe;
                }
                QPushButton#squatBtn[active="true"] {
                    background-color: #fab387;
                    color: #11111b;
                }
                QPushButton#standBtn[active="true"] {
                    background-color: #a6e3a1;
                    color: #11111b;
                }
                QPushButton#stopBtn[active="true"] {
                    background-color: #f38ba8;
                    color: #11111b;
                }
                QSlider::groove:horizontal {
                    height: 6px;
                    background: #313244;
                    border-radius: 3px;
                }
                QSlider::sub-page:horizontal {
                    background: #89b4fa;
                    border-radius: 3px;
                }
                QSlider::handle:horizontal {
                    background: #cdd6f4;
                    width: 14px;
                    margin-top: -4px;
                    margin-bottom: -4px;
                    border-radius: 7px;
                }
            """)

            central = QWidget()
            self.setCentralWidget(central)
            main_layout = QVBoxLayout(central)
            main_layout.setContentsMargins(16, 14, 16, 14)
            main_layout.setSpacing(12)

            # Header
            header_layout = QHBoxLayout()
            title_lbl = QLabel("🤖 WOBBLE REMOTE CONTROL")
            title_lbl.setFont(QFont("Arial", 14, QFont.Bold))
            title_lbl.setStyleSheet("color: #89b4fa; letter-spacing: 1px;")
            header_layout.addWidget(title_lbl)

            self.status_badge = QLabel("READY")
            self.status_badge.setFont(QFont("Arial", 10, QFont.Bold))
            self.status_badge.setStyleSheet("background: #a6e3a1; color: #11111b; border-radius: 4px; padding: 3px 8px;")
            header_layout.addWidget(self.status_badge, alignment=Qt.AlignRight)
            main_layout.addLayout(header_layout)

            # Telemetry Card
            telemetry_card = QFrame()
            telemetry_card.setObjectName("card")
            t_layout = QGridLayout(telemetry_card)
            t_layout.setContentsMargins(12, 10, 12, 10)

            t_layout.addWidget(QLabel("Pitch Tilt:"), 0, 0)
            self.pitch_val = QLabel("0.0°")
            self.pitch_val.setFont(QFont("Monospace", 11, QFont.Bold))
            self.pitch_val.setStyleSheet("color: #a6e3a1;")
            t_layout.addWidget(self.pitch_val, 0, 1)

            t_layout.addWidget(QLabel("Linear Speed:"), 0, 2)
            self.speed_val = QLabel("0.00 m/s")
            self.speed_val.setFont(QFont("Monospace", 11, QFont.Bold))
            t_layout.addWidget(self.speed_val, 0, 3)

            t_layout.addWidget(QLabel("Posture Angle:"), 1, 0)
            self.posture_val = QLabel("0.00 rad (Stand)")
            self.posture_val.setFont(QFont("Monospace", 11, QFont.Bold))
            t_layout.addWidget(self.posture_val, 1, 1)

            t_layout.addWidget(QLabel("Balance State:"), 1, 2)
            self.balance_state = QLabel("BALANCED")
            self.balance_state.setFont(QFont("Arial", 10, QFont.Bold))
            self.balance_state.setStyleSheet("color: #a6e3a1;")
            t_layout.addWidget(self.balance_state, 1, 3)

            main_layout.addWidget(telemetry_card)

            # Movement Controls Card (WASD)
            wasd_card = QFrame()
            wasd_card.setObjectName("card")
            w_layout = QVBoxLayout(wasd_card)
            w_layout.setContentsMargins(12, 10, 12, 10)

            move_hdr = QLabel("🕹️ DIRECTIONAL MOVEMENT (WASD)")
            move_hdr.setFont(QFont("Arial", 11, QFont.Bold))
            move_hdr.setStyleSheet("color: #bac2de;")
            w_layout.addWidget(move_hdr, alignment=Qt.AlignCenter)

            grid = QGridLayout()
            grid.setSpacing(8)

            self.btn_w = QPushButton("▲ W\nForward")
            self.btn_w.setFixedSize(100, 60)
            grid.addWidget(self.btn_w, 0, 1)

            self.btn_a = QPushButton("◀ A\nTurn Left")
            self.btn_a.setFixedSize(100, 60)
            grid.addWidget(self.btn_a, 1, 0)

            self.btn_s = QPushButton("▼ S\nBackward")
            self.btn_s.setFixedSize(100, 60)
            grid.addWidget(self.btn_s, 1, 1)

            self.btn_d = QPushButton("▶ D\nTurn Right")
            self.btn_d.setFixedSize(100, 60)
            grid.addWidget(self.btn_d, 1, 2)

            w_layout.addLayout(grid)

            # Connect mouse press/release on movement buttons
            self.btn_w.pressed.connect(lambda: self.add_virtual_key('W'))
            self.btn_w.released.connect(lambda: self.remove_virtual_key('W'))
            self.btn_s.pressed.connect(lambda: self.add_virtual_key('S'))
            self.btn_s.released.connect(lambda: self.remove_virtual_key('S'))
            self.btn_a.pressed.connect(lambda: self.add_virtual_key('A'))
            self.btn_a.released.connect(lambda: self.remove_virtual_key('A'))
            self.btn_d.pressed.connect(lambda: self.add_virtual_key('D'))
            self.btn_d.released.connect(lambda: self.remove_virtual_key('D'))

            main_layout.addWidget(wasd_card)

            # Posture Controls Card (O / P / Space)
            posture_card = QFrame()
            posture_card.setObjectName("card")
            p_layout = QVBoxLayout(posture_card)
            p_layout.setContentsMargins(12, 10, 12, 10)

            posture_hdr = QLabel("🦿 4-BAR POSTURE CONTROL")
            posture_hdr.setFont(QFont("Arial", 11, QFont.Bold))
            posture_hdr.setStyleSheet("color: #bac2de;")
            p_layout.addWidget(posture_hdr, alignment=Qt.AlignCenter)

            p_btn_layout = QHBoxLayout()
            p_btn_layout.setSpacing(10)

            self.btn_squat = QPushButton("[O] SQUAT\n-0.42 rad (24°)")
            self.btn_squat.setObjectName("squatBtn")
            self.btn_squat.setFixedHeight(50)
            self.btn_squat.clicked.connect(self.on_squat_clicked)
            p_btn_layout.addWidget(self.btn_squat)

            self.btn_stand = QPushButton("[P] STAND\n0.00 rad (0°)")
            self.btn_stand.setObjectName("standBtn")
            self.btn_stand.setFixedHeight(50)
            self.btn_stand.clicked.connect(self.on_stand_clicked)
            p_btn_layout.addWidget(self.btn_stand)

            p_layout.addLayout(p_btn_layout)

            # Emergency Stop Button
            self.btn_stop = QPushButton("[SPACE] EMERGENCY BRAKE / STOP")
            self.btn_stop.setObjectName("stopBtn")
            self.btn_stop.setFixedHeight(40)
            self.btn_stop.setStyleSheet("""
                QPushButton#stopBtn {
                    background-color: #452834;
                    color: #f38ba8;
                    border: 1px solid #f38ba8;
                }
                QPushButton#stopBtn:hover {
                    background-color: #f38ba8;
                    color: #11111b;
                }
            """)
            self.btn_stop.clicked.connect(self.on_stop_clicked)
            p_layout.addWidget(self.btn_stop)

            main_layout.addWidget(posture_card)

            # Speed Tuning Slider
            speed_card = QFrame()
            speed_card.setObjectName("card")
            s_layout = QHBoxLayout(speed_card)
            s_layout.setContentsMargins(12, 8, 12, 8)
            s_layout.addWidget(QLabel("Max Speed:"))
            self.speed_slider = QSlider(Qt.Horizontal)
            self.speed_slider.setRange(10, 80)
            self.speed_slider.setValue(int(self.node.max_linear_vel * 100))
            self.speed_slider.valueChanged.connect(self.on_speed_slider_changed)
            s_layout.addWidget(self.speed_slider)
            self.speed_slider_lbl = QLabel(f"{self.node.max_linear_vel:.2f} m/s")
            self.speed_slider_lbl.setFont(QFont("Monospace", 10, QFont.Bold))
            s_layout.addWidget(self.speed_slider_lbl)
            main_layout.addWidget(speed_card)

            # Instructions footer
            footer = QLabel("⌨️ Keyboard: Focus window and hold W/A/S/D to move, O to squat, P to stand, Space to stop.")
            footer.setFont(QFont("Arial", 9))
            footer.setStyleSheet("color: #6c7086;")
            footer.setWordWrap(True)
            main_layout.addWidget(footer)

        def add_virtual_key(self, key_char: str):
            self.active_keys.add(key_char)
            self.update_motion_from_keys()

        def remove_virtual_key(self, key_char: str):
            self.active_keys.discard(key_char)
            self.update_motion_from_keys()

        def on_speed_slider_changed(self, value):
            new_speed = value / 100.0
            self.node.max_linear_vel = new_speed
            self.speed_slider_lbl.setText(f"{new_speed:.2f} m/s")
            self.update_motion_from_keys()

        def on_squat_clicked(self):
            self.node.command_squat()
            self.btn_squat.setProperty("active", True)
            self.btn_stand.setProperty("active", False)
            self.btn_squat.style().unpolish(self.btn_squat)
            self.btn_squat.style().polish(self.btn_squat)
            self.btn_stand.style().unpolish(self.btn_stand)
            self.btn_stand.style().polish(self.btn_stand)

        def on_stand_clicked(self):
            self.node.command_stand()
            self.btn_stand.setProperty("active", True)
            self.btn_squat.setProperty("active", False)
            self.btn_stand.style().unpolish(self.btn_stand)
            self.btn_stand.style().polish(self.btn_stand)
            self.btn_squat.style().unpolish(self.btn_squat)
            self.btn_squat.style().polish(self.btn_squat)

        def on_stop_clicked(self):
            self.active_keys.clear()
            self.node.emergency_stop()
            self.update_motion_from_keys()

        def keyPressEvent(self, event: QKeyEvent):
            key = event.key()
            if key == Qt.Key_W:
                self.active_keys.add('W')
            elif key == Qt.Key_S:
                self.active_keys.add('S')
            elif key == Qt.Key_A:
                self.active_keys.add('A')
            elif key == Qt.Key_D:
                self.active_keys.add('D')
            elif key == Qt.Key_O:
                self.on_squat_clicked()
            elif key == Qt.Key_P:
                self.on_stand_clicked()
            elif key == Qt.Key_Space:
                self.on_stop_clicked()
            elif key == Qt.Key_Escape or key == Qt.Key_Q:
                self.close()
            else:
                super().keyPressEvent(event)

            self.update_motion_from_keys()

        def keyReleaseEvent(self, event: QKeyEvent):
            if event.isAutoRepeat():
                return

            key = event.key()
            if key == Qt.Key_W:
                self.active_keys.discard('W')
            elif key == Qt.Key_S:
                self.active_keys.discard('S')
            elif key == Qt.Key_A:
                self.active_keys.discard('A')
            elif key == Qt.Key_D:
                self.active_keys.discard('D')
            else:
                super().keyReleaseEvent(event)

            self.update_motion_from_keys()

        def update_motion_from_keys(self):
            """Calculates target forward and turn velocities based on active keys."""
            forward = 0.0
            turn = 0.0

            if 'W' in self.active_keys and 'S' not in self.active_keys:
                forward = 1.0
            elif 'S' in self.active_keys and 'W' not in self.active_keys:
                forward = -1.0

            if 'A' in self.active_keys and 'D' not in self.active_keys:
                turn = 1.0
            elif 'D' in self.active_keys and 'A' not in self.active_keys:
                turn = -1.0

            self.node.set_motion_intent(forward, turn)

            # Update visual button states
            self.set_button_active(self.btn_w, 'W' in self.active_keys)
            self.set_button_active(self.btn_s, 'S' in self.active_keys)
            self.set_button_active(self.btn_a, 'A' in self.active_keys)
            self.set_button_active(self.btn_d, 'D' in self.active_keys)

        def set_button_active(self, btn: QPushButton, is_active: bool):
            if btn.property("active") != is_active:
                btn.setProperty("active", is_active)
                btn.style().unpolish(btn)
                btn.style().polish(btn)

        def update_tick(self):
            """Processes ROS 2 callbacks and updates GUI indicators."""
            rclpy.spin_once(self.node, timeout_sec=0)

            # Update Telemetry labels
            pitch = self.node.pitch_deg
            self.pitch_val.setText(f"{pitch:+.1f}°")
            if abs(pitch) < 6.0:
                self.pitch_val.setStyleSheet("color: #a6e3a1;")  # Green
            elif abs(pitch) < 18.0:
                self.pitch_val.setStyleSheet("color: #fab387;")  # Amber
            else:
                self.pitch_val.setStyleSheet("color: #f38ba8;")  # Red

            self.speed_val.setText(f"{self.node.measured_vel:+.2f} m/s")

            sq = self.node.squat_angle
            if sq < -0.2:
                self.posture_val.setText(f"{sq:.2f} rad (Squat)")
                self.posture_val.setStyleSheet("color: #fab387;")
            else:
                self.posture_val.setText(f"{sq:.2f} rad (Stand)")
                self.posture_val.setStyleSheet("color: #a6e3a1;")

            if not self.node.is_upright:
                self.balance_state.setText("FALLEN")
                self.balance_state.setStyleSheet("color: #f38ba8;")
                self.status_badge.setText("FALLEN")
                self.status_badge.setStyleSheet("background: #f38ba8; color: #11111b; border-radius: 4px; padding: 3px 8px;")
            elif abs(self.node.current_linear_vel) > 0.05 or abs(self.node.current_angular_vel) > 0.05:
                self.balance_state.setText("DRIVING")
                self.balance_state.setStyleSheet("color: #89b4fa;")
                self.status_badge.setText("MOVING")
                self.status_badge.setStyleSheet("background: #89b4fa; color: #11111b; border-radius: 4px; padding: 3px 8px;")
            else:
                self.balance_state.setText("BALANCED")
                self.balance_state.setStyleSheet("color: #a6e3a1;")
                self.status_badge.setText("BALANCED")
                self.status_badge.setStyleSheet("background: #a6e3a1; color: #11111b; border-radius: 4px; padding: 3px 8px;")


def run_cli_mode(ros_node: RemoteControlRosNode):
    """Terminal CLI interactive controller using non-blocking termios."""
    import select
    import termios
    import tty

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("\n" + "=" * 58)
    print("       🤖 WOBBLE ROBOT TERMINAL REMOTE CONTROL")
    print("=" * 58)
    print("  Controls:")
    print("    [W] / [S]        : Drive Forward / Backward")
    print("    [A] / [D]        : Turn Left / Right")
    print("    [O]              : Squat (-0.42 rad / 24°)")
    print("    [P]              : Stand (0.00 rad / Upright)")
    print("    [SPACE] or [X]   : Stop / Brake")
    print("    [+] / [-]        : Increase / Decrease Speed")
    print("    [Q] or [Ctrl+C]  : Quit Controller")
    print("=" * 58 + "\n")

    current_forward = 0.0
    current_turn = 0.0

    try:
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(ros_node, timeout_sec=0.02)

            # Check for terminal key input
            rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
            if rlist:
                ch = sys.stdin.read(1)
                if ch in ['w', 'W']:
                    current_forward = 1.0
                elif ch in ['s', 'S']:
                    current_forward = -1.0
                elif ch in ['a', 'A']:
                    current_turn = 1.0
                elif ch in ['d', 'D']:
                    current_turn = -1.0
                elif ch in [' ', 'x', 'X']:
                    current_forward = 0.0
                    current_turn = 0.0
                    ros_node.emergency_stop()
                elif ch in ['o', 'O']:
                    ros_node.command_squat()
                elif ch in ['p', 'P']:
                    ros_node.command_stand()
                elif ch == '+':
                    ros_node.max_linear_vel = min(0.8, ros_node.max_linear_vel + 0.05)
                    print(f"\n[Speed Set] Max: {ros_node.max_linear_vel:.2f} m/s")
                elif ch == '-':
                    ros_node.max_linear_vel = max(0.1, ros_node.max_linear_vel - 0.05)
                    print(f"\n[Speed Set] Max: {ros_node.max_linear_vel:.2f} m/s")
                elif ch in ['q', 'Q', '\x03']:
                    print("\nExiting Remote Control...")
                    break

                ros_node.set_motion_intent(current_forward, current_turn)

            # Print status line
            posture_str = "SQUAT" if ros_node.current_posture_cmd < -0.2 else "STAND"
            state_str = "OK" if ros_node.is_upright else "FALLEN!"
            sys.stdout.write(
                f"\r[Wobble] Pitch: {ros_node.pitch_deg:+5.1f}° | "
                f"Vel: {ros_node.measured_vel:+4.2f} m/s | "
                f"Target: {ros_node.current_linear_vel:+4.2f} m/s | "
                f"Posture: {posture_str} | Status: {state_str}   "
            )
            sys.stdout.flush()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ros_node.emergency_stop()


def main(args=None):
    parser = argparse.ArgumentParser(description="Wobble Robot Remote Control")
    parser.add_argument('--cli', action='store_true', help='Force terminal CLI mode instead of GUI')
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    ros_node = RemoteControlRosNode()

    # Determine execution mode: GUI if DISPLAY and PyQt5 available, else CLI
    use_gui = HAS_PYQT and (not parsed_args.cli) and ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ)

    if use_gui:
        try:
            # Set XCB platform for Linux Wayland compatibility if not set
            if 'QT_QPA_PLATFORM' not in os.environ:
                os.environ['QT_QPA_PLATFORM'] = 'xcb'

            app = QApplication(sys.argv)
            gui = WobbleRemoteGUI(ros_node)
            gui.show()
            exit_code = app.exec_()
        except Exception as e:
            print(f"[Wobble Remote] GUI initialization failed ({e}). Falling back to CLI mode...")
            use_gui = False

    if not use_gui:
        run_cli_mode(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
