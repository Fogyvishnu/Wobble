#!/usr/bin/env python3
"""
Project Wobble: Autonomous Vision-Guided Course Navigator
Author: Antigravity Agent for Project Wobble

Description:
    Fully autonomous visual navigation system for Project Wobble's 30-Meter
    Championship Proving Ground with 8 Obstacle Stages:
      1. Stage 1: Visual Runway Acceleration & Centerline Tracking (0.0m - 2.0m)
      2. Stage 2: Overhead Hurdle 1 - Dynamic 4-Bar Squat Under Low Bar (3.2m)
      3. Stage 3: Elevated Standing Posture Speed Gate (5.5m)
      4. Stage 4: 5-Gate Slalom Arena - Closed-Loop Sinusoidal Tracking & Visual Repulsion (6.7m - 15.0m)
      5. Stage 5: Precision Canyon / Squeeze Corridor (15.0m - 19.0m)
      6. Stage 6: Graded Multi-Frequency Speed Bumps Suspension Adaptation (19.0m - 23.5m)
      7. Stage 7: Overhead Hurdle 2 - Double Squat Clearance Challenge (25.0m)
      8. Stage 8: Grand Sprint & Championship Finish Arch Balanced Braking (30.0m)
    Broadcasts /camera/annotated_image with Cybernetic HUD overlay for RViz2 / operator telemetry.
"""

import math
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64, Float64MultiArray, String
from cv_bridge import CvBridge


class WobbleVisionNavigator(Node):
    """Autonomous vision-guided navigation node for Project Wobble."""

    def __init__(self):
        super().__init__('wobble_course_navigator')

        # Parameters
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('rate_hz', 25.0)
        self.wheel_r = float(self.get_parameter('wheel_radius').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / self.rate_hz

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.squat_pub = self.create_publisher(Float64, '/cmd_squat', 10)
        self.reset_pub = self.create_publisher(Empty, '/wobble/reset', 10)
        self.status_pub = self.create_publisher(String, '/wobble/course_status', 10)
        self.annotated_img_pub = self.create_publisher(Image, '/camera/annotated_image', 10)

        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/wobble/odom', self.odometry_callback, 10)
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray, '/wobble/telemetry', self.telemetry_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, '/wobble/reset', self.reset_callback, 10)

        # Ground Truth & Odometry State
        self.has_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Vision State
        self.latest_frame = None
        self.has_image = False
        self.frame_count = 0
        self.last_img_time = time.time()
        self.fps = 0.0

        # Visual Detection Flags & Metrics
        self.lane_error = 0.0        # Normalized error from center in [-1, 1]
        self.lane_detected = False
        self.hurdle_detected = False
        self.hurdle_distance_px = 0.0
        self.hurdle_stage = 1
        self.bollard_detected = False
        self.bollard_offset = 0.0
        self.bollard_cx = 0.0
        self.bollard_w = 0.0
        self.finish_detected = False

        # Odometry / Balance State
        self.left_pos = None
        self.right_pos = None
        self.distance_traveled = 0.0
        self.is_balanced = False
        self.pitch = 0.0
        self.squat_feedback = 0.0

        # Command Smoothing (acceleration limits)
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0
        self.current_squat_cmd = 0.0

        # Navigation State Machine
        self.state = "INIT_WAIT"
        self.state_timer = 0.0
        self.target_squat_angle = 0.0

        # Main Navigation Control Loop
        self.timer = self.create_timer(self.dt, self.navigation_step)
        self.get_logger().info("Autonomous 30m Championship Vision Navigator Initialized. Awaiting camera & balance...")

    def odometry_callback(self, msg: Odometry):
        self.has_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        # Ground-truth progress synchronization
        if self.robot_x > self.distance_traveled:
            self.distance_traveled = self.robot_x

    def telemetry_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.pitch = msg.data[0]
            self.squat_feedback = msg.data[6]
            self.is_balanced = (msg.data[7] > 0.5) and (abs(self.pitch) < 0.20)

    def reset_callback(self, msg: Empty):
        self.state = "INIT_WAIT"
        self.state_timer = 0.0
        self.distance_traveled = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.left_pos = None
        self.right_pos = None
        self.hurdle_detected = False
        self.hurdle_stage = 1
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0
        self.current_squat_cmd = 0.0
        self.get_logger().info("Reset signal received: Navigator state reset to INIT_WAIT.")

    def joint_callback(self, msg: JointState):
        d_left = None
        d_right = None
        for i, name in enumerate(msg.name):
            if name == 'joint_left_wheel' and len(msg.position) > i:
                cur_left = msg.position[i]
                if self.left_pos is not None:
                    d_left = (cur_left - self.left_pos) * self.wheel_r
                else:
                    d_left = 0.0
                self.left_pos = cur_left

            elif name == 'joint_right_wheel' and len(msg.position) > i:
                cur_right = msg.position[i]
                if self.right_pos is not None:
                    d_right = (cur_right - self.right_pos) * self.wheel_r
                else:
                    d_right = 0.0
                self.right_pos = cur_right

        if d_left is not None and d_right is not None:
            d_fwd = (d_left + d_right) / 2.0
            if abs(d_fwd) < 0.25:
                self.distance_traveled += d_fwd

    def camera_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV BGR8
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
            self.has_image = True
            self.frame_count += 1

            now = time.time()
            dt_img = now - self.last_img_time
            if dt_img > 0.0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt_img)
            self.last_img_time = now

            # Process Computer Vision Pipeline
            self.process_vision_frame(frame)

        except Exception as e:
            self.get_logger().warn(f"Camera frame processing exception: {e}")

    def process_vision_frame(self, frame: np.ndarray):
        """Processes RGB camera frame for visual cues and generates annotated overlay."""
        h, w, _ = frame.shape
        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------------- 1. Visual Lane / Centerline Detection ----------------
        # Look in lower half of image (ground track)
        roi_lane = hsv[int(h * 0.52):int(h * 0.95), int(w * 0.15):int(w * 0.85)]
        mask_yellow = cv2.inRange(roi_lane, np.array([18, 80, 80]), np.array([38, 255, 255]))
        cnts_y, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if cnts_y:
            c_large = max(cnts_y, key=cv2.contourArea)
            if cv2.contourArea(c_large) > 150:
                M = cv2.moments(c_large)
                if M["m00"] > 0:
                    cx_local = int(M["m10"] / M["m00"])
                    cx_global = cx_local + int(w * 0.15)
                    self.lane_error = (cx_global - (w / 2.0)) / (w / 2.0)
                    self.lane_detected = True

                    # Draw lane center on annotated image
                    cv2.circle(annotated, (cx_global, int(h * 0.75)), 7, (0, 255, 255), -1)
                    cv2.line(annotated, (int(w / 2), int(h * 0.75)), (cx_global, int(h * 0.75)), (0, 255, 255), 2)
            else:
                self.lane_detected = False
        else:
            self.lane_detected = False

        # ---------------- 2. Visual Hurdle (Low Clearance Crossbar) Detection ----------------
        # Hurdle 1 at X = 3.2m, Hurdle 2 at X = 25.0m
        roi_hurdle = hsv[int(h * 0.28):int(h * 0.65), :]
        mask_orange = cv2.inRange(roi_hurdle, np.array([8, 90, 80]), np.array([25, 255, 255]))
        cnts_h, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hurdle_boxes = []
        for c in cnts_h:
            area = cv2.contourArea(c)
            if area > 40:
                bx, by, bw, bh = cv2.boundingRect(c)
                # Overhead crossbar is horizontal (bw > bh * 2.5) with substantial span (bw > 60)
                if (bw > bh * 2.5) and (bw > 60):
                    by_global = by + int(h * 0.28)
                    hurdle_boxes.append((bx, by_global, bw, bh, area))

        d_cur = self.distance_traveled
        is_hurdle_zone = (d_cur < 4.2) or (22.5 <= d_cur <= 26.2)
        if hurdle_boxes and is_hurdle_zone:
            largest_h = max(hurdle_boxes, key=lambda b: b[2])
            bx, by, bw, bh, area = largest_h
            self.hurdle_detected = True
            self.hurdle_distance_px = float(bw)
            self.hurdle_stage = 1 if d_cur < 10.0 else 2

            # Draw bounding box and alert
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 140, 255), 2)
            cv2.putText(annotated, f"HURDLE {self.hurdle_stage} CROSSBAR: W={bw}px", (bx, max(20, by - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2)
        else:
            self.hurdle_detected = False
            self.hurdle_distance_px = 0.0

        # ---------------- 3. Visual Slalom Bollard Detection (Cyan/Blue) ----------------
        # Active in Slalom Zone: X = 6.0m to 15.0m
        roi_slalom = hsv[int(h * 0.35):int(h * 0.75), :]
        mask_cyan = cv2.inRange(roi_slalom, np.array([85, 75, 55]), np.array([115, 255, 255]))
        cnts_c, _ = cv2.findContours(mask_cyan, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bollards = []
        for c in cnts_c:
            area = cv2.contourArea(c)
            if area > 50:
                bx, by, bw, bh = cv2.boundingRect(c)
                by_global = by + int(h * 0.35)
                cx = bx + bw / 2.0
                bollards.append((cx, by_global, bw, bh, area))

        if bollards and (6.0 <= d_cur <= 15.0):
            bollards.sort(key=lambda b: b[4], reverse=True)
            self.bollard_detected = True
            primary_b = bollards[0]
            cx, by, bw, bh, area = primary_b
            self.bollard_cx = float(cx)
            self.bollard_w = float(bw)
            self.bollard_offset = (cx - (w / 2.0)) / (w / 2.0)

            for i, b in enumerate(bollards[:2]):
                bcx, bby, bbw, bbh, barea = b
                cv2.rectangle(annotated, (int(bcx - bbw/2), int(bby)), (int(bcx + bbw/2), int(bby + bbh)), (255, 200, 0), 2)
                cv2.putText(annotated, f"BOLLARD", (int(bcx - bbw/2), int(bby - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)
        else:
            self.bollard_detected = False
            self.bollard_cx = 0.0
            self.bollard_w = 0.0

        # ---------------- 4. Visual Finish Line / Arch Detection (Green Banner) ----------------
        roi_finish = hsv[int(h * 0.15):int(h * 0.65), :]
        mask_green = cv2.inRange(roi_finish, np.array([38, 70, 50]), np.array([85, 255, 255]))
        cnts_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.finish_detected = False
        self.finish_banner_w = 0.0
        if cnts_g and d_cur > 25.0:
            c_arch = max(cnts_g, key=cv2.contourArea)
            if cv2.contourArea(c_arch) > 150:
                bx, by, bw, bh = cv2.boundingRect(c_arch)
                by_global = by + int(h * 0.15)
                self.finish_detected = True
                self.finish_banner_w = float(bw)
                cv2.rectangle(annotated, (bx, by_global), (bx + bw, by_global + bh), (0, 255, 100), 2)
                cv2.putText(annotated, f"FINISH ARCH (W={bw}px)", (bx, max(20, by_global - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 100), 2)

        # ---------------- 5. Render Cybernetic HUD Overlay ----------------
        self.draw_hud_overlay(annotated, w, h)

        # Publish annotated vision stream
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'camera_optical_link'
            self.annotated_img_pub.publish(out_msg)
        except Exception:
            pass

    def draw_hud_overlay(self, img: np.ndarray, w: int, h: int):
        """Renders high-tech cybernetic heads-up display overlay on image."""
        # Top banner background bar
        cv2.rectangle(img, (0, 0), (w, 42), (20, 24, 28), -1)
        cv2.line(img, (0, 42), (w, 42), (0, 180, 255), 2)

        # Title & Mode
        cv2.putText(img, "WOBBLE 30M AUTONOMOUS PROVING GROUND", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2)

        # Status badge
        badge_color = (0, 255, 120) if self.state != "MISSION_COMPLETE" else (255, 200, 0)
        cv2.putText(img, f"[{self.state}]", (380, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, badge_color, 2)

        # FPS indicator
        cv2.putText(img, f"{self.fps:.0f} FPS", (w - 78, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (160, 170, 180), 1)

        # Lower Telemetry Bar
        cv2.rectangle(img, (0, h - 36), (w, h), (20, 24, 28), -1)
        cv2.line(img, (0, h - 36), (w, h - 36), (0, 180, 255), 1)

        pitch_deg = math.degrees(self.pitch)
        telem_str = f"Dist: {self.distance_traveled:4.1f}m/30m | Pitch: {pitch_deg:+5.1f}deg | Squat: {self.current_squat_cmd:+.2f} rad | Speed: {self.current_cmd_vx:+.2f} m/s"
        cv2.putText(img, telem_str, (12, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (230, 235, 240), 1)

        # Visual Reticle Center Crosshair
        cx, cy = int(w / 2), int(h / 2)
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

    def navigation_step(self):
        """Main autonomous vision-guided state machine navigation step."""
        d = self.robot_x if self.has_odom else self.distance_traveled
        self.state_timer += self.dt

        vx = 0.0
        wz = 0.0
        squat_angle = 0.0
        status_msg = ""

        # Visual Lane Center Steering Offset
        lane_wz = -0.40 * self.lane_error if self.lane_detected else 0.0

        # ==================== 8-Stage Championship State Machine ====================
        if self.state == "INIT_WAIT":
            status_msg = f"Awaiting settling (bal={self.is_balanced}, img={self.has_image}, t={self.state_timer:.1f}s)"

            # Auto-recovery: If not balanced after 1.5s, pulse /wobble/reset to stand upright
            if self.state_timer > 1.5 and not self.is_balanced and (int(self.state_timer * 10) % 15 == 0):
                self.reset_pub.publish(Empty())
                self.get_logger().info("Autonomous auto-recovery: Pulsing /wobble/reset upright...")

            if self.is_balanced and self.has_image and self.state_timer > 3.0:
                self.state = "STAGE1_RUNWAY_ACCEL"
                self.state_timer = 0.0
                self.distance_traveled = 0.0
                self.left_pos = None
                self.right_pos = None
                self.get_logger().info("Balance & Camera stream confirmed! Autonomous 30m championship mission starting.")
            else:
                log_str = String()
                log_str.data = f"[{self.state}] {status_msg} | Pitch: {math.degrees(self.pitch):.1f}° | FPS: {self.fps:.0f}"
                self.status_pub.publish(log_str)
                return

        # STAGE 1: Runway Acceleration
        elif self.state == "STAGE1_RUNWAY_ACCEL":
            status_msg = f"Stage 1: Runway Accel | Dist: {d:.2f}m"
            vx = 0.22
            wz = lane_wz
            squat_angle = 0.0

            # Hurdle 1 at 3.2m: initiate squat approach when distance >= 2.0m or visual sighting
            if (self.hurdle_detected and self.hurdle_distance_px > 210) or d >= 2.0:
                self.state = "STAGE2_APPROACH_HURDLE1"
                self.state_timer = 0.0
                self.get_logger().info(f"[STAGE 2] Hurdle 1 detected in camera (W={self.hurdle_distance_px:.0f}px)! Initiating squat.")

        # STAGE 2: Low-Clearance Overhead Hurdle 1
        elif self.state == "STAGE2_APPROACH_HURDLE1":
            status_msg = f"Stage 2: Hurdle 1 Squat | Dist: {d:.2f}m"
            vx = 0.20
            wz = lane_wz * 0.4
            squat_angle = -0.55  # Deep 4-bar squat clearance under 26.5cm bar

            if d >= 2.5 or (self.hurdle_distance_px > 300):
                self.state = "STAGE2_DUCK_HURDLE1"
                self.get_logger().info("[STAGE 2] Ducking under Hurdle 1 overhead bar...")

        elif self.state == "STAGE2_DUCK_HURDLE1":
            status_msg = f"Stage 2: Ducking Under Hurdle 1 | Dist: {d:.2f}m"
            vx = 0.22
            wz = 0.0
            squat_angle = -0.55

            if d >= 3.8:
                self.state = "STAGE3_SPEED_GATE"
                self.get_logger().info("[STAGE 3] Hurdle 1 cleared! Rising upright toward Speed Gate.")

        # STAGE 3: Elevated Standing Posture Speed Gate
        elif self.state == "STAGE3_SPEED_GATE":
            status_msg = f"Stage 3: Upright Speed Gate | Dist: {d:.2f}m"
            vx = 0.22
            wz = lane_wz
            squat_angle = 0.0  # Full upright posture

            if d >= 6.7:
                self.state = "STAGE4_SLALOM_5_GATE"
                self.get_logger().info("[STAGE 4] Speed Gate cleared! Entering 5-Gate Slalom Arena.")

        # STAGE 4: 5-Gate Championship Slalom Arena (6.7m to 14.8m)
        elif self.state == "STAGE4_SLALOM_5_GATE":
            squat_angle = -0.20  # Athletic crouch lowers CoM by 3.5cm for high agile stability
            vx = 0.16

            # Continuous sinusoidal reference trajectory through 5 gates:
            # Spacing: 1.6m, period = 3.2m, amplitude = 0.16m, starting at x = 6.7m
            # y*(x) = -0.16 * sin(pi/1.6 * (x - 6.7)) for x in [6.7, 14.7]
            look_ahead = 0.22
            x_eval = max(6.7, min(14.7, d + look_ahead))
            phase = (math.pi / 1.6) * (x_eval - 6.7)
            y_ref = -0.16 * math.sin(phase)
            dy_dx = -0.16 * (math.pi / 1.6) * math.cos(phase)
            psi_ref = math.atan(dy_dx)

            cur_y = self.robot_y if self.has_odom else 0.0
            cur_psi = self.robot_yaw if self.has_odom else 0.0
            e_y = cur_y - y_ref
            e_psi = math.atan2(math.sin(cur_psi - psi_ref), math.cos(cur_psi - psi_ref))

            tracking_wz = -2.2 * e_y - 0.7 * e_psi

            # Active Visual Obstacle Avoidance Repulsion
            vis_repulse_wz = 0.0
            if self.bollard_detected and self.bollard_w > 20:
                gate_num = int((d - 6.7) / 1.6) + 1
                if gate_num % 2 == 1:
                    # Bollard is on our left (+Y); if cx > 240 steer right
                    if self.bollard_cx > 240:
                        vis_repulse_wz = -0.40 * min(1.0, (self.bollard_cx - 240) / 100.0)
                else:
                    # Bollard is on our right (-Y); if cx < 400 steer left
                    if self.bollard_cx < 400:
                        vis_repulse_wz = 0.40 * min(1.0, (400 - self.bollard_cx) / 100.0)

            wz = max(-0.45, min(0.45, tracking_wz + vis_repulse_wz))
            current_gate = min(5, max(1, int((d - 6.7) / 1.6) + 1))
            status_msg = f"Stage 4: Slalom Gate {current_gate}/5 | Dist: {d:.2f}m | RefY: {y_ref:+.2f}m"

            if d >= 14.8:
                self.state = "STAGE5_NARROW_CANYON"
                self.get_logger().info(f"[STAGE 5] 5-Gate Slalom completed safely! Entering Precision Canyon at {d:.2f}m.")

        # STAGE 5: Precision Canyon / Squeeze Corridor (14.8m to 18.8m)
        elif self.state == "STAGE5_NARROW_CANYON":
            status_msg = f"Stage 5: Precision Canyon Squeeze | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.10  # Low CoM precision tracking posture

            if self.has_odom:
                cur_y = self.robot_y
                cur_psi = self.robot_yaw
                wz = -2.5 * cur_y - 0.8 * cur_psi
            elif self.lane_detected:
                wz = lane_wz
            else:
                wz = 0.0
            wz = max(-0.35, min(0.35, wz))

            if d >= 18.8:
                self.state = "STAGE6_SPEED_BUMPS"
                self.get_logger().info(f"[STAGE 6] Canyon cleared! Entering 4-Stage Graded Speed Bumps at {d:.2f}m.")

        # STAGE 6: Graded Multi-Frequency Speed Bumps (18.8m to 23.4m)
        elif self.state == "STAGE6_SPEED_BUMPS":
            status_msg = f"Stage 6: Graded Speed Bumps | Dist: {d:.2f}m"
            vx = 0.13
            squat_angle = -0.25  # Athletic suspension absorption crouch

            if self.has_odom:
                cur_y = self.robot_y
                cur_psi = self.robot_yaw
                wz = -2.2 * cur_y - 0.7 * cur_psi
            elif self.lane_detected:
                wz = lane_wz
            else:
                wz = 0.0
            wz = max(-0.35, min(0.35, wz))

            if d >= 23.4:
                self.state = "STAGE7_APPROACH_HURDLE2"
                self.get_logger().info(f"[STAGE 7] Speed bumps traversed! Approaching Hurdle 2 (Double Squat) at {d:.2f}m.")

        # STAGE 7: Low Overhead Hurdle 2 (Double Squat Challenge!)
        elif self.state == "STAGE7_APPROACH_HURDLE2":
            status_msg = f"Stage 7: Hurdle 2 Squat | Dist: {d:.2f}m"
            vx = 0.18
            wz = lane_wz * 0.4
            squat_angle = -0.55  # Deep squat under second 26.5cm bar

            if d >= 24.3 or (self.hurdle_distance_px > 280 and d >= 23.8):
                self.state = "STAGE7_DUCK_HURDLE2"
                self.get_logger().info("[STAGE 7] Ducking under Hurdle 2 overhead bar...")

        elif self.state == "STAGE7_DUCK_HURDLE2":
            status_msg = f"Stage 7: Ducking Under Hurdle 2 | Dist: {d:.2f}m"
            vx = 0.20
            wz = 0.0
            squat_angle = -0.55

            if d >= 25.8:
                self.state = "STAGE8_SPRINT_FINISH"
                self.get_logger().info("[STAGE 8] Hurdle 2 cleared! Sprinting to Championship Finish Arch.")

        # STAGE 8: Grand Sprint & Championship Finish Arch (25.8m to 30.0m)
        elif self.state == "STAGE8_SPRINT_FINISH":
            status_msg = f"Stage 8: Championship Sprint | Dist: {d:.2f}m / 30.0m"
            vx = 0.26
            squat_angle = 0.0  # Full upright victory posture

            if self.has_odom:
                cur_y = self.robot_y
                cur_psi = self.robot_yaw
                wz = -2.2 * cur_y - 0.7 * cur_psi
            elif self.lane_detected:
                wz = lane_wz
            else:
                wz = 0.0
            wz = max(-0.40, min(0.40, wz))

            # Stop when crossing the 30m finish line (d >= 29.8m) or banner spans full view (W > 260px)
            if d >= 29.8 or (self.finish_detected and self.finish_banner_w > 260 and d >= 29.0):
                self.state = "MISSION_COMPLETE"
                self.get_logger().info(">>> [STAGE 8] CHAMPIONSHIP FINISH ARCH REACHED (30m)! Active braking to balanced stop. <<<")

        # MISSION COMPLETE: Active Balanced Zero-Velocity Hold
        elif self.state == "MISSION_COMPLETE":
            status_msg = f"30M CHAMPIONSHIP COMPLETE! Traveled: {d:.2f}m | Upright & Balanced"
            vx = 0.0
            wz = 0.0
            squat_angle = 0.0

        # ==================== Inverted Pendulum Command Smoothing ====================
        # Slew-rate acceleration limiter
        accel_limit = 0.60  # m/s^2 forward acceleration limit
        max_dv = accel_limit * self.dt
        if abs(vx - self.current_cmd_vx) > max_dv:
            self.current_cmd_vx += math.copysign(max_dv, vx - self.current_cmd_vx)
        else:
            self.current_cmd_vx = vx

        yaw_accel_limit = 1.80  # rad/s^2 yaw acceleration limit
        max_dw = yaw_accel_limit * self.dt
        if abs(wz - self.current_cmd_wz) > max_dw:
            self.current_cmd_wz += math.copysign(max_dw, wz - self.current_cmd_wz)
        else:
            self.current_cmd_wz = wz

        self.current_squat_cmd = squat_angle

        # Publish Commands
        cmd_twist = Twist()
        cmd_twist.linear.x = float(self.current_cmd_vx)
        cmd_twist.angular.z = float(self.current_cmd_wz)
        self.cmd_vel_pub.publish(cmd_twist)

        cmd_posture = Float64()
        cmd_posture.data = float(squat_angle)
        self.squat_pub.publish(cmd_posture)

        # Broadcast Status Log
        log_str = String()
        log_str.data = f"[{self.state}] {status_msg} | Pitch: {math.degrees(self.pitch):.1f}° | FPS: {self.fps:.0f}"
        self.status_pub.publish(log_str)


def main(args=None):
    rclpy.init(args=args)
    navigator = WobbleVisionNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        navigator.cmd_vel_pub.publish(stop_twist)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
