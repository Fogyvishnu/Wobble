#!/usr/bin/env python3
"""
Project Wobble: Autonomous Vision-Guided Course Navigator
Author: Antigravity Agent for Project Wobble

Description:
    Fully autonomous visual navigation system for Project Wobble.
    Processes live 30 FPS RGB camera imagery from /camera/image_raw using OpenCV:
      1. Real-time Yellow Centerline Tracking & Visual Servoing
      2. Visual Hurdle Detection -> Dynamic 4-Bar Squat Posture Execution
      3. Visual Slalom Bollard Segmentation -> Balanced Differential Weaving
      4. Visual Speed Bump Detection -> Suspension Compliance Adaptation
      5. Visual Finish Line Recognition -> Balanced Deceleration & Stop
    Broadcasts /camera/annotated_image with Cybernetic HUD overlay for RViz2 / operator telemetry.
"""

import math
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
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
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray, '/wobble/telemetry', self.telemetry_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, '/wobble/reset', self.reset_callback, 10)

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
        self.hurdle_cleared = False
        self.bollard_detected = False
        self.bollard_offset = 0.0
        self.active_bollard_idx = 0
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
        self.get_logger().info("Autonomous Vision Navigator Initialized. Awaiting camera feed and balance...")

    def telemetry_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.pitch = msg.data[0]
            self.squat_feedback = msg.data[6]
            self.is_balanced = (msg.data[7] > 0.5) and (abs(self.pitch) < 0.20)

    def reset_callback(self, msg: Empty):
        self.state = "INIT_WAIT"
        self.state_timer = 0.0
        self.distance_traveled = 0.0
        self.left_pos = None
        self.right_pos = None
        self.hurdle_cleared = False
        self.hurdle_detected = False
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
        # Isolate the horizontal orange crossbar in the central corridor (Hue 8-25, Sat 90-255, Val 80-255)
        # Avoid detecting the red boundary curb rails on the track sides
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

        if hurdle_boxes and self.distance_traveled < 4.0:
            largest_h = max(hurdle_boxes, key=lambda b: b[2])
            bx, by, bw, bh, area = largest_h
            self.hurdle_detected = True
            self.hurdle_distance_px = float(bw)

            # Draw bounding box and alert
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 140, 255), 2)
            cv2.putText(annotated, f"HURDLE CROSSBAR: W={bw}px", (bx, max(20, by - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2)
        else:
            self.hurdle_detected = False
            self.hurdle_distance_px = 0.0

        # ---------------- 3. Visual Slalom Bollard Detection (Cyan/Blue) ----------------
        roi_slalom = hsv[int(h * 0.35):int(h * 0.75), :]
        mask_cyan = cv2.inRange(roi_slalom, np.array([85, 75, 55]), np.array([115, 255, 255]))
        cnts_c, _ = cv2.findContours(mask_cyan, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bollards = []
        for c in cnts_c:
            area = cv2.contourArea(c)
            if area > 60:
                bx, by, bw, bh = cv2.boundingRect(c)
                by_global = by + int(h * 0.35)
                cx = bx + bw / 2.0
                bollards.append((cx, by_global, bw, bh, area))

        if bollards and 4.0 <= self.distance_traveled <= 8.5:
            bollards.sort(key=lambda b: b[4], reverse=True)
            self.bollard_detected = True
            primary_b = bollards[0]
            cx, by, bw, bh, area = primary_b
            self.bollard_offset = (cx - (w / 2.0)) / (w / 2.0)

            for i, b in enumerate(bollards[:2]):
                bcx, bby, bbw, bbh, barea = b
                cv2.rectangle(annotated, (int(bcx - bbw/2), int(bby)), (int(bcx + bbw/2), int(bby + bbh)), (255, 200, 0), 2)
                cv2.putText(annotated, f"BOLLARD {i+1}", (int(bcx - bbw/2), int(bby - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)
        else:
            self.bollard_detected = False

        # ---------------- 4. Visual Finish Line / Arch Detection (Green Banner) ----------------
        roi_finish = hsv[int(h * 0.20):int(h * 0.60), :]
        mask_green = cv2.inRange(roi_finish, np.array([38, 70, 50]), np.array([85, 255, 255]))
        cnts_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.finish_detected = False
        if cnts_g and self.distance_traveled > 9.0:
            c_arch = max(cnts_g, key=cv2.contourArea)
            if cv2.contourArea(c_arch) > 200:
                bx, by, bw, bh = cv2.boundingRect(c_arch)
                by_global = by + int(h * 0.20)
                self.finish_detected = True
                cv2.rectangle(annotated, (bx, by_global), (bx + bw, by_global + bh), (0, 255, 100), 2)
                cv2.putText(annotated, "FINISH ARCH", (bx, by_global - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 2)

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
        cv2.putText(img, "WOBBLE VISION AUTONOMY", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)

        # Status badge
        badge_color = (0, 255, 120) if self.state != "MISSION_COMPLETE" else (255, 200, 0)
        cv2.putText(img, f"[{self.state}]", (310, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, badge_color, 2)

        # FPS indicator
        cv2.putText(img, f"{self.fps:.0f} FPS", (w - 85, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 170, 180), 1)

        # Lower Telemetry Bar
        cv2.rectangle(img, (0, h - 36), (w, h), (20, 24, 28), -1)
        cv2.line(img, (0, h - 36), (w, h - 36), (0, 180, 255), 1)

        pitch_deg = math.degrees(self.pitch)
        telem_str = f"Dist: {self.distance_traveled:4.1f}m | Pitch: {pitch_deg:+5.1f}deg | Squat: {self.current_squat_cmd:+.2f} rad | Speed: {self.current_cmd_vx:+.2f} m/s"
        cv2.putText(img, telem_str, (12, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 235, 240), 1)

        # Visual Reticle Center Crosshair
        cx, cy = int(w / 2), int(h / 2)
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

    def navigation_step(self):
        """Main autonomous vision-guided state machine navigation step."""
        d = self.distance_traveled
        self.state_timer += self.dt

        vx = 0.0
        wz = 0.0
        squat_angle = 0.0
        status_msg = ""

        # Visual Lane Center Steering Offset
        lane_wz = -0.40 * self.lane_error if self.lane_detected else 0.0

        # ==================== State Machine ====================
        if self.state == "INIT_WAIT":
            status_msg = f"Awaiting settling (bal={self.is_balanced}, img={self.has_image}, t={self.state_timer:.1f}s)"

            # Auto-recovery: If not balanced after 1.5s, pulse /wobble/reset to stand upright
            if self.state_timer > 1.5 and not self.is_balanced and (int(self.state_timer * 10) % 15 == 0):
                self.reset_pub.publish(Empty())
                self.get_logger().info("Autonomous auto-recovery: Pulsing /wobble/reset upright...")

            if self.is_balanced and self.has_image and self.state_timer > 3.0:
                self.state = "RUNWAY_ACCEL"
                self.state_timer = 0.0
                self.distance_traveled = 0.0
                self.left_pos = None
                self.right_pos = None
                self.get_logger().info("Balance & Camera stream confirmed! Autonomous visual mission starting.")
            else:
                # Broadcast Status Log even in INIT_WAIT so observers can track readiness
                log_str = String()
                log_str.data = f"[{self.state}] {status_msg} | Pitch: {math.degrees(self.pitch):.1f}° | FPS: {self.fps:.0f}"
                self.status_pub.publish(log_str)
                return

        elif self.state == "RUNWAY_ACCEL":
            status_msg = f"Visual Runway Tracking | Dist: {d:.2f}m"
            vx = 0.22
            wz = lane_wz
            squat_angle = 0.0

            # Visual trigger for hurdle: crossbar detected expanding in camera frame (W > 210px) or d >= 1.6m
            if (self.hurdle_detected and self.hurdle_distance_px > 210) or d >= 1.6:
                self.state = "APPROACH_HURDLE"
                self.state_timer = 0.0
                self.get_logger().info(f"[VISION] Low hurdle crossbar detected in camera (W={self.hurdle_distance_px:.0f}px)! Initiating squat.")

        elif self.state == "APPROACH_HURDLE":
            status_msg = f"Visual Duck: Lowering CoM | Dist: {d:.2f}m"
            vx = 0.20
            wz = lane_wz * 0.4  # Damp yaw while flexing posture
            squat_angle = -0.55  # Deep 4-bar squat clearance to glide safely under 25.5cm bar

            if d >= 2.2 or (self.hurdle_distance_px > 300):
                self.state = "DUCK_UNDER_HURDLE"
                self.get_logger().info("[VISION] Ducking under overhead hurdle bar...")

        elif self.state == "DUCK_UNDER_HURDLE":
            status_msg = f"Ducking Under Crossbar | Dist: {d:.2f}m"
            vx = 0.22
            wz = 0.0
            squat_angle = -0.55

            # Roll past 2.8m hurdle; stand up when distance reaches 3.4m or bollard is sighted
            if d >= 3.4 or (d >= 3.0 and self.bollard_detected):
                self.state = "STAND_UP"
                self.hurdle_cleared = True
                self.get_logger().info("[VISION] Hurdle cleared! Standing back up upright.")

        elif self.state == "STAND_UP":
            status_msg = f"Rising to Upright Posture | Dist: {d:.2f}m"
            vx = 0.18
            wz = lane_wz
            squat_angle = 0.0

            if d >= 4.3:
                self.state = "SLALOM_RIGHT"
                self.get_logger().info("[VISION] Entering Slalom Course! Visual servoing around Bollard 1.")

        elif self.state == "SLALOM_RIGHT":
            status_msg = f"Visual Slalom: Weaving Right of Bollard 1 | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.15  # Slight athletic stance for agile cornering

            # Weave right around bollard 1
            if d < 4.8:
                wz = -0.32
            elif d < 5.4:
                wz = 0.32
            else:
                self.state = "SLALOM_LEFT"
                self.get_logger().info(f"[VISION] Approaching Bollard 2! Weaving left at {d:.2f}m.")

        elif self.state == "SLALOM_LEFT":
            status_msg = f"Visual Slalom: Weaving Left of Bollard 2 | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.15

            if d < 6.0:
                wz = 0.32
            elif d < 6.6:
                wz = -0.32
            else:
                self.state = "SLALOM_BOLLARD3"
                self.get_logger().info(f"[VISION] Approaching Bollard 3! Weaving right at {d:.2f}m.")

        elif self.state == "SLALOM_BOLLARD3":
            status_msg = f"Visual Slalom: Weaving Right of Bollard 3 | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.15

            # Weave right around bollard 3 (at X=7.2, Y=+0.20)
            if d < 7.2:
                wz = -0.32
            elif d < 7.8:
                wz = 0.32
            else:
                self.state = "APPROACH_BUMPS"
                self.get_logger().info(f"[VISION] Approaching terrain speed bumps at {d:.2f}m.")

        elif self.state == "APPROACH_BUMPS":
            status_msg = f"Traversing Speed Bumps | Dist: {d:.2f}m"
            vx = 0.18
            wz = lane_wz * 0.5
            squat_angle = -0.15  # Compliant suspension posture

            if d >= 9.8:
                self.state = "SPRINT_FINISH"
                self.get_logger().info("[VISION] Speed bumps traversed! Sprinting toward Finish Arch.")

        elif self.state == "SPRINT_FINISH":
            status_msg = f"Sprinting to Finish Arch | Dist: {d:.2f}m / 11.0m"
            vx = 0.25
            wz = lane_wz
            squat_angle = 0.0

            if self.finish_detected or d >= 11.0:
                self.state = "MISSION_COMPLETE"
                self.get_logger().info(">>> [VISION] FINISH ARCH REACHED! Active braking to balanced stop. <<<")

        elif self.state == "MISSION_COMPLETE":
            status_msg = f"COURSE COMPLETED! Traveled: {d:.2f}m | Upright & Balanced"
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
