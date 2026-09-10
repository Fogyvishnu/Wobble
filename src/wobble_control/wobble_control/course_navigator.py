#!/usr/bin/env python3
"""
Autonomous Hurdle Course Navigator for Project Wobble.
Controls the robot through:
  1. Straight Runway Acceleration
  2. Low Clearance Hurdle (Squat under overhead crossbar)
  3. Slalom Bollards (Dynamic balanced turning)
  4. Speed Bumps (Suspension compliance traversal)
  5. Finish Line Arrival & Balanced Stop
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, String


class WobbleCourseNavigator(Node):

    def __init__(self):
        super().__init__('wobble_course_navigator')

        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('rate_hz', 20.0)
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.dt = 1.0 / self.get_parameter('rate_hz').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.squat_pub = self.create_publisher(Float64, '/cmd_squat', 10)
        self.status_pub = self.create_publisher(String, '/wobble/course_status', 10)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.telemetry_sub = self.create_subscription(
            Float64MultiArray, '/wobble/telemetry', self.telemetry_callback, 10)

        # Odometry tracking state
        self.left_pos = None
        self.right_pos = None
        self.distance_traveled = 0.0
        self.is_balanced = False
        self.pitch = 0.0

        # Command smoothing to eliminate inverted-pendulum step shock
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0

        # State machine
        self.state = "INIT_WAIT"
        self.state_timer = 0.0

        self.timer = self.create_timer(self.dt, self.navigation_step)
        self.get_logger().info("Wobble Hurdle Course Navigator Initialized. Waiting for balance...")

    def telemetry_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.pitch = msg.data[0]
            self.is_balanced = (msg.data[7] > 0.5) and (abs(self.pitch) < 0.15)

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
            # Filter out startup jumps or excessive anomalies
            if abs(d_fwd) < 0.2:
                self.distance_traveled += d_fwd

    def navigation_step(self):
        d = self.distance_traveled
        self.state_timer += self.dt

        vx = 0.0
        wz = 0.0
        squat_angle = 0.0
        status_msg = ""

        # ---------------- State Machine ----------------
        if self.state == "INIT_WAIT":
            status_msg = "Checking upright balance..."
            if self.is_balanced and self.state_timer > 1.5:
                self.state = "RUNWAY_ACCEL"
                self.state_timer = 0.0
                self.get_logger().info("Balance confirmed. Starting Runway Acceleration!")

        elif self.state == "RUNWAY_ACCEL":
            status_msg = f"Runway Sprint | Dist: {d:.2f}m / 11.0m"
            vx = 0.24
            wz = 0.0
            squat_angle = 0.0  # Upright standing
            if d >= 1.7:
                self.state = "APPROACH_HURDLE"
                self.state_timer = 0.0
                self.get_logger().info(f"Approaching low hurdle at {d:.2f}m! Initiating squat posture.")

        elif self.state == "APPROACH_HURDLE":
            status_msg = f"Squatting for Low Hurdle | Dist: {d:.2f}m"
            vx = 0.20
            wz = 0.0
            squat_angle = -0.42  # Lower 4-bar linkage CoM to duck under 25.5cm gate
            if d >= 2.3:
                self.state = "DUCK_UNDER_HURDLE"
                self.get_logger().info(f"Ducking under low hurdle at {d:.2f}m...")

        elif self.state == "DUCK_UNDER_HURDLE":
            status_msg = f"Ducking Under Hurdle Bar (25.5cm) | Dist: {d:.2f}m"
            vx = 0.20
            wz = 0.0
            squat_angle = -0.42  # Stable athletic squat holds under crossbar
            if d >= 3.4:
                self.state = "STAND_UP"
                self.get_logger().info(f"Cleared hurdle gate at {d:.2f}m! Standing back up.")

        elif self.state == "STAND_UP":
            status_msg = f"Standing Upright | Dist: {d:.2f}m"
            vx = 0.18
            wz = 0.0
            squat_angle = 0.0  # Rise to normal upright posture
            if d >= 4.3:
                self.state = "SLALOM_RIGHT"
                self.get_logger().info(f"Entering Slalom Course! Swerving right around Bollard 1.")

        elif self.state == "SLALOM_RIGHT":
            status_msg = f"Slalom: Passing Right of Bollard 1 | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.15  # Slight athletic crouch for cornering stability
            if d < 4.8:
                wz = -0.30  # Turn right
            elif d < 5.4:
                wz = 0.30   # Counter-steer back to center
            else:
                self.state = "SLALOM_LEFT"
                self.get_logger().info(f"Swerving left around Bollard 2 at {d:.2f}m.")

        elif self.state == "SLALOM_LEFT":
            status_msg = f"Slalom: Passing Left of Bollard 2 | Dist: {d:.2f}m"
            vx = 0.20
            squat_angle = -0.15
            if d < 6.0:
                wz = 0.30   # Turn left
            elif d < 6.6:
                wz = -0.30  # Counter-steer back to center
            else:
                self.state = "SLALOM_STRAIGHT"
                self.get_logger().info(f"Straightening out through Bollard 3 at {d:.2f}m.")

        elif self.state == "SLALOM_STRAIGHT":
            status_msg = f"Exiting Slalom Course | Dist: {d:.2f}m"
            vx = 0.22
            wz = 0.0
            squat_angle = 0.0
            if d >= 7.8:
                self.state = "APPROACH_BUMPS"
                self.get_logger().info(f"Approaching terrain speed bumps at {d:.2f}m.")

        elif self.state == "APPROACH_BUMPS":
            status_msg = f"Traversing Speed Bumps | Dist: {d:.2f}m"
            vx = 0.18  # Slower steady speed for bump absorption
            wz = 0.0
            squat_angle = -0.15  # Athletic squat lets passive suspension absorb bump
            if d >= 9.8:
                self.state = "SPRINT_FINISH"
                self.get_logger().info(f"Bumps cleared! Sprinting to finish arch at {d:.2f}m.")

        elif self.state == "SPRINT_FINISH":
            status_msg = f"Final Sprint to Finish Gate | Dist: {d:.2f}m / 11.0m"
            vx = 0.25
            wz = 0.0
            squat_angle = 0.0
            if d >= 11.0:
                self.state = "MISSION_COMPLETE"
                self.get_logger().info(">>> FINISH LINE CROSSED! Braking to balanced stop. <<<")

        elif self.state == "MISSION_COMPLETE":
            status_msg = f"HURDLE COURSE COMPLETED! Final Distance: {d:.2f}m"
            vx = 0.0
            wz = 0.0
            squat_angle = 0.0

        # Smooth velocity ramp to eliminate step disturbances
        accel_limit = 0.5  # m/s^2 forward acceleration limit
        max_dv = accel_limit * self.dt
        if abs(vx - self.current_cmd_vx) > max_dv:
            self.current_cmd_vx += math.copysign(max_dv, vx - self.current_cmd_vx)
        else:
            self.current_cmd_vx = vx

        yaw_accel_limit = 1.5  # rad/s^2
        max_dw = yaw_accel_limit * self.dt
        if abs(wz - self.current_cmd_wz) > max_dw:
            self.current_cmd_wz += math.copysign(max_dw, wz - self.current_cmd_wz)
        else:
            self.current_cmd_wz = wz

        # Send commands
        cmd_twist = Twist()
        cmd_twist.linear.x = self.current_cmd_vx
        cmd_twist.angular.z = self.current_cmd_wz
        self.cmd_vel_pub.publish(cmd_twist)

        cmd_posture = Float64()
        cmd_posture.data = squat_angle
        self.squat_pub.publish(cmd_posture)

        # Broadcast live telemetry status
        log_str = String()
        log_str.data = f"[{self.state}] {status_msg} | Pitch: {math.degrees(self.pitch):.1f}°"
        self.status_pub.publish(log_str)


def main(args=None):
    rclpy.init(args=args)
    navigator = WobbleCourseNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Safe stop
        stop_twist = Twist()
        navigator.cmd_vel_pub.publish(stop_twist)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
