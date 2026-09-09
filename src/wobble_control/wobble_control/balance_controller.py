#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, String


class WobbleBalanceController(Node):
    """
    Dynamically balancing wheeled biped controller for Project Wobble.
    Implements a cascaded PID control loop:
      - Outer Loop: Linear velocity / odometry tracking -> target pitch lean
      - Inner Loop: MPU6050 pitch angle and pitch rate -> balancing wheel torque
      - Yaw Loop: Differential steering torque
      - Posture Loop: MG995 4-bar linkage height & squatting control
    """

    def __init__(self):
        super().__init__('wobble_balance_controller')

        # Declare parameters with default tuning
        self.declare_parameter('loop_rate_hz', 200.0)
        self.declare_parameter('pitch_kp', 18.0)
        self.declare_parameter('pitch_kd', 1.2)
        self.declare_parameter('pitch_ki', 0.8)
        self.declare_parameter('pitch_integral_max', 0.5)
        self.declare_parameter('pitch_offset', 0.0)

        self.declare_parameter('vel_kp', 0.35)
        self.declare_parameter('vel_ki', 0.08)
        self.declare_parameter('vel_integral_max', 0.15)
        self.declare_parameter('max_target_pitch', 0.25)

        self.declare_parameter('yaw_kp', 0.4)
        self.declare_parameter('yaw_kd', 0.02)

        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('max_wheel_torque', 1.5)
        self.declare_parameter('fall_angle_threshold', 0.8)

        self.declare_parameter('servo_limit_min', -1.570796)
        self.declare_parameter('servo_limit_max', 1.570796)
        self.declare_parameter('default_squat_angle', 0.0)

        # Cache parameters
        self.rate_hz = self.get_parameter('loop_rate_hz').value
        self.dt = 1.0 / self.rate_hz

        # State variables
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.measured_linear_vel = 0.0
        self.measured_yaw_rate = 0.0

        # Command inputs
        self.cmd_linear_vel = 0.0
        self.cmd_yaw_vel = 0.0
        self.target_squat_angle = self.get_parameter('default_squat_angle').value
        self.last_cmd_vel_time = self.get_clock().now()

        # Integrator states
        self.pitch_integral = 0.0
        self.vel_integral = 0.0

        # Safety & status
        self.is_fallen = False

        # Publishers
        self.left_wheel_pub = self.create_publisher(
            Float64MultiArray, '/left_wheel_effort_controller/commands', 10)
        self.right_wheel_pub = self.create_publisher(
            Float64MultiArray, '/right_wheel_effort_controller/commands', 10)
        self.hip_servo_pub = self.create_publisher(
            Float64MultiArray, '/hip_position_controller/commands', 10)
        self.telemetry_pub = self.create_publisher(
            Float64MultiArray, '/wobble/telemetry', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.squat_sub = self.create_subscription(
            Float64, '/cmd_squat', self.squat_callback, 10)

        # High-frequency control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Project Wobble Cascaded PID Balance Controller Initialized.")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        # Compute pitch angle from quaternion:
        # pitch (Y-axis tilt): sin(pitch) = 2 * (w*y - z*x)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.pitch = math.asin(sinp)

        # Pitch rate (gyro Y) & Yaw rate (gyro Z)
        self.pitch_rate = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'joint_left_wheel' and len(msg.velocity) > i:
                self.left_wheel_vel = msg.velocity[i]
            elif name == 'joint_right_wheel' and len(msg.velocity) > i:
                self.right_wheel_vel = msg.velocity[i]

        wheel_r = self.get_parameter('wheel_radius').value
        wheel_sep = self.get_parameter('wheel_separation').value

        # Forward linear velocity and yaw rate from encoders
        self.measured_linear_vel = wheel_r * (self.left_wheel_vel + self.right_wheel_vel) / 2.0
        self.measured_yaw_rate = wheel_r * (self.right_wheel_vel - self.left_wheel_vel) / wheel_sep

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_vel = msg.linear.x
        self.cmd_yaw_vel = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def squat_callback(self, msg: Float64):
        # Desired squat angle in radians
        min_limit = self.get_parameter('servo_limit_min').value
        max_limit = self.get_parameter('servo_limit_max').value
        self.target_squat_angle = max(min_limit, min(max_limit, msg.data))
        self.get_logger().info(f"Squat posture command received: {self.target_squat_angle:.3f} rad")

    def control_loop(self):
        # Command timeout watchdog: reset targets if no cmd_vel received in 0.5s
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.cmd_linear_vel = 0.0
            self.cmd_yaw_vel = 0.0

        fall_thresh = self.get_parameter('fall_angle_threshold').value
        if abs(self.pitch) > fall_thresh:
            if not self.is_fallen:
                self.get_logger().warn(f"Fall detected! Pitch: {math.degrees(self.pitch):.1f} deg. Disabling motors.")
                self.is_fallen = True
            # Zero motor output for safety
            self.publish_wheel_efforts(0.0, 0.0)
            self.publish_posture()
            return

        if self.is_fallen and abs(self.pitch) < (fall_thresh * 0.4):
            self.get_logger().info("Robot upright recovered. Re-enabling balance controller.")
            self.is_fallen = False
            self.pitch_integral = 0.0
            self.vel_integral = 0.0

        # ========== 1. Outer Loop: Linear Velocity Control ==========
        vel_err = self.cmd_linear_vel - self.measured_linear_vel
        self.vel_integral += vel_err * self.dt
        vel_int_max = self.get_parameter('vel_integral_max').value
        self.vel_integral = max(-vel_int_max, min(vel_int_max, self.vel_integral))

        vel_kp = self.get_parameter('vel_kp').value
        vel_ki = self.get_parameter('vel_ki').value
        pitch_offset = self.get_parameter('pitch_offset').value
        max_lean = self.get_parameter('max_target_pitch').value

        # Desired pitch lean to accelerate or maintain speed
        target_pitch = pitch_offset - (vel_kp * vel_err + vel_ki * self.vel_integral)
        target_pitch = max(-max_lean, min(max_lean, target_pitch))

        # ========== 2. Inner Loop: Pitch Balancing ==========
        pitch_err = self.pitch - target_pitch
        self.pitch_integral += pitch_err * self.dt
        pitch_int_max = self.get_parameter('pitch_integral_max').value
        self.pitch_integral = max(-pitch_int_max, min(pitch_int_max, self.pitch_integral))

        pitch_kp = self.get_parameter('pitch_kp').value
        pitch_kd = self.get_parameter('pitch_kd').value
        pitch_ki = self.get_parameter('pitch_ki').value

        tau_balance = (pitch_kp * pitch_err) + (pitch_kd * self.pitch_rate) + (pitch_ki * self.pitch_integral)

        # ========== 3. Yaw / Steering Control ==========
        yaw_err = self.cmd_yaw_vel - self.yaw_rate
        yaw_kp = self.get_parameter('yaw_kp').value
        delta_tau = yaw_kp * yaw_err

        # ========== 4. Mixer & Saturation ==========
        max_torque = self.get_parameter('max_wheel_torque').value
        tau_left = max(-max_torque, min(max_torque, tau_balance - delta_tau))
        tau_right = max(-max_torque, min(max_torque, tau_balance + delta_tau))

        # Send actuator commands
        self.publish_wheel_efforts(tau_left, tau_right)
        self.publish_posture()

        # Telemetry: [pitch, pitch_rate, linear_vel, cmd_vel, tau_L, tau_R, squat_angle, status]
        telemetry = Float64MultiArray()
        telemetry.data = [
            float(self.pitch),
            float(self.pitch_rate),
            float(self.measured_linear_vel),
            float(self.cmd_linear_vel),
            float(tau_left),
            float(tau_right),
            float(self.target_squat_angle),
            0.0 if self.is_fallen else 1.0
        ]
        self.telemetry_pub.publish(telemetry)

    def publish_wheel_efforts(self, tau_l: float, tau_r: float):
        msg_l = Float64MultiArray()
        msg_l.data = [tau_l]
        self.left_wheel_pub.publish(msg_l)

        msg_r = Float64MultiArray()
        msg_r.data = [tau_r]
        self.right_wheel_pub.publish(msg_r)

    def publish_posture(self):
        # Publish symmetric target angles to both MG995 servos
        msg = Float64MultiArray()
        msg.data = [self.target_squat_angle, self.target_squat_angle]
        self.hip_servo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = WobbleBalanceController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
