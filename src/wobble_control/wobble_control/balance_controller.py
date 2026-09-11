#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float64, Float64MultiArray, String

try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    HAS_GZ_INTERFACES = True
except ImportError:
    HAS_GZ_INTERFACES = False


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
        self.declare_parameter('pitch_kp', 3.6)
        self.declare_parameter('pitch_kd', 0.14)
        self.declare_parameter('pitch_ki', 0.40)
        self.declare_parameter('pitch_integral_max', 0.15)
        self.declare_parameter('pitch_offset', 0.000)

        self.declare_parameter('pos_kp', 0.015)
        self.declare_parameter('vel_kd', 0.020)
        self.declare_parameter('vel_kp', 0.080)
        self.declare_parameter('vel_ki', 0.005)
        self.declare_parameter('max_target_pitch', 0.040)

        self.declare_parameter('yaw_kp', 0.25)
        self.declare_parameter('yaw_kd', 0.01)

        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('max_wheel_torque', 0.60)
        self.declare_parameter('fall_angle_threshold', 0.80)

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

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.measured_linear_vel = 0.0
        self.measured_yaw_rate = 0.0
        self.robot_position = 0.0
        self.target_position = 0.0
        self.position_initialized = False

        # Command inputs
        self.cmd_linear_vel = 0.0
        self.cmd_yaw_vel = 0.0
        self.target_squat_angle = self.get_parameter('default_squat_angle').value
        self.last_cmd_vel_time = self.get_clock().now()

        # Integrator states
        self.pitch_integral = 0.0
        self.vel_integral = 0.0
        self.filtered_linear_vel = 0.0
        self.theta_trim = 0.0
        self.initial_reset_done = False
        self.current_target_pitch = float(self.get_parameter('pitch_offset').value)

        # Gazebo service client for upright reset
        if HAS_GZ_INTERFACES:
            self.set_pose_cli = self.create_client(SetEntityPose, '/world/wobble_hurdle_course/set_pose')
        else:
            self.set_pose_cli = None

        # Safety & status
        self.is_fallen = False
        self.startup_grace = 2.0  # 2.0s grace period for initial upright settling
        self.current_squat_angle = float(self.target_squat_angle)

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
        self.reset_sub = self.create_subscription(
            Empty, '/wobble/reset', self.reset_callback, 10)

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

        # Pitch rate (gyro Y) with low-pass filter & Yaw rate (gyro Z)
        raw_rate = msg.angular_velocity.y
        self.pitch_rate = 0.8 * raw_rate + 0.2 * self.pitch_rate
        raw_yaw = msg.angular_velocity.z
        self.yaw_rate = 0.8 * raw_yaw + 0.2 * self.yaw_rate

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'joint_left_wheel':
                if len(msg.velocity) > i:
                    self.left_wheel_vel = msg.velocity[i]
                if len(msg.position) > i:
                    self.left_wheel_pos = msg.position[i]
            elif name == 'joint_right_wheel':
                if len(msg.velocity) > i:
                    self.right_wheel_vel = msg.velocity[i]
                if len(msg.position) > i:
                    self.right_wheel_pos = msg.position[i]

        wheel_r = self.get_parameter('wheel_radius').value
        wheel_sep = self.get_parameter('wheel_separation').value

        # Forward linear velocity, position, and yaw rate from encoders
        self.measured_linear_vel = wheel_r * (self.left_wheel_vel + self.right_wheel_vel) / 2.0
        self.measured_yaw_rate = wheel_r * (self.right_wheel_vel - self.left_wheel_vel) / wheel_sep
        self.robot_position = wheel_r * (self.left_wheel_pos + self.right_wheel_pos) / 2.0
        if not self.position_initialized:
            self.target_position = self.robot_position
            self.position_initialized = True

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_vel = msg.linear.x
        self.cmd_yaw_vel = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def squat_callback(self, msg: Float64):
        # Desired squat angle in radians
        min_limit = self.get_parameter('servo_limit_min').value
        max_limit = self.get_parameter('servo_limit_max').value
        new_angle = max(min_limit, min(max_limit, msg.data))
        if abs(new_angle - self.target_squat_angle) > 0.02:
            self.get_logger().info(f"Squat posture transition: {new_angle:.3f} rad")
        self.target_squat_angle = new_angle

    def control_loop(self):
        # Ensure robot is initialized upright in simulation as soon as service is ready
        if not self.initial_reset_done:
            if self.set_pose_cli and self.set_pose_cli.service_is_ready():
                self.reset_sim_pose()
                self.initial_reset_done = True
                self.startup_grace = 2.0
                self.is_fallen = False
                self.get_logger().info("Reset model pose upright at controller startup!")

        # Command timeout watchdog: reset targets if no cmd_vel received in 0.5s
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.cmd_linear_vel = 0.0
            self.cmd_yaw_vel = 0.0

        # Initial settling: hold target_position at current position until settled
        if self.startup_grace > 0.0:
            self.startup_grace -= self.dt
            self.target_position = self.robot_position

        # Fall detection & motor cut-off: if pitch exceeds threshold, cut motors to prevent wheel spin
        fall_thresh = self.get_parameter('fall_angle_threshold').value
        if abs(self.pitch) > fall_thresh:
            if not self.is_fallen:
                self.get_logger().warn(f"Fall detected! Pitch: {math.degrees(self.pitch):.1f} deg. Disabling wheel motors.")
                self.is_fallen = True
            self.pitch_integral = 0.0
            self.publish_wheel_efforts(0.0, 0.0)
            self.publish_posture()
            self.publish_telemetry(0.0, 0.0)
            return

        if self.is_fallen and abs(self.pitch) < 0.20:
            self.get_logger().info("Robot upright recovered. Re-enabling balance controller.")
            self.is_fallen = False
            self.pitch_integral = 0.0
            self.theta_trim = 0.0
            self.target_position = self.robot_position

        # Posture smoothing (slew-rate limit 0.8 rad/s) to prevent abrupt momentum transfer
        max_rate = 0.8 * self.dt
        diff = self.target_squat_angle - self.current_squat_angle
        if abs(diff) > max_rate:
            self.current_squat_angle += math.copysign(max_rate, diff)
        else:
            self.current_squat_angle = self.target_squat_angle

        # Kinematic CoM compensation for 4-bar parallelogram linkage squatting:
        # As linkage flexes, wheel axle translates forward: dx = -crank * sin(squat)
        crank_len = 0.08
        coupler_len = 0.085
        dx_com = -crank_len * math.sin(self.current_squat_angle)
        dz_com = crank_len * math.cos(self.current_squat_angle) + coupler_len + 0.02
        kinematic_pitch_bias = math.atan2(dx_com, dz_com)

        # Low-pass filter linear velocity to prevent fast wheel slip from shaking the pitch target
        alpha_v = 0.10
        self.filtered_linear_vel = alpha_v * self.measured_linear_vel + (1.0 - alpha_v) * self.filtered_linear_vel

        # ========== 1. Cascaded Tilt-Demand Outer Loop ==========
        pitch_offset = self.get_parameter('pitch_offset').value + kinematic_pitch_bias
        max_lean = self.get_parameter('max_target_pitch').value
        vel_kp = self.get_parameter('vel_kp').value
        pos_kp = self.get_parameter('pos_kp').value
        vel_kd = self.get_parameter('vel_kd').value
        vel_ki = self.get_parameter('vel_ki').value

        if abs(self.cmd_linear_vel) < 0.01:
            # Station-keeping mode: hold target position and damp velocity
            pos_err = max(-0.30, min(0.30, self.robot_position - self.target_position))
            v_err = self.filtered_linear_vel

            desired_target_pitch = pitch_offset - (pos_kp * pos_err) - (vel_kd * v_err)
        else:
            # Driving mode: advance position setpoint to current position, track commanded velocity
            self.target_position = self.robot_position
            v_err = self.filtered_linear_vel - self.cmd_linear_vel
            desired_target_pitch = pitch_offset + (vel_kp * self.cmd_linear_vel) - (vel_kd * v_err)

        desired_target_pitch = max(pitch_offset - max_lean, min(pitch_offset + max_lean, desired_target_pitch))

        # Slew-rate limit target pitch changes (0.5 rad/s)
        max_d_target = 0.5 * self.dt
        diff_target = desired_target_pitch - self.current_target_pitch
        if abs(diff_target) > max_d_target:
            self.current_target_pitch += math.copysign(max_d_target, diff_target)
        else:
            self.current_target_pitch = desired_target_pitch

        target_pitch = self.current_target_pitch

        # ========== 2. Inner Loop: Pitch Balancing ==========
        pitch_err = self.pitch - target_pitch
        self.pitch_integral += pitch_err * self.dt
        pitch_int_max = self.get_parameter('pitch_integral_max').value
        self.pitch_integral = max(-pitch_int_max, min(pitch_int_max, self.pitch_integral))

        pitch_kp = self.get_parameter('pitch_kp').value
        pitch_kd = self.get_parameter('pitch_kd').value
        pitch_ki = self.get_parameter('pitch_ki').value

        # Balance torque: pure pitch stabilization to track target_pitch
        tau_balance = (pitch_kp * pitch_err) + (pitch_kd * self.pitch_rate) + (pitch_ki * self.pitch_integral)

        # ========== 3. Yaw / Steering Control ==========
        yaw_err = self.cmd_yaw_vel - self.yaw_rate
        if abs(yaw_err) < 0.05 and abs(self.cmd_yaw_vel) < 0.01:
            yaw_err = 0.0
        yaw_kp = self.get_parameter('yaw_kp').value
        delta_tau = yaw_kp * yaw_err

        # ========== 4. Mixer & Saturation ==========
        max_torque = self.get_parameter('max_wheel_torque').value
        tau_left = max(-max_torque, min(max_torque, tau_balance - delta_tau))
        tau_right = max(-max_torque, min(max_torque, tau_balance + delta_tau))

        # Send actuator commands
        self.publish_wheel_efforts(tau_left, tau_right)
        self.publish_posture()
        self.publish_telemetry(tau_left, tau_right)

    def reset_sim_pose(self):
        if self.set_pose_cli and self.set_pose_cli.service_is_ready():
            req = SetEntityPose.Request()
            req.entity.name = 'wobble'
            req.entity.type = Entity.MODEL
            req.pose.position.x = 0.0
            req.pose.position.y = 0.0
            req.pose.position.z = 0.041
            req.pose.orientation.w = 0.999925
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.01225
            req.pose.orientation.z = 0.0
            self.set_pose_cli.call_async(req)
            self.target_position = self.robot_position
            self.filtered_linear_vel = 0.0
            self.pitch_integral = 0.0
            self.theta_trim = 0.0
            self.current_target_pitch = float(self.get_parameter('pitch_offset').value)
            self.get_logger().info("Reset model pose to upright via ROS-Gazebo bridge service.")

    def reset_callback(self, msg: Empty):
        self.get_logger().info("Reset command received. Re-enabling balance controller.")
        self.is_fallen = False
        self.startup_grace = 2.0
        self.pitch_integral = 0.0
        self.theta_trim = 0.0
        self.cmd_linear_vel = 0.0
        self.cmd_yaw_vel = 0.0
        self.filtered_linear_vel = 0.0
        self.current_target_pitch = float(self.get_parameter('pitch_offset').value)
        self.target_squat_angle = 0.0
        self.current_squat_angle = 0.0
        self.position_initialized = False
        self.target_position = self.robot_position
        self.reset_sim_pose()

    def publish_telemetry(self, tau_left: float = 0.0, tau_right: float = 0.0):
        # Telemetry: [pitch, pitch_rate, linear_vel, cmd_vel, tau_L, tau_R, squat_angle, status]
        telemetry = Float64MultiArray()
        telemetry.data = [
            float(self.pitch),
            float(self.pitch_rate),
            float(self.measured_linear_vel),
            float(self.cmd_linear_vel),
            float(tau_left),
            float(tau_right),
            float(self.current_squat_angle),
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
        # 4-bar parallelogram linkage: knee angle = -hip angle
        msg = Float64MultiArray()
        msg.data = [
            self.current_squat_angle,
            -self.current_squat_angle,
            self.current_squat_angle,
            -self.current_squat_angle
        ]
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
