# Project Wobble: Dynamically Balancing Two-Wheeled Biped Robot

Wobble is a dynamically balancing, two-wheeled bipedal robot designed to explore dynamic inverted pendulum stabilization combined with variable-height posture control via a 4-bar linkage.

---

## 1. System Architecture

* **Simulation Platform:** ROS 2 Jazzy & Gazebo Harmonic (via RoboStack / Pixi)
* **Physical Target Platform:** ESP32-S3 microcontroller + FreeRTOS
* **Posture Actuation:** 2x MG995 180° Servos controlling symmetric/differential 4-bar squat linkages ($[-\pi/2, +\pi/2]$ rad)
* **Balancing Actuation:** 2x DC Motors with high-resolution magnetic quadrature encoders
* **Sensing:** MPU6050 6-axis IMU (gyroscope + accelerometer) on torso center

---

## 2. Kinematic & Dynamic Design

### 4-Bar Squat Mechanism
Each leg consists of a planar 4-bar parallelogram linkage:
1. **Base Link:** Chassis hip mount.
2. **Crank Link:** Driven directly by the MG995 servo ($[-1.57, +1.57]$ rad).
3. **Coupler Link (Lower Leg):** Carries the wheel motor assembly, maintaining upright alignment as the leg squats.
4. **Passive Compliance:** Passively sprung shock-absorbing suspension between the lower leg and wheel hub to cushion impacts and eliminate continuous stall torque on the MG995 servos.

### Cascaded Balance Control Loop
The simulation uses a Cascaded PID architecture mirroring the physical ESP32-S3 firmware:
* **Outer Velocity Loop (50 Hz):** Tracks desired velocity $(v_{cmd}, \omega_{cmd})$ from `/cmd_vel` and calculates the lean angle offset $\theta_{target}$.
* **Inner Pitch Balance Loop (200 Hz):** Reads pitch angle $\theta$ and pitch angular velocity $\dot{\theta}$ from the MPU6050 `/imu/data` to compute balancing wheel torque $\tau_{bal}$.
* **Yaw Steering Loop (100 Hz):** Computes differential wheel torque $\Delta \tau$ for directional steering.
* **Fall Detection Watchdog:** Shuts down wheel motors if $|\theta| > 45^\circ$ to prevent motor runaway or hardware burnout.

---

## 3. Package Structure

```
Wobble/
├── pixi.toml                   # Non-root package manager (ROS 2 Jazzy + Gazebo Harmonic)
├── AGENT.md                    # Hardware & kinematic specification
├── src/
│   ├── wobble_description/     # URDF/Xacro models, materials, RViz visualizer
│   │   ├── urdf/
│   │   │   ├── wobble.urdf.xacro
│   │   │   ├── chassis.xacro
│   │   │   ├── leg_4bar.xacro
│   │   │   ├── wheels.xacro
│   │   │   ├── ros2_control.xacro
│   │   │   └── gazebo.xacro
│   │   ├── launch/display.launch.py
│   │   └── rviz/wobble.rviz
│   │
│   ├── wobble_control/         # Cascaded PID balance & posture controller
│   │   ├── wobble_control/balance_controller.py
│   │   ├── config/controllers.yaml
│   │   ├── config/balance_params.yaml
│   │   └── launch/control.launch.py
│   │
│   ├── wobble_gazebo/          # Gazebo Harmonic world, bridge, and spawner
│   │   ├── worlds/wobble_world.sdf
│   │   ├── config/ros_gz_bridge.yaml
│   │   └── launch/sim.launch.py
│   │
│   └── wobble_bringup/         # Master launch files
│       └── launch/wobble_sim.launch.py
```

---

## 4. Quickstart Guide

All commands run inside the isolated Pixi environment without requiring root permissions.

### 1. Build the Workspace
```bash
pixi run build
```

### 2. Inspect Robot Kinematics in RViz2
Launch RViz with interactive joint sliders to articulate the 4-bar squat mechanism and wheels:
```bash
pixi run display
```

### 3. Launch Full Gazebo Harmonic Balance Simulation
Launch Gazebo Harmonic, spawn Wobble, start the bridge, and activate the Cascaded PID balance controller:
```bash
pixi run sim
```

### 4. Teleoperate Wobble (Drive & Steer)
In a separate terminal:
```bash
pixi run teleop
```
* `i` / `,` : Drive Forward / Backward
* `j` / `l` : Turn Left / Right
* `k` : Stop

### 5. Control Squat Posture (MG995 Servos)
Adjust Wobble's center of mass and height on the fly:
```bash
# Deep Squat (-0.5 rad)
pixi run squat

# Stand Upright (0.0 rad)
pixi run stand

# Custom Angle command (range: [-1.57, +1.57] rad)
pixi exec ros2 topic pub /cmd_squat std_msgs/msg/Float64 "{data: 0.4}" -1
```

---

## 5. Tuning & Sim-to-Real Transfer

* Controller tuning parameters can be modified in `src/wobble_control/config/balance_params.yaml`.
* The pitch, velocity, and yaw loop formulations are structured to be directly ported into the C++/ESP-IDF firmware for the physical ESP32-S3 microcontroller.
