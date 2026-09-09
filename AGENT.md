**Project Wobble: Agent Architecture**

**Overview**
Wobble is a dynamically balancing, two-wheeled bipedal robot. It utilizes a 4-bar linkage to control squatting posture via 180-degree servos(MG995), while relying on DC encoder motors and a continuous PID control loop to maintain an inverted pendulum balance. 

**Hardware Stack**
* Processing: ESP32-S3 (Physical) / ROS 2 Jazzy (Simulation)
* Actuators (Posture): 2x MG995 180-degree Servos
* Actuators (Drive/Balance): 2x DC Motors with Magnetic Encoders
* Sensors: MPU6050 (6-axis IMU)

**Kinematic Constraints**
* Hip servos are physically restricted to [-1.57, 1.57] radians.
* Drive wheels require continuous high-speed hardware interrupts on the microcontroller for accurate quadrature encoder counting.
* Leg linkages are passively sprung to absorb shock and reduce continuous holding torque on the MG995 servos.