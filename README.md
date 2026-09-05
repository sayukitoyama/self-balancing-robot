# Self-Balancing Robot

This project is a two-wheel self-balancing robot designed and built using an ESP32, an MPU-6050 inertial measurement unit, and a TB6612FNG motor driver. The robot uses accelerometer and gyroscope data from the MPU-6050 to estimate its tilt angle in real time.

The estimated tilt angle is compared with the desired upright position, and a PID controller calculates the motor output needed to correct the robot's motion. The TB6612FNG motor driver controls two DC gear motors that move the wheels in response to the controller output.

## Mechanical Design

The robot chassis and drive components were modeled in CAD to support the motors, electronics, and wheel assembly. Custom components were designed around the dimensions of the physical hardware and incorporated into the final prototype.
