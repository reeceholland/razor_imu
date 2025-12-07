# razor_imu
**ROS 2 C++ Driver for the 9DoF Razor IMU (MPU-9250 / MPU-9150)**

`razor_imu` is a lightweight, high-performance ROS 2 driver that reads IMU data over a serial connection from the SparkFun 9DoF Razor IMU board and publishes standard ROS messages:

- `sensor_msgs/Imu`
- `sensor_msgs/MagneticField`

It supports both classic firmware output and the **quat-enabled firmware** producing fused orientation directly from the onboard MPU-9250 DMP.

This repository is written in modern C++ (C++17), uses **standalone ASIO** for fast, dependency-free serial I/O, and integrates cleanly into larger robotics stacks such as the **Rugged Rover** platform.

---

## Features

- Reads IMU data over USB serial connection.
- Su