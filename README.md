# Razor IMU ROS 2 Driver

This package provides a ROS 2 driver node for the **Razor IMU** sensor.
It reads serial data from the IMU, parses accelerometer, gyroscope,
magnetometer, and quaternion orientation data, and publishes it as
standard ROS 2 messages.

This driver is designed for robotics applications requiring real-time
inertial measurement data and integrates cleanly into a standard ROS 2
control and navigation stack.

------------------------------------------------------------------------

## Features

-   Serial communication with Razor IMU
-   Publishes:
    -   `sensor_msgs/msg/Imu`
    -   `sensor_msgs/msg/MagneticField`
-   Supports quaternion orientation output
-   Parameter-driven configuration via YAML
-   Includes unit tests using `gtest`
-   Designed for ROS 2 Humble and newer

------------------------------------------------------------------------

## Data Format

The firmware is expected to output serial data in the following format:

    timestamp, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz

Where:

-   `a*` = acceleration (m/s²)
-   `g*` = angular velocity (rad/s)
-   `m*` = magnetic field (µT)
-   `q*` = quaternion orientation

------------------------------------------------------------------------

## Package Structure

    razor_imu/
    ├── include/
    │   └── razor_imu/
    │       └── razor_imu_node.hpp
    ├── src/
    │   └── razor_imu_node.cpp
    ├── config/
    │   └── razor.yaml
    ├── launch/
    │   └── razor.launch.py
    ├── test/
    │   └── test_razor_imu_node.cpp
    ├── CMakeLists.txt
    ├── package.xml
    └── README.md

------------------------------------------------------------------------

## Dependencies

-   ROS 2 Humble
-   `rclcpp`
-   `sensor_msgs`
-   `std_msgs`
-   `serial` (or equivalent serial backend)
-   `ament_cmake`
-   `gtest` (for testing)

------------------------------------------------------------------------

## Parameters

Loaded from `config/razor.yaml`:

  Parameter        Type     Description
  ---------------- -------- ------------------------
  `port`           string   Serial device path
  `baudrate`       int      Serial baud rate
  `frame_id`       string   TF frame for IMU
  `publish_rate`   double   Publish frequency (Hz)

------------------------------------------------------------------------

## Running the Node

``` bash
ros2 launch razor_imu razor.launch.py
```

Or directly:

``` bash
ros2 run razor_imu razor_imu_node --ros-args --params-file config/razor.yaml
```

------------------------------------------------------------------------

## Topics

### Subscribed

*(None)*

### Published

  Topic         Type
  ------------- ---------------------------------
  `/imu/data`   `sensor_msgs/msg/Imu`
  `/imu/mag`    `sensor_msgs/msg/MagneticField`

------------------------------------------------------------------------

## Future Improvements

-   Dynamic reconfiguration
-   ROS 2 diagnostics integration
-   Timestamp synchronization

------------------------------------------------------------------------

## Author

Reece Holland\
Robotics Software Engineer

------------------------------------------------------------------------

## License

MIT License