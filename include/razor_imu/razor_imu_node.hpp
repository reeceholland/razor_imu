#ifndef RAZOR_IMU_NODE_HPP_
#define RAZOR_IMU_NODE_HPP_

#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

// Standalone ASIO (header-only)
#include <asio.hpp>

class RazorIMUNode : public rclcpp::Node
{
public:
  RazorIMUNode();
  ~RazorIMUNode() override;

private:
  // String helpers
  static void ltrim(std::string & s);
  static void rtrim(std::string & s);
  static void trim(std::string & s);

  void openSerial();
  void startReadThread();
  void readLoop();

  bool parseSampleLine(const std::string & line,
                       double & time_ms,
                       double & ax, double & ay, double & az,
                       double & gx, double & gy, double & gz,
                       double & mx, double & my, double & mz,
                       double & qw, double & qx, double & qy, double & qz);

  void processAndPublish(double time_ms,
                         double ax_g, double ay_g, double az_g,
                         double gx_dps, double gy_dps, double gz_dps,
                         double mx_uT, double my_uT, double mz_uT,
                         double qw, double qx, double qy, double qz);

  // Parameters
  std::string port_;
  int baudrate_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string mag_topic_;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // Serial / threading
  asio::io_context io_context_;
  asio::serial_port serial_port_;
  std::thread read_thread_;
  std::atomic<bool> running_;
};

#endif  // RAZOR_IMU_NODE_HPP_
