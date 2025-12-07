#include "razor_imu/razor_imu_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Main function for the Razor IMU node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RazorIMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}