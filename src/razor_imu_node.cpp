#include "razor_imu/razor_imu_node.hpp"

#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Constructor for the RazorIMUNode class.
 */
RazorIMUNode::RazorIMUNode()
: Node("razor_imu_node"),
  io_context_(),
  serial_port_(io_context_),
  running_(false)
{
  // Parameters
  port_      = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  baudrate_  = this->declare_parameter<int>("baudrate", 9600);
  frame_id_  = this->declare_parameter<std::string>("frame_id", "imu_link");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu/data");
  mag_topic_ = this->declare_parameter<std::string>("mag_topic", "imu/mag");

  RCLCPP_INFO(get_logger(), "Opening Razor IMU on %s @ %d",
              port_.c_str(), baudrate_);

  // Create publishers
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_, 10);

  // Open serial port
  openSerial();

  // Start reading thread
  startReadThread();
}

/**
 * @brief Destructor for the RazorIMUNode class.
 */
RazorIMUNode::~RazorIMUNode()
{
  // Signal the read thread to stop
  running_.store(false);

  // Cancel and close the serial port to unblock any pending reads
  if (serial_port_.is_open()) {
    asio::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
  }

  // Join the read thread if it's running
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

/**
 * @brief Trim leading whitespace from a string.
 * 
 * @param s The string to trim.
 */
void RazorIMUNode::ltrim(std::string & s)
{
  // Remove leading whitespace characters from the string
  s.erase(s.begin(),
    std::find_if(s.begin(), s.end(),
      [](unsigned char ch){ return !std::isspace(ch); }));
}

/**
 * @brief Trim trailing whitespace from a string.
 * 
 * @param s The string to trim.
 */
void RazorIMUNode::rtrim(std::string & s)
{
  // Remove trailing whitespace characters from the string
  s.erase(
    std::find_if(s.rbegin(), s.rend(),
      [](unsigned char ch){ return !std::isspace(ch); }).base(),
    s.end());
}

/**
 * @brief Trim leading and trailing whitespace from a string.
 * 
 * @param s The string to trim.
 */
void RazorIMUNode::trim(std::string & s)
{
  ltrim(s);
  rtrim(s);
}

/**
 * @brief Open the serial port with the specified parameters.
 */
void RazorIMUNode::openSerial()
{
  // Create an error code object to capture any errors.
  asio::error_code ec;

  // Attempt to open the serial port
  serial_port_.open(port_, ec);

  // Check for errors during port opening
  if (ec) {
    RCLCPP_FATAL(get_logger(), "Failed to open port %s: %s",
                 port_.c_str(), ec.message().c_str());
    throw std::runtime_error("Could not open serial port");
  }

  // Set serial port options
  serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate_));
  serial_port_.set_option(asio::serial_port_base::character_size(8));
  serial_port_.set_option(asio::serial_port_base::stop_bits(
      asio::serial_port_base::stop_bits::one));
  serial_port_.set_option(asio::serial_port_base::parity(
      asio::serial_port_base::parity::none));
  serial_port_.set_option(asio::serial_port_base::flow_control(
      asio::serial_port_base::flow_control::none));

  RCLCPP_INFO(get_logger(), "Serial port %s opened.", port_.c_str());
}

/**
 * @brief Start the thread that reads data from the serial port.
 */
void RazorIMUNode::startReadThread()
{
  // Set running to true to start the read thread
  running_.store(true);

  // Launch the read thread
  read_thread_ = std::thread([this]() { this->readLoop(); });
}

/**
 * @brief Main loop for reading data from the serial port.
 */
void RazorIMUNode::readLoop()
{
  // Buffer to hold incoming data
  asio::streambuf buffer;

  // Error code for read operations
  asio::error_code ec;

  // Read data from the serial port until a newline character is encountered
  while (rclcpp::ok() && running_.load()) {
    std::size_t n = asio::read_until(serial_port_, buffer, '\n', ec);
    (void)n;

    // Check for errors during read operation
    if (ec) {
      if (running_.load()) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Serial read error: %s", ec.message().c_str());
      }
      continue;
    }

    // Convert the buffer into a string stream for line extraction
    std::istream is(&buffer);

    // Extract a line from the stream
    std::string line;
    std::getline(is, line);

    // Remove carriage return if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    double time_ms;
    double ax_g, ay_g, az_g;
    double gx_dps, gy_dps, gz_dps;
    double mx_uT, my_uT, mz_uT;
    double qw, qx, qy, qz;

    // Parse the line to extract sensor data
    if (!parseSampleLine(line, time_ms, ax_g, ay_g, az_g,
                         gx_dps, gy_dps, gz_dps, mx_uT, my_uT, mz_uT,
                         qw, qx, qy, qz))
    {
      RCLCPP_DEBUG(get_logger(), "Ignored line: '%s'", line.c_str());
      continue;
    }

    // Process and publish the parsed sensor data
    processAndPublish(time_ms,
                      ax_g, ay_g, az_g,
                      gx_dps, gy_dps, gz_dps,
                      mx_uT, my_uT, mz_uT,
                      qw, qx, qy, qz);
  }
}


/**
 * @brief Parse a line of sensor data from the IMU.
 * line format:
 * time_ms, ax, ay, az, gx, gy, gz, mx, my, mz, qw, qx, qy, qz
 * 
 * @param line The input line containing sensor data.
 * @param time_ms Timestamp in milliseconds.
 * @param ax Acceleration in X (g).
 * @param ay Acceleration in Y (g).
 * @param az Acceleration in Z (g).
 * @param gx Gyroscope in X (degrees per second).
 * @param gy Gyroscope in Y (degrees per second).
 * @param gz Gyroscope in Z (degrees per second).
 * @param mx Magnetometer in X (microTesla).
 * @param my Magnetometer in Y (microTesla).
 * @param mz Magnetometer in Z (microTesla).
 * @param qw Quaternion W component.
 * @param qx Quaternion X component.
 * @param qy Quaternion Y component.
 * @param qz Quaternion Z component.
 * @return true if parsing was successful, false otherwise.
 */
bool RazorIMUNode::parseSampleLine(const std::string & line,
                                   double & time_ms,
                                   double & ax, double & ay, double & az,
                                   double & gx, double & gy, double & gz,
                                   double & mx, double & my, double & mz,
                                   double & qw, double & qx, double & qy, double & qz)
{
  // Copy the input line to a modifiable string
  std::string data = line;

  // Trim whitespace from the entire line
  trim(data);

  // Return false if the line is empty after trimming
  if (data.empty()) {
    return false;
  }

  std::stringstream ss(data);
  std::string token;
  std::vector<std::string> tokens;

  // Split the line by commas and trim each token
  while (std::getline(ss, token, ',')) {
    trim(token);
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }

  // Ensure we have the correct number of tokens
  if (tokens.size() < 14) {
    return false;
  }

  try {

    // Parse values from tokens
    time_ms = std::stod(tokens[0]);

    // Accelerometer order: ax, ay, az
    ax = std::stod(tokens[1]);
    ay = std::stod(tokens[2]);
    az = std::stod(tokens[3]);

    // Gyroscope order: gx, gy, gz
    gx = std::stod(tokens[4]);
    gy = std::stod(tokens[5]);
    gz = std::stod(tokens[6]);

    // Magnetometer order: mx, my, mz
    mx = std::stod(tokens[7]);
    my = std::stod(tokens[8]);
    mz = std::stod(tokens[9]);

    // Quaternion order: qw, qx, qy, qz
    qw = std::stod(tokens[10]);
    qx = std::stod(tokens[11]);
    qy = std::stod(tokens[12]);
    qz = std::stod(tokens[13]);
  } catch (...) {
    return false;
  }

  return true;
}

/**
 * @brief Process the parsed sensor data and publish ROS messages.
 * 
 * @param time_ms Timestamp in milliseconds.
 * @param ax_g Acceleration in X (g).
 * @param ay_g Acceleration in Y (g).
 * @param az_g Acceleration in Z (g).
 * @param gx_dps Gyroscope in X (degrees per second).
 * @param gy_dps Gyroscope in Y (degrees per second).
 * @param gz_dps Gyroscope in Z (degrees per second).
 * @param mx_uT Magnetometer in X (microTesla).
 * @param my_uT Magnetometer in Y (microTesla).
 * @param mz_uT Magnetometer in Z (microTesla).
 * @param qw Quaternion W component.
 * @param qx Quaternion X component.
 * @param qy Quaternion Y component.
 * @param qz Quaternion Z component.
 */
void RazorIMUNode::processAndPublish(double time_ms,
                                     double ax_g, double ay_g, double az_g,
                                     double gx_dps, double gy_dps, double gz_dps,
                                     double mx_uT, double my_uT, double mz_uT,
                                     double qw, double qx, double qy, double qz)
{
  (void)time_ms;  // Unused parameter for now
  auto stamp = get_clock()->now();

  // Unit conversions
  const double g_to_ms2 = 9.80665;      // g -> m/s^2
  const double deg2rad  = M_PI / 180.0; // deg/s -> rad/s
  const double uT_to_T  = 1e-6;         // microTesla -> Tesla

  // Convert g to m/s^2
  double ax = ax_g * g_to_ms2;
  double ay = ay_g * g_to_ms2;
  double az = az_g * g_to_ms2;

  // Convert deg/s to rad/s
  double gx = gx_dps * deg2rad;
  double gy = gy_dps * deg2rad;
  double gz = gz_dps * deg2rad;

  // Convert microTesla to Tesla
  double mx = mx_uT * uT_to_T;
  double my = my_uT * uT_to_T;
  double mz = mz_uT * uT_to_T;

  // IMU message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = frame_id_;

  // Calculate norm of quaternion
  double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

  // If norm is too small or NaN, skip this frame
  if (norm < 1e-6 || std::isnan(norm)) {
    RCLCPP_WARN(this->get_logger(), "Invalid quaternion, skipping frame");
    return;
  }

  // Normalize quaternion
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;

  // Orientation from firmware quaternion
  imu_msg.orientation.w = qw;
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;

  // Orientation covariance TODO(Reece): Tune these values based on sensor characteristics.
  imu_msg.orientation_covariance[0] = 0.01;  // roll var
  imu_msg.orientation_covariance[4] = 0.01;  // pitch var
  imu_msg.orientation_covariance[8] = 0.03;  // yaw var

  // Angular velocity
  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  // Angular velocity covariance TODO(Reece): Tune these values based on sensor characteristics.
  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;

  // Linear acceleration
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  // Linear acceleration covariance TODO(Reece): Tune these values based on sensor characteristics.
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  // Publish IMU message
  imu_pub_->publish(imu_msg);

  // Magnetometer message
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp = stamp;
  mag_msg.header.frame_id = frame_id_;

  // Magnetic field from firmware magnetometer
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;

  // Magnetic field covariance TODO(Reece): Tune these values based on sensor characteristics.
  mag_msg.magnetic_field_covariance[0] = -1.0;

  // Publish magnetometer message
  mag_pub_->publish(mag_msg);
}
