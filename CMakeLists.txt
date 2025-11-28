cmake_minimum_required(VERSION 3.16)
project(razor_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_definitions(-DASIO_STANDALONE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_library(razor_imu_lib
  src/razor_imu_node.cpp
)
ament_target_dependencies(razor_imu_lib
  rclcpp
  sensor_msgs
)

add_executable(razor_imu_node
  src/razor_imu_main.cpp
)
target_link_libraries(razor_imu_node
  razor_imu_lib
)

ament_target_dependencies(razor_imu_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  razor_imu_lib
  razor_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
