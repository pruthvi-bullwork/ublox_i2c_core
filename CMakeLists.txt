cmake_minimum_required(VERSION 3.8)
project(ublox_i2c_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)

include_directories(include)

# --- BASE ---
add_executable(base_node src/base_node.cpp src/ublox_i2c_handler.cpp)
ament_target_dependencies(base_node rclcpp sensor_msgs std_msgs)

# --- ROVER ---
add_executable(rover_node src/rover_node.cpp)
ament_target_dependencies(rover_node rclcpp sensor_msgs std_msgs)

# --- MANAGER ---
add_executable(i2c_manager_node src/i2c_manager_node.cpp src/ublox_i2c_handler.cpp)
ament_target_dependencies(i2c_manager_node rclcpp rtcm_msgs)

# --- INSTALL BINARIES ---
install(TARGETS 
  base_node 
  rover_node 
  i2c_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- INSTALL LAUNCH FILES ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()