cmake_minimum_required(VERSION 3.8)
project(dexi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

# Find Python dependencies
find_package(ament_cmake_python REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  camera_calibration_parsers
)
target_link_libraries(camera_node
  ${OpenCV_LIBS}
)

add_executable(raw_camera_node src/raw_camera_node.cpp)
ament_target_dependencies(raw_camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  camera_calibration_parsers
)
target_link_libraries(raw_camera_node
  ${OpenCV_LIBS}
)

# Install
install(TARGETS
  camera_node
  raw_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS
  src/csi_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()