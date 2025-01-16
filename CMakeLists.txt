cmake_minimum_required(VERSION 3.0.2)
project(rmp220_middleware)

# Set compiler options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  segway_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs segway_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  rmp220_middleware/rmp220_middleware.py  # Replace with the actual script path
  rmp220_middleware/rmp220_middleware_extended.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Testing (optional)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add tests here if needed
endif()
