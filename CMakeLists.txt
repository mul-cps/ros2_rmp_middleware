cmake_minimum_required(VERSION 3.5)
project(rmp220_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(segway_msgs REQUIRED)

add_executable(rmp220_middleware rmp220_middleware.cpp)
ament_target_dependencies(rmp220_middleware rclcpp std_msgs geometry_msgs sensor_msgs segway_msgs)

install(TARGETS rmp220_middleware DESTINATION lib/${PROJECT_NAME})

ament_package()
