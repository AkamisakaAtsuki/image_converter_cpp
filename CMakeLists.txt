cmake_minimum_required(VERSION 3.5)
project(image_converter_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_converter src/image_converter.cpp)

ament_target_dependencies(image_converter
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  yaml-cpp
)

# yaml-cppをリンク
target_link_libraries(image_converter yaml-cpp)

install(TARGETS
  image_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
