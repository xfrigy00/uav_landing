cmake_minimum_required(VERSION 3.8)
project(uav_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)         # ROS2 C++ client library
find_package(sensor_msgs REQUIRED)    # ROS2 message types for sensor data
find_package(cv_bridge REQUIRED)      # Bridge between ROS2 and OpenCV
find_package(OpenCV REQUIRED)         # OpenCV for image processing
find_package(geometry_msgs REQUIRED)  # ROS2 message of type geometry data

# Regulator node
add_executable(regulator_node src/regulator_node.cpp)
ament_target_dependencies(regulator_node rclcpp geometry_msgs)

# If building tests, set up linting and testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install both nodes
install(TARGETS
  regulator_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

#>>

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Landing.action"
)

#<<

ament_package()

