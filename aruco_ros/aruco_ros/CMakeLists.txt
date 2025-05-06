cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils SHARED src/aruco_ros_utils.cpp)
target_include_directories(aruco_ros_utils
  PUBLIC
  include)

target_include_directories(aruco_ros_utils
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_ros_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(aruco_ros_utils ${OpenCV_LIBRARIES})

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)

target_include_directories(single
  PUBLIC
  include)

target_include_directories(single
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(single ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(single ${OpenCV_LIBRARIES})

add_executable(single_1 src/simple_single_1.cpp
                           src/aruco_ros_utils.cpp)

target_include_directories(single_1
  PUBLIC
  include)

target_include_directories(single_1
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(single_1 ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(single_1 ${OpenCV_LIBRARIES})

add_executable(single_2 src/simple_single_2.cpp
                           src/aruco_ros_utils.cpp)

target_include_directories(single_2
  PUBLIC
  include)

target_include_directories(single_2
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(single_2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(single_2 ${OpenCV_LIBRARIES})

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
target_include_directories(double
  PUBLIC
  include)

target_include_directories(double
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(double ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(double ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
target_include_directories(marker_publisher
  PUBLIC
  include)

target_include_directories(marker_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(marker_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})


#>>
# Add executable for CameraInfoPublisher
add_executable(camera_info_publisher src/camera_info_publisher.cpp)

# Include directories for the new node
target_include_directories(camera_info_publisher
  PUBLIC
  include
)

# Add system-specific includes if necessary
target_include_directories(camera_info_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Link the necessary dependencies
ament_target_dependencies(camera_info_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(camera_info_publisher ${OpenCV_LIBRARIES})

#|#|#

# Add executable for MarkerSubscriber
add_executable(marker_subscriber src/marker_subscriber.cpp)

# Include directories for the new node
target_include_directories(marker_subscriber
  PUBLIC
  include
)

# Add system-specific includes if necessary
target_include_directories(marker_subscriber
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Link the necessary dependencies
ament_target_dependencies(marker_subscriber ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_subscriber ${OpenCV_LIBRARIES})

#|#|#

# Add executable for VelocityPublisher
add_executable(velocity_publisher src/velocity_publisher.cpp)

# Include directories for the VelocityPublisher node
target_include_directories(velocity_publisher
  PUBLIC
  include
)

# Add system-specific includes if necessary
target_include_directories(velocity_publisher
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Link the necessary dependencies
ament_target_dependencies(velocity_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(velocity_publisher ${OpenCV_LIBRARIES})

#|#|#

# Add executable for marker_detector
add_executable(marker_detector src/marker_detector.cpp)

# Include directories for the new node
target_include_directories(marker_detector
  PUBLIC
  include
)

# Add system-specific includes if necessary
target_include_directories(marker_detector
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Link the necessary dependencies
ament_target_dependencies(marker_detector ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(marker_detector ${OpenCV_LIBRARIES})

#|#|#
#<<



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(TARGETS aruco_ros_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS marker_publisher single single_1 single_2 double
  DESTINATION lib/${PROJECT_NAME}
)
  
#>>
# Install the new executable
install(TARGETS camera_info_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install the new executable
install(TARGETS marker_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install the VelocityPublisher node
install(TARGETS velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install the marker_detector node
install(TARGETS marker_detector
  DESTINATION lib/${PROJECT_NAME}
)
#<<

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()


