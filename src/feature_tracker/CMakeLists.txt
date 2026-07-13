cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(message_filters REQUIRED)
find_package(plvins_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    src/line_descriptor/include
)

# point feature tracker
add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

ament_target_dependencies(feature_tracker
    rclcpp sensor_msgs cv_bridge camera_model message_filters plvins_interfaces OpenCV)

target_link_libraries(feature_tracker ${OpenCV_LIBS})

# line feature tracker (build line descriptor from source)
add_executable(LineFeature_tracker
    src/linefeature_tracker_node.cpp
    src/parameters.cpp
    src/linefeature_tracker.cpp
    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
)

ament_target_dependencies(LineFeature_tracker
    rclcpp sensor_msgs cv_bridge camera_model message_filters plvins_interfaces OpenCV)

target_link_libraries(LineFeature_tracker ${OpenCV_LIBS})

install(TARGETS feature_tracker LineFeature_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY ../support_files/
  DESTINATION share/${PROJECT_NAME}/support_files
)

ament_package()
