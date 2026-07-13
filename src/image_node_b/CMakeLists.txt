cmake_minimum_required(VERSION 3.8)
project(image_node_b)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(camera_model REQUIRED)
find_package(plvins_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(/usr/include/eigen3)

add_executable(image_node_b src/image_node_b.cpp)
ament_target_dependencies(image_node_b
  rclcpp sensor_msgs std_msgs cv_bridge
  message_filters camera_model plvins_interfaces)
target_link_libraries(image_node_b ${OpenCV_LIBS})

install(TARGETS image_node_b DESTINATION lib/${PROJECT_NAME})
ament_package()
