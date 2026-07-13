cmake_minimum_required(VERSION 3.8)
project(sim_data_pub)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plvins_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(pub_imu src/pub_imu.cpp)
ament_target_dependencies(pub_imu rclcpp sensor_msgs)

add_executable(pub_feature src/pub_feature.cpp)
ament_target_dependencies(pub_feature rclcpp sensor_msgs geometry_msgs plvins_interfaces)

install(TARGETS pub_imu pub_feature DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/euroc_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
