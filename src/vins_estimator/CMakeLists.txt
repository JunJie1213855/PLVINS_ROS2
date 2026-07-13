cmake_minimum_required(VERSION 3.8)
project(plvins_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(plvins_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    src
    src/loop-closure/ThirdParty/DBoW
    src/loop-closure/ThirdParty/DUtils
    src/loop-closure/ThirdParty/DVision
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

set(SOURCEFILES
    src/parameters.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/loop-closure/ThirdParty/DBoW/BowVector.cpp
    src/loop-closure/ThirdParty/DBoW/FBrief.cpp
    src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    src/loop-closure/ThirdParty/DUtils/Random.cpp
    src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    src/loop-closure/ThirdParty/DVision/BRIEF.cpp
    src/loop-closure/ThirdParty/VocabularyBinary.cpp
    src/loop-closure/loop_closure.cpp
    src/loop-closure/keyframe.cpp
    src/loop-closure/keyframe_database.cpp
    src/utility/line_geometry.cpp
    src/factor/line_parameterization.cpp
    src/factor/line_projection_factor.cpp
)

add_library(vinsEstimatorLib SHARED ${SOURCEFILES})
target_link_libraries(vinsEstimatorLib ${OpenCV_LIBS} ${CERES_LIBRARIES})

ament_target_dependencies(vinsEstimatorLib
    rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
    tf2 tf2_ros tf2_eigen cv_bridge camera_model plvins_interfaces visualization_msgs)

add_executable(plvins_estimator
    src/estimator_node.cpp
    src/estimator.cpp
)
target_link_libraries(plvins_estimator vinsEstimatorLib ${OpenCV_LIBS})

install(TARGETS vinsEstimatorLib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS plvins_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
