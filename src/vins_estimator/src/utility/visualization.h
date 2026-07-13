#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

#include <plvins_interfaces/msg/plvins_cloud.hpp>
#include <plvins_interfaces/msg/channel_float32.hpp>

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(rclcpp::Node *n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::msg::Header &header);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r);

void pubInitialGuess(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                Eigen::Matrix3d loop_correct_r);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

void pubLinesCloud(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

void pubPoseGraph(CameraPoseVisualization* posegraph, const std_msgs::msg::Header &header);

void updateLoopPath(nav_msgs::msg::Path _loop_path);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header, Eigen::Vector3d loop_correct_t,
                   Eigen::Matrix3d loop_correct_r);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);
