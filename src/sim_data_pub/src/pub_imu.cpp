#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
using namespace std;

void LoadPose(std::string filename, std::vector<double>& timestamp,
              std::vector<Eigen::Vector3d>& gyros, std::vector<Eigen::Vector3d>& accs)
{
    std::ifstream f;
    f.open(filename.c_str());
    if(!f.is_open())
    {
        std::cerr << " can't open LoadFeatures file " << std::endl;
        return;
    }
    while (!f.eof()) {
        std::string s;
        std::getline(f, s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t, gyro, acc;
            ss >> time;
            ss >> q.w(); ss >> q.x(); ss >> q.y(); ss >> q.z();
            ss >> t(0); ss >> t(1); ss >> t(2);
            ss >> gyro(0); ss >> gyro(1); ss >> gyro(2);
            ss >> acc(0); ss >> acc(1); ss >> acc(2);
            timestamp.push_back(time);
            gyros.push_back(gyro);
            accs.push_back(acc);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu_publisher");
    
    auto out_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu0", 1000);
    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<Eigen::Vector3d> accs;
    
    std::string sim_file;
    node->declare_parameter<std::string>("sim_file_path", "/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/");
    sim_file = node->get_parameter("sim_file_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Loaded sim_file_path: " << sim_file);
    
    LoadPose(sim_file + "imu_pose.txt", timestamp, gyros, accs);
    sleep(1);
    
    rclcpp::Rate loop_rate(200);
    for (size_t i = 0; i < timestamp.size(); ++i)
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp[i] * 1e9));
        imu_msg.header.frame_id = "imu0";
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1.0;
        Eigen::Vector3d gyro = gyros[i];
        Eigen::Vector3d acc = accs[i];
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        out_pub->publish(imu_msg);
        RCLCPP_INFO(node->get_logger(), "send an imu message");
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
