#include <rclcpp/rclcpp.hpp>
#include <plvins_interfaces/msg/plvins_cloud.hpp>
#include <plvins_interfaces/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <sstream>

using namespace std;

void LoadPose(std::string filename, std::vector<double>& timestamp,
              std::vector<Eigen::Vector3d>& gyros, std::vector<Eigen::Vector3d>& accs)
{
    std::ifstream f(filename.c_str());
    if(!f.is_open()) { std::cerr << "can't open LoadPose file" << std::endl; return; }
    while (!f.eof()) {
        std::string s; std::getline(f, s);
        if(!s.empty()) {
            std::stringstream ss; ss << s;
            double time; Eigen::Quaterniond q; Eigen::Vector3d t, gyro, acc;
            ss >> time;
            ss >> q.w(); ss >> q.x(); ss >> q.y(); ss >> q.z();
            ss >> t(0); ss >> t(1); ss >> t(2);
            ss >> gyro(0); ss >> gyro(1); ss >> gyro(2);
            ss >> acc(0); ss >> acc(1); ss >> acc(2);
            timestamp.push_back(time); gyros.push_back(gyro); accs.push_back(acc);
        }
    }
}

void LoadPointObs(std::string filename, std::vector<Eigen::Vector2d>& obs)
{
    std::ifstream f(filename.c_str());
    if(!f.is_open()) { std::cerr << "can't open LoadPointObs file" << std::endl; return; }
    while (!f.eof()) {
        std::string s; std::getline(f, s);
        if(!s.empty()) {
            std::stringstream ss; ss << s;
            Eigen::Vector3d p; Eigen::Vector2d ob; double temp;
            ss >> p(0); ss >> p(1); ss >> p(2); ss >> temp; ss >> ob(0); ss >> ob(1);
            obs.push_back(ob);
        }
    }
}

void LoadLineObs(std::string filename, std::vector<Eigen::Vector4d>& obs)
{
    std::ifstream f(filename.c_str());
    if(!f.is_open()) { std::cerr << "can't open LoadLineObs file" << std::endl; return; }
    while (!f.eof()) {
        std::string s; std::getline(f, s);
        if(!s.empty()) {
            std::stringstream ss; ss << s;
            Eigen::Vector4d ob; ss >> ob(0); ss >> ob(1); ss >> ob(2); ss >> ob(3);
            obs.push_back(ob);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("feature_publisher");

    auto pub_pobs = node->create_publisher<plvins_interfaces::msg::PlvinsCloud>("/feature_tracker/feature", 1000);
    auto pub_lobs = node->create_publisher<plvins_interfaces::msg::PlvinsCloud>("/linefeature_tracker/linefeature", 1000);

    std::vector<double> timestamp;
    std::vector<Eigen::Vector3d> gyros, accs;

    node->declare_parameter<std::string>("sim_file_path", "/home/hyj/my_slam/vio_sim/vio_pl_sim/bin/");
    std::string sim_file = node->get_parameter("sim_file_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(), "Loaded sim_file_path: " << sim_file);

    LoadPose(sim_file + "cam_pose.txt", timestamp, gyros, accs);
    sleep(1);
    RCLCPP_INFO(node->get_logger(), "START PUB POINTS AND LINES FEATURE");

    rclcpp::Rate loop_rate(30);
    for (size_t i = 0; i < timestamp.size(); ++i)
    {
        stringstream ss;
        ss << sim_file << "keyframe/all_points_" << i << ".txt";
        std::vector<Eigen::Vector2d> pobs;
        LoadPointObs(ss.str(), pobs);

        stringstream ss1;
        ss1 << sim_file << "keyframe/all_lines_" << i << ".txt";
        std::vector<Eigen::Vector4d> lobs;
        LoadLineObs(ss1.str(), lobs);

        // point feature cloud
        auto feature_points = std::make_unique<plvins_interfaces::msg::PlvinsCloud>();
        auto id_of_point = plvins_interfaces::msg::ChannelFloat32();
        auto u_of_point = plvins_interfaces::msg::ChannelFloat32();
        auto v_of_point = plvins_interfaces::msg::ChannelFloat32();

        feature_points->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp[i] * 1e9));
        feature_points->header.frame_id = "world";

        for (size_t j = 0; j < pobs.size(); ++j) {
            geometry_msgs::msg::Point32 p;
            p.x = pobs[j].x(); p.y = pobs[j].y(); p.z = 1;
            feature_points->points.push_back(p);
            id_of_point.values.push_back(j * 1.0f);
            u_of_point.values.push_back(static_cast<float>(pobs[j].x()));
            v_of_point.values.push_back(static_cast<float>(pobs[j].y()));
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        pub_pobs->publish(std::move(feature_points));

        // line feature cloud
        auto feature_lines = std::make_unique<plvins_interfaces::msg::PlvinsCloud>();
        auto id_of_line = plvins_interfaces::msg::ChannelFloat32();
        auto u_of_endpoint = plvins_interfaces::msg::ChannelFloat32();
        auto v_of_endpoint = plvins_interfaces::msg::ChannelFloat32();

        feature_lines->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp[i] * 1e9));
        feature_lines->header.frame_id = "world";

        for (size_t j = 0; j < lobs.size(); j++) {
            geometry_msgs::msg::Point32 p;
            p.x = lobs[j](0); p.y = lobs[j](1); p.z = 1;
            feature_lines->points.push_back(p);
            id_of_line.values.push_back(static_cast<float>(j));
            u_of_endpoint.values.push_back(lobs[j](2));
            v_of_endpoint.values.push_back(lobs[j](3));
        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);
        pub_lobs->publish(std::move(feature_lines));

        RCLCPP_INFO(node->get_logger(), "publish Points and lines timestamp: %f", timestamp[i]);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
