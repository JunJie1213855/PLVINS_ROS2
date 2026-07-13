#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <message_filters/subscriber.h>

#include <plvins_interfaces/msg/plvins_cloud.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

camodocal::CameraPtr m_camera;
cv::Mat undist_map1_, undist_map2_, K_;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_point_line;

void callback(const plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr &point_feature_msg,
              const plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr &line_feature_msg,
              const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
    cv::Mat img1;
    cv::cvtColor(show_img, img1, cv::COLOR_GRAY2BGR);

    for(size_t i = 0; i < point_feature_msg->points.size(); i++)
    {
        cv::Point endPoint(point_feature_msg->channels[1].values[i],
                           point_feature_msg->channels[2].values[i]);
        cv::circle(img1, endPoint, 2, cv::Scalar(0, 255, 0), 2);
    }

    cv::remap(img1, show_img, undist_map1_, undist_map2_, cv::INTER_LINEAR);
    for(size_t i = 0; i < line_feature_msg->points.size(); i++)
    {
        cv::Point startPoint(line_feature_msg->channels[3].values[i],
                             line_feature_msg->channels[4].values[i]);
        cv::Point endPoint(line_feature_msg->channels[5].values[i],
                           line_feature_msg->channels[6].values[i]);
        cv::line(show_img, startPoint, endPoint, cv::Scalar(0, 0, 255), 2, 8);
    }

    auto output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", show_img).toImageMsg();
    pub_point_line->publish(*output_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sync_control_node");

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        "/../../config/euroc/euroc_config_fix_extrinsic.yaml");
    K_ = m_camera->initUndistortRectifyMap(undist_map1_, undist_map2_);

    message_filters::Subscriber<plvins_interfaces::msg::PlvinsCloud> point_feature_sub(node, "/feature_tracker/feature");
    message_filters::Subscriber<plvins_interfaces::msg::PlvinsCloud> line_feature_sub(node, "/linefeature_tracker/linefeature");
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node, "/cam0/image_raw");

    pub_point_line = node->create_publisher<sensor_msgs::msg::Image>("/PointLine_image", 1000);

    typedef message_filters::sync_policies::ExactTime<
        plvins_interfaces::msg::PlvinsCloud,
        plvins_interfaces::msg::PlvinsCloud,
        sensor_msgs::msg::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_feature_sub, line_feature_sub, image_sub);
    sync.registerCallback(&callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
