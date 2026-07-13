#include <plvins_interfaces/msg/plvins_cloud.hpp>
#include <plvins_interfaces/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "linefeature_tracker.h"

#define SHOW_UNDISTORTION 0

std::vector<uchar> r_status;
std::vector<float> r_err;
std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;

class LineFeatureTrackerNode : public rclcpp::Node
{
public:
    LineFeatureTrackerNode() : Node("linefeature_tracker")
    {
        readParameters(this);

        for (int i = 0; i < NUM_OF_CAM; i++)
            trackerData.readIntrinsicParameter(CAM_NAMES[i]);

        RCLCPP_INFO(this->get_logger(), "start line feature");

        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, 100, std::bind(&LineFeatureTrackerNode::img_callback, this, std::placeholders::_1));

        pub_img_ = this->create_publisher<plvins_interfaces::msg::PlvinsCloud>("linefeature", 1000);
        pub_match_ = this->create_publisher<sensor_msgs::msg::Image>("linefeature_img", 1000);
    }

    rclcpp::Publisher<plvins_interfaces::msg::PlvinsCloud>::SharedPtr pub_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;

    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        if(first_image_flag)
        {
            first_image_flag = false;
            first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
        }

        if (round(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            if (abs(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
                pub_count = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;

        TicToc t_r;

        if (PUB_THIS_FRAME)
        {
            cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            cv::Mat show_img = ptr->image;

            frame_cnt++;
            trackerData.readImage(ptr->image.rowRange(0, ROW));

            pub_count++;
            auto feature_lines = std::make_unique<plvins_interfaces::msg::PlvinsCloud>();
            auto id_of_line = plvins_interfaces::msg::ChannelFloat32();
            auto u_of_endpoint = plvins_interfaces::msg::ChannelFloat32();
            auto v_of_endpoint = plvins_interfaces::msg::ChannelFloat32();

            feature_lines->header = img_msg->header;
            feature_lines->header.frame_id = "world";

            std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if (i != 1 || !STEREO_TRACK)
                {
                    auto un_lines = trackerData.undistortedLineEndPoints();
                    auto &ids = trackerData.curframe_->lineID;

                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::msg::Point32 p;
                        p.x = un_lines[j].StartPt.x;
                        p.y = un_lines[j].StartPt.y;
                        p.z = 1;
                        feature_lines->points.push_back(p);
                        id_of_line.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
                        v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
                    }
                }
            }
            feature_lines->channels.push_back(id_of_line);
            feature_lines->channels.push_back(u_of_endpoint);
            feature_lines->channels.push_back(v_of_endpoint);
            RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f",
                rclcpp::Time(feature_lines->header.stamp).seconds(), this->now().seconds());
            pub_img_->publish(std::move(feature_lines));

            // publish visualization image with detected lines
            if (pub_match_->get_subscription_count() > 0)
            {
                cv::Mat line_show_img;
                cv::cvtColor(show_img, line_show_img, cv::COLOR_GRAY2BGR);
                auto &curframe = trackerData.curframe_;
                for (size_t j = 0; j < curframe->lineID.size(); j++)
                {
                    cv::Point startPt(curframe->vecLine[j].StartPt.x,
                                      curframe->vecLine[j].StartPt.y);
                    cv::Point endPt(curframe->vecLine[j].EndPt.x,
                                    curframe->vecLine[j].EndPt.y);
                    cv::line(line_show_img, startPt, endPt, cv::Scalar(0, 0, 255), 2);
                }
                cv::imshow("line image", line_show_img);
                cv::waitKey(1);
                auto line_vis_msg = cv_bridge::CvImage(img_msg->header, "bgr8", line_show_img).toImageMsg();
                pub_match_->publish(*line_vis_msg);
            }
        }
        sum_time += t_r.toc();
        mean_time = sum_time/frame_cnt;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
