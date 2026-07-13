#include <plvins_interfaces/msg/plvins_cloud.hpp>
#include <plvins_interfaces/msg/channel_float32.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

std::vector<uchar> r_status;
std::vector<float> r_err;
std::queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;

double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;

class FeatureTrackerNode : public rclcpp::Node
{
public:
    FeatureTrackerNode() : Node("feature_tracker")
    {
        readParameters(this);

        for (int i = 0; i < NUM_OF_CAM; i++)
            trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

        if(FISHEYE)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
                if(!trackerData[i].fisheye_mask.data)
                {
                    RCLCPP_INFO(this->get_logger(), "load mask fail");
                    rclcpp::shutdown();
                }
                else
                    RCLCPP_INFO(this->get_logger(), "load mask success");
            }
        }

        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            IMAGE_TOPIC, 100, std::bind(&FeatureTrackerNode::img_callback, this, std::placeholders::_1));

        pub_img_ = this->create_publisher<plvins_interfaces::msg::PlvinsCloud>("feature", 1000);
        pub_match_ = this->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
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

        // frequency control
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

        cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat show_img = ptr->image;

        TicToc t_r;
        frame_cnt++;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)));
            else
            {
                if (EQUALIZE)
                {
                    std::cout <<" EQUALIZE " << std::endl;
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                }
                else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }
#if SHOW_UNDISTORTION
//        trackerData[i].showUndistortion();
#endif
        }

        // stereo tracking
        if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
        {
            pub_count++;
            r_status.clear();
            r_err.clear();
            TicToc t_o;
            cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
            RCLCPP_DEBUG(this->get_logger(), "spatial optical flow costs: %fms", t_o.toc());
            std::vector<cv::Point2f> ll, rr;
            std::vector<int> idx;
            for (unsigned int i = 0; i < r_status.size(); i++)
            {
                if (!inBorder(trackerData[1].cur_pts[i]))
                    r_status[i] = 0;

                if (r_status[i])
                {
                    idx.push_back(i);
                    Eigen::Vector3d tmp_p;
                    trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
                    trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                    tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                    tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                    rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
                }
            }
            if (ll.size() >= 8)
            {
                std::vector<uchar> status;
                TicToc t_f;
                cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
                RCLCPP_DEBUG(this->get_logger(), "find f cost: %f", t_f.toc());
                int r_cnt = 0;
                for (unsigned int i = 0; i < status.size(); i++)
                {
                    if (status[i] == 0)
                        r_status[idx[i]] = 0;
                    r_cnt += r_status[idx[i]];
                }
            }
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }

       if (PUB_THIS_FRAME)
       {
            pub_count++;
            auto feature_points = std::make_unique<plvins_interfaces::msg::PlvinsCloud>();
            auto id_of_point = plvins_interfaces::msg::ChannelFloat32();
            auto u_of_point = plvins_interfaces::msg::ChannelFloat32();
            auto v_of_point = plvins_interfaces::msg::ChannelFloat32();

            feature_points->header = img_msg->header;
            feature_points->header.frame_id = "world";

            std::vector<std::set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if (i != 1 || !STEREO_TRACK)
                {
                    auto un_pts = trackerData[i].undistortedPoints();
                    auto &cur_pts = trackerData[i].cur_pts;
                    auto &ids = trackerData[i].ids;
                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::msg::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;
                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        assert(inBorder(cur_pts[j]));
                    }
                }
                else if (STEREO_TRACK)
                {
                    auto r_un_pts = trackerData[1].undistortedPoints();
                    auto &ids = trackerData[0].ids;
                    for (unsigned int j = 0; j < ids.size(); j++)
                    {
                        if (r_status[j])
                        {
                            int p_id = ids[j];
                            hash_ids[i].insert(p_id);
                            geometry_msgs::msg::Point32 p;
                            p.x = r_un_pts[j].x;
                            p.y = r_un_pts[j].y;
                            p.z = 1;
                            feature_points->points.push_back(p);
                            id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        }
                    }
                }
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f",
                rclcpp::Time(feature_points->header.stamp).seconds(), this->now().seconds());
            pub_img_->publish(std::move(feature_points));

            {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                cv::Mat stereo_img = ptr->image;
                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                    cv::cvtColor(show_img, tmp_img, cv::COLOR_GRAY2RGB);
                    cv::cvtColor(trackerData[0].cur_img, tmp_img, cv::COLOR_GRAY2RGB);
                    if (i != 1 || !STEREO_TRACK)
                    {
                        for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                        {
                            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                            cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                        }
                    }
                    else
                    {
                        for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                        {
                            if (r_status[j])
                            {
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255, 0), 2);
                                cv::line(stereo_img, trackerData[i - 1].cur_pts[j], trackerData[i].cur_pts[j] + cv::Point2f(0, ROW), cv::Scalar(0, 255, 0));
                            }
                        }
                    }
                }
                pub_match_->publish(*ptr->toImageMsg());
            }
        }
        sum_time += t_r.toc();
        mean_time = sum_time/frame_cnt;
        RCLCPP_INFO(this->get_logger(), "whole point feature tracker processing costs: %f", mean_time);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
