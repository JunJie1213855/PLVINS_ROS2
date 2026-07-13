#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

Estimator estimator;
std::queue<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> relo_buf;
std::condition_variable con;
double current_time = -1;
std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf;
std::queue<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> feature_buf;
std::queue<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> linefeature_buf;
std::mutex m_posegraph_buf;
std::queue<int> optimize_posegraph_buf;
std::queue<KeyFrame*> keyframe_buf;
std::queue<RetriveData> retrive_data_buf;
int sum_of_wait = 0;
std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;
double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
std::queue<std::pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;
int global_frame_cnt = 0;
camodocal::CameraPtr m_camera;
std::vector<int> erase_index;
std_msgs::msg::Header cur_header;
Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

void predict(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    double t = rclcpp::Time(imu_msg->header.stamp).seconds();
    double dt = t - latest_time;
    latest_time = t;
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;
    std::queue<sensor_msgs::msg::Imu::ConstSharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::ConstSharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
        std::pair<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr,plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> >>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
            std::pair<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr,plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> >> measurements;
    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty() || linefeature_buf.empty())
            return measurements;
        if (!(rclcpp::Time(imu_buf.back()->header.stamp) > rclcpp::Time(feature_buf.front()->header.stamp)))
        {
            std::cout << "wait for imu, only should happen at the beginning\n";
            sum_of_wait++;
            return measurements;
        }
        if (!(rclcpp::Time(imu_buf.front()->header.stamp) < rclcpp::Time(feature_buf.front()->header.stamp)))
        {
            std::cout << "throw img, only should happen at the beginning\n";
            feature_buf.pop();
            linefeature_buf.pop();
            continue;
        }
        plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr img_msg = feature_buf.front();
        feature_buf.pop();
        plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr linefeature_msg = linefeature_buf.front();
        linefeature_buf.pop();
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp) <= rclcpp::Time(img_msg->header.stamp))
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        measurements.emplace_back(IMUs, std::make_pair(img_msg,linefeature_msg) );
    }
    return measurements;
}

class VINSEstimatorNode : public rclcpp::Node
{
public:
    VINSEstimatorNode() : Node("vins_estimator")
    {
        readParameters(this);
        estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
        RCLCPP_DEBUG(this->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
        RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");
        registerPub(this);

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 2000, std::bind(&VINSEstimatorNode::imu_callback, this, std::placeholders::_1));
        sub_feature_ = this->create_subscription<plvins_interfaces::msg::PlvinsCloud>(
            "/feature", 2000, std::bind(&VINSEstimatorNode::feature_callback, this, std::placeholders::_1));
        sub_linefeature_ = this->create_subscription<plvins_interfaces::msg::PlvinsCloud>(
            "/linefeature", 2000, std::bind(&VINSEstimatorNode::linefeature_callback, this, std::placeholders::_1));

        running_ = true;
        measurement_thread_ = std::thread(&VINSEstimatorNode::process, this);
    }

    ~VINSEstimatorNode()
    {
        running_ = false;
        con.notify_all();
        if (measurement_thread_.joinable())
            measurement_thread_.join();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<plvins_interfaces::msg::PlvinsCloud>::SharedPtr sub_feature_;
    rclcpp::Subscription<plvins_interfaces::msg::PlvinsCloud>::SharedPtr sub_linefeature_;
    std::thread measurement_thread_;
    std::atomic<bool> running_{false};

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        m_buf.lock();
        imu_buf.push(imu_msg);
        m_buf.unlock();
        con.notify_one();
        {
            std::lock_guard<std::mutex> lg(m_state);
            predict(imu_msg);
            std_msgs::msg::Header header = imu_msg->header;
            header.frame_id = "world";
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
        }
    }

    void feature_callback(const plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr &feature_msg)
    {
        m_buf.lock();
        feature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
    }

    void linefeature_callback(const plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr &feature_msg)
    {
        m_buf.lock();
        linefeature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
    }

    void send_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        double t = rclcpp::Time(imu_msg->header.stamp).seconds();
        if (current_time < 0)
            current_time = t;
        double dt = t - current_time;
        current_time = t;
        double ba[]{0.0, 0.0, 0.0};
        double bg[]{0.0, 0.0, 0.0};
        double dx = imu_msg->linear_acceleration.x - ba[0];
        double dy = imu_msg->linear_acceleration.y - ba[1];
        double dz = imu_msg->linear_acceleration.z - ba[2];
        double rx = imu_msg->angular_velocity.x - bg[0];
        double ry = imu_msg->angular_velocity.y - bg[1];
        double rz = imu_msg->angular_velocity.z - bg[2];
        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
    }

    void process()
    {
        while (running_)
        {
            std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>,
                    std::pair<plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr,plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr> >> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&] {
                return !running_ || (measurements = getMeasurements()).size() != 0;
            });
            if (!running_) break;
            lk.unlock();

            for (auto &measurement : measurements)
            {
                for (auto &imu_msg : measurement.first)
                    send_imu(imu_msg);

                plvins_interfaces::msg::PlvinsCloud::ConstSharedPtr relo_msg = NULL;
                while (!relo_buf.empty())
                {
                    relo_msg = relo_buf.front();
                    relo_buf.pop();
                }
                if (relo_msg != NULL)
                {
                    std::vector<Vector3d> match_points;
                    double frame_stamp = rclcpp::Time(relo_msg->header.stamp).seconds();
                    for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                    {
                        Vector3d u_v_id;
                        u_v_id.x() = relo_msg->points[i].x;
                        u_v_id.y() = relo_msg->points[i].y;
                        u_v_id.z() = relo_msg->points[i].z;
                        match_points.push_back(u_v_id);
                    }
                    Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                    Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                    Matrix3d relo_r = relo_q.toRotationMatrix();
                    int frame_index;
                    frame_index = relo_msg->channels[0].values[7];
                    estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
                }

                auto point_and_line_msg = measurement.second;
                auto img_msg = point_and_line_msg.first;
                auto line_msg = point_and_line_msg.second;

                TicToc t_s;
                std::map<int, std::vector<std::pair<int, Vector3d>>> image;
                std::map<int, std::vector<std::pair<int, Vector4d>>> image1;
                std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 5, 1>>>> image2;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    assert(z == 1);
                    image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
                    image1[feature_id].emplace_back(camera_id, Vector4d(x, y, z, 0));
                    Eigen::Matrix<double, 5, 1> xyz_uv;
                    xyz_uv << x, y, z, p_u, p_v;
                    image2[feature_id].emplace_back(camera_id, xyz_uv);
                }
                std::map<int, std::vector<std::pair<int, Vector4d>>> lines;
                for (unsigned int i = 0; i < line_msg->points.size(); i++)
                {
                    int v = line_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x_startpoint = line_msg->points[i].x;
                    double y_startpoint = line_msg->points[i].y;
                    double x_endpoint = line_msg->channels[1].values[i];
                    double y_endpoint = line_msg->channels[2].values[i];
                    lines[feature_id].emplace_back(camera_id, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));
                }
                estimator.processImage(image2, lines, img_msg->header);

                double whole_t = t_s.toc();
                printStatistics(estimator, whole_t);
                std_msgs::msg::Header header = img_msg->header;
                header.frame_id = "world";
                cur_header = header;
                m_loop_drift.lock();
                pubOdometry(estimator, header, relocalize_t, relocalize_r);
                pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
                pubCameraPose(estimator, header, relocalize_t, relocalize_r);
                pubLinesCloud(estimator, header, relocalize_t, relocalize_r);
                pubPointCloud(estimator, header, relocalize_t, relocalize_r);
                pubTF(estimator, header, relocalize_t, relocalize_r);
                pubKeyframe(estimator);
                if (relo_msg != NULL)
                    pubRelocalization(estimator);
                m_loop_drift.unlock();
            }
            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state.unlock();
            m_buf.unlock();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VINSEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
