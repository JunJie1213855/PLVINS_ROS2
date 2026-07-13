#!/usr/bin/env python3
"""
EuRoC Dataset Publisher for ORB_SLAM3 ROS2

从 EuRoC MAV 数据集发布图像和 IMU 数据到 ROS2 topics。

用法:
    # 终端1: 先启动 SLAM (以 mono-inertial 为例)
    source install/setup.sh
    ros2 run orbslam3 mono-inertial \
        ~/lib/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt \
        ~/lib/SLAM/ORB_SLAM3/Examples/Monocular-Inertial/EuRoC.yaml

    # 终端2: 发布数据集
    python3 scripts/publish_euroc.py ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0

发布的 topics:
    /camera  — sensor_msgs/Image (单目图像)
    /imu     — sensor_msgs/Imu

选项:
    --speed N    播放速度倍率 (默认 1.0 实时)
    --stereo     双目模式: 发布 camera/left 和 camera/right (配合 stereo/stereo-inertial 使用)
"""

import os
import sys
import csv
import time
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from builtin_interfaces.msg import Time


class EurocPublisher(Node):
    def __init__(self, dataset_dir: str, speed: float = 1.0, stereo: bool = False):
        super().__init__('euroc_publisher')

        self.dataset_dir = Path(dataset_dir)
        self.speed = speed
        self.stereo = stereo
        self.bridge = CvBridge()

        # 创建发布者
        if stereo:
            self.pub_left = self.create_publisher(Image, 'camera/left', 10)
            self.pub_right = self.create_publisher(Image, 'camera/right', 10)
        else:
            self.pub_image = self.create_publisher(Image, 'camera', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu', 1000)

        # 加载数据索引
        self.cam0_data = self._load_csv('cam0')
        self.imu_data = self._load_csv('imu0')
        if stereo:
            self.cam1_data = self._load_csv('cam1')

        self.get_logger().info(
            f'已加载 cam0: {len(self.cam0_data)} 帧, imu0: {len(self.imu_data)} 条')

    def _load_csv(self, sensor: str) -> list:
        """加载 EuRoC 传感器 CSV 文件"""
        csv_path = self.dataset_dir / sensor / 'data.csv'
        rows = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row[0].startswith('#'):
                    continue
                rows.append(row)
        return rows

    def _nanosec_to_ros_time(self, ts_ns: int) -> Time:
        """纳秒时间戳 → ROS2 Time"""
        t = Time()
        t.sec = ts_ns // 1_000_000_000
        t.nanosec = ts_ns % 1_000_000_000
        return t

    def publish(self):
        """按时间顺序交错发布图像和 IMU 数据（实时播放）"""
        # 合并图像和 IMU 事件，按时间戳排序
        events = []

        for row in self.cam0_data:
            ts_ns = int(row[0])
            events.append(('cam0', ts_ns, row[1]))

        if self.stereo:
            for row in self.cam1_data:
                ts_ns = int(row[0])
                events.append(('cam1', ts_ns, row[1]))

        for row in self.imu_data:
            ts_ns = int(row[0])
            gyro = (float(row[1]), float(row[2]), float(row[3]))
            accel = (float(row[4]), float(row[5]), float(row[6]))
            events.append(('imu', ts_ns, (gyro, accel)))

        events.sort(key=lambda e: e[1])

        # 实时播放
        t0_ns = events[0][1]
        t0_wall = time.time()
        frame_count = 0

        for evt_type, ts_ns, payload in events:
            # 计算应该等待的时间
            elapsed_dataset = (ts_ns - t0_ns) / 1e9  # 秒
            elapsed_wall = time.time() - t0_wall
            sleep_time = (elapsed_dataset / self.speed) - elapsed_wall

            if sleep_time > 0:
                time.sleep(sleep_time)

            # 发布数据
            if evt_type == 'cam0':
                msg = self._make_image_msg('cam0', payload, ts_ns)
                if self.stereo:
                    self.pub_left.publish(msg)
                else:
                    self.pub_image.publish(msg)
                frame_count += 1
                if frame_count % 100 == 0:
                    self.get_logger().info(f'已发布 {frame_count} 帧')

            elif evt_type == 'cam1' and self.stereo:
                msg = self._make_image_msg('cam1', payload, ts_ns)
                self.pub_right.publish(msg)

            elif evt_type == 'imu':
                msg = self._make_imu_msg(payload, ts_ns)
                self.pub_imu.publish(msg)

        self.get_logger().info(f'播放完成，共发布 {frame_count} 帧图像')

    def _make_image_msg(self, sensor: str, filename: str, ts_ns: int) -> Image:
        """读取图像文件并生成 ROS Image 消息"""
        img_path = self.dataset_dir / sensor / 'data' / filename
        cv_img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)

        if cv_img is None:
            self.get_logger().error(f'无法读取图像: {img_path}')
            return Image()

        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='mono8')
        msg.header.stamp = self._nanosec_to_ros_time(ts_ns)
        msg.header.frame_id = 'camera'
        return msg

    def _make_imu_msg(self, payload: tuple, ts_ns: int) -> Imu:
        """生成 ROS IMU 消息"""
        gyro, accel = payload

        msg = Imu()
        msg.header.stamp = self._nanosec_to_ros_time(ts_ns)
        msg.header.frame_id = 'imu'

        # EuRoC IMU: 角速度 rad/s, 线加速度 m/s²
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        # 协方差矩阵: -1 表示未知
        for i in range(9):
            msg.orientation_covariance[i] = -1.0
            msg.angular_velocity_covariance[i] = -1.0
            msg.linear_acceleration_covariance[i] = -1.0

        return msg


def main():
    parser = argparse.ArgumentParser(description='EuRoC 数据集 ROS2 发布器')
    parser.add_argument('dataset_dir', help='mav0 目录路径')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='播放速度倍率 (默认: 1.0 实时)')
    parser.add_argument('--stereo', action='store_true',
                        help='双目模式: 发布 camera/left 和 camera/right')
    args = parser.parse_args()

    rclpy.init()
    node = EurocPublisher(args.dataset_dir, speed=args.speed, stereo=args.stereo)

    try:
        node.publish()
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
