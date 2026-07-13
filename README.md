# PL-VINS

点线视觉惯性 SLAM 系统，基于 VINS-Mono 扩展，增加线特征跟踪和回环检测，已移植至 ROS2 Humble。

**参考论文：**
- *PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and  Line Features*(Qiang Fu)

## 环境依赖

| 依赖 | 安装方式 |
|------|---------|
| Ubuntu 22.04 | — |
| ROS2 Humble | `apt install ros-humble-desktop` |
| OpenCV 4.5+ | 随 ROS2 安装 |
| Eigen3 | `apt install libeigen3-dev` |
| Ceres Solver | `apt install libceres-dev` |
| evo（评估工具） | `pip install evo --upgrade` |

**注意：** 如果使用 conda/anaconda Python，需要调整 `PYTHONPATH` / `LD_LIBRARY_PATH` 避免与系统 ROS2 Python 包冲突。

## 编译

```bash
cd /home/ros/rosws/PL_VINS_ros2_ws
source /opt/ros/humble/setup.bash

# 先编译消息包（其他包都依赖它）
colcon build --symlink-install --packages-select plvins_interfaces
source install/setup.bash

# 再编译全部
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 常见编译问题

**1. `plvins_interfaces` 符号链接冲突**

```
failed to create symbolic link ... because existing path cannot be removed: Is a directory
```

解决：`rm -rf build/plvins_interfaces install/plvins_interfaces` 后重新编译。

**2. 并行编译时找不到 `plvins_interfaces`**

```
Could not find a package configuration file provided by "plvins_interfaces"
```

解决：先单独编译 `plvins_interfaces`（见上方命令），再编译全部。

**3. matplotlib 版本冲突（evo 绘图报错）**

```
ImportError: cannot import name 'docstring' from 'matplotlib'
```

解决：`pip uninstall matplotlib -y && sudo apt install python3-matplotlib`

**4. Ctrl+C 异常退出**

```
terminate called without an active exception
```

已修复（添加了线程正确关闭逻辑）。如果仍遇到，确保重新编译了最新代码。

## 运行

详细说明见 [docs/example.md](docs/example.md)，以下为快速参考。

### EuRoC 数据集

```bash
# 终端1：发布数据集
python3 src/sim_data_pub/scripts/euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0

# 终端2：VIO + 回环
ros2 launch plvins_estimator plvins.launch.py
```

### 模拟数据集（测试 VIO 后端）

```bash
# 终端1：IMU
ros2 run sim_data_pub pub_imu --ros-args -p sim_file_path:="$(pwd)/src/config/simdata/data/"

# 终端2：预计算特征 → topic 重映射
ros2 run sim_data_pub pub_feature --ros-args \
  -p sim_file_path:="$(pwd)/src/config/simdata/data/" \
  -r /feature_tracker/feature:=/feature \
  -r /linefeature_tracker/linefeature:=/linefeature

# 终端3：VIO 估计器
ros2 run plvins_estimator plvins_estimator --ros-args \
  -p config_file:="$(pwd)/src/config/simdata/simdata_config.yaml" \
  -p vins_folder:="$(pwd)/src"
```

### 轨迹评估

```bash
./eval_euroc.sh Trajactory/tum_fast_no_loop.txt ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0/
```

## 架构

```
Camera ──┬──► feature_tracker (KLT 点跟踪) ──► /feature
         │
         └──► LineFeature_tracker (LSD 线检测) ──► /linefeature

IMU ────────────────────────────────────────────► /imu0

/feature + /linefeature + /imu0
         │
         ▼
  plvins_estimator (滑窗 VIO, Ceres 优化)
         │
         ▼
  pose_graph (DBoW 回环检测 + 位姿图优化)
```

| 包 | 用途 |
|----|------|
| `plvins_interfaces` | 自定义 ROS2 消息 |
| `camera_model` | 相机模型库（Pinhole, MEI, Equidistant） |
| `feature_tracker` | 点特征（KLT）+ 线特征（LSD）提取 |
| `plvins_estimator` | 核心 VIO：IMU 预积分 + BA + 边缘化 + 回环 |
| `pose_graph` | 全局位姿图优化 |
| `image_node_b` | 可视化：同步显示点/线/图像 |
| `sim_data_pub` | 数据集回放（EuRoC）+ 模拟数据发布 |
| `benchmark_publisher` | 轨迹评估基准发布 |

## 配置

配置文件位于 `src/config/`，三种预设：

- **EuRoC** (`config/euroc/`) — `euroc_config.yaml`、`loop.yaml`
- **PennCOSYVIO** (`config/pennCOSYVIO/`)
- **模拟** (`config/simdata/`)

关键参数：相机模型、IMU-相机外参 (`estimate_extrinsic`)、IMU 噪声、特征跟踪阈值、回环开关 (`loop_closure`)。

## Docker

```bash
# 构建并进入容器
xhost +local:docker
docker compose build
docker compose run plvins bash

# 容器内运行（注意工作区路径变化）
ros2 launch plvins_estimator plvins.launch.py vins_path:=/home/ros/plvins_ws/src
```

详细说明见 [docs/docker.md](docs/docker.md)。
