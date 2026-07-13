# PL-VINS Docker 使用指南

## 文件说明

| 文件 | 用途 |
|------|------|
| `Dockerfile` | 基于 `osrf/ros:humble-desktop-full`，安装 Ceres + evo，编译全部 ROS2 包 |
| `docker-compose.yml` | 编排容器：挂载 X11 显示、数据集（只读）、输出目录 |
| `docker_entrypoint.sh` | 入口脚本：source ROS2 环境后执行命令 |
| `.dockerignore` | 排除 `build/` `install/` `log/` `Trajactory/` 等构建和运行产物 |

## 构建镜像

```bash
cd /home/ros/rosws/PL_VINS_ros2_ws
docker compose build
```

首次构建约 5-10 分钟（安装依赖 + 编译 8 个包）。

## 运行容器

### 带 RViz 图形界面

```bash
# 允许 Docker 访问 X11
xhost +local:docker

# 启动并进入容器
docker compose run plvins bash
```

容器内的 ROS2 环境已自动 source。

### 纯命令行（无 GUI）

```bash
docker run -it --rm \
  -v $(pwd)/Trajactory:/home/ros/plvins_ws/Trajactory \
  plvins:humble bash
```

## 容器内运行 PL-VINS

**注意：** 容器内工作区路径为 `/home/ros/plvins_ws`，需覆盖 `vins_path` 参数。

### EuRoC 数据集

```bash
# 终端1（宿主机）：发布数据集
python3 src/sim_data_pub/scripts/euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0

# 终端2（容器内）：VIO + 回环
ros2 launch plvins_estimator plvins.launch.py vins_path:=/home/ros/plvins_ws/src
```

### 模拟数据集

```bash
# 3 个终端均在容器内
# 终端1
ros2 run sim_data_pub pub_imu --ros-args -p sim_file_path:=/home/ros/plvins_ws/src/config/simdata/data/

# 终端2
ros2 run sim_data_pub pub_feature --ros-args \
  -p sim_file_path:=/home/ros/plvins_ws/src/config/simdata/data/ \
  -r /feature_tracker/feature:=/feature \
  -r /linefeature_tracker/linefeature:=/linefeature

# 终端3
ros2 run plvins_estimator plvins_estimator --ros-args \
  -p config_file:=/home/ros/plvins_ws/src/config/simdata/simdata_config.yaml \
  -p vins_folder:=/home/ros/plvins_ws/src
```

### 轨迹评估

数据集挂载在容器的 `/home/ros/dataset/`（只读）：

```bash
# 容器内
./eval_euroc.sh Trajactory/tum_fast_no_loop.txt /home/ros/dataset/Euroc/vicon_room1/V1_02_medium/mav0/
```

轨迹输出到 `Trajactory/`，宿主机可直接访问（挂载为可写）。

## 挂载说明

`docker-compose.yml` 中的卷挂载：

| 宿主机路径 | 容器内路径 | 权限 | 说明 |
|-----------|-----------|------|------|
| `~/dataset/` | `/home/ros/dataset/` | 只读 | EuRoC 等数据集 |
| `./Trajactory/` | `/home/ros/plvins_ws/Trajactory/` | 读写 | 轨迹输出 |
| `./results/` | `/home/ros/plvins_ws/results/` | 读写 | 评估结果 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | 只读 | RViz 显示 |

如果数据集在其他路径，修改 `docker-compose.yml` 的 volumes 部分，或使用 `docker run -v` 手动挂载。

## 故障排除

**`xhost: unable to open display`**
```bash
# 确认 DISPLAY 变量已设置
echo $DISPLAY
# 如果为空，设置
export DISPLAY=:0
xhost +local:docker
```

**RViz 启动后无画面或闪退**
```bash
# 容器内检查 OpenGL
glxinfo | grep "OpenGL"
# 如使用 NVIDIA 显卡，需要 nvidia-docker2
sudo apt install nvidia-docker2
```

**`docker compose` 命令不存在**
```bash
# 旧版用 docker-compose（带横杠）
docker-compose build
docker-compose run plvins bash
```
