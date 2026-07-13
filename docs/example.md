# PL-VINS 使用示例

## 1. 构建

```bash
cd /home/ros/rosws/PL_VINS_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2. 发布 EuRoC 数据集

```bash
# 终端1：发布图像和 IMU
python3 src/sim_data_pub/scripts/euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0

# 可选参数
#   --speed 2.0   2倍速播放
#   --stereo      双目模式
```

## 3. 运行 PL-VINS

### 3.1 仅 VIO（无回环）

```bash
ros2 launch plvins_estimator euroc_fix_extrinsic.launch.py
```

### 3.2 VIO + 回环检测（推荐）

```bash
ros2 launch plvins_estimator plvins.launch.py
```

回环要求：config 中 `loop_closure: 1`，且 `support_files/` 下有 `brief_k10L6.bin` 和 `brief_pattern.yml`。

### 3.3 自定义参数

```bash
ros2 launch plvins_estimator plvins.launch.py \
    config_path:=/path/to/my_config.yaml \
    vins_path:=/home/ros/rosws/PL_VINS_ros2_ws/src
```

## 4. 轨迹评估

运行结束后，轨迹文件保存在 `Trajactory/` 目录下：

| 文件 | 格式 |
|------|------|
| `tum_fast_no_loop.txt` | 纳秒时间戳 + `x y z qw qx qy qz` |
| `evo_fast_no_loop.txt` | 秒时间戳 + `x y z qx qy qz qw` |

### 4.1 使用 eval_euroc.sh 评估

```bash
# 使用 TUM 轨迹（纳秒时间戳）
./eval_euroc.sh Trajactory/tum_fast_no_loop.txt ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0/ --plot

# 使用 EVO 轨迹（秒时间戳）
./eval_euroc.sh Trajactory/evo_fast_no_loop.txt ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0/ --plot

# 不加 --plot 可避免 matplotlib 版本冲突问题
./eval_euroc.sh Trajactory/tum_fast_no_loop.txt ~/dataset/Euroc/vicon_room1/V1_02_medium/mav0/
```

### 4.2 手动 evo 评估

```bash
# 先转换轨迹（以 TUM 格式为例）
python3 -c "
with open('Trajactory/tum_fast_no_loop.txt') as fin, open('est.tum','w') as fout:
    for line in fin:
        cols = line.strip().split()
        ts = float(cols[0]) / 1e9  # 纳秒 → 秒
        fout.write(f'{ts:.9f} {cols[1]} {cols[2]} {cols[3]} {cols[5]} {cols[6]} {cols[7]} {cols[4]}\n')
"

# 评估
evo_ape tum <groundtruth.tum> est.tum -va --plot
```

### 4.3 evo 常用命令

```bash
# 绝对位姿误差
evo_ape tum gt.tum est.tum -va --plot

# 相对位姿误差
evo_rpe tum gt.tum est.tum -va --plot

# 多轨迹对比
evo_traj tum traj1.tum traj2.tum --ref=gt.tum -p --plot_mode=xyz
```

## 5. 调整线特征质量

编辑 `src/feature_tracker/src/linefeature_tracker.cpp` 的 `readImage()` 函数，关键参数：

| 行号 | 参数 | 默认 | 建议 |
|------|------|------|------|
| 453 | `opts.log_eps` | `1.0` | `2.0`（提高置信度阈值） |
| 454 | `opts.density_th` | `0.6` | `0.7`（提高密度要求） |
| 456 | `min_line_length` | `0.125` | `0.15`（滤除短线） |
| 493 | `lineLength >= 60` | `60` | `80`（硬过滤最短线） |

修改后重新构建：
```bash
colcon build --symlink-install --packages-select feature_tracker
```

## 6. 结果示例

V1_01_easy 序列（无回环）：
```
APE RMSE:   0.068 m
APE Mean:   0.059 m
APE Median: 0.051 m
APE Max:    0.159 m
```
