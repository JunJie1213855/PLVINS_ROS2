# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build a single package
colcon build --symlink-install --packages-select <package_name>

# Source the workspace after building
source install/setup.bash
```

All packages use `ament_cmake`, C++17 (`-O3 -Wall -g`), and depend on OpenCV, Eigen3, and Ceres Solver. The `plvins_interfaces` package (custom ROS2 messages) must be built first since all other packages depend on it.

## Architecture

This is **PL-VINS**: a point-and-line visual-inertial SLAM system ported to ROS2 Humble. It extends VINS-Mono by adding line feature tracking and loop closure with pose graph optimization.

### Data Flow Pipeline

```
Camera images ──┬──► feature_tracker (KLT point tracking)
                │        │
                │        ▼  /feature_tracker/feature (PlvinsCloud)
                │
                └──► LineFeature_tracker (LSD line detection)
                         │
                         ▼  /linefeature_tracker/linefeature (PlvinsCloud)

IMU messages ────────────────────────────────────┐
                                                  ▼
                      plvins_estimator (sliding-window VIO with Ceres)
                         │
                         ▼  /vins_estimator/odometry, /vins_estimator/key_poses, etc.
                         │
                      pose_graph (DBoW loop closure + pose graph optimization)
                         │
                         ▼  /pose_graph/_path, relocalization
```

### Package Map

| Package | Purpose |
|---------|---------|
| `plvins_interfaces` | Custom ROS2 messages: `PlvinsCloud` (point array + float channels), `ChannelFloat32` |
| `camera_model` | camodocal library — Pinhole, Catadioptric (MEI), Equidistant camera models with Ceres-based calibration |
| `feature_tracker` | Two executables: `feature_tracker` (KLT optical flow point tracking) and `LineFeature_tracker` (LSD line segment detection with custom binary descriptors) |
| `plvins_estimator` | Core VIO node. IMU pre-integration, visual-inertial bundle adjustment, marginalization, loop closure interface, line factor support. Built as a shared lib (`vinsEstimatorLib`) + executable |
| `pose_graph` | Global pose graph optimization. DBoW2-based loop detection, keyframe database, 4-DoF pose graph optimization with Ceres, relocalization |
| `image_node_b` | Debug visualization — synchronizes point features, line features, and raw images via `message_filters::Synchronizer`, renders overlays |
| `sim_data_pub` | Dataset playback nodes: IMU publisher, mono/stereo feature publishers, Euroc bag converter (Python) |
| `benchmark_publisher` | Publishes ground-truth trajectory from EuRoC-style CSV for accuracy evaluation |

### Estimator Internals (`vins_estimator/src/`)

- **`estimator.cpp`** — Main VIO pipeline: `processIMU()` → `processImage()` → `initialStructure()` → `solveOdometry()` → `optimization()`/`optimizationwithLine()` → `slideWindow()`
- **`factor/`** — Ceres residual blocks: `imu_factor.h` (IMU pre-integration), `projection_factor.h` (point reprojection), `line_projection_factor.h` (line re-projection with 4-DoF parameterization), `marginalization_factor.h` (Schur complement), `pose_local_parameterization.h`
- **`initial/`** — Bootstrapping: `solve_5pts.h` (5-point algorithm), `initial_sfm.h`, `initial_alignment.h` (visual-inertial alignment), `initial_ex_rotation.h` (gyro bias + extrinsic rotation calibration)
- **`loop-closure/`** — DBoW2 vocabulary tree for BRIEF descriptors, `loop_closure.h` (detection + relative pose computation), `keyframe.h`, `keyframe_database.h`
- **`utility/`** — Camera pose visualization markers, line geometry helpers, timing utilities

### Key Constants (defined in `vins_estimator/src/parameters.h`)

- `WINDOW_SIZE = 10` — sliding window frames
- `NUM_OF_CAM = 1` — monocular (stereo uses `STEREO_TRACK` flag in feature_tracker)
- `SIZE_LINE = 4` — 4-DoF line parameterization (orthonormal representation)

## Launch

```bash
# Full PL-VINS pipeline (feature tracking + VIO + pose graph + loop closure)
ros2 launch plvins_estimator plvins.launch.py

# VIO only, no loop closure (EuRoC, fixed extrinsic)
ros2 launch plvins_estimator euroc_fix_extrinsic.launch.py

# Override config and workspace paths
ros2 launch plvins_estimator plvins.launch.py config_path:=/path/to/config.yaml vins_path:=/path/to/src
```

## Configuration

All parameters live in YAML files under `src/config/`. Three dataset presets exist:

- **EuRoC MAV** (`config/euroc/`) — `euroc_config.yaml` (estimate extrinsic), `euroc_config_fix_extrinsic.yaml`, `loop.yaml` (with loop closure + pose graph save)
- **PennCOSYVIO** (`config/pennCOSYVIO/`)
- **Simulation** (`config/simdata/`) — mono and stereo variants

Key parameters in each config: camera intrinsics, IMU-Camera extrinsic (`estimate_extrinsic: 0/1/2`), IMU noise values (`acc_n`, `gyr_n`, `acc_w`, `gyr_w`), feature tracking thresholds, optimization limits (`max_solver_time`, `max_num_iterations`), loop closure settings.

The parameter system uses global variables read from YAML in `readParameters()`; config paths are relative to the `vins_folder` ROS parameter (defaults to `/home/ros/rosws/PL_VINS_ros2_ws/src`).

## Custom ROS2 Messages

- **`PlvinsCloud`** — Header + `geometry_msgs/Point32[]` points + `ChannelFloat32[]` channels. Used to transmit tracked features with metadata (feature ID, pixel coordinates) packed in channels.
- **`ChannelFloat32`** — Named float array. Channels encode different metadata: channel[0] = feature ID + camera ID, channel[1-2] = pixel u,v, channel[3-6] = endpoint coordinates (lines).

## Known Issues (from learned project skills)

- **Pangolin headless hang**: Pangolin-based processes won't exit after Ctrl+C in headless environments. This repo does NOT use Pangolin, but the `plvins_estimator` uses a custom processing thread that must be joined on shutdown.
- **ROS2 shared_ptr double-free**: Avoid storing `shared_ptr<Node>` in global/static variables; the node destructor races with the ROS2 executor shutdown.
- **Conda Python**: If using conda, set `PYTHONPATH` / `LD_LIBRARY_PATH` to avoid import conflicts with system ROS2 Python packages.

## Coding Conventions

- C++17, Eigen3 for linear algebra, OpenCV `cv::Mat` for images, `std::vector` for containers
- `TicToc` utility class (in `utility/tic_toc.h`) for performance timing — used pervasively
- Mutex-guarded global queues for inter-thread communication (e.g., `imu_buf`, `feature_buf` in estimator_node)
- Parameters are global variables populated by `readParameters(rclcpp::Node*)`; config values are relative to `VINS_FOLDER_PATH`
- Coordinate frames: IMU world frame is gravity-aligned; camera-to-IMU extrinsic is `imu^R_cam`, `imu^T_cam`
- Line features use Plücker coordinates with orthonormal 4-DoF parameterization for optimization
