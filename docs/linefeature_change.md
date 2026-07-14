# 线特征检测替换指南

## 函数结构

`src/feature_tracker/src/linefeature_tracker.cpp` 的 `readImage()` 包含 5 个阶段：

```
readImage()  (第411行)
│
├─ [阶段1] 第413-443行: 图像预处理
│    undistort → equalize → 初始化 forwframe_/curframe_
│    ★ 不需要修改
│
├─ [阶段2] 第444-498行: LSD 线检测
│    第445行: Ptr<LSDDetectorC> lsd_ = ...createLSDDetectorC();
│    第447-466行: opts 参数（log_eps, density_th, min_length...）
│    第470行: lsd_->detect(img, lsd, 2, 1, opts);
│    输出: std::vector<KeyLine> lsd  ← OpenCV 线格式
│    ★ 主要替换点
│
├─ [阶段3] 第482-512行: LBD 描述子提取 + 过滤
│    第488行: bd_->compute(img, lsd, lbd_descr);
│    第491-498行: 过滤 octave==0 && lineLength >= 60 → keylsd
│    第507-512行: 首帧分配 ID，非首帧 ID = -1
│    ★ 如果用方案B也需替换
│
├─ [阶段4] 第525-670行: LBD 匹配 + ID 跟踪
│    第531-532行: BinaryDescriptorMatcher 前后帧匹配
│    第543行: distance < 30 过滤
│    第551行: 几何验证（端点 < 200px, 角度 < 0.1rad）
│    第593-665行: 按水平/垂直线分类，各保留 35 条
│    第559-562行: 匹配成功的继承旧 ID，新线分配 allfeature_cnt++
│    ★ 如果用方案B也需替换
│
├─ [阶段5] 第673-682行: KeyLine → Line 转换
│    将 OpenCV KeyLine 转为自定义 Line 结构
│    forwframe_->vecLine ← 最终输出
│    ★ 如果用方案B可跳过此转换
│
└─ 第682行: curframe_ = forwframe_;  // 当前帧成为下一帧的参考帧
```

## 输出格式（必须满足）

`linefeature_tracker_node.cpp` 的 `img_callback()` 从 `trackerData.curframe_` 读取结果：

```cpp
// 线段端点（像素坐标）
auto &ids = trackerData.curframe_->lineID;                    // vector<int>
auto un_lines = trackerData.undistortedLineEndPoints();        // vector<Line>

// Line 结构 (linefeature_tracker.h:25)
struct Line {
    Point2f StartPt;   // 起点（像素坐标）
    Point2f EndPt;     // 终点（像素坐标）
    float lineWidth;
    Point2f Center;    // 中心点
    Point2f unitDir;   // 单位方向
    float length;      // 长度（像素）
    float theta;       // 角度
    // ... 其他字段
};
```

最终发布到 `/linefeature` topic 的 `PlvinsCloud` 消息格式：
```
points[i].x     = 起点 x（归一化坐标，即 (StartPt.x - cx) / fx）
points[i].y     = 起点 y（归一化坐标）
points[i].z     = 1
channels[0].values[i] = feature_id * NUM_OF_CAM + camera_id
channels[1].values[i] = 终点 x（归一化坐标）
channels[2].values[i] = 终点 y（归一化坐标）
```

`undistortedLineEndPoints()` (第20行) 已包含去畸变 + 归一化逻辑，新检测器仍需调用它。

## 方案 A：只换检测器（最小改动）

**适用场景：** 替换 LSD 为其他线检测算法（如 EDLines、FLD、M-LSD），但保留 LBD 描述子和匹配逻辑。

**修改范围：** 仅替换第 470 行。

### 步骤

1. 在 `linefeature_tracker.h` 中添加新检测器方法声明：
```cpp
// 在 LineFeatureTracker 类中添加
std::vector<KeyLine> detectWithNewMethod(const cv::Mat& img);
```

2. 在 `linefeature_tracker.cpp` 中实现：
```cpp
std::vector<KeyLine> LineFeatureTracker::detectWithNewMethod(const cv::Mat& img) {
    std::vector<KeyLine> lines;
    // 你的检测逻辑
    // 确保输出 KeyLine 格式（包含 startPointX/Y, endPointX/Y, lineLength, angle, octave）
    return lines;
}
```

3. 修改第 470 行：
```cpp
// 替换前：
lsd_->detect(img, lsd, 2, 1, opts);

// 替换后：
lsd = detectWithNewMethod(img);
```

4. 重新编译：
```bash
colcon build --symlink-install --packages-select feature_tracker
```

**优点：** 改动最小，LBD 匹配 + ID 管理全部保留，向后兼容好。
**缺点：** 仍受 LBD 描述子质量影响，匹配逻辑未升级。

## 方案 B：替换整个检测+匹配管线

**适用场景：** 用学习型方法（如 SOLD2、HAWPv3、PLNet）一次性完成检测和匹配。

**修改范围：** 替换第 444-670 行（阶段2+3+4）。

### 步骤

1. 在 `linefeature_tracker.h` 中添加方法声明：
```cpp
void detectAndTrackWithNewMethod(const cv::Mat& img);
```

2. 重写 `readImage()`：
```cpp
void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    frame_cnt++;
    cv::remap(_img, img, undist_map1_, undist_map2_, cv::INTER_LINEAR);

    if (EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    } else {
        forwframe_.reset(new FrameLines);
        forwframe_->img = img;
    }

    // 调用新检测器
    TicToc t_detect;
    detectAndTrackWithNewMethod(img);
    RCLCPP_INFO(rclcpp::get_logger("linefeature_tracker"),
        "new line detect+track costs: %fms", t_detect.toc());

    curframe_ = forwframe_;
}
```

3. `detectAndTrackWithNewMethod` 伪代码：
```cpp
void LineFeatureTracker::detectAndTrackWithNewMethod(const cv::Mat& img) {
    // 1. 检测线段
    auto detected = your_detector.detect(img);

    // 2. 转为 Line 结构
    for (auto& dl : detected) {
        Line l;
        l.StartPt = cv::Point2f(dl.x1, dl.y1);
        l.EndPt   = cv::Point2f(dl.x2, dl.y2);
        l.length  = sqrt((dl.x2-dl.x1)*(dl.x2-dl.x1) + (dl.y2-dl.y1)*(dl.y2-dl.y1));
        l.Center  = (l.StartPt + l.EndPt) * 0.5;
        forwframe_->vecLine.push_back(l);
    }

    // 3. 匹配 + ID 管理
    if (curframe_->vecLine.empty()) {
        for (size_t i = 0; i < forwframe_->vecLine.size(); i++)
            forwframe_->lineID.push_back(allfeature_cnt++);
    } else {
        for (size_t i = 0; i < forwframe_->vecLine.size(); i++) {
            int mid = matchWithPrevious(forwframe_->vecLine[i], curframe_->vecLine);
            if (mid >= 0)
                forwframe_->lineID.push_back(mid);
            else
                forwframe_->lineID.push_back(allfeature_cnt++);
        }
    }
}
```

**优点：** 可以利用端到端学习方法，检测和匹配质量更高；代码更简洁。
**缺点：** 改动量大，需要处理 ID 一致性（跨帧的 feature_id 必须连贯）。

## 验证方法

### 1. 格式验证（模拟数据集）

```bash
# 终端1: 发布预计算特征
ros2 run sim_data_pub pub_feature --ros-args \
  -p sim_file_path:=src/config/simdata/data/ \
  -r /feature_tracker/feature:=/feature \
  -r /linefeature_tracker/linefeature:=/linefeature

# 终端2: estimator 检查能否正确解包
ros2 run plvins_estimator plvins_estimator --ros-args \
  -p config_file:=src/config/simdata/simdata_config.yaml \
  -p vins_folder:=$(pwd)/src
```

### 2. 可视化验证（真实数据）

```bash
ros2 launch plvins_estimator plvins.launch.py
# 在 RViz 中检查 /linefeature_img 是否有线段绘制
```

### 3. 精度验证

```bash
./eval_euroc.sh Trajactory/tum_fast_no_loop.txt ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0/
```

## 不需要修改的文件

以下文件与线检测算法无关，**不需要修改**：

- `src/vins_estimator/src/estimator_node.cpp` — 只订阅 `/linefeature` 话题
- `src/vins_estimator/src/estimator.cpp` — `processImage()` 不变
- `src/vins_estimator/src/factor/line_projection_factor.cpp` — Ceres 线重投影因子不变
- `src/plvins_interfaces/msg/PlvinsCloud.msg` — 消息定义不变
- `src/vins_estimator/launch/plvins.launch.py` — 如果节点名不变则不需改
