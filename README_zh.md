# FAST-Calib MID360 + Hikvision

[English](README.md)

面向 Livox MID360 激光雷达与海康工业相机的 ROS 2 外参标定工具。

本项目提供一套可复用的静态标定流程：

- 采集一张海康相机图像和一段 MID360 `PointCloud2`；
- 运行 FAST-Calib，提取相机观测并累计静态雷达点云；
- 在 RViz2 中显示静态点云和四个可交互小球；
- 使用小球标记标定板四个孔的大致位置；
- 保存标记结果并继续计算相机与雷达外参。

本仓库按独立开源项目维护。使用时应加载本工作区的
`install/setup.bash`，不应依赖其他业务项目的 install 空间。

## 硬件与默认配置

- Ubuntu 22.04 + ROS 2 Humble
- Livox MID360
- 安装海康 MVS SDK 的 USB/GigE 工业相机
- 四孔 ArUco 标定板

仓库默认标定板参数：

- ArUco 字典：`DICT_4X4_50`
- ArUco ID：`[0, 1, 3, 2]`
- 四孔中心间距：`0.500 m × 0.400 m`
- 孔半径：`0.120 m`
- LiDAR topic：`/livox/lidar`
- LiDAR frame：`livox_frame`

## 依赖安装

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rosbag2 \
  ros-humble-interactive-markers \
  python3-colcon-common-extensions \
  python3-yaml \
  libopencv-dev \
  libpcl-dev
```

还需要安装：

- 海康 MVS SDK，默认位于 `/opt/MVS`
- Livox-SDK2，提供 `livox_lidar_sdk_shared`
- `livox_ros_driver2`，安装到系统或与本项目放在同一个 ROS 2 工作区构建

推荐工作区结构：

```text
calib_ws/
  src/
    FAST-Calib/
    livox_ros_driver2/
```

构建：

```bash
cd ~/calib_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果直接在本仓库根目录构建：

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 配置文件

- `config/livox_mid360_fast_calib.json`：MID360 IP、主机地址和端口
- `config/camera_params_hikvision_20260814.yaml`：海康相机原始内参结果
- `config/qr_params.yaml`：当前内参、标定板参数、ROI 和默认输入输出路径

启动 MID360 PointCloud2：

```bash
ros2 launch fast_calib mid360_pointcloud2_launch.py
```

## 现场交互式标定

标准工作区运行方式：

```bash
cd ~/calib_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/FAST-Calib
./scripts/interactive_calibration_workflow.sh scene_001 25
```

直接从仓库根目录运行：

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/interactive_calibration_workflow.sh scene_001 25
```

脚本会依次执行：

1. 抓取一张海康相机图像；
2. 录制指定时长的 MID360 bag；
3. 生成 `config/qr_params_<scene>.yaml`；
4. 生成累计静态点云 `output/<scene>/filtered_cloud.ply`；
5. 启动四球交互编辑器；
6. 打开 RViz2；
7. 保存四个孔的大致位置；
8. 继续执行后续标定。

RViz2 示例：

![RViz2 四球交互](docs/assets/rviz_interactive_hole_spheres.png)

![RViz2 局部视图](docs/assets/rviz_interactive_closeup.png)

![RViz2 小球调整](docs/assets/rviz_interactive_adjustment.png)

在 RViz2 中选择 `Interact` 工具，然后拖动四个彩色小球。完成后在另一个终端保存：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {}
```

返回主流程终端按 Enter。

生成文件：

```text
calib_data/<scene>/image.png
calib_data/<scene>/lidar_bag/
config/qr_params_<scene>.yaml
output/<scene>/filtered_cloud.ply
output/<scene>/manual_lidar_holes.yaml
output/<scene>_manual_four_holes/calib_result.txt
```

## 从相机和雷达合并 Rosbag 离线标定

静态标定可以将相机和雷达一起录进同一个 ROS 2 bag，撤掉标定板后再离线处理。录制期间相机、雷达和标定板必须保持静止。

推荐使用一键录包脚本。它会清理旧的标定发布进程、启动 MID360 和海康相机、
检查两个 topic 确实有消息，然后只录制这两个 topic：

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/record_calibration_bag.sh ~/calibration_bags/scene_001
```

开始录制后按一次 Ctrl+C。脚本会自动：

1. 等待 rosbag 刷新缓存并写入 metadata；
2. 停止 MID360 和海康相机发布节点；
3. 检查两个 topic 的消息数量不为 0；
4. 从 bag 解码一张相机图验证图像有效；
5. 将验证结果写入 `<bag_path>_verification/`。

bag 必须包含：

- 一个 `sensor_msgs/msg/PointCloud2` 类型的雷达 topic；
- 一个 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage` 类型的相机 topic。

默认相机参数为 `1440×1080`、曝光 `30000 us`、增益 `8`、发布频率 1 Hz；
MID360 PointCloud2 通常约为 10 Hz。

导入已录制的合并 bag：

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/interactive_calibration_from_bag.sh \
  offline_scene_001 \
  ~/calibration_bags/scene_001 \
  /camera/image_raw \
  /livox/lidar
```

脚本会：

1. 将 bag 硬链接或复制到场景目录；
2. 从相机 topic 自动选择时间中部的一帧；
3. 保存为 `calib_data/<scene>/image.png`；
4. 生成场景 YAML；
5. 累计静态点云；
6. 打开相同的 RViz 四球流程。

相机 topic 可以连续录制，但当前静态标定算法最终只使用其中一张图像。

一键脚本会使用本项目新增的 `hikvision_image_publisher` 发布
`/camera/image_raw`，不需要另外启动相机节点。

如果只想导入和检查数据，不启动处理与 RViz：

```bash
PREPARE_ONLY=1 ./scripts/interactive_calibration_from_bag.sh \
  offline_scene_001 \
  ~/calibration_bags/scene_001 \
  /camera/image_raw \
  /livox/lidar
```

## 处理已有场景

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_fast_calib_scene.sh config/qr_params_<scene>.yaml
```

已准备好场景配置和数据时，也可以直接进入离线交互流程：

```bash
./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

## 常见问题

如果遇到：

```text
undefined symbol: libusb_set_option
```

说明 MVS SDK 的旧版 libusb 排在系统库之前。运行 FAST-Calib 时需要从
`LD_LIBRARY_PATH` 中过滤 `/opt/MVS/lib`。

如果 RViz2 能看到点云但小球不能拖动：

- Display 必须是 `rviz_default_plugins/InteractiveMarkers`；
- `Interactive Markers Namespace` 必须是 `/manual_lidar_holes`；
- 当前工具必须选择 `Interact`。

## 标定记录

`calibration_record/` 保存了首次成功标定的设备配置、问题记录和参考结果：

- `calibration_record/interactive_workflow.md`
- `calibration_record/device_config.md`
- `calibration_record/pitfalls_and_solutions.md`
- `calibration_record/final_result_20260617.md`

历史参考结果：

```text
output/final_success_20260617/calib_result.txt
```
