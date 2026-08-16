# RViz 可视化拖球标定流程

这个流程把标定拆成可操作步骤：

1. 脚本采集 Hikvision 图像和 MID360 点云 bag。
2. FAST-Calib 检查相机是否能识别 ArUco 标定板。
3. 如果相机识别通过并生成 `filtered_cloud.ply`，脚本把累计点云作为静态点云发布。
4. RViz2 显示静态点云和四个可拖动实心球。
5. 用户把四个球拖到标定板四个孔上。
6. 用户保存球心 YAML。
7. 脚本读取这四个 LiDAR 球心，结合相机侧四孔结果出外参。

## 一键入口

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash

scripts/interactive_calibration_workflow.sh <scene_name> 25
```

例如：

```bash
scripts/interactive_calibration_workflow.sh current_static_pair_interactive_01 25
```

脚本会生成：

```text
calib_data/<scene_name>/image.png
calib_data/<scene_name>/lidar_bag/
config/qr_params_<scene_name>.yaml
output/<scene_name>/filtered_cloud.ply
output/<scene_name>/manual_lidar_holes.yaml
output/<scene_name>/manual_lidar_hole_editor.rviz
output/<scene_name>_manual_four_holes/calib_result.txt
```

## RViz 操作

脚本打开 RViz 后：

1. 使用 RViz 顶部的 `Interact` 工具。
2. 拖动四个彩色球，让球心落到四个圆孔中心。
3. 保存球心：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {}
```

4. 回到运行 `interactive_calibration_workflow.sh` 的终端按 Enter。

脚本会继续运行：

```bash
ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file config/qr_params_<scene_name>.yaml \
  -p manual_lidar_centers_path:=output/<scene_name>/manual_lidar_holes.yaml \
  -p output_path:=/home/vision/FAST-Calib/output/<scene_name>_manual_four_holes
```

## 单独启动编辑器

如果已经有 `filtered_cloud.ply`，只想重新拖球：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash

python3 scripts/interactive_lidar_hole_editor.py \
  --cloud output/<scene_name>/filtered_cloud.ply \
  --output output/<scene_name>/manual_lidar_holes.yaml \
  --initial-centers output/<scene_name>/manual_lidar_holes.yaml \
  --rate 0.2
```

另开 RViz：

```bash
rviz2 -d output/<scene_name>/manual_lidar_hole_editor.rviz
```

保存：

```bash
ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {}
```

## 保存文件格式

`manual_lidar_holes.yaml` 格式：

```yaml
frame_id: livox_frame
topic: /static_accumulated_cloud
centers:
- name: upper +Y
  x: 3.57454
  y: 0.07096
  z: 0.11873
- name: upper -Y
  x: 3.5539
  y: -0.4318
  z: 0.14098
- name: lower +Y
  x: 3.54801
  y: 0.061
  z: -0.275
- name: lower -Y
  x: 3.52758
  y: -0.4502
  z: -0.255
```

四个点的顺序不需要非常严格，`manual_lidar_centers_calib` 会复用 FAST-Calib 的排序逻辑。不过建议仍按上面名字摆放，便于人工复核。

## 注意事项

- RDP 下不要播放实时 rosbag；这个流程只发布低频静态点云。
- 如果没有生成 `filtered_cloud.ply`，通常是相机没有识别到 ArUco 或 FAST-Calib 在相机阶段失败。
- RViz 中只需要 `InteractiveMarkers` display；`Interactive Markers Namespace` 必须是 `/manual_lidar_holes`。
- 小球本体是 `MOVE_3D` control。选择工具栏 `Interact` 后，直接拖动小球本体，不需要拖轴。
- 如果运行标定时报 `libusb_set_option`，按 `pitfalls_and_solutions.md` 过滤 `/opt/MVS/lib`。

## 从合并 rosbag 离线进入交互流程

如果一个 ROS 2 bag 同时包含 `/livox/lidar` PointCloud2 和相机 Image/CompressedImage，
可以在标定板撤走后离线提取中间相机帧并进入同一套 RViz 四球流程：

```bash
./scripts/interactive_calibration_from_bag.sh \
  <scene_name> \
  <bag_path> \
  <camera_topic> \
  /livox/lidar
```

当前静态标定最终只使用相机 topic 中的一张图像。录制期间相机、雷达和标定板必须保持静止。

推荐使用一键录制脚本生成合并 bag：

```bash
./scripts/record_calibration_bag.sh ~/calibration_bags/<bag_name>
```

脚本启动并检查 `/livox/lidar` 与 `/camera/image_raw`，按 Ctrl+C 后自动刷盘、
停止两个发布节点、检查消息数量并解码一张相机图。
