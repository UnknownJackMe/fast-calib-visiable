# RViz 四球粗定位与 LiDAR 孔心自动精修流程

## 1. 流程定义

四个 RViz 小球只用于告诉程序“每个孔大概在哪里”，不是精确孔心。

完整流程：

```text
相机图像 + LiDAR bag
  -> 累计静态点云
  -> RViz 四球粗定位
  -> 保存 rough seeds
  -> 每个 seed 附近局部搜索板面空洞
  -> 圆孔中心自动拟合
  -> 四孔几何验证
  -> RViz 显示绿色 refined centers
  -> 使用 refined centers 计算外参
```

粗 seed 不会直接参与外参计算。外参工具默认拒绝未验证的 seed YAML。

## 2. 现场采集入口

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/interactive_calibration_workflow.sh <scene_name> 25
```

脚本会抓取一张海康图像并录制约 25 秒 MID360 PointCloud2。

## 3. 合并 rosbag 离线入口

录制相机和 LiDAR 合并 bag：

```bash
./scripts/record_calibration_bag.sh ~/calibration_bags/<bag_name>
```

按 Ctrl+C 后脚本自动刷盘、停止传感器、检查消息数量并解码一张相机图。

撤掉标定板后，从合并 bag 进入相同流程：

```bash
./scripts/interactive_calibration_from_bag.sh \
  <scene_name> \
  ~/calibration_bags/<bag_name> \
  /camera/image_raw \
  /livox/lidar
```

## 4. 已有场景入口

如果场景已经包含图像、bag、配置和静态点云：

```bash
./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

## 5. RViz 操作

RViz 打开后：

1. 选择顶部 `Interact` 工具；
2. 将四个彩色球分别拖到对应孔附近；
3. 不要求球心与孔心精确重合；
4. 每个球都有独立的 X/Y/Z 平移手柄，可分别调整前后、左右和上下位置；
5. 雷达或标定板倾斜时，四球不需要保持水平、垂直或相同深度；
6. 标签表示标定板上的孔位对应关系，不表示孔必须沿 LiDAR 坐标轴排列；
7. 确保球没有放到错误孔、板边缘或其他空洞；
8. 回到主流程终端按 Enter。

也可以手动保存 seed：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
ros2 service call /save_lidar_hole_seeds std_srvs/srv/Trigger {}
```

旧服务名 `/save_lidar_hole_markers` 仍保留兼容。

## 6. 自动精修

保存 seed 后，脚本运行：

```bash
python3 scripts/refine_lidar_hole_seeds.py \
  --plane-cloud output/<scene>/plane_cloud.ply \
  --filtered-cloud output/<scene>/filtered_cloud.ply \
  --seeds output/<scene>/manual_lidar_hole_seeds.yaml \
  --config config/hole_refinement_params.yaml \
  --output output/<scene>/refined_lidar_holes.yaml \
  --report output/<scene>/hole_refinement_report.yaml \
  --debug-dir output/<scene>/refinement_debug
```

算法步骤：

1. 使用预分割板面；没有 `plane_cloud.ply` 时从 filtered cloud RANSAC 提取主平面；
2. 将板面展开到二维坐标；
3. 在每个 seed 附近搜索最大空圆；
4. 按角度提取孔边界最近点；
5. 鲁棒拟合圆心和半径；
6. 在多个候选中联合选择最符合 `0.500 × 0.400 m` 四孔几何的组合；
7. 输出质量报告。

精修成功后，RViz 中出现更小的绿色球。用户可以：

- 按 Enter 接受并继续；
- 输入 `r` 返回调整 rough seeds。

精修失败时不会计算外参。用户调整失败 seed 后可以直接重试，不需要重新采集。

## 7. 输出文件

```text
calib_data/<scene>/image.png
calib_data/<scene>/lidar_bag/
config/qr_params_<scene>.yaml
output/<scene>/filtered_cloud.ply
output/<scene>/plane_cloud.ply
output/<scene>/manual_lidar_hole_seeds.yaml
output/<scene>/refined_lidar_holes.yaml
output/<scene>/hole_refinement_report.yaml
output/<scene>/refinement_debug/
output/<scene>_refined_four_holes/calib_result.txt
```

### rough seed 文件

```yaml
kind: rough_lidar_hole_seeds
version: 1
frame_id: livox_frame
centers:
- name: upper +Y
  x: 2.37
  y: 0.20
  z: 1.15
```

### refined center 文件

```yaml
kind: refined_lidar_hole_centers
version: 1
status: pass
frame_id: livox_frame
centers:
- name: upper +Y
  x: 2.212001
  y: 0.208652
  z: 1.204813
  seed: [2.377052, 0.199761, 1.15318]
  seed_distance_m: 0.173167
  fitted_radius_m: 0.10581
  radial_rmse_m: 0.0053
  status: pass
```

## 8. 质量门限

默认参数位于：

```text
config/hole_refinement_params.yaml
```

主要检查：

- 每孔圆半径；
- 径向拟合 RMSE；
- seed 到 refined center 的最大距离；
- 上下两排宽度；
- 左右两列高度；
- 两条对角线。

只有报告 `status: pass` 时才允许外参工具继续。

## 9. 已验证结果

```text
recalib_202260816_02
  粗 seed 直接计算 RMSE：0.025906 m
  自动精修后 RMSE：      0.002541 m
  最大 seed 偏差：        0.1732 m

final_success_20260617
  历史人工结果 RMSE：约   0.004935 m
  自动精修回归 RMSE：     0.002671 m
```

## 10. 注意事项

- 录制期间相机、雷达和标定板必须静止；
- RDP 下使用静态累计点云，不循环播放实时点云；
- 四个 seed 必须分别对应四个不同孔；
- 精修失败时不要通过 legacy 开关强行出最终外参；
- `allow_unvalidated_manual_centers:=true` 只用于显式回放历史数据；
- 如果 PCL 报 `libusb_set_option`，过滤 `/opt/MVS/lib` 中的旧 libusb。
