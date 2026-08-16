# 下次重新标定快速流程

## 1. 构建与环境

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2. 推荐：一键录制相机和 LiDAR 合并 bag

```bash
./scripts/record_calibration_bag.sh \
  /home/vision/calibration_bags/<bag_name>
```

看到两个 topic 检查通过后录制约 25 秒，按一次 Ctrl+C。脚本会自动刷盘、停止传感器并验证 bag。

## 3. 从合并 bag 进入标定

```bash
./scripts/interactive_calibration_from_bag.sh \
  <scene_name> \
  /home/vision/calibration_bags/<bag_name> \
  /camera/image_raw \
  /livox/lidar
```

如果不使用合并 bag，也可以现场抓一张图并录 LiDAR：

```bash
./scripts/interactive_calibration_workflow.sh <scene_name> 25
```

## 4. RViz 任意多边形选择标定板

脚本首先打开完整场景的低分辨率预览：

1. 按 `P` 激活 `标定板多边形选择`，确认鼠标变成十字光标；
2. 不需要点中具体点云点；
3. 按住左键，在标定板外围连续拖动一圈；
4. 松开左键自动完成；右键或 Esc 取消；
5. 检查绿色点云是否覆盖完整标定板；
6. 回到终端按 Enter 接受，或输入 `r` 清除候选并重新拖动套索。

程序使用低分辨率点云交互，但会从高分辨率点云提取最终板面，因此不会因为预览降采样影响孔心精修。

## 5. RViz 四球操作

1. 选择 `Interact`；
2. 将四个彩色球分别拖到对应孔附近；
3. 不需要精确对心，小球只是 rough seeds；
4. 雷达或标定板倾斜时，使用每个球的 X/Y/Z 三轴手柄分别调整前后、左右和上下位置；
5. 四个球互不约束，不要求在 LiDAR 坐标系中保持水平、垂直或相同深度；
6. 回到主终端按 Enter。

程序会自动：

- 在每个 seed 附近搜索板面空洞；
- 拟合真实圆孔中心和半径；
- 验证 `0.500 × 0.400 m` 四孔几何；
- 在 RViz 中显示更小的绿色 refined centers。

精修失败时调整对应 seed 后重试，不需要重新采集。

## 6. 输出

```text
output/<scene>/manual_lidar_hole_seeds.yaml
output/<scene>/static_cloud_full.ply
output/<scene>/static_cloud_preview.ply
output/<scene>/selected_board_cloud.ply
output/<scene>/board_extraction_report.yaml
output/<scene>/refined_lidar_holes.yaml
output/<scene>/hole_refinement_report.yaml
output/<scene>/refinement_debug/
output/<scene>_refined_four_holes/calib_result.txt
```

## 7. 已有场景重新进入

```bash
./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

已有标定板提取结果时默认复用。强制重新框选：

```bash
RESELECT_BOARD=1 ./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

## 8. 验收

- `hole_refinement_report.yaml` 必须为 `status: pass`；
- `board_extraction_report.yaml` 不能为 `status: fail`；
- 外参工具默认拒绝 rough seed 文件；
- 目标 RMSE 小于 `0.008 m`，理想值约 `0.005 m` 或更低；
- 检查 `colored_cloud.ply` 投影效果；
- 当前回归场景已达到 `0.002541 m` 和 `0.002671 m`。

## 9. 常见错误

如果出现：

```text
undefined symbol: libusb_set_option
```

运行 FAST-Calib 前过滤 MVS SDK 的旧 libusb：

```bash
CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
```

详细说明见：

```text
calibration_record/interactive_workflow.md
calibration_record/rough_seed_refinement_plan.md
calibration_record/pitfalls_and_solutions.md
```
