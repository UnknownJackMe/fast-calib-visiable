# 设备和工程配置

## 主机环境

- 主机路径：`/home/vision/FAST-Calib`
- ROS：ROS2 Humble
- FAST-Calib 版本：ROS2 fork
- 主要构建命令：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## LiDAR

- 设备：Livox MID360
- ROS topic：`/livox/lidar`
- Topic 类型：`sensor_msgs/msg/PointCloud2`
- 当前 LiDAR IP：`192.168.1.30`
- 当前主机 Livox 有线侧 IP：`192.168.1.50`
- Livox driver config：

```text
/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json
```

重要：不要把 MID360 改回 `192.168.1.3`。这台机器的 Wi-Fi/RDP 使用 `wlo1 192.168.1.3/24`，LiDAR 占用同一地址会影响远程连接。

启动 MID360 点云：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash
ros2 launch fast_calib mid360_pointcloud2_launch.py
```

检查：

```bash
ros2 topic list -t | grep /livox/lidar
```

期望：

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

## 相机

- 相机：Hikvision
- 当前使用静态 PNG 输入，不依赖 ROS 相机 topic
- 相机序列号：`DA3217436`
- 常用抓图命令：

```bash
ros2 run fast_calib grab_hikvision_png \
  /home/vision/FAST-Calib/calib_data/<scene>/image.png \
  DA3217436 \
  30000 \
  8
```

当前相机内参来源（2026-08-14，51 张图，平均重投影误差约 0.071 px）：

```text
/home/vision/FAST-Calib/config/camera_params_hikvision_20260814.yaml
```

当前新采集场景使用的内参：

```yaml
cam_width: 1440
cam_height: 1080
cam_fx: 1793.8419173990653
cam_fy: 1793.8080728731852
cam_cx: 704.449662990624
cam_cy: 550.5285395821396
cam_d0: -0.070460678323384
cam_d1: 0.1318047353351998
cam_d2: -0.0018212418630035034
cam_d3: 0.0006867908135533604
cam_d4: -0.005040654198261228
```

`final_success_20260617` 等历史场景仍保留当时使用的旧内参，不应回写修改。

## 标定板参数

本次配置文件：

```text
/home/vision/FAST-Calib/config/qr_params_final_success_20260617.yaml
```

关键参数：

```yaml
aruco_dictionary: "DICT_4X4_50"
aruco_ids: [0, 1, 3, 2] # top-left, top-right, bottom-right, bottom-left
marker_size: 0.200
delta_width_qr_center: 0.550
delta_height_qr_center: 0.350
delta_width_circles: 0.500
delta_height_circles: 0.400
circle_radius: 0.120
```

注意：FAST-Calib 相机侧代码会把 `delta_width_circles` 和 `delta_height_circles` 除以 2，四个孔中心在标定板坐标中是 `±0.250 m` 和 `±0.200 m`。

## 本次最终场景数据

```text
Scene: current_static_pair_y0_full_20260617
Image: /home/vision/FAST-Calib/calib_data/final_success_20260617/image.png
LiDAR bag: /home/vision/FAST-Calib/calib_data/final_success_20260617/lidar_bag
Config: /home/vision/FAST-Calib/config/qr_params_final_success_20260617.yaml
Output: /home/vision/FAST-Calib/output/final_success_20260617
Manual four-hole output: /home/vision/FAST-Calib/output/final_success_20260617
```

LiDAR bag 信息：

```text
duration: about 24.799 s
messages: 249
topic: /livox/lidar
type: sensor_msgs/msg/PointCloud2
size: about 123.9 MiB
```

最终使用的 ROI：

```yaml
x_min: 2.20
x_max: 4.40
y_min: -0.80
y_max: 0.80
z_min: -0.60
z_max: 1.50
```
