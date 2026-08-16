# FAST-Calib MID360 + Hikvision

[中文说明](README_zh.md)

ROS 2 calibration toolkit for Livox MID360 LiDAR and Hikvision industrial cameras.

This project packages a practical FAST-Calib workflow for static target calibration:

- capture one Hikvision image and one MID360 `sensor_msgs/msg/PointCloud2` bag;
- run FAST-Calib to extract camera observations and a static accumulated LiDAR cloud;
- open RViz2 with four independently movable rough-seed spheres;
- move every seed freely along LiDAR X/Y/Z near its corresponding board hole,
  including scenes where the LiDAR or target is tilted;
- refine the four hole centers from local point-cloud geometry;
- validate the known four-hole rectangle before solving the extrinsic parameters.

The repository is intended to be usable as a standalone open-source project. You should source this workspace's `install/setup.bash`, not another project's install space.

## Hardware

- Ubuntu 22.04 + ROS 2 Humble
- Livox MID360
- Hikvision USB/GigE industrial camera supported by MVS SDK
- Four-hole ArUco calibration board

Default target parameters used by the included configs:

- ArUco dictionary: `DICT_4X4_50`
- ArUco IDs: `[0, 1, 3, 2]`
- Hole spacing: `0.500 m x 0.400 m`
- Hole radius: `0.120 m`
- LiDAR topic: `/livox/lidar`
- LiDAR frame: `livox_frame`

## Dependencies

Install ROS 2 and common build/runtime packages:

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
  python3-scipy \
  libopencv-dev \
  libpcl-dev
```

Install vendor SDKs:

- Hikvision MVS SDK, expected under `/opt/MVS`
- Livox-SDK2, providing `livox_lidar_sdk_shared`
- `livox_ros_driver2`, either installed system-wide or built in the same ROS 2 workspace as this package

Recommended workspace layout:

```text
calib_ws/
  src/
    FAST-Calib/
    livox_ros_driver2/
```

Build from the workspace root:

```bash
cd ~/calib_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If you build directly from this repository root, the same rule applies:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Do not source another application repository such as `moving_scaning_hku`. If `livox_ros_driver2` is not found after sourcing this workspace, install it or build it in the same workspace.

## Configuration

Edit these files before collecting new data:

- `config/livox_mid360_fast_calib.json`: MID360 IP and Livox connection settings
- `config/camera_params_hikvision_20260814.yaml`: source Hikvision calibration result
- `config/qr_params.yaml`: active camera intrinsics, target geometry, input/output defaults

The included MID360 launch file starts `livox_ros_driver2` and publishes `/livox/lidar` as `sensor_msgs/msg/PointCloud2`:

```bash
ros2 launch fast_calib mid360_pointcloud2_launch.py
```

The capture workflow starts this driver automatically if `/livox/lidar` is not already available.

## Interactive Calibration

Run the full workflow:

```bash
cd ~/calib_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/FAST-Calib
./scripts/interactive_calibration_workflow.sh scene_001 25
```

If running from the source repository root:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/interactive_calibration_workflow.sh scene_001 25
```

The script will:

1. capture a Hikvision image;
2. record a MID360 bag for the requested duration;
3. generate `config/qr_params_<scene>.yaml`;
4. run FAST-Calib once to create `output/<scene>/filtered_cloud.ply`;
5. start `scripts/interactive_lidar_hole_editor.py`;
6. open RViz2 with `output/<scene>/manual_lidar_hole_editor.rviz`;
7. save four approximate hole seeds;
8. fit and validate the true LiDAR hole centers near those seeds;
9. show refined centers as smaller green spheres;
10. calculate the extrinsic parameters only from validated refined centers.

Example RViz2 views:

![RViz2 interactive LiDAR hole spheres](docs/assets/rviz_interactive_hole_spheres.png)

![RViz2 interactive close-up](docs/assets/rviz_interactive_closeup.png)

![RViz2 interactive sphere adjustment](docs/assets/rviz_interactive_adjustment.png)

In RViz2:

1. Select the `Interact` tool.
2. Drag each colored sphere near its corresponding physical hole. Mouse-level precision is not required.
3. Use the sphere's `move_x`, `move_y`, and `move_z` handles for explicit
   forward/backward, left/right, and up/down adjustment.
4. Each sphere is independent. The four rough seeds do not need to remain
   horizontal, vertical, or at the same LiDAR-frame depth when the sensor or
   calibration target is tilted.
5. Return to the workflow terminal and press Enter, or save the rough seeds manually:

```bash
source /opt/ros/humble/setup.bash
ros2 service call /save_lidar_hole_seeds std_srvs/srv/Trigger {}
```

The local refiner searches the board plane around each seed, fits the circular
empty region, checks the `0.500 x 0.400 m` layout, and displays successful
centers as green spheres. Failed refinement returns to the same RViz editor for
seed adjustment; rough seeds are never silently used as final centers.

The hole labels describe correspondence on the calibration target, not a
required alignment with the LiDAR coordinate axes. The refiner estimates the
target plane in 3D before fitting the holes, so an approximately 45-degree
LiDAR or target installation is supported.

If your workspace setup file is not in one of the standard locations above, set it explicitly:

```bash
export ROS_WORKSPACE_SETUP=/path/to/your/workspace/install/setup.bash
```

Generated files:

```text
calib_data/<scene>/image.png
calib_data/<scene>/lidar_bag/
config/qr_params_<scene>.yaml
output/<scene>/filtered_cloud.ply
output/<scene>/manual_lidar_hole_seeds.yaml
output/<scene>/refined_lidar_holes.yaml
output/<scene>/hole_refinement_report.yaml
output/<scene>/refinement_debug/
output/<scene>_refined_four_holes/calib_result.txt
```

## Offline Calibration from a Combined Rosbag

For a static calibration scene, it is valid to record the LiDAR and camera in
one ROS 2 bag and perform all processing after the target has been removed. The
camera, LiDAR, and target must remain stationary for the whole recording.

The recommended recorder starts both sensor publishers, isolates them in ROS
Domain 77, checks for actual messages, records only the two calibration topics,
and validates the bag after Ctrl+C:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/record_calibration_bag.sh ~/calibration_bags/scene_001
```

Press Ctrl+C once to stop. The script will wait for rosbag to flush, stop the
MID360 and Hikvision publishers, verify nonzero message counts for both topics,
and decode a sample camera frame. Verification artifacts are written to:

```text
~/calibration_bags/scene_001_verification/
```

The resulting bag contains:

- a LiDAR topic with type `sensor_msgs/msg/PointCloud2`;
- a camera topic with type `sensor_msgs/msg/Image` or
  `sensor_msgs/msg/CompressedImage`.

The Hikvision publisher defaults to `1440x1080`, exposure `30000 us`, gain `8`,
and 1 Hz. MID360 PointCloud2 is normally published at approximately 10 Hz.

Import the completed bag and start the same static-cloud/RViz workflow:

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

The importer selects the temporal-middle camera message, saves it as
`calib_data/<scene>/image.png`, imports the bag into the scene directory,
generates the scene config, builds the accumulated LiDAR cloud, and opens the
RViz sphere editor. Recording the camera continuously is allowed, but only one
image is used by this static calibration algorithm.

To prepare and inspect a scene without launching processing or RViz:

```bash
PREPARE_ONLY=1 ./scripts/interactive_calibration_from_bag.sh \
  offline_scene_001 \
  ~/calibration_bags/scene_001 \
  /camera/image_raw \
  /livox/lidar
```

Continue a prepared scene later with:

```bash
./scripts/interactive_calibration_workflow.sh offline_scene_001 --existing
```

## Run Existing Data

For an existing config:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run_fast_calib_scene.sh config/qr_params_<scene>.yaml
```

For already validated refined LiDAR centers:

```bash
CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$CLEAN_LD" ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file config/qr_params_<scene>.yaml \
  -p manual_lidar_centers_path:=output/<scene>/refined_lidar_holes.yaml \
  -p output_path:=output/<scene>_refined_four_holes
```

## Troubleshooting

If PCL or FAST-Calib fails with:

```text
undefined symbol: libusb_set_option
```

Hikvision MVS probably put an older `libusb` ahead of the system library. Run calibration commands with `/opt/MVS/lib` removed from `LD_LIBRARY_PATH`, as shown above.

If RViz2 shows the cloud but the spheres are not draggable:

- the display must be `rviz_default_plugins/InteractiveMarkers`;
- `Interactive Markers Namespace` must be `/manual_lidar_holes`;
- the active RViz tool must be `Interact`;
- select a colored sphere and use `move_x`, `move_y`, or `move_z` when depth
  motion is difficult with direct sphere dragging.

If `/livox/lidar` is missing, verify:

```bash
ros2 topic list -t | grep /livox/lidar
ros2 topic hz /livox/lidar
```

## Reference Records

The `calibration_record/` directory contains the field notes from the first successful MID360 + Hikvision calibration, including device settings, pitfalls, and the final verified result.

Useful files:

- `calibration_record/interactive_workflow.md`
- `calibration_record/device_config.md`
- `calibration_record/pitfalls_and_solutions.md`
- `calibration_record/final_result_20260617.md`

Final reference output from that run:

```text
output/final_success_20260617/calib_result.txt
```

## Upstream

This repository is based on the ROS 2 port work discussed from FAST-Calib and extends it with MID360/Hikvision capture tools plus an RViz2 interactive four-hole workflow.
