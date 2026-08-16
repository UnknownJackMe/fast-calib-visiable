# FAST-Calib: Livox MID360 and Hikvision Industrial Camera Extrinsic Calibration

[中文说明](README.md)

FAST-Calib is a ROS 2 static extrinsic-calibration tool for **Livox MID360 LiDAR** and **Hikvision industrial cameras**.

This project provides two usage modes:

- **Offline calibration**: record the camera and LiDAR into the same ROS 2 bag first, then pass the bag to the calibration scripts;
- **One-click on-site calibration**: run one script to automatically capture a camera image, record a LiDAR bag, and continue with the subsequent interactive calibration steps.

To support tilted installation of the LiDAR or calibration board, the current workflow no longer relies on a fixed-orientation calibration-board ROI. Instead, it first uses a lasso in RViz2 to outline the board, extracts the board plane from the high-resolution static point cloud, and then performs four-hole rough positioning, automatic hole-center refinement, and extrinsic-parameter calculation.

---

## Table of Contents

- [Features](#features)
- [Software and Hardware Requirements](#software-and-hardware-requirements)
- [Installing Dependencies](#installing-dependencies)
- [Configuring the Devices](#configuring-the-devices)
- [Building](#building)
- [Calibration Workflow Overview](#calibration-workflow-overview)
- [Mode 1: Offline Calibration](#mode-1-offline-calibration)
- [Mode 2: One-Click On-Site Calibration](#mode-2-one-click-on-site-calibration)
- [Subsequent RViz2 Operations](#subsequent-rviz2-operations)
- [Output Files](#output-files)
- [Reprocessing an Existing Scene](#reprocessing-an-existing-scene)
- [Frequently Asked Questions](#frequently-asked-questions)
- [Calibration Records](#calibration-records)
- [License](#license)

---

## Features

- Supports Livox MID360 `sensor_msgs/msg/PointCloud2`;
- Supports Hikvision industrial cameras through the MVS SDK;
- Supports offline calibration from a combined camera-and-LiDAR bag;
- Supports automatic image capture, LiDAR recording, and subsequent calibration on site;
- Uses a low-resolution full-scene preview to reduce the RViz2 display load;
- Uses the RViz2 lasso tool to select calibration-board regions with arbitrary perspective shapes;
- Automatically fits calibration-board planes with arbitrary orientations from the high-resolution point cloud;
- Allows the four rough-seed spheres to be moved independently along the LiDAR X/Y/Z axes;
- Automatically fits the true centers of the four circular holes;
- Performs joint validation using the `0.500 m × 0.400 m` four-hole geometry;
- Rough seeds or hole centers that fail validation cannot be used for the final extrinsic-parameter calculation;
- Supports LiDAR or calibration-board installations tilted by approximately 45° or other angles.

---

## Software and Hardware Requirements

### Software Environment

| Item | Requirement |
| --- | --- |
| Operating system | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Point-cloud library | PCL 1.10 or later |
| Image library | OpenCV |
| Build tools | CMake, colcon, GCC/Clang |
| Python | Python 3, NumPy, SciPy, PyYAML |

### Hardware

- Livox MID360;
- Hikvision USB/GigE industrial camera;
- Four-hole ArUco calibration board;
- A graphical environment capable of running RViz2.

### Current Default Calibration-Board Parameters

| Parameter | Default value |
| --- | --- |
| ArUco dictionary | `DICT_4X4_50` |
| ArUco IDs | `[0, 1, 3, 2]` |
| Horizontal spacing between hole centers | `0.500 m` |
| Vertical spacing between hole centers | `0.400 m` |
| Hole radius | `0.120 m` |
| LiDAR topic | `/livox/lidar` |
| LiDAR frame | `livox_frame` |

---

## Installing Dependencies

Install ROS 2, PCL, RViz2 plugin-development dependencies, and Python dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rosbag2 \
  ros-humble-interactive-markers \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  qtbase5-dev \
  libopencv-dev \
  libpcl-dev
```

You also need to install:

1. **Hikvision MVS SDK**
   - Default installation path: `/opt/MVS`
   - It must provide the `MvCameraControl` library and header files;
2. **Livox-SDK2**
   - It must provide `livox_lidar_sdk_shared`;
3. **livox_ros_driver2**
   - It can be installed system-wide or built in the same ROS 2 workspace as FAST-Calib.

Recommended workspace layout:

```text
calib_ws/
└── src/
    ├── FAST-Calib/
    └── livox_ros_driver2/
```

---

## Configuring the Devices

### Configuring the MID360

Modify:

```text
config/livox_mid360_fast_calib.json
```

Confirm that the following settings match the actual device:

- MID360 IP address;
- Host IP address;
- Data and command ports.

After startup, the following topic should be available:

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

You can check it with:

```bash
ros2 topic list -t | grep /livox/lidar
ros2 topic hz /livox/lidar
```

### Configuring the Hikvision Camera

The current default parameters are:

| Parameter | Default value |
| --- | --- |
| Camera serial number | `DA3217436` |
| Exposure time | `30000 us` |
| Gain | `8` |
| Camera publishing rate for the combined bag | `1.0 Hz` |
| Camera topic | `/camera/image_raw` |

You can override them with environment variables:

```bash
export CAMERA_SERIAL=<camera-serial-number>
export EXPOSURE_US=30000
export GAIN=8
export CAMERA_RATE=1.0
```

The current camera intrinsics are located in:

```text
config/camera_params_hikvision_20260814.yaml
config/qr_params.yaml
```

After changing the camera, lens, focal length, or image resolution, update the camera intrinsics first.

---

## Building

### Building from the standalone repository root

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select fast_calib \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

### Building in a workspace that contains livox_ros_driver2

```bash
cd ~/calib_ws
source /opt/ros/humble/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
cd src/FAST-Calib
```

### Checking the build result

```bash
ros2 pkg prefix fast_calib
```

Confirm that the RViz2 lasso plugin has been generated:

```bash
ls install/fast_calib/lib/libfast_calib_rviz_plugins.so
```

If you built in a parent workspace, the file is usually located at:

```text
~/calib_ws/install/fast_calib/lib/libfast_calib_rviz_plugins.so
```

> Each time you open a new terminal, source `/opt/ros/humble/setup.bash` and the workspace's `install/setup.bash` again.

---

## Calibration Workflow Overview

The workflow is the same after data preparation, regardless of whether you use offline calibration or one-click on-site calibration:

```text
Camera image + LiDAR bag
        ↓
Generate the high-resolution static cloud static_cloud_full.ply
        ↓
Generate the low-resolution RViz preview static_cloud_preview.ply
        ↓
Select the calibration board with a lasso in RViz2
        ↓
Fit and extract selected_board_cloud.ply
        ↓
Drag four rough seeds on the board cloud
        ↓
Automatically refine the four true hole centers
        ↓
Validate the four-hole geometry
        ↓
Calculate T_cam_lidar
```

---

## Mode 1: Offline Calibration

Offline calibration is suitable when you:

- record the complete data on site first;
- remove the calibration board afterward;
- then pass the bag to FAST-Calib for processing.

### 1. Which topics must be recorded in the bag?

The combined bag must contain both:

| Data | Default topic | Message type | Required |
| --- | --- | --- | --- |
| MID360 point cloud | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Yes |
| Camera image | `/camera/image_raw` | `sensor_msgs/msg/Image` | Yes |

The offline importer also supports:

```text
sensor_msgs/msg/CompressedImage
```

If you use compressed images, pass the actual compressed-image topic when importing, for example:

```text
/camera/image_raw/compressed
```

> The camera, LiDAR, and calibration board must remain stationary during recording. Recording for approximately 20–30 seconds is recommended. Remove the calibration board only after rosbag has completely stopped and metadata has been written.

### 2. Recommended: record a combined bag with the project script

The script automatically:

- uses a separate `ROS_DOMAIN_ID=77`;
- cleans up old calibration-sensor processes;
- starts the MID360 PointCloud2 publisher;
- starts the Hikvision camera ROS publisher;
- checks that both topics contain actual messages;
- records only the LiDAR and camera topics;
- waits for rosbag to flush its data after Ctrl+C;
- verifies the message counts for both topics;
- decodes one camera image from the bag for verification.

Run:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/record_calibration_bag.sh \
  ~/calibration_bags/scene_001
```

After recording starts, keep the devices and calibration board stationary for approximately 20–30 seconds, then press once:

```text
Ctrl+C
```

On success, the following directories will be generated:

```text
~/calibration_bags/scene_001/
~/calibration_bags/scene_001_verification/
```

Verify the bag:

```bash
ros2 bag info ~/calibration_bags/scene_001
```

You should see entries similar to:

```text
Topic: /camera/image_raw | Type: sensor_msgs/msg/Image
Topic: /livox/lidar      | Type: sensor_msgs/msg/PointCloud2
```

### 3. Optional: record a combined bag manually

If the camera and LiDAR topics are already being published by other nodes, you can record them directly:

```bash
ros2 bag record \
  -o ~/calibration_bags/scene_001 \
  /livox/lidar \
  /camera/image_raw
```

If you need to use this project's sensor launch file:

```bash
ros2 launch fast_calib calibration_sensors_launch.py
```

Then run the `ros2 bag record` command above in another terminal.

After recording, you must check:

```bash
ros2 bag info ~/calibration_bags/scene_001
```

### 4. Pass the combined bag to the calibration script

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

Parameter meanings:

```text
interactive_calibration_from_bag.sh \
  <new-scene-name> \
  <bag-directory> \
  <camera-topic> \
  <LiDAR-topic>
```

The script will:

1. check the bag and topic types;
2. hard-link or copy the bag to `calib_data/<scene>/lidar_bag/`;
3. select a frame from the temporal middle of the camera topic;
4. save it as `calib_data/<scene>/image.png`;
5. generate `config/qr_params_<scene>.yaml`;
6. generate a high-resolution static cloud and a low-resolution RViz preview;
7. open the RViz2 board-selection lasso interface;
8. continue with four-hole rough positioning, hole-center refinement, and extrinsic-parameter calculation.

> The scene name must be new. By default, the script refuses to overwrite an existing `calib_data/<scene>`, `output/<scene>`, or `config/qr_params_<scene>.yaml`.

### 5. Import the bag without launching RViz2 immediately

```bash
PREPARE_ONLY=1 \
./scripts/interactive_calibration_from_bag.sh \
  offline_scene_001 \
  ~/calibration_bags/scene_001 \
  /camera/image_raw \
  /livox/lidar
```

Then run:

```bash
./scripts/interactive_calibration_workflow.sh \
  offline_scene_001 \
  --existing
```

---

## Mode 2: One-Click On-Site Calibration

The one-click on-site mode is suitable when the calibration board is still in place and you want one command to collect data and enter the remaining workflow.

Run:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/interactive_calibration_workflow.sh \
  scene_001 \
  25
```

Here:

- `scene_001`: the new scene name;
- `25`: the LiDAR bag recording duration, in seconds.

This mode will:

1. check whether `/livox/lidar` exists;
2. automatically start the MID360 PointCloud2 driver if it does not;
3. capture one PNG image through the Hikvision MVS SDK;
4. record a `/livox/lidar` bag for the specified duration;
5. generate the scene configuration;
6. generate the full static cloud and the RViz preview cloud;
7. open the board-selection lasso interface;
8. open the four-sphere rough-positioning interface;
9. automatically refine the hole centers and calculate the extrinsic parameters.

> One-click on-site mode does not record a combined camera-and-LiDAR bag. It captures one camera PNG first and then records the LiDAR bag, so the camera, LiDAR, and calibration board must remain stationary from image capture until LiDAR recording ends.

You can override the default camera parameters:

```bash
CAMERA_SERIAL=<camera-serial-number> \
EXPOSURE_US=30000 \
GAIN=8 \
./scripts/interactive_calibration_workflow.sh scene_001 25
```

The scene name must also be different from any existing data.

---

## Subsequent RViz2 Operations

After offline import or on-site data collection is complete, the script automatically enters the following two RViz2 stages.

### Step 1: Lasso-select the calibration board

After RViz2 opens:

1. Select `Move Camera` first if you need to adjust the viewing angle;
2. adjust the view so that the entire calibration board is clearly visible;
3. press `P` to activate `Board Polygon Selection`;
4. confirm that the cursor changes to a crosshair;
5. hold the left mouse button and continuously drag one loop around the outside of the board;
6. release the left mouse button; the program automatically closes the lasso and extracts the board plane;
7. you do not need to hit any specific cloud points; simply make the on-screen lasso enclose the board;
8. right-click or press Esc to cancel the current lasso.

Display colors:

| Color | Meaning |
| --- | --- |
| Gray | Low-resolution full-scene preview |
| Yellow | Visible points directly selected by the lasso |
| Green | Calibration board extracted from the high-resolution point cloud |
| Orange wireframe | Selection boundary |
| Blue arrow | Calibration-board plane normal |

After confirming that the green calibration board is complete, return to the terminal:

- Press Enter: confirm and proceed to the four-sphere stage;
- enter `r`: delete the unconfirmed candidate and draw the lasso again;
- press Ctrl+C: exit; unconfirmed results will not be reused next time.

![Unannotated point cloud](README_images/未标注点云.png)

![Calibration-board selection](README_images/框选标定板位置.png)

### Step 2: Rough positioning with four spheres

1. Select `Interact` at the top of RViz2;
2. place the four colored spheres near the four holes respectively;
3. precise centering is not required; the spheres are only rough seeds;
4. each sphere independently supports `move_x`, `move_y`, and `move_z`;
5. when the LiDAR or calibration board is tilted, the four spheres do not need to remain horizontal, vertical, or at the same depth;
6. make sure the four spheres correspond to four different holes;
7. return to the terminal and press Enter when finished.

![Four-hole sphere placement 1](README_images/四孔小球摆放1.png)

![Four-hole sphere placement 2](README_images/四孔小球摆放2.png)

![Four-hole sphere placement 3](README_images/四孔小球摆放3.png)

### Step 3: Automatic refinement and confirmation

The program will automatically:

1. search the local board-plane point cloud around each rough seed;
2. fit the circular-hole boundary and radius and recover the true hole center;
3. validate the four-hole width, height, and diagonals;
4. display smaller green refined centers in RViz2;
5. allow extrinsic-parameter calculation only after validation succeeds.

After the terminal reports that refinement succeeded:

- Press Enter: accept the refined hole centers and calculate the extrinsic parameters;
- enter `r`: return to RViz2 and adjust the rough seeds;
- if refinement fails, enter `q` to stop the workflow.

![Runtime screenshot 1](README_images/运行截图1.png)

![Runtime screenshot 2](README_images/运行截图2.png)

![Runtime screenshot 3](README_images/运行截图3.png)

---

## Output Files

### Scene inputs

```text
calib_data/<scene>/image.png
calib_data/<scene>/lidar_bag/
config/qr_params_<scene>.yaml
```

### Static cloud and calibration-board extraction

```text
output/<scene>/static_cloud_full.ply
output/<scene>/static_cloud_preview.ply
output/<scene>/selected_board_cloud.ply
output/<scene>/board_extraction_report.yaml
```

### Four-hole rough positioning and refinement

```text
output/<scene>/manual_lidar_hole_seeds.yaml
output/<scene>/refined_lidar_holes.yaml
output/<scene>/hole_refinement_report.yaml
output/<scene>/refinement_debug/
```

### Final extrinsic parameters

```text
output/<scene>_refined_four_holes/calib_result.txt
output/<scene>_refined_four_holes/colored_cloud.pcd
output/<scene>_refined_four_holes/colored_cloud.ply
```

The final extrinsic-parameter file mainly contains:

```text
Rcl: rotation matrix from LiDAR to camera
Pcl: translation vector from LiDAR to camera
cam_fx / cam_fy / cam_cx / cam_cy: camera intrinsics
cam_d0 ... cam_d4: camera distortion parameters
```

Acceptance recommendations:

- `board_extraction_report.yaml` contains `accepted: true`;
- `hole_refinement_report.yaml` has `status: pass`;
- the four-hole registration RMSE is less than `0.008 m`;
- ideally, the RMSE is close to or below `0.005 m`;
- inspect the projection result in `colored_cloud.ply` or `colored_cloud.pcd`.

---

## Reprocessing an Existing Scene

After bag import and scene configuration are complete, you can directly re-enter the interactive workflow:

```bash
./scripts/interactive_calibration_workflow.sh \
  <scene_name> \
  --existing
```

Force regeneration of the high- and low-resolution static clouds:

```bash
REBUILD_STATIC_CLOUD=1 \
./scripts/interactive_calibration_workflow.sh \
  <scene_name> \
  --existing
```

Force lasso re-selection of the calibration board:

```bash
RESELECT_BOARD=1 \
./scripts/interactive_calibration_workflow.sh \
  <scene_name> \
  --existing
```

Adjust the full-scene preview range:

```bash
export FAST_CALIB_PREVIEW_MIN_X=1.0
export FAST_CALIB_PREVIEW_MAX_X=5.0
export FAST_CALIB_PREVIEW_MIN_Y=-2.0
export FAST_CALIB_PREVIEW_MAX_Y=2.0
export FAST_CALIB_PREVIEW_MIN_Z=-1.0
export FAST_CALIB_PREVIEW_MAX_Z=3.0

REBUILD_STATIC_CLOUD=1 \
./scripts/interactive_calibration_workflow.sh \
  <scene_name> \
  --existing
```

---

## Frequently Asked Questions

### 1. `undefined symbol: libusb_set_option` appears

This is usually caused by an old version of libusb from the Hikvision MVS SDK being placed before the system library.

Temporarily filter it out:

```bash
CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" \
  | tr ':' '\n' \
  | grep -v '^/opt/MVS/lib' \
  | paste -sd:)
```

The main calibration scripts in this project already apply this filtering automatically to the relevant PCL commands.

### 2. `Board Polygon Selection` is not available in RViz2

Rebuild and source the workspace again:

```bash
cd ~/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Confirm that the plugin exists:

```bash
ls install/fast_calib/lib/libfast_calib_rviz_plugins.so
```

### 3. The lasso tool does not activate

Press:

```text
P
```

The cursor should become a crosshair. The lasso does not need to hit cloud points; hold the left mouse button, surround the calibration board, and release.

### 4. The complete calibration board is not visible in RViz2

Expand the preview range and regenerate the static cloud:

```bash
export FAST_CALIB_PREVIEW_MIN_X=1.0
export FAST_CALIB_PREVIEW_MAX_X=5.0
export FAST_CALIB_PREVIEW_MIN_Y=-2.0
export FAST_CALIB_PREVIEW_MAX_Y=2.0
export FAST_CALIB_PREVIEW_MIN_Z=-1.0
export FAST_CALIB_PREVIEW_MAX_Z=3.0

REBUILD_STATIC_CLOUD=1 \
RESELECT_BOARD=1 \
./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

### 5. The next run still reuses an incorrect result after entering `r`

The current version reuses a result only when both of the following conditions are satisfied:

```text
selected_board_cloud.ply exists
board_extraction_report.yaml contains accepted: true
```

Unconfirmed candidate files are not used as official results.

### 6. The four spheres cannot be dragged

Check the following:

- whether the RViz2 tool is `Interact`;
- whether the Display is `rviz_default_plugins/InteractiveMarkers`;
- whether the Namespace is `/manual_lidar_holes`;
- if depth-direction movement is difficult, use the `move_x`, `move_y`, and `move_z` axis handles.

### 7. The script refuses to overwrite an existing scene

This prevents the bag, image, and calibration results from being overwritten. Use a new scene name, for example:

```text
scene_001_v2
scene_002_light_v2
```

Or use the existing-scene entry point:

```bash
./scripts/interactive_calibration_workflow.sh <scene_name> --existing
```

### 8. `livox_ros_driver2` cannot be found

Make sure the driver is discoverable in the current environment:

```bash
ros2 pkg prefix livox_ros_driver2
```

If the driver is installed in a separate directory, set:

```bash
export LIVOX_ROS_DRIVER2_PREFIX=/path/to/livox_ros_driver2/install/livox_ros_driver2
```

---

## Calibration Records

Detailed design notes, historical results, and issue records are located in:

```text
calibration_record/
```

Main documents:

- `calibration_record/quick_start.md`
- `calibration_record/interactive_workflow.md`
- `calibration_record/rough_seed_refinement_plan.md`
- `calibration_record/device_config.md`
- `calibration_record/pitfalls_and_solutions.md`
- `calibration_record/final_result_20260617.md`

Historical reference result:

```text
output/final_success_20260617/calib_result.txt
```

---

## License

This project follows the [LICENSE](LICENSE) in the repository.

Reference notes for the RViz2 point-cloud lasso interaction design can be found in:

```text
docs/THIRD_PARTY_NOTICES.md
```
