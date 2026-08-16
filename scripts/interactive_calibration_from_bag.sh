#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
用法：interactive_calibration_from_bag.sh 场景名 BAG路径 图像TOPIC [雷达TOPIC]

导入同时包含相机和 LiDAR 的 ROS 2 bag，提取时间中间位置的相机图像，
然后进入“完整静态点云 → 多边形框选标定板 → 四球精修 → 外参计算”流程。

支持的相机消息类型：
  sensor_msgs/msg/Image
  sensor_msgs/msg/CompressedImage

示例：
  ./scripts/interactive_calibration_from_bag.sh \
    offline_001 /data/calibration_bag /camera/image_raw /livox/lidar
EOF
}

if [[ $# -lt 3 || $# -gt 4 ]]; then
  usage
  exit 64
fi

scene_name=$1
source_bag=$2
image_topic=$3
lidar_topic=${4:-/livox/lidar}

if [[ ! "$scene_name" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]*$ ]]; then
  echo "场景名只能包含字母、数字、点、下划线和连字符。" >&2
  exit 64
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fast_calib_root=${FAST_CALIB_ROOT:-$(cd "${script_dir}/.." && pwd)}
cd "$fast_calib_root"

source_bag=$(realpath "$source_bag")
if [[ ! -f "${source_bag}/metadata.yaml" ]]; then
  echo "该目录不是有效的 ROS 2 bag：$source_bag" >&2
  exit 66
fi

data_dir="${fast_calib_root}/calib_data/${scene_name}"
bag_dir="${data_dir}/lidar_bag"
image_path="${data_dir}/image.png"
output_dir="${fast_calib_root}/output/${scene_name}"
config_path="${fast_calib_root}/config/qr_params_${scene_name}.yaml"
base_config=${BASE_CONFIG:-${fast_calib_root}/config/qr_params.yaml}

if [[ -e "$data_dir" || -e "$output_dir" || -e "$config_path" ]]; then
  echo "为避免覆盖数据，检测到同名场景文件：" >&2
  [[ -e "$data_dir" ]] && echo "  $data_dir" >&2
  [[ -e "$output_dir" ]] && echo "  $output_dir" >&2
  [[ -e "$config_path" ]] && echo "  $config_path" >&2
  exit 73
fi

set +u
source /opt/ros/humble/setup.bash
workspace_setup=${ROS_WORKSPACE_SETUP:-}
if [[ -n "$workspace_setup" ]]; then
  source "$workspace_setup"
elif [[ -f "${fast_calib_root}/install/setup.bash" ]]; then
  source "${fast_calib_root}/install/setup.bash"
elif [[ -f "${fast_calib_root}/../../install/setup.bash" ]]; then
  source "${fast_calib_root}/../../install/setup.bash"
else
  echo "找不到工作空间 setup.bash，请先构建或设置 ROS_WORKSPACE_SETUP。" >&2
  exit 69
fi
set -u

mkdir -p "$data_dir" "$output_dir"

echo "正在导入 ROS 2 bag：$source_bag"
if ! cp -al -- "$source_bag" "$bag_dir" 2>/dev/null; then
  cp -a --reflink=auto -- "$source_bag" "$bag_dir"
fi

ros2 bag info "$bag_dir" | tee "${output_dir}/rosbag_info.txt"
if ! grep -Fq "Topic: ${lidar_topic} | Type: sensor_msgs/msg/PointCloud2" \
  "${output_dir}/rosbag_info.txt"; then
  echo "bag 中没有 ${lidar_topic} [sensor_msgs/msg/PointCloud2]。" >&2
  echo "可用 topic 已保存到：${output_dir}/rosbag_info.txt" >&2
  exit 65
fi

python3 scripts/extract_image_from_rosbag.py \
  --bag "$bag_dir" \
  --image-topic "$image_topic" \
  --output "$image_path" \
  --metadata-output "${output_dir}/image_extraction.yaml"

python3 - "$base_config" "$config_path" "$bag_dir" "$image_path" "$output_dir" "$lidar_topic" <<'PY'
from pathlib import Path
import sys

src, dst, bag, image, out = map(Path, sys.argv[1:6])
lidar_topic = sys.argv[6]
replacements = {
    "bag_path:": f'    bag_path: "{bag}"',
    "image_path:": f'    image_path: "{image}"',
    "output_path:": f'    output_path: "{out}"',
    "lidar_topic:": f'    lidar_topic: "{lidar_topic}"',
}
lines = []
for raw in src.read_text(encoding="utf-8").splitlines():
    stripped = raw.strip()
    replacement = next(
        (value for key, value in replacements.items() if stripped.startswith(key)),
        None,
    )
    lines.append(replacement if replacement is not None else raw)
dst.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "离线场景配置已生成：$config_path"
if [[ "${PREPARE_ONLY:-0}" == "1" ]]; then
  echo "PREPARE_ONLY=1，离线场景准备完成，不启动交互标定。"
  exit 0
fi
exec scripts/interactive_calibration_workflow.sh "$scene_name" --existing
