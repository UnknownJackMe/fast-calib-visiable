#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: capture_and_run_scene.sh SCENE_NAME [DURATION_S]

Capture one FAST-Calib static pair, generate a scene YAML, then run FAST-Calib
and the board-plane diagnostic.

Environment overrides:
  CAMERA_SERIAL   Hikvision serial, default DA3217436
  EXPOSURE_US     Hikvision exposure, default 30000
  GAIN            Hikvision gain, default 15
  BASE_CONFIG     Source YAML to copy parameters from, default config/qr_params.yaml
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 64
fi

scene_name=$1
duration_s=${2:-25}
camera_serial=${CAMERA_SERIAL:-DA3217436}
exposure_us=${EXPOSURE_US:-30000}
gain=${GAIN:-15}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}

if [[ ! "$scene_name" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]*$ ]]; then
  echo "SCENE_NAME must contain only letters, digits, '.', '_' or '-', and must not start with '-'" >&2
  exit 64
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fast_calib_root=${FAST_CALIB_ROOT:-$(cd "${script_dir}/.." && pwd)}
cd "$fast_calib_root"

base_config=${BASE_CONFIG:-config/qr_params.yaml}
data_dir="${fast_calib_root}/calib_data/${scene_name}"
bag_dir="${data_dir}/lidar_bag"
image_path="${data_dir}/image.png"
output_dir="${fast_calib_root}/output/${scene_name}"
config_path="${fast_calib_root}/config/qr_params_${scene_name}.yaml"

if [[ -e "$data_dir" || -e "$output_dir" || -e "$config_path" ]]; then
  echo "Refusing to overwrite existing scene artifacts:" >&2
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
  echo "Cannot find workspace setup.bash. Build the package first or set ROS_WORKSPACE_SETUP." >&2
  exit 69
fi
set -u

mkdir -p "$data_dir" "$output_dir"

lidar_ip=$(python3 - "$fast_calib_root/config/livox_mid360_fast_calib.json" <<'PY'
import json
from pathlib import Path
import sys

config = Path(sys.argv[1])
data = json.loads(config.read_text(encoding="utf-8"))
print(data["lidar_configs"][0]["ip"])
PY
)

started_driver=0
launch_pid=""
cleanup() {
  if [[ "$started_driver" -eq 1 && -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 5); do
      if ! kill -0 "$launch_pid" 2>/dev/null; then
        wait "$launch_pid" 2>/dev/null || true
        return
      fi
      sleep 1
    done
    kill -TERM "$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 3); do
      if ! kill -0 "$launch_pid" 2>/dev/null; then
        wait "$launch_pid" 2>/dev/null || true
        return
      fi
      sleep 1
    done
    kill -KILL "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! ros2 topic list --no-daemon -t 2>/dev/null \
  | grep -Eq '^/livox/lidar \[[^]]*sensor_msgs/msg/PointCloud2'; then
  if ip route get "$lidar_ip" 2>/dev/null | grep -q '^local .* dev lo'; then
    {
      echo "LiDAR IP $lidar_ip is currently assigned to this host, so the Livox driver cannot reach the sensor."
      echo "Release the conflicting local address first, for example with administrator privileges:"
      echo "  sudo ip addr del ${lidar_ip}/24 dev wlo1"
      echo "Then rerun this script with a new scene name."
    } | tee "${output_dir}/network_preflight_error.txt" >&2
    exit 78
  fi
  echo "Starting MID360 PointCloud2 driver..."
  ros2 launch fast_calib mid360_pointcloud2_launch.py >"${output_dir}/livox_driver.log" 2>&1 &
  launch_pid=$!
  started_driver=1
fi

echo "Waiting for /livox/lidar PointCloud2..."
set +e
timeout 30 bash -c '
  until ros2 topic list --no-daemon -t 2>/dev/null \
    | grep -Eq "^/livox/lidar \\[[^]]*sensor_msgs/msg/PointCloud2"; do
    sleep 1
  done
'
topic_wait_status=$?
set -e
if [[ "$topic_wait_status" -ne 0 ]]; then
  echo "Timed out waiting for /livox/lidar PointCloud2 on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}." >&2
  if [[ -f "${output_dir}/livox_driver.log" ]]; then
    echo "Last Livox driver log lines:" >&2
    tail -n 40 "${output_dir}/livox_driver.log" >&2
  fi
  exit 76
fi

set +e
timeout --foreground --signal=INT --kill-after=5s 8 ros2 topic hz /livox/lidar >"${output_dir}/livox_hz.txt" 2>&1 </dev/null
hz_status=$?
set -e
if [[ "$hz_status" -ne 0 && "$hz_status" -ne 124 ]]; then
  cat "${output_dir}/livox_hz.txt" >&2
  exit "$hz_status"
fi

echo "Capturing Hikvision image..."
ros2 run fast_calib grab_hikvision_png "$image_path" "$camera_serial" "$exposure_us" "$gain" \
  | tee "${output_dir}/camera_capture.log"

echo "Recording ${duration_s}s LiDAR bag..."
set +e
timeout --foreground --signal=INT --kill-after=10s "${duration_s}" \
  ros2 bag record /livox/lidar -o "$bag_dir" >"${output_dir}/rosbag_record.log" 2>&1 </dev/null
record_status=$?
set -e
if [[ "$record_status" -ne 0 && "$record_status" -ne 124 && "$record_status" -ne 130 ]]; then
  cat "${output_dir}/rosbag_record.log" >&2
  exit "$record_status"
fi

if ! find "$bag_dir" -maxdepth 1 -name '*.db3' -type f -size +0c | grep -q .; then
  echo "LiDAR bag recording did not produce a non-empty db3 file: $bag_dir" >&2
  cat "${output_dir}/rosbag_record.log" >&2 || true
  exit 75
fi

ros2 bag info "$bag_dir" >"${output_dir}/rosbag_info.txt"

python3 - "$base_config" "$config_path" "$bag_dir" "$image_path" "$output_dir" <<'PY'
from pathlib import Path
import sys

src, dst, bag, image, out = map(Path, sys.argv[1:])
text = src.read_text(encoding="utf-8")
replacements = {
    "bag_path:": f'    bag_path: "{bag}"',
    "image_path:": f'    image_path: "{image}"',
    "output_path:": f'    output_path: "{out}"',
}
lines = []
for raw in text.splitlines():
    stripped = raw.strip()
    replaced = False
    for key, value in replacements.items():
        if stripped.startswith(key):
            lines.append(value)
            replaced = True
            break
    if not replaced:
        lines.append(raw)
dst.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "Generated scene config: $config_path"

set +e
scripts/run_fast_calib_scene.sh "$config_path"
run_status=$?
set -e

echo "Scene data: $data_dir"
echo "Scene output: $output_dir"
echo "Scene config: $config_path"
exit "$run_status"
