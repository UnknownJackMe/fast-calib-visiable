#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: record_calibration_bag.sh BAG_PATH

Start the MID360 and Hikvision ROS publishers in an isolated ROS domain, verify
that both topics are producing messages, and record only:
  /livox/lidar
  /camera/image_raw

Press Ctrl+C to stop. The script waits for rosbag to flush, stops both sensor
publishers, validates per-topic message counts, and decodes a camera frame.

Environment overrides:
  ROS_DOMAIN_ID             default 77
  LIDAR_TOPIC               default /livox/lidar
  CAMERA_TOPIC              default /camera/image_raw
  CAMERA_SERIAL             default DA3217436
  EXPOSURE_US               default 30000
  GAIN                      default 8
  CAMERA_RATE               default 1.0 Hz
  LIVOX_ROS_DRIVER2_PREFIX  package prefix used when livox_ros_driver2 is not found
EOF
}

if [[ $# -ne 1 ]]; then
  usage
  exit 64
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fast_calib_root=${FAST_CALIB_ROOT:-$(cd "${script_dir}/.." && pwd)}
cd "$fast_calib_root"

bag_path=$(realpath -m "$1")
lidar_topic=${LIDAR_TOPIC:-/livox/lidar}
camera_topic=${CAMERA_TOPIC:-/camera/image_raw}
camera_serial=${CAMERA_SERIAL:-DA3217436}
exposure_us=${EXPOSURE_US:-30000}
gain=${GAIN:-8}
camera_rate=${CAMERA_RATE:-1.0}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}

if [[ -e "$bag_path" ]]; then
  echo "Refusing to overwrite existing bag path: $bag_path" >&2
  exit 73
fi
mkdir -p "$(dirname "$bag_path")"

set +u
source /opt/ros/humble/setup.bash

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  livox_prefix=${LIVOX_ROS_DRIVER2_PREFIX:-/home/vision/moving_scaning_hku/ros2_livox_ws/install/livox_ros_driver2}
  if [[ ! -d "$livox_prefix" ]]; then
    echo "livox_ros_driver2 is not available. Set LIVOX_ROS_DRIVER2_PREFIX." >&2
    exit 69
  fi
  export AMENT_PREFIX_PATH="${livox_prefix}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${livox_prefix}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${livox_prefix}/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="${livox_prefix}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
fi

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

sensor_pid=""
recorder_pid=""
sensor_log="${bag_path}_sensors.log"

stop_pid() {
  local pid=$1
  local signal=${2:-INT}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -"$signal" "$pid" 2>/dev/null || true
  fi
}

wait_for_exit() {
  local pid=$1
  local attempts=${2:-8}
  for _ in $(seq 1 "$attempts"); do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 1
  done
  return 1
}

stop_matching_processes() {
  local label=$1
  local pattern=$2
  mapfile -t pids < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  for pid in "${pids[@]}"; do
    if [[ "$pid" == "$$" || "$pid" == "$PPID" ]]; then
      continue
    fi
    echo "Stopping existing ${label} process PID ${pid}"
    kill -INT "$pid" 2>/dev/null || true
  done
}

stop_sensor_stack() {
  if [[ -n "$sensor_pid" ]] && kill -0 "$sensor_pid" 2>/dev/null; then
    kill -INT "$sensor_pid" 2>/dev/null || true
    if ! wait_for_exit "$sensor_pid" 8; then
      kill -TERM "$sensor_pid" 2>/dev/null || true
      wait_for_exit "$sensor_pid" 3 || true
    fi
  fi
  sensor_pid=""
}

cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait_for_exit "$recorder_pid" 15 || true
  fi
  recorder_pid=""
  stop_sensor_stack
}
trap cleanup EXIT

echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Cleaning old calibration publishers..."
stop_matching_processes "calibration sensor launch" "ros2 launch fast_calib calibration_sensors_launch.py"
stop_matching_processes "Livox driver" "livox_ros_driver2_node"
stop_matching_processes "Hikvision publisher" "hikvision_image_publisher"
stop_matching_processes "rosbag player" "ros2 bag play"
sleep 2

existing_topics=$(ros2 topic list --no-daemon -t 2>/dev/null || true)
if printf '%s\n' "$existing_topics" | grep -Eq "^${lidar_topic//\//\\/} "; then
  echo "A publisher for $lidar_topic is still visible in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}:" >&2
  printf '%s\n' "$existing_topics" | grep -F "$lidar_topic" >&2 || true
  exit 75
fi
if printf '%s\n' "$existing_topics" | grep -Eq "^${camera_topic//\//\\/} "; then
  echo "A publisher for $camera_topic is still visible in ROS_DOMAIN_ID=${ROS_DOMAIN_ID}:" >&2
  printf '%s\n' "$existing_topics" | grep -F "$camera_topic" >&2 || true
  exit 75
fi

echo "Starting MID360 and Hikvision publishers..."
ros2 launch fast_calib calibration_sensors_launch.py \
  camera_serial:="$camera_serial" \
  exposure_us:="$exposure_us" \
  gain:="$gain" \
  publish_rate:="$camera_rate" \
  camera_topic:="$camera_topic" \
  >"$sensor_log" 2>&1 &
sensor_pid=$!

wait_for_topic_type() {
  local topic=$1
  local type=$2
  for _ in $(seq 1 30); do
    if ! kill -0 "$sensor_pid" 2>/dev/null; then
      echo "Sensor launch exited before $topic appeared. Log: $sensor_log" >&2
      tail -n 80 "$sensor_log" >&2 || true
      return 1
    fi
    if ros2 topic list --no-daemon -t 2>/dev/null \
      | grep -Eq "^${topic//\//\\/} \\[[^]]*${type//\//\\/}"; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for $topic [$type]. Log: $sensor_log" >&2
  tail -n 80 "$sensor_log" >&2 || true
  return 1
}

wait_for_topic_type "$lidar_topic" "sensor_msgs/msg/PointCloud2"
wait_for_topic_type "$camera_topic" "sensor_msgs/msg/Image"

echo "Checking that both topics contain real messages..."
timeout --foreground 10 ros2 topic echo "$lidar_topic" sensor_msgs/msg/PointCloud2 --once \
  >/dev/null
timeout --foreground 10 ros2 topic echo "$camera_topic" sensor_msgs/msg/Image --once \
  >/dev/null

echo "Topic checks passed:"
ros2 topic list --no-daemon -t | grep -E "^(${lidar_topic}|${camera_topic}) "
echo
echo "Recording to: $bag_path"
echo "Press Ctrl+C once to stop, flush, validate, and shut down both publishers."

ros2 bag record "$lidar_topic" "$camera_topic" -o "$bag_path" &
recorder_pid=$!

on_interrupt() {
  echo
  echo "Stop requested; asking rosbag to flush remaining messages..."
  stop_pid "$recorder_pid" INT
}
trap on_interrupt INT TERM

record_status=0
wait "$recorder_pid" || record_status=$?
recorder_pid=""
trap - INT TERM

if [[ "$record_status" -ne 0 && "$record_status" -ne 130 ]]; then
  echo "ros2 bag record exited with status $record_status" >&2
  exit "$record_status"
fi

echo "Recording stopped. Stopping sensor publishers..."
stop_sensor_stack

if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "Bag metadata was not created: ${bag_path}/metadata.yaml" >&2
  exit 76
fi

verification_dir="${bag_path}_verification"
mkdir -p "$verification_dir"
ros2 bag info "$bag_path" | tee "${verification_dir}/rosbag_info.txt"

python3 - "$bag_path" "$lidar_topic" "$camera_topic" <<'PY'
from pathlib import Path
import sys
import yaml

bag_path = Path(sys.argv[1])
required = {
    sys.argv[2]: "sensor_msgs/msg/PointCloud2",
    sys.argv[3]: "sensor_msgs/msg/Image",
}
metadata = yaml.safe_load((bag_path / "metadata.yaml").read_text(encoding="utf-8"))
topics = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
found = {}
for entry in topics:
    item = entry["topic_metadata"]
    found[item["name"]] = (item["type"], int(entry["message_count"]))

errors = []
for topic, expected_type in required.items():
    if topic not in found:
        errors.append(f"missing topic {topic}")
        continue
    actual_type, count = found[topic]
    if actual_type != expected_type:
        errors.append(f"{topic}: expected {expected_type}, got {actual_type}")
    if count <= 0:
        errors.append(f"{topic}: message count is zero")
    print(f"verified {topic} [{actual_type}] messages={count}")

if errors:
    raise SystemExit("Bag verification failed: " + "; ".join(errors))
PY

python3 scripts/extract_image_from_rosbag.py \
  --bag "$bag_path" \
  --image-topic "$camera_topic" \
  --output "${verification_dir}/camera_sample.png" \
  --metadata-output "${verification_dir}/camera_sample.yaml"

echo
echo "Bag verification passed."
echo "Bag: $bag_path"
echo "Sensor log: $sensor_log"
echo "Verification: $verification_dir"
