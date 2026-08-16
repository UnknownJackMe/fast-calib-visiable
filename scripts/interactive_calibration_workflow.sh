#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
用法：interactive_calibration_workflow.sh 场景名 [录制秒数]
      interactive_calibration_workflow.sh 场景名 --existing

流程：
  1. 生成完整静态点云和低分辨率 RViz 预览点云；
  2. 在 RViz 中用任意多边形框选标定板；
  3. 从高分辨率点云中提取选中的标定板；
  4. 在提取后的标定板点云上拖动四个 rough seed；
  5. 自动精修四个孔心并计算外参。

RViz 多边形工具：
  按 P 激活工具；按住左键拖动套索，松开完成；右键或 Esc 取消。
USAGE
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 64
fi

scene_name=$1
mode=${2:-25}
use_existing=0
if [[ "$mode" == "--existing" ]]; then
  use_existing=1
  duration_s=0
else
  duration_s=$mode
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fast_calib_root=${FAST_CALIB_ROOT:-$(cd "${script_dir}/.." && pwd)}
cd "$fast_calib_root"

data_dir="${fast_calib_root}/calib_data/${scene_name}"
output_dir="${fast_calib_root}/output/${scene_name}"
config_path="${fast_calib_root}/config/qr_params_${scene_name}.yaml"
full_cloud="${output_dir}/static_cloud_full.ply"
preview_cloud="${output_dir}/static_cloud_preview.ply"
selected_board_cloud="${output_dir}/selected_board_cloud.ply"
board_report="${output_dir}/board_extraction_report.yaml"
candidate_board_cloud="${output_dir}/selected_board_candidate.ply"
candidate_board_report="${output_dir}/board_extraction_candidate.yaml"
seeds_file="${output_dir}/manual_lidar_hole_seeds.yaml"
legacy_centers_file="${output_dir}/manual_lidar_holes.yaml"
refined_centers_file="${output_dir}/refined_lidar_holes.yaml"
refinement_report="${output_dir}/hole_refinement_report.yaml"
refinement_debug_dir="${output_dir}/refinement_debug"
refinement_config="${fast_calib_root}/config/hole_refinement_params.yaml"
manual_output_dir="${fast_calib_root}/output/${scene_name}_refined_four_holes"
board_rviz_file="${output_dir}/board_polygon_selection.rviz"
hole_rviz_file="${output_dir}/manual_lidar_hole_editor.rviz"

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
  echo "找不到 ROS 工作空间 setup.bash，请先构建或设置 ROS_WORKSPACE_SETUP。" >&2
  exit 69
fi
set -u

config_value() {
  python3 - "$config_path" "$1" <<'PY'
from pathlib import Path
import sys
key = sys.argv[2]
for raw in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines():
    line = raw.split("#", 1)[0].strip()
    if line.startswith(key + ":"):
        print(line.split(":", 1)[1].strip().strip('"'))
        break
PY
}

if [[ "$use_existing" -eq 0 ]]; then
  echo "正在采集场景：${scene_name}"
  set +e
  scripts/capture_and_run_scene.sh "$scene_name" "$duration_s"
  capture_status=$?
  set -e
else
  echo "正在使用已有离线场景：${scene_name}"
  capture_status=0
fi

if [[ ! -f "$config_path" ]]; then
  echo "场景配置不存在：$config_path" >&2
  exit "$capture_status"
fi

mkdir -p "$output_dir" "$manual_output_dir"

if [[ ! -f "$full_cloud" || ! -f "$preview_cloud" || "${REBUILD_STATIC_CLOUD:-0}" == "1" ]]; then
  bag_path=$(config_value bag_path)
  lidar_topic=$(config_value lidar_topic)
  if [[ -z "$bag_path" || -z "$lidar_topic" ]]; then
    echo "无法从配置读取 bag_path 或 lidar_topic。" >&2
    exit 66
  fi

  echo "正在从完整 bag 生成静态点云和 RViz 预览点云..."
  echo "这一步不使用旧的固定标定板 ROI，避免倾斜标定板被裁切。"
  clean_ld=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
  env LD_LIBRARY_PATH="$clean_ld" ros2 run fast_calib prepare_static_cloud \
    --bag "$bag_path" \
    --topic "$lidar_topic" \
    --output-full "$full_cloud" \
    --output-preview "$preview_cloud" \
    --full-leaf "${FAST_CALIB_FULL_LEAF:-0.01}" \
    --preview-leaf "${FAST_CALIB_PREVIEW_LEAF:-0.03}" \
    --min-range "${FAST_CALIB_MIN_RANGE:-0.3}" \
    --max-range "${FAST_CALIB_MAX_RANGE:-8.0}" \
    --min-x "${FAST_CALIB_PREVIEW_MIN_X:-1.5}" \
    --max-x "${FAST_CALIB_PREVIEW_MAX_X:-4.0}" \
    --min-y "${FAST_CALIB_PREVIEW_MIN_Y:--1.2}" \
    --max-y "${FAST_CALIB_PREVIEW_MAX_Y:-1.2}" \
    --min-z "${FAST_CALIB_PREVIEW_MIN_Z:--0.5}" \
    --max-z "${FAST_CALIB_PREVIEW_MAX_Z:-2.0}"
fi

if [[ ! -f "$full_cloud" || ! -f "$preview_cloud" ]]; then
  echo "静态点云生成失败：$full_cloud 或 $preview_cloud 不存在。" >&2
  exit 74
fi

board_processor_pid=""
board_rviz_pid=""
hole_editor_pid=""
hole_rviz_pid=""
board_selection_confirmed=0
clear_candidate_files() {
  python3 - "$candidate_board_cloud" "$candidate_board_report" <<'PY'
from pathlib import Path
import sys
for value in sys.argv[1:]:
    Path(value).unlink(missing_ok=True)
PY
}
stop_board_stage() {
  if [[ -n "$board_rviz_pid" ]] && kill -0 "$board_rviz_pid" 2>/dev/null; then
    kill "$board_rviz_pid" 2>/dev/null || true
  fi
  if [[ -n "$board_processor_pid" ]] && kill -0 "$board_processor_pid" 2>/dev/null; then
    kill "$board_processor_pid" 2>/dev/null || true
  fi
  board_rviz_pid=""
  board_processor_pid=""
}
cleanup() {
  stop_board_stage
  if [[ "$board_selection_confirmed" -eq 0 ]]; then
    clear_candidate_files
  fi
  if [[ -n "$hole_rviz_pid" ]] && kill -0 "$hole_rviz_pid" 2>/dev/null; then
    kill "$hole_rviz_pid" 2>/dev/null || true
  fi
  if [[ -n "$hole_editor_pid" ]] && kill -0 "$hole_editor_pid" 2>/dev/null; then
    kill "$hole_editor_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

accepted_board=0
if [[ -f "$selected_board_cloud" && -f "$board_report" ]] && grep -q '^accepted: true' "$board_report"; then
  accepted_board=1
fi

if [[ "$accepted_board" -eq 0 || "${RESELECT_BOARD:-0}" == "1" ]]; then
  clear_candidate_files
  echo
  echo "===== 第一步：套索框选标定板 ====="
  echo "RViz 正在显示完整静态点云的低分辨率预览。"
  echo "请按键盘 P 激活“标定板多边形选择”工具，确认鼠标变成十字光标。"
  echo "按住左键，在标定板外围连续拖动一圈，松开左键自动完成。"
  echo "不需要点中任何具体点云点，只要屏幕套索围住标定板即可。"
  echo "右键或 Esc 可以取消当前套索。"

  python3 scripts/board_polygon_selection_processor.py \
    --full-cloud "$full_cloud" \
    --preview-cloud "$preview_cloud" \
    --output "$candidate_board_cloud" \
    --report "$candidate_board_report" \
    --frame-id livox_frame \
    --plane-thickness "${FAST_CALIB_BOARD_PLANE_THICKNESS:-0.025}" \
    --hull-margin "${FAST_CALIB_BOARD_HULL_MARGIN:-0.05}" \
    >"${output_dir}/board_polygon_selection_processor.log" 2>&1 &
  board_processor_pid=$!

  cp "$fast_calib_root/rviz_cfg/board_polygon_selection.rviz" "$board_rviz_file"
  rviz2 -d "$board_rviz_file" >"${output_dir}/board_polygon_selection_rviz.log" 2>&1 &
  board_rviz_pid=$!

  while true; do
    read -r -p "看到绿色标定板完整后按 Enter 确认；输入 r 清除并重画：" board_decision
    if [[ "${board_decision:-}" == "r" || "${board_decision:-}" == "R" ]]; then
      ros2 topic pub --once /clear_board_polygon_selection std_msgs/msg/Empty '{}' >/dev/null 2>&1 || true
      clear_candidate_files
      echo "候选结果已清除。请按住左键重新拖动套索。"
      continue
    fi

    ready=0
    for _ in $(seq 1 20); do
      if [[ -f "$candidate_board_cloud" && -f "$candidate_board_report" ]]; then
        ready=1
        break
      fi
      sleep 0.25
    done
    if [[ "$ready" -eq 1 ]] && ! grep -q '^status: fail' "$candidate_board_report"; then
      cp "$candidate_board_cloud" "$selected_board_cloud"
      python3 - "$candidate_board_report" "$board_report" <<'PY'
from pathlib import Path
import sys
import yaml
source, destination = map(Path, sys.argv[1:])
data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
data["accepted"] = True
destination.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
      board_selection_confirmed=1
      clear_candidate_files
      echo "标定板提取结果已确认：$selected_board_cloud"
      break
    fi
    echo "还没有得到有效的标定板提取结果，请输入 r 重新框选。" >&2
  done
else
  board_selection_confirmed=1
  echo "检测到已确认的标定板提取结果，直接复用：$selected_board_cloud"
fi

stop_board_stage

cat >"$hole_rviz_file" <<EOF2
Panels:
  - Class: rviz_common/Displays
    Name: 显示
  - Class: rviz_common/Views
    Name: 视图
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.35
      Class: rviz_default_plugins/Grid
      Color: 90; 90; 90
      Enabled: true
      Name: XY 网格
      Plane: XY
      Plane Cell Count: 24
      Reference Frame: livox_frame
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 1.0
      Name: LiDAR 坐标轴
      Reference Frame: livox_frame
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 70; 220; 100
      Color Transformer: FlatColor
      Enabled: true
      Name: 已提取标定板
      Position Transformer: XYZ
      Size (Pixels): 2
      Size (m): 0.01
      Style: Points
      Topic:
        Value: /static_accumulated_cloud
      Use Fixed Frame: true
      Value: true
    - Class: rviz_default_plugins/InteractiveMarkers
      Enabled: true
      Interactive Markers Namespace: /manual_lidar_holes
      Name: 四个粗定位小球
      Show Axes: true
      Show Descriptions: true
      Show Visual Aids: true
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /manual_lidar_holes/update
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: 自动精修孔心
      Namespaces:
        refined_lidar_hole_labels: true
        refined_lidar_holes: true
      Topic:
        Value: /refined_lidar_hole_markers
      Value: true
  Enabled: true
  Global Options:
    Background Color: 18; 18; 18
    Fixed Frame: livox_frame
    Frame Rate: 10
  Name: 根节点
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.4
      Focal Point:
        X: 2.6
        Y: 0.0
        Z: 0.9
      Name: 当前视图
      Pitch: 0.28
      Target Frame: livox_frame
      Yaw: 2.7
Window Geometry:
  Height: 900
  Width: 1400
EOF2

echo
echo "===== 第二步：四个小球粗定位和自动精修 ====="
initial_centers_file="$seeds_file"
if [[ ! -f "$initial_centers_file" && -f "$legacy_centers_file" ]]; then
  initial_centers_file="$legacy_centers_file"
fi
python3 scripts/interactive_lidar_hole_editor.py \
  --cloud "$selected_board_cloud" \
  --output "$seeds_file" \
  --initial-centers "$initial_centers_file" \
  --refined-centers "$refined_centers_file" \
  --rate 0.2 >"${output_dir}/interactive_lidar_hole_editor.log" 2>&1 &
hole_editor_pid=$!

sleep 1
rviz2 -d "$hole_rviz_file" >"${output_dir}/manual_lidar_hole_editor_rviz.log" 2>&1 &
hole_rviz_pid=$!

cat <<EOF3

RViz 已打开。

第一步的多边形已经提取为：
  $selected_board_cloud

现在请选择 Interact 工具，调整四个彩色 rough seed：
  - 每个球都可以沿 X/Y/Z 三轴独立移动；
  - 雷达或标定板倾斜时，四个球不需要水平、垂直或同一深度；
  - 只要靠近正确的孔即可，不需要精确对心。

调整完成后回到此终端按 Enter。
EOF3

read -r -p "四个 rough seed 放好后按 Enter 开始自动精修："
ros2 service call /save_lidar_hole_seeds std_srvs/srv/Trigger {} >/dev/null

while true; do
  echo "正在根据局部点云自动精修四个孔心..."
  set +e
  python3 scripts/refine_lidar_hole_seeds.py \
    --plane-cloud "$selected_board_cloud" \
    --filtered-cloud "$selected_board_cloud" \
    --seeds "$seeds_file" \
    --config "$refinement_config" \
    --output "$refined_centers_file" \
    --report "$refinement_report" \
    --debug-dir "$refinement_debug_dir"
  refinement_status=$?
  set -e

  if [[ "$refinement_status" -eq 0 ]]; then
    echo
    echo "自动精修成功。RViz 中的绿色小球是真实拟合孔心。"
    read -r -p "按 Enter 接受并计算外参；输入 r 返回重新调整 rough seed：" decision
    if [[ "${decision:-}" != "r" && "${decision:-}" != "R" ]]; then
      break
    fi
  else
    echo "自动精修失败，请查看：$refinement_report" >&2
    read -r -p "输入 r 返回调整 rough seed；输入 q 退出：" decision
    if [[ "${decision:-}" == "q" || "${decision:-}" == "Q" ]]; then
      exit 75
    fi
  fi

done

if [[ ! -f "$refined_centers_file" ]]; then
  echo "精修孔心文件不存在：$refined_centers_file" >&2
  exit 74
fi

echo "正在使用自动精修孔心计算外参..."
clean_ld=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$clean_ld" ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file "$config_path" \
  -p manual_lidar_centers_path:="$refined_centers_file" \
  -p output_path:="$manual_output_dir"

echo
echo "===== 标定完成 ====="
echo "外参结果：$manual_output_dir/calib_result.txt"
echo "标定板提取报告：$board_report"
echo "孔心精修报告：$refinement_report"
