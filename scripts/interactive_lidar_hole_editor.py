#!/usr/bin/env python3
import argparse
import math
import struct
from pathlib import Path

import rclpy
import yaml
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, MarkerArray


DEFAULT_NAMES = ["upper +Y", "upper -Y", "lower +Y", "lower -Y"]
DEFAULT_COLORS = [
    (1.0, 0.0, 0.0, 1.0),
    (0.0, 0.2, 1.0, 1.0),
    (0.0, 0.85, 0.2, 1.0),
    (1.0, 0.85, 0.0, 1.0),
]


def make_move_axis_control(name, quaternion):
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation.w = quaternion[0]
    control.orientation.x = quaternion[1]
    control.orientation.y = quaternion[2]
    control.orientation.z = quaternion[3]
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control


def read_ascii_ply_xyz(path):
    vertex_count = None
    points = []
    with path.open("r", encoding="ascii") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break

        if vertex_count is None:
            raise ValueError(f"{path} does not declare an element vertex count")

        for _ in range(vertex_count):
            line = f.readline()
            if not line:
                break
            values = line.split()
            if len(values) >= 3:
                points.append((float(values[0]), float(values[1]), float(values[2])))
    return points


def load_centers(path, require_validated_refinement=False):
    if not path or not path.exists():
        return None
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError):
        return None
    if not isinstance(data, dict) or "centers" not in data:
        return None
    if require_validated_refinement and (
        data.get("kind") != "refined_lidar_hole_centers" or data.get("status") != "pass"
    ):
        return None
    if not isinstance(data["centers"], list) or len(data["centers"]) != 4:
        return None
    centers = []
    try:
        for index, item in enumerate(data["centers"]):
            centers.append(
                {
                    "name": str(item.get("name", DEFAULT_NAMES[index])),
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "z": float(item["z"]),
                }
            )
    except (KeyError, TypeError, ValueError):
        return None
    return centers


def initial_centers_from_cloud(points, target_width, target_height):
    xs = sorted(p[0] for p in points)
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    x0 = xs[len(xs) // 2]
    y0 = (min(ys) + max(ys)) * 0.5
    z0 = (min(zs) + max(zs)) * 0.5
    half_w = target_width * 0.5
    half_h = target_height * 0.5
    return [
        {"name": "upper +Y", "x": x0, "y": y0 + half_w, "z": z0 + half_h},
        {"name": "upper -Y", "x": x0, "y": y0 - half_w, "z": z0 + half_h},
        {"name": "lower +Y", "x": x0, "y": y0 + half_w, "z": z0 - half_h},
        {"name": "lower -Y", "x": x0, "y": y0 - half_w, "z": z0 - half_h},
    ]


def make_cloud(points, frame_id):
    header = Header()
    header.frame_id = frame_id
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    data = bytearray(12 * len(points))
    for i, point in enumerate(points):
        struct.pack_into("<fff", data, i * 12, point[0], point[1], point[2])

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * len(points)
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


class LidarHoleEditor(Node):
    def __init__(self, args):
        super().__init__("interactive_lidar_hole_editor")
        self.frame_id = args.frame_id
        self.output_path = args.output.resolve()
        self.refined_centers_path = args.refined_centers.resolve() if args.refined_centers else None
        self.marker_scale = args.marker_scale
        self.points = read_ascii_ply_xyz(args.cloud.resolve())
        self.cloud_msg = make_cloud(self.points, self.frame_id)
        self.centers = load_centers(args.initial_centers)
        if self.centers is None:
            self.centers = initial_centers_from_cloud(self.points, args.target_width, args.target_height)

        self.cloud_pub = self.create_publisher(PointCloud2, "/static_accumulated_cloud", 1)
        self.refined_pub = self.create_publisher(MarkerArray, "/refined_lidar_hole_markers", 1)
        self.server = InteractiveMarkerServer(self, "manual_lidar_holes")
        self.save_srv = self.create_service(Trigger, "/save_lidar_hole_markers", self.save_callback)
        self.save_seed_srv = self.create_service(Trigger, "/save_lidar_hole_seeds", self.save_callback)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_cloud)
        self.create_markers()
        self.write_centers()
        self.get_logger().info(
            f"已从 {args.cloud.resolve()} 加载 {len(self.points)} 个点；"
            f"请在 RViz 中放置四个 rough seed，完成后调用 /save_lidar_hole_seeds"
        )
        self.get_logger().info(f"当前 rough seed 文件：{self.output_path}")

    def publish_cloud(self):
        stamp = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = stamp
        self.cloud_pub.publish(self.cloud_msg)
        refined = load_centers(
            self.refined_centers_path, require_validated_refinement=True
        )
        self.refined_pub.publish(self.make_refined_markers(refined, stamp))

    def make_refined_markers(self, centers, stamp):
        marker_array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        for idx, center in enumerate(centers or []):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "refined_lidar_holes"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = center["x"]
            sphere.pose.position.y = center["y"]
            sphere.pose.position.z = center["z"]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.marker_scale * 0.55
            sphere.scale.y = self.marker_scale * 0.55
            sphere.scale.z = self.marker_scale * 0.55
            sphere.color.r = 0.1
            sphere.color.g = 1.0
            sphere.color.b = 0.1
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "refined_lidar_hole_labels"
            label.id = 100 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = center["x"]
            label.pose.position.y = center["y"]
            label.pose.position.z = center["z"] + self.marker_scale * 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = 0.1
            label.color.g = 1.0
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = f"refined {center['name']}"
            marker_array.markers.append(label)
        return marker_array

    def create_markers(self):
        self.server.clear()
        for idx, center in enumerate(self.centers):
            marker = InteractiveMarker()
            marker.header.frame_id = self.frame_id
            marker.name = f"manual_hole_{idx}"
            marker.description = center["name"]
            marker.scale = max(self.marker_scale * 2.8, 0.25)
            marker.pose.position.x = center["x"]
            marker.pose.position.y = center["y"]
            marker.pose.position.z = center["z"]
            marker.pose.orientation.w = 1.0

            visual = Marker()
            visual.type = Marker.SPHERE
            visual.scale.x = self.marker_scale
            visual.scale.y = self.marker_scale
            visual.scale.z = self.marker_scale
            visual.color.r, visual.color.g, visual.color.b, visual.color.a = DEFAULT_COLORS[idx]

            visual_control = InteractiveMarkerControl()
            visual_control.name = "drag_sphere"
            visual_control.always_visible = True
            visual_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
            visual_control.markers.append(visual)
            marker.controls.append(visual_control)

            # Give every rough seed independent translation handles in all
            # three LiDAR-frame axes. MOVE_3D remains useful for quick motion
            # in the current view, while these handles make depth adjustment
            # explicit when the sensor or board is tilted.
            marker.controls.append(make_move_axis_control("move_x", (1.0, 1.0, 0.0, 0.0)))
            marker.controls.append(make_move_axis_control("move_y", (1.0, 0.0, 0.0, 1.0)))
            marker.controls.append(make_move_axis_control("move_z", (1.0, 0.0, 1.0, 0.0)))
            self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if not feedback.marker_name.startswith("manual_hole_"):
            return
        idx = int(feedback.marker_name.rsplit("_", 1)[1])
        self.centers[idx]["x"] = feedback.pose.position.x
        self.centers[idx]["y"] = feedback.pose.position.y
        self.centers[idx]["z"] = feedback.pose.position.z

    def save_callback(self, _request, response):
        self.write_centers()
        response.success = True
        response.message = f"已保存 4 个 LiDAR 孔位 rough seed：{self.output_path}"
        self.get_logger().info(response.message)
        return response

    def write_centers(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "kind": "rough_lidar_hole_seeds",
            "version": 1,
            "frame_id": self.frame_id,
            "topic": "/static_accumulated_cloud",
            "centers": [
                {
                    "name": center["name"],
                    "x": round(float(center["x"]), 6),
                    "y": round(float(center["y"]), 6),
                    "z": round(float(center["z"]), 6),
                }
                for center in self.centers
            ],
        }
        self.output_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="RViz interactive editor for four LiDAR calibration holes")
    parser.add_argument("--cloud", required=True, type=Path, help="ASCII PLY cloud to display as static accumulated cloud")
    parser.add_argument("--output", required=True, type=Path, help="YAML file to save the four LiDAR hole centers")
    parser.add_argument("--initial-centers", type=Path, help="Optional existing YAML centers file")
    parser.add_argument("--refined-centers", type=Path, help="Optional refined centers YAML to display")
    parser.add_argument("--frame-id", default="livox_frame")
    parser.add_argument("--rate", type=float, default=0.2)
    parser.add_argument("--marker-scale", type=float, default=0.18)
    parser.add_argument("--target-width", type=float, default=0.5)
    parser.add_argument("--target-height", type=float, default=0.4)
    args = parser.parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if not math.isfinite(args.marker_scale) or args.marker_scale <= 0:
        raise ValueError("--marker-scale must be positive")

    rclpy.init()
    node = LidarHoleEditor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
