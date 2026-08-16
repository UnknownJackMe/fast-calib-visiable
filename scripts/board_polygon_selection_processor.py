#!/usr/bin/env python3
"""从 RViz 任意多边形选择结果提取高分辨率标定板点云。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial import ConvexHull
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


EXPECTED_WIDTH = 0.500
EXPECTED_HEIGHT = 0.400


def read_ascii_ply(path: Path) -> np.ndarray:
    vertex_count = None
    header_lines = 0
    with path.open("r", encoding="ascii") as stream:
        for line in stream:
            header_lines += 1
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break
    if vertex_count is None:
        raise ValueError(f"PLY 文件没有 vertex 数量: {path}")
    data = np.loadtxt(
        path,
        skiprows=header_lines,
        max_rows=vertex_count,
        usecols=(0, 1, 2),
        dtype=np.float32,
    )
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return np.asarray(data[:, :3], dtype=np.float32)


def write_ascii_ply(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("end_header\n")
        np.savetxt(stream, points, fmt="%.7f")


def make_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.height = 1
    message.width = int(len(points))
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * int(len(points))
    message.is_dense = True
    message.data = np.asarray(points, dtype=np.float32).tobytes()
    return message


def fit_plane_ransac(points: np.ndarray, threshold: float = 0.02) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(points) < 3:
        raise ValueError("多边形选择点太少，无法拟合平面")
    rng = np.random.default_rng(20260816)
    iterations = min(1200, max(200, len(points) * 4))
    best_mask = None
    best_count = -1
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)].astype(np.float64)
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        distance = np.abs((points - sample[0]) @ normal)
        mask = distance <= threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 100:
        raise ValueError("多边形选择区域内没有足够的平面点")

    inliers = points[best_mask].astype(np.float64)
    origin = inliers.mean(axis=0)
    _, _, vectors = np.linalg.svd(inliers - origin, full_matrices=False)
    normal = vectors[-1]
    normal /= np.linalg.norm(normal)
    distances = np.abs((points - origin) @ normal)
    refined_mask = distances <= threshold
    refined = points[refined_mask].astype(np.float64)
    origin = refined.mean(axis=0)
    _, _, vectors = np.linalg.svd(refined - origin, full_matrices=False)
    normal = vectors[-1]
    normal /= np.linalg.norm(normal)
    return origin, normal, refined


def plane_coordinates(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    origin = points.mean(axis=0)
    _, _, vectors = np.linalg.svd(points - origin, full_matrices=False)
    axis_u = vectors[0]
    axis_v = vectors[1]
    normal = vectors[2]
    axis_u /= np.linalg.norm(axis_u)
    axis_v /= np.linalg.norm(axis_v)
    normal /= np.linalg.norm(normal)
    uv = np.column_stack(((points - origin) @ axis_u, (points - origin) @ axis_v))
    return origin, axis_u, axis_v, uv


def hull_mask(points_uv: np.ndarray, hull_uv: np.ndarray, margin: float) -> tuple[np.ndarray, np.ndarray]:
    if len(hull_uv) < 3:
        raise ValueError("选中平面点不足以构成多边形")
    hull = ConvexHull(hull_uv)
    equations = hull.equations
    normals = equations[:, :-1]
    offsets = equations[:, -1]
    norms = np.linalg.norm(normals, axis=1)
    expanded = points_uv @ normals.T + offsets[None, :] <= margin * norms[None, :]
    return np.all(expanded, axis=1), hull_uv[hull.vertices]


def serializable_vector(vector: np.ndarray) -> list[float]:
    return [round(float(value), 6) for value in vector]


class BoardPolygonSelectionProcessor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("board_polygon_selection_processor")
        self.args = args
        self.frame_id = args.frame_id
        self.full_cloud = read_ascii_ply(args.full_cloud.resolve())
        self.preview_cloud = read_ascii_ply(args.preview_cloud.resolve())
        self.selected_board: np.ndarray | None = None
        self.board_markers: MarkerArray | None = None
        if args.output.exists():
            self.selected_board = read_ascii_ply(args.output.resolve())
        static_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self.preview_publisher = self.create_publisher(
            PointCloud2, "/static_cloud_preview", static_qos
        )
        self.board_publisher = self.create_publisher(
            PointCloud2, "/selected_board_cloud", static_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/selected_board_markers", static_qos
        )
        self.selected_subscription = self.create_subscription(
            PointCloud2,
            "/board_polygon_selected_points",
            self.on_selected_points,
            1,
        )
        self.clear_subscription = self.create_subscription(
            Empty, "/clear_board_polygon_selection", self.on_clear_selection, 1
        )
        self.initial_publish_timer = self.create_timer(0.5, self.publish_initial_clouds)
        self.get_logger().info(
            f"已加载完整处理点云 {len(self.full_cloud)} 点，RViz 预览点云 {len(self.preview_cloud)} 点。"
        )

    def on_clear_selection(self, _message: Empty):
        self.selected_board = None
        self.board_markers = None
        for path in (self.args.output, self.args.report):
            try:
                path.unlink(missing_ok=True)
            except OSError as error:
                self.get_logger().warning(f"清理候选文件失败：{error}")

        stamp = self.get_clock().now().to_msg()
        self.board_publisher.publish(
            make_cloud(np.empty((0, 3), dtype=np.float32), self.frame_id, stamp)
        )
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(delete_all)
        self.marker_publisher.publish(markers)
        self.get_logger().info("已清除未确认的标定板候选结果，请重新拖动套索。")

    def publish_initial_clouds(self):
        stamp = self.get_clock().now().to_msg()
        self.preview_publisher.publish(make_cloud(self.preview_cloud, self.frame_id, stamp))
        if self.selected_board is not None:
            self.board_publisher.publish(make_cloud(self.selected_board, self.frame_id, stamp))
        if self.board_markers is not None:
            self.marker_publisher.publish(self.board_markers)
        self.initial_publish_timer.cancel()

    def make_board_markers(
        self,
        origin: np.ndarray,
        normal: np.ndarray,
        axis_u: np.ndarray,
        axis_v: np.ndarray,
        hull_uv: np.ndarray,
    ) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        outline = Marker()
        outline.header.frame_id = self.frame_id
        outline.header.stamp = stamp
        outline.ns = "selected_board_outline"
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.018
        outline.color.r = 1.0
        outline.color.g = 0.65
        outline.color.b = 0.05
        outline.color.a = 1.0
        for uv in np.vstack((hull_uv, hull_uv[0])):
            xyz = origin + axis_u * uv[0] + axis_v * uv[1]
            outline.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        markers.markers.append(outline)

        arrow = Marker()
        arrow.header.frame_id = self.frame_id
        arrow.header.stamp = stamp
        arrow.ns = "selected_board_normal"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.025
        arrow.scale.y = 0.05
        arrow.scale.z = 0.07
        arrow.color.r = 0.1
        arrow.color.g = 0.65
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.points = [
            Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
            Point(
                x=float(origin[0] + normal[0] * 0.35),
                y=float(origin[1] + normal[1] * 0.35),
                z=float(origin[2] + normal[2] * 0.35),
            ),
        ]
        markers.markers.append(arrow)
        return markers

    def on_selected_points(self, message: PointCloud2):
        selected = np.frombuffer(message.data, dtype=np.float32).reshape(-1, message.point_step // 4)
        selected = selected[:, :3].copy()
        if len(selected) < 100:
            self.get_logger().error(f"多边形选择只有 {len(selected)} 个点，至少需要 100 个点。")
            return

        try:
            origin, normal, plane_points = fit_plane_ransac(selected)
            plane_origin, axis_u, axis_v, selected_uv = plane_coordinates(plane_points)
            full_uv = np.column_stack(
                ((self.full_cloud - plane_origin) @ axis_u, (self.full_cloud - plane_origin) @ axis_v)
            )
            full_distance = np.abs((self.full_cloud - plane_origin) @ normal)
            hull, hull_uv = hull_mask(full_uv, selected_uv, margin=self.args.hull_margin)
            board_mask = (full_distance <= self.args.plane_thickness) & hull
            board = self.full_cloud[board_mask]
            if len(board) < self.args.min_board_points:
                raise ValueError(
                    f"提取后的标定板点太少：{len(board)}，至少需要 {self.args.min_board_points}"
                )

            _, _, _, board_uv = plane_coordinates(board)
            extents = np.ptp(board_uv, axis=0)
            plane_rmse = float(np.sqrt(np.mean(((plane_points - plane_origin) @ normal) ** 2)))
            status = "pass" if plane_rmse <= self.args.max_plane_rmse else "warn"
            self.selected_board = board
            self.board_markers = self.make_board_markers(
                plane_origin, normal, axis_u, axis_v, hull_uv
            )
            write_ascii_ply(self.args.output.resolve(), board)
            self.board_publisher.publish(
                make_cloud(board, self.frame_id, self.get_clock().now().to_msg())
            )
            self.marker_publisher.publish(self.board_markers)
            report = {
                "kind": "selected_board_cloud",
                "version": 1,
                "status": status,
                "frame_id": self.frame_id,
                "source_full_cloud": str(self.args.full_cloud.resolve()),
                "source_preview_cloud": str(self.args.preview_cloud.resolve()),
                "selected_preview_points": int(len(selected)),
                "selected_plane_points": int(len(plane_points)),
                "output_points": int(len(board)),
                "plane_origin": serializable_vector(plane_origin),
                "plane_normal": serializable_vector(normal),
                "plane_rmse_m": plane_rmse,
                "board_projected_extents_m": [float(value) for value in extents],
                "selection_hull_uv": [
                    [round(float(value), 6) for value in point] for point in hull_uv
                ],
                "plane_thickness_m": self.args.plane_thickness,
                "hull_margin_m": self.args.hull_margin,
                "warnings": [],
            }
            if extents.min() < 0.30:
                report["status"] = "warn"
                report["warnings"].append("选区提取的平面尺寸偏小，请确认标定板没有被框选截断。")
            self.args.report.parent.mkdir(parents=True, exist_ok=True)
            self.args.report.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
            self.get_logger().info(
                f"标定板提取完成：{len(board)} 点，平面 RMSE={plane_rmse:.4f} m，"
                f"投影尺寸={extents[0]:.3f} x {extents[1]:.3f} m。"
            )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"标定板提取失败：{error}")


def main() -> None:
    parser = argparse.ArgumentParser(description="RViz 多边形选区驱动的标定板提取器")
    parser.add_argument("--full-cloud", required=True, type=Path)
    parser.add_argument("--preview-cloud", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--frame-id", default="livox_frame")
    parser.add_argument("--plane-thickness", type=float, default=0.025)
    parser.add_argument("--hull-margin", type=float, default=0.05)
    parser.add_argument("--max-plane-rmse", type=float, default=0.02)
    parser.add_argument("--min-board-points", type=int, default=500)
    args = parser.parse_args()

    rclpy.init()
    node = BoardPolygonSelectionProcessor(args)
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
