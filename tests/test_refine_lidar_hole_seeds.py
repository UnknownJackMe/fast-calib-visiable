#!/usr/bin/env python3
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from scripts.interactive_lidar_hole_editor import (
    LidarHoleEditor,
    load_centers,
    make_move_axis_control,
)


ROOT = Path(__file__).resolve().parents[1]


def write_ply(path, points):
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("end_header\n")
        np.savetxt(stream, points, fmt="%.9f")


class RefineLidarHoleSeedsTest(unittest.TestCase):
    def test_move_axis_control_is_fixed_translation(self):
        control = make_move_axis_control("move_x", (1.0, 1.0, 0.0, 0.0))
        self.assertEqual(control.interaction_mode, control.MOVE_AXIS)
        self.assertEqual(control.orientation_mode, control.FIXED)

    def test_refined_marker_array_clears_stale_markers(self):
        editor = SimpleNamespace(frame_id="livox_frame", marker_scale=0.18)
        markers = LidarHoleEditor.make_refined_markers(editor, None, Time())
        self.assertEqual(len(markers.markers), 1)
        self.assertEqual(markers.markers[0].action, Marker.DELETEALL)

    def test_refined_markers_require_top_level_pass_status(self):
        centers = [
            {"name": name, "x": 1.0, "y": float(index), "z": 2.0, "status": "pass"}
            for index, name in enumerate(
                ["upper +Y", "upper -Y", "lower +Y", "lower -Y"]
            )
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "refined.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "kind": "refined_lidar_hole_centers",
                        "status": "fail",
                        "centers": centers,
                    },
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            self.assertIsNone(load_centers(path, require_validated_refinement=True))

            data = yaml.safe_load(path.read_text(encoding="utf-8"))
            data["status"] = "pass"
            path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            self.assertEqual(
                len(load_centers(path, require_validated_refinement=True)), 4
            )

    def test_refines_four_rough_seeds_on_synthetic_plane(self):
        rng = np.random.default_rng(9)
        y_values = np.arange(-0.65, 0.651, 0.010)
        z_values = np.arange(-0.50, 0.501, 0.010)
        grid_y, grid_z = np.meshgrid(y_values, z_values, indexing="xy")
        centers_yz = np.asarray(
            [[0.25, 0.20], [-0.25, 0.20], [0.25, -0.20], [-0.25, -0.20]]
        )
        keep = np.ones(grid_y.size, dtype=bool)
        flat_y = grid_y.ravel()
        flat_z = grid_z.ravel()
        for center_y, center_z in centers_yz:
            keep &= np.hypot(flat_y - center_y, flat_z - center_z) >= 0.11
        flat_y = flat_y[keep]
        flat_z = flat_z[keep]
        flat_x = 3.0 + rng.normal(0.0, 0.002, len(flat_y))
        points = np.column_stack((flat_x, flat_y, flat_z))

        rough_offsets = np.asarray(
            [[0.04, -0.07, 0.06], [-0.03, 0.06, -0.05], [0.02, -0.08, 0.05], [-0.04, 0.07, -0.04]]
        )
        names = ["upper +Y", "upper -Y", "lower +Y", "lower -Y"]
        true_xyz = np.column_stack((np.full(4, 3.0), centers_yz[:, 0], centers_yz[:, 1]))
        rough_xyz = true_xyz + rough_offsets

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            plane_path = temp / "plane.ply"
            seed_path = temp / "seeds.yaml"
            output_path = temp / "refined.yaml"
            report_path = temp / "report.yaml"
            debug_dir = temp / "debug"
            write_ply(plane_path, points)
            seed_path.write_text(
                yaml.safe_dump(
                    {
                        "kind": "rough_lidar_hole_seeds",
                        "frame_id": "livox_frame",
                        "centers": [
                            {
                                "name": name,
                                "x": float(point[0]),
                                "y": float(point[1]),
                                "z": float(point[2]),
                            }
                            for name, point in zip(names, rough_xyz)
                        ],
                    },
                    sort_keys=False,
                ),
                encoding="utf-8",
            )

            subprocess.run(
                [
                    "python3",
                    str(ROOT / "scripts/refine_lidar_hole_seeds.py"),
                    "--plane-cloud",
                    str(plane_path),
                    "--seeds",
                    str(seed_path),
                    "--config",
                    str(ROOT / "config/hole_refinement_params.yaml"),
                    "--output",
                    str(output_path),
                    "--report",
                    str(report_path),
                    "--debug-dir",
                    str(debug_dir),
                ],
                check=True,
                capture_output=True,
                text=True,
            )

            result = yaml.safe_load(output_path.read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "pass")
            refined = np.asarray(
                [[item["x"], item["y"], item["z"]] for item in result["centers"]]
            )
            errors = np.linalg.norm(refined - true_xyz, axis=1)
            self.assertLess(float(errors.max()), 0.020)


if __name__ == "__main__":
    unittest.main()
