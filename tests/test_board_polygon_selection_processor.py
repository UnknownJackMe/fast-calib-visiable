#!/usr/bin/env python3
import unittest

import numpy as np

from scripts.board_polygon_selection_processor import fit_plane_ransac, hull_mask, plane_coordinates


class BoardPolygonSelectionProcessorTest(unittest.TestCase):
    def test_tilted_board_plane_wins_over_ground_outliers(self):
        rng = np.random.default_rng(12)
        normal = np.asarray([0.72, 0.05, 0.692], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        axis_u = np.asarray([normal[2], 0.0, -normal[0]], dtype=np.float64)
        axis_u /= np.linalg.norm(axis_u)
        axis_v = np.cross(normal, axis_u)
        axis_v /= np.linalg.norm(axis_v)
        origin = np.asarray([2.3, 0.0, 1.0], dtype=np.float64)

        u = rng.uniform(-0.55, 0.55, 900)
        v = rng.uniform(-0.45, 0.45, 900)
        board = origin + u[:, None] * axis_u + v[:, None] * axis_v
        board += rng.normal(0.0, 0.003, (len(board), 1)) * normal

        ground_y = rng.uniform(-0.7, 0.7, 180)
        ground_x = rng.uniform(1.8, 3.2, 180)
        ground = np.column_stack((ground_x, ground_y, np.full(180, 0.35)))
        selected = np.vstack((board, ground)).astype(np.float32)

        fitted_origin, fitted_normal, inliers = fit_plane_ransac(selected, threshold=0.015)
        angle = np.degrees(np.arccos(np.clip(abs(float(fitted_normal @ normal)), 0.0, 1.0)))
        self.assertLess(angle, 2.0)
        self.assertGreater(len(inliers), 800)
        self.assertLess(abs(float((fitted_origin - origin) @ normal)), 0.02)

    def test_convex_hull_keeps_board_region_and_rejects_outside_points(self):
        selected_uv = np.asarray(
            [[-0.5, -0.4], [0.5, -0.4], [0.5, 0.4], [-0.5, 0.4]],
            dtype=np.float64,
        )
        query = np.asarray(
            [[0.0, 0.0], [0.53, 0.0], [0.70, 0.0], [0.0, -0.46]],
            dtype=np.float64,
        )
        mask, hull = hull_mask(query, selected_uv, margin=0.05)
        self.assertEqual(mask.tolist(), [True, True, False, False])
        self.assertEqual(len(hull), 4)

    def test_plane_coordinates_flatten_tilted_points(self):
        points = np.asarray(
            [[2.0, -0.5, 1.0], [2.2, 0.5, 0.8], [2.1, 0.0, 0.9], [1.9, 0.2, 1.1]],
            dtype=np.float64,
        )
        origin, axis_u, axis_v, uv = plane_coordinates(points)
        reconstructed = origin + uv[:, :1] * axis_u + uv[:, 1:] * axis_v
        self.assertLess(float(np.max(np.linalg.norm(points - reconstructed, axis=1))), 1e-6)


if __name__ == "__main__":
    unittest.main()
