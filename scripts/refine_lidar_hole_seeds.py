#!/usr/bin/env python3
import argparse
import itertools
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree


EXPECTED_NAMES = ["upper +Y", "upper -Y", "lower +Y", "lower -Y"]


def read_ascii_ply(path):
    vertex_count = None
    points = []
    with path.open("r", encoding="ascii") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[-1])
            if stripped == "end_header":
                break
        if vertex_count is None:
            raise ValueError(f"{path} has no PLY vertex count")
        for _ in range(vertex_count):
            values = stream.readline().split()
            if len(values) >= 3:
                points.append([float(values[0]), float(values[1]), float(values[2])])
    if not points:
        raise ValueError(f"{path} contains no points")
    return np.asarray(points, dtype=np.float64)


def write_ascii_ply(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    with path.open("w", encoding="ascii") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("end_header\n")
        for point in points:
            stream.write(f"{point[0]:.9f} {point[1]:.9f} {point[2]:.9f}\n")


def load_seeds(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    centers = data.get("centers", []) if data else []
    by_name = {
        str(item.get("name")): np.asarray(
            [float(item["x"]), float(item["y"]), float(item["z"])],
            dtype=np.float64,
        )
        for item in centers
    }
    missing = [name for name in EXPECTED_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"Missing named seeds: {', '.join(missing)}")
    return data.get("frame_id", "livox_frame"), by_name


def load_config(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not data or "hole_refinement" not in data:
        raise ValueError(f"Missing hole_refinement section in {path}")
    return data["hole_refinement"]


def fit_circle(points):
    matrix = np.column_stack(
        (2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points)))
    )
    target = np.sum(points * points, axis=1)
    solution = np.linalg.lstsq(matrix, target, rcond=None)[0]
    center = solution[:2]
    radius_sq = solution[2] + float(center @ center)
    return center, math.sqrt(max(0.0, radius_sq))


def plane_coordinates(points):
    origin = points.mean(axis=0)
    _u, _s, vectors = np.linalg.svd(points - origin, full_matrices=False)
    normal = vectors[2]
    normal /= np.linalg.norm(normal)

    lidar_y = np.asarray([0.0, 1.0, 0.0])
    axis_u = lidar_y - normal * float(lidar_y @ normal)
    if np.linalg.norm(axis_u) < 1e-6:
        lidar_z = np.asarray([0.0, 0.0, 1.0])
        axis_u = lidar_z - normal * float(lidar_z @ normal)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    axis_v /= np.linalg.norm(axis_v)
    if axis_v[2] < 0.0:
        axis_v = -axis_v

    uv = np.column_stack(((points - origin) @ axis_u, (points - origin) @ axis_v))
    plane_residuals = np.abs((points - origin) @ normal)
    return origin, normal, axis_u, axis_v, uv, plane_residuals


def extract_dominant_plane(points, config):
    rng = np.random.default_rng(17)
    threshold = float(config["plane_ransac_distance_threshold"])
    best_mask = None
    best_count = 0
    for _ in range(int(config["plane_ransac_iterations"])):
        first, second, third = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(second - first, third - first)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        mask = np.abs((points - first) @ normal) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < int(config["plane_ransac_min_inliers"]):
        raise RuntimeError(f"Dominant plane RANSAC found only {best_count} inliers")

    initial_plane = points[best_mask]
    origin = initial_plane.mean(axis=0)
    _u, _s, vectors = np.linalg.svd(initial_plane - origin, full_matrices=False)
    normal = vectors[2]
    refined_mask = np.abs((points - origin) @ normal) < threshold
    refined = points[refined_mask]
    if len(refined) < int(config["plane_ransac_min_inliers"]):
        raise RuntimeError(f"Refined dominant plane has only {len(refined)} inliers")
    return refined


def project_point(point, origin, normal, axis_u, axis_v):
    projected = point - normal * float((point - origin) @ normal)
    uv = np.asarray(
        [float((projected - origin) @ axis_u), float((projected - origin) @ axis_v)]
    )
    return projected, uv


def uv_to_xyz(center, origin, axis_u, axis_v):
    return origin + axis_u * center[0] + axis_v * center[1]


def boundary_points_for_candidate(local_uv, center, config):
    delta = local_uv - center
    radii = np.linalg.norm(delta, axis=1)
    mask = radii < float(config["boundary_search_radius"])
    delta = delta[mask]
    radii = radii[mask]
    if len(delta) == 0:
        return np.empty((0, 2), dtype=np.float64)

    angles = (np.arctan2(delta[:, 1], delta[:, 0]) + 2.0 * np.pi) % (2.0 * np.pi)
    angular_bins = int(config["angular_bins"])
    bins = np.floor(angles / (2.0 * np.pi) * angular_bins).astype(int)
    selected = []
    for bin_index in range(angular_bins):
        indices = np.flatnonzero(bins == bin_index)
        if len(indices) == 0:
            continue
        nearest = indices[np.argmin(radii[indices])]
        radius = radii[nearest]
        if float(config["boundary_min_radius"]) < radius < float(
            config["boundary_max_radius"]
        ):
            selected.append(delta[nearest] + center)
    return np.asarray(selected, dtype=np.float64).reshape((-1, 2))


def refine_circle(boundary_points, config):
    center, radius = fit_circle(boundary_points)
    residuals = np.abs(np.linalg.norm(boundary_points - center, axis=1) - radius)
    cutoff = max(0.012, float(np.quantile(residuals, 0.80)))
    inlier_mask = residuals < cutoff
    if int(inlier_mask.sum()) < int(config["min_boundary_bins"]) - 4:
        return None
    inliers = boundary_points[inlier_mask]
    center, radius = fit_circle(inliers)
    residuals = np.abs(np.linalg.norm(inliers - center, axis=1) - radius)
    return center, radius, residuals, inliers


def generate_candidates(
    name,
    seed,
    plane_points,
    plane_uv,
    tree,
    origin,
    normal,
    axis_u,
    axis_v,
    config,
    debug_dir,
):
    _seed_projected, seed_uv = project_point(seed, origin, normal, axis_u, axis_v)
    local_radius = max(float(config["search_radius_u"]), float(config["search_radius_v"]))
    local_radius += float(config["boundary_search_radius"])
    local_mask = np.linalg.norm(plane_uv - seed_uv, axis=1) < local_radius
    local_uv = plane_uv[local_mask]
    local_xyz = plane_points[local_mask]

    safe_name = name.lower().replace(" ", "_").replace("+", "pos").replace("-", "neg")
    write_ascii_ply(debug_dir / f"{safe_name}_roi.ply", local_xyz)

    step = float(config["grid_step"])
    offsets_u = np.arange(
        -float(config["search_radius_u"]), float(config["search_radius_u"]) + step * 0.5, step
    )
    offsets_v = np.arange(
        -float(config["search_radius_v"]), float(config["search_radius_v"]) + step * 0.5, step
    )
    grid_u, grid_v = np.meshgrid(offsets_u, offsets_v, indexing="xy")
    grid = np.column_stack((seed_uv[0] + grid_u.ravel(), seed_uv[1] + grid_v.ravel()))
    clearances, _indices = tree.query(grid, k=1)
    valid = np.flatnonzero(
        (clearances > float(config["min_clearance"]))
        & (clearances < float(config["max_clearance"]))
    )
    valid = valid[np.argsort(clearances[valid])[::-1]]

    candidates = []
    for grid_index in valid[: int(config["max_grid_candidates"])]:
        initial_center = grid[grid_index]
        boundary = boundary_points_for_candidate(local_uv, initial_center, config)
        if len(boundary) < int(config["min_boundary_bins"]):
            continue
        fitted = refine_circle(boundary, config)
        if fitted is None:
            continue
        center, radius, residuals, inliers = fitted
        if not float(config["min_fitted_radius"]) < radius < float(
            config["max_fitted_radius"]
        ):
            continue

        center_xyz = uv_to_xyz(center, origin, axis_u, axis_v)
        seed_distance = float(np.linalg.norm(center_xyz - seed))
        radial_rmse = float(np.sqrt(np.mean(residuals * residuals)))
        local_score = (
            radial_rmse / 0.010
            + abs(radius - float(config["expected_radius"])) / 0.035
            + 0.15 * seed_distance / float(config["max_seed_distance"])
            - 0.005 * len(inliers)
        )
        candidate = {
            "name": name,
            "score": float(local_score),
            "center_uv": center,
            "center_xyz": center_xyz,
            "radius": float(radius),
            "radial_rmse": radial_rmse,
            "inlier_count": int(len(inliers)),
            "boundary_bin_count": int(len(boundary)),
            "seed_distance": seed_distance,
            "inliers_uv": inliers,
        }

        duplicate_index = next(
            (
                index
                for index, existing in enumerate(candidates)
                if np.linalg.norm(existing["center_uv"] - center)
                < float(config["candidate_dedup_distance"])
            ),
            None,
        )
        if duplicate_index is None:
            candidates.append(candidate)
        elif candidate["score"] < candidates[duplicate_index]["score"]:
            candidates[duplicate_index] = candidate

    candidates.sort(key=lambda item: item["score"])
    return seed_uv, candidates[: int(config["max_candidates_per_seed"])]


def combination_metrics(combo, config):
    points = [item["center_xyz"] for item in combo]

    def distance(first, second):
        return float(np.linalg.norm(points[first] - points[second]))

    metrics = {
        "upper_width": distance(0, 1),
        "lower_width": distance(2, 3),
        "positive_y_height": distance(0, 2),
        "negative_y_height": distance(1, 3),
        "diagonal_upper_pos_to_lower_neg": distance(0, 3),
        "diagonal_upper_neg_to_lower_pos": distance(1, 2),
    }
    expected_diagonal = math.hypot(
        float(config["expected_width"]), float(config["expected_height"])
    )
    geometry_score = (
        (
            abs(metrics["upper_width"] - float(config["expected_width"]))
            + abs(metrics["lower_width"] - float(config["expected_width"]))
        )
        / 0.025
        + (
            abs(metrics["positive_y_height"] - float(config["expected_height"]))
            + abs(metrics["negative_y_height"] - float(config["expected_height"]))
        )
        / 0.025
        + (
            abs(metrics["diagonal_upper_pos_to_lower_neg"] - expected_diagonal)
            + abs(metrics["diagonal_upper_neg_to_lower_pos"] - expected_diagonal)
        )
        / 0.040
    )
    total_score = sum(item["score"] for item in combo) + geometry_score
    return float(total_score), float(geometry_score), metrics


def choose_combination(candidate_lists, config):
    best = None
    best_valid = None
    for combo in itertools.product(*candidate_lists):
        total_score, geometry_score, metrics = combination_metrics(combo, config)
        selection = (total_score, geometry_score, metrics, combo)
        if best is None or total_score < best[0]:
            best = selection

        geometry_failures = validate_geometry(metrics, config)
        candidate_failures = [
            failure
            for candidate in combo
            for failure in validate_candidate(candidate, config)
        ]
        if not geometry_failures and not candidate_failures:
            if best_valid is None or total_score < best_valid[0]:
                best_valid = selection
    return best_valid if best_valid is not None else best


def validate_candidate(candidate, config):
    failures = []
    if candidate["radial_rmse"] > float(config["max_radial_rmse"]):
        failures.append("radial_rmse")
    if candidate["seed_distance"] > float(config["max_seed_distance"]):
        failures.append("seed_distance")
    if not float(config["min_fitted_radius"]) < candidate["radius"] < float(
        config["max_fitted_radius"]
    ):
        failures.append("radius")
    return failures


def validate_geometry(metrics, config):
    failures = []
    for key in ("upper_width", "lower_width"):
        if abs(metrics[key] - float(config["expected_width"])) > float(
            config["max_width_error"]
        ):
            failures.append(key)
    for key in ("positive_y_height", "negative_y_height"):
        if abs(metrics[key] - float(config["expected_height"])) > float(
            config["max_height_error"]
        ):
            failures.append(key)
    expected_diagonal = math.hypot(
        float(config["expected_width"]), float(config["expected_height"])
    )
    for key in (
        "diagonal_upper_pos_to_lower_neg",
        "diagonal_upper_neg_to_lower_pos",
    ):
        if abs(metrics[key] - expected_diagonal) > float(config["max_diagonal_error"]):
            failures.append(key)
    return failures


def serializable_candidate(candidate, seed, failures):
    center = candidate["center_xyz"]
    return {
        "name": candidate["name"],
        "x": round(float(center[0]), 6),
        "y": round(float(center[1]), 6),
        "z": round(float(center[2]), 6),
        "seed": [round(float(value), 6) for value in seed],
        "seed_distance_m": round(candidate["seed_distance"], 6),
        "fitted_radius_m": round(candidate["radius"], 6),
        "radial_rmse_m": round(candidate["radial_rmse"], 6),
        "inlier_count": candidate["inlier_count"],
        "boundary_bin_count": candidate["boundary_bin_count"],
        "status": "pass" if not failures else "fail",
        "failures": failures,
    }


def main():
    parser = argparse.ArgumentParser(description="Refine four LiDAR hole centers from rough RViz seeds")
    parser.add_argument("--plane-cloud", type=Path)
    parser.add_argument("--filtered-cloud", type=Path)
    parser.add_argument("--seeds", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--debug-dir", required=True, type=Path)
    args = parser.parse_args()

    config = load_config(args.config.resolve())
    frame_id, seeds = load_seeds(args.seeds.resolve())
    if args.plane_cloud and args.plane_cloud.exists():
        plane_source = args.plane_cloud.resolve()
        plane_points = read_ascii_ply(plane_source)
        plane_source_kind = "presegmented_plane_cloud"
    elif args.filtered_cloud and args.filtered_cloud.exists():
        plane_source = args.filtered_cloud.resolve()
        filtered_points = read_ascii_ply(plane_source)
        plane_points = extract_dominant_plane(filtered_points, config)
        plane_source_kind = "ransac_from_filtered_cloud"
        write_ascii_ply(args.debug_dir.resolve() / "ransac_plane.ply", plane_points)
    else:
        raise RuntimeError("Provide an existing --plane-cloud or --filtered-cloud")
    origin, normal, axis_u, axis_v, plane_uv, plane_residuals = plane_coordinates(plane_points)
    tree = cKDTree(plane_uv)

    candidate_lists = []
    seed_uv_values = {}
    failure_reasons = []
    for name in EXPECTED_NAMES:
        seed_uv, candidates = generate_candidates(
            name,
            seeds[name],
            plane_points,
            plane_uv,
            tree,
            origin,
            normal,
            axis_u,
            axis_v,
            config,
            args.debug_dir.resolve(),
        )
        seed_uv_values[name] = seed_uv
        candidate_lists.append(candidates)
        print(f"{name}: {len(candidates)} local circle candidates")
        if not candidates:
            failure_reasons.append(f"{name}: no valid local circle candidates")

    selected = None if failure_reasons else choose_combination(candidate_lists, config)
    if selected is None:
        overall_status = "fail"
        geometry_score = None
        metrics = {}
        combo = []
    else:
        total_score, geometry_score, metrics, combo = selected
        geometry_failures = validate_geometry(metrics, config)
        per_hole_failures = [
            validate_candidate(candidate, config) for candidate in combo
        ]
        failure_reasons.extend(f"geometry: {item}" for item in geometry_failures)
        for name, failures in zip(EXPECTED_NAMES, per_hole_failures):
            failure_reasons.extend(f"{name}: {item}" for item in failures)
        overall_status = "pass" if not failure_reasons else "fail"

    refined_centers = []
    if combo:
        for name, candidate in zip(EXPECTED_NAMES, combo):
            failures = validate_candidate(candidate, config)
            refined_centers.append(serializable_candidate(candidate, seeds[name], failures))
            inlier_xyz = np.asarray(
                [uv_to_xyz(point, origin, axis_u, axis_v) for point in candidate["inliers_uv"]]
            )
            safe_name = name.lower().replace(" ", "_").replace("+", "pos").replace("-", "neg")
            write_ascii_ply(args.debug_dir.resolve() / f"{safe_name}_circle_inliers.ply", inlier_xyz)

    output_data = {
        "kind": "refined_lidar_hole_centers",
        "version": 1,
        "status": overall_status,
        "frame_id": frame_id,
        "source_plane_cloud": str(plane_source),
        "plane_source_kind": plane_source_kind,
        "source_seeds": str(args.seeds.resolve()),
        "centers": refined_centers,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(output_data, sort_keys=False), encoding="utf-8")

    report = {
        "status": overall_status,
        "algorithm": "maximum_empty_circle_with_joint_geometry_v1",
        "plane": {
            "point_count": int(len(plane_points)),
            "source": str(plane_source),
            "source_kind": plane_source_kind,
            "origin": [float(value) for value in origin],
            "normal": [float(value) for value in normal],
            "rmse_m": float(np.sqrt(np.mean(plane_residuals * plane_residuals))),
        },
        "candidate_counts": {
            name: len(candidates) for name, candidates in zip(EXPECTED_NAMES, candidate_lists)
        },
        "geometry_score": geometry_score,
        "geometry": {key: float(value) for key, value in metrics.items()},
        "failures": failure_reasons,
        "refined_centers": refined_centers,
        "config": config,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")

    if combo:
        write_ascii_ply(
            args.debug_dir.resolve() / "refined_centers.ply",
            np.asarray([candidate["center_xyz"] for candidate in combo]),
        )

    print(f"Refinement status: {overall_status}")
    for item in refined_centers:
        print(
            f"  {item['name']}: ({item['x']:.6f}, {item['y']:.6f}, {item['z']:.6f}) "
            f"r={item['fitted_radius_m']:.4f} rmse={item['radial_rmse_m']:.4f} "
            f"seed_delta={item['seed_distance_m']:.4f}"
        )
    if metrics:
        print("Geometry:")
        for key, value in metrics.items():
            print(f"  {key}: {value:.6f} m")
    if failure_reasons:
        print("Failures:")
        for reason in failure_reasons:
            print(f"  {reason}")
    print(f"Refined centers: {args.output}")
    print(f"Report: {args.report}")
    raise SystemExit(0 if overall_status == "pass" else 3)


if __name__ == "__main__":
    main()
