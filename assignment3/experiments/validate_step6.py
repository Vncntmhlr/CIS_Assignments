"""Offline Step 6 validation: synthetic truth and replay of saved practice measurements.

Run from assignment3 with PYTHONPATH=.; no server or hidden-truth methods are used.
"""

import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from src import Frame, Rot, load_patch_boxes, load_sur, vct3
from src.session import frame_to_json
from src.student import closest_point, icp, report
from experiments.test_step6_registration import apply, surface_case


ROOT = Path(__file__).resolve().parents[1]


def summarize(result):
    """Serialize final ICP fields and Euclidean inlier residual statistics (mm)."""
    residuals = result.residuals[result.inlier_mask]
    return {"F_reg_reference_to_ct": frame_to_json(result.F_reg),
            "converged": result.converged, "iterations": result.n_iter,
            "n_samples": len(result.residuals), "n_used": int(result.inlier_mask.sum()),
            "inlier_rms_mm": float(np.sqrt(np.mean(residuals ** 2))),
            "inlier_residual_stats_mm": report.residual_statistics(residuals),
            "history_inlier_rms_mm": result.history}


def main():
    """Write reproducible numerical evidence without changing the notebook."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "experiments/results/step5_practice.json")
    parser.add_argument("--output", type=Path, default=ROOT / "experiments/results/step6_validation.json")
    args = parser.parse_args()
    evidence = {"validation": "offline only; saved practice observations and synthetic truth",
                "configuration": {"max_iter": 100, "tol_mm": 1e-4, "reject": "mad",
                    "mad_normal_scale": icp._MAD_NORMAL_SCALE, "mad_cutoff": icp._MAD_CUTOFF,
                    "mad_scale_floor_mm": icp._MAD_SCALE_FLOOR_MM,
                    "motion_tolerance_factor": icp._MOTION_TOL_FACTOR,
                    "implementation_sha256": hashlib.sha256(Path(icp.__file__).read_bytes()).hexdigest()},
                "synthetic": []}
    for seed in (11, 22, 33):
        mesh, surface, truth, samples = surface_case(seed, count=120)
        rng = np.random.default_rng(seed + 100)
        bad = np.zeros(len(samples), dtype=bool)
        bad[rng.choice(len(samples), 12, replace=False)] = True
        samples[bad] += rng.normal(size=(12, 3)) * 20
        initial = Frame(Rot("z", 0.04), vct3(1, -1, 0.5)) * truth
        start = perf_counter()
        result = icp.icp(samples, mesh, initial)
        seconds = perf_counter() - start
        clean_errors = np.linalg.norm(apply(result.F_reg, samples[~bad]) - surface[~bad], axis=1)
        mesh_errors = np.linalg.norm(apply(result.F_reg * truth.inv(), mesh.vertices) - mesh.vertices, axis=1)
        record = summarize(result)
        record.update(seed=seed, input_kind="synthetic surface samples with known Reference-to-CT truth",
                      n_injected_outliers=12, wall_icp_seconds=seconds,
                      clean_max_paired_error_mm=float(clean_errors.max()),
                      synthetic_mesh_displacement_rms_mm=float(np.sqrt(np.mean(mesh_errors ** 2))),
                      rejected_injected_outliers=int(np.count_nonzero(bad & ~result.inlier_mask)),
                      rejected_clean_samples=int(np.count_nonzero(~bad & ~result.inlier_mask)))
        evidence["synthetic"].append(record)
        print(json.dumps({k: record[k] for k in ("seed", "converged", "iterations", "n_used",
              "clean_max_paired_error_mm", "synthetic_mesh_displacement_rms_mm", "wall_icp_seconds")}))

    saved = json.loads(args.input.read_text())
    rows = saved["samples"]
    samples = np.array([row["d_ref_mm"] for row in rows])
    by_patch = {}
    for row in rows:
        by_patch.setdefault(row["patch_id"], []).append(row["d_ref_mm"])
    by_patch = {key: np.array(value) for key, value in by_patch.items()}
    mesh = load_sur(ROOT / "data/ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(ROOT / "data/box-centroids-rotations-world.json")
    start = perf_counter()
    coarse = icp.coarse_registration(by_patch, boxes)
    coarse_seconds = perf_counter() - start
    _, _, initial_distances = closest_point.closest_points_on_mesh(apply(coarse, samples), mesh)
    start = perf_counter()
    result = icp.icp(samples, mesh, coarse)
    seconds = perf_counter() - start
    replay = summarize(result)
    replay.update(input_kind="replay of saved actual practice observations; truth unavailable offline",
                  source_file=args.input.name, practice_collection_seed=saved["seed"],
                  mesh_vertices=mesh.n_vertices, mesh_triangles=mesh.n_triangles,
                  n_patches=len(by_patch), F_coarse_reference_to_ct=frame_to_json(coarse),
                  coarse_all_sample_rms_mm=float(np.sqrt(np.mean(initial_distances ** 2))),
                  wall_coarse_seconds=coarse_seconds, wall_icp_seconds=seconds,
                  sample_results=[{"sample_index": row["sample_index"], "patch_id": row["patch_id"],
                                   "residual_mm": float(r), "used": bool(used)}
                                  for row, r, used in zip(rows, result.residuals, result.inlier_mask)])
    evidence["practice_replay"] = replay
    evidence["practice_actual_tre"] = "NOT RUN; no live scoring or hidden outlier labels requested"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({k: replay[k] for k in ("input_kind", "converged", "iterations", "n_used",
                                           "inlier_rms_mm", "wall_coarse_seconds", "wall_icp_seconds")}))


if __name__ == "__main__":
    main()
