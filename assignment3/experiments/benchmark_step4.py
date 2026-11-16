"""Time the notebook's Step 4 query set and optionally check Ahmed's baseline."""

import argparse
import hashlib
import importlib.util
import json
import platform
from pathlib import Path
from statistics import median
from time import perf_counter

import numpy as np

from src import load_sur
from src.student.closest_point import closest_points_on_mesh


def main():
    """Benchmark 200 CT queries on the real mesh and optionally save JSON evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ahmed-source", type=Path, help="Local copy of Ahmed's closest_point.py")
    parser.add_argument("--output", type=Path, help="Write settings and measurements as JSON")
    args = parser.parse_args()

    data = Path(__file__).resolve().parents[1] / "data"
    mesh = load_sur(data / "ProximalFemurMeshFile.sur")
    queries = np.random.default_rng(0).uniform(-60, 60, size=(200, 3))
    mesh.tri_vertices  # Preload the mesh's triangle data before timing.
    times = []
    for _ in range(5):
        start = perf_counter()
        closest, indices, distances = closest_points_on_mesh(queries, mesh)
        times.append(perf_counter() - start)

    print(f"{len(queries)} queries, {mesh.n_triangles} triangles")
    print(f"NumPy batched median {median(times):.6f} s, range {min(times):.6f}–{max(times):.6f} s")
    print(f"Mean distance {distances.mean():.6f} mm")
    evidence = {
        "validation": "offline geometry; no simulator calls",
        "python": platform.python_version(), "numpy": np.__version__,
        "platform": platform.platform(), "seed": 0, "query_count": len(queries),
        "query_distribution": "uniform(-60, 60) mm per CT axis",
        "vertex_count": mesh.n_vertices, "triangle_count": mesh.n_triangles,
        "query_batch_size": 32, "mesh_triangle_cache_preloaded": True,
        "batched_seconds": times, "batched_median_seconds": median(times),
        "mean_distance_mm": float(distances.mean()),
        "simulated_seconds": 0,
    }

    if args.ahmed_source:
        spec = importlib.util.spec_from_file_location("src.student.ahmed_closest_point", args.ahmed_source)
        baseline = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(baseline)
        start = perf_counter()
        baseline_points, baseline_indices, baseline_distances = baseline.closest_points_on_mesh(queries, mesh)
        elapsed = perf_counter() - start
        np.testing.assert_allclose(closest, baseline_points, atol=1e-6, rtol=0)
        np.testing.assert_allclose(distances, baseline_distances, atol=1e-6, rtol=0)
        print(f"Ahmed's baseline {elapsed:.6f} s, speedup {elapsed / median(times):.1f}x")
        print(f"Largest distance difference {np.max(np.abs(distances - baseline_distances)):.3g} mm")
        print(f"Largest point difference {np.max(np.linalg.norm(closest - baseline_points, axis=1)):.3g} mm")
        print(f"Matching triangle indices {np.count_nonzero(indices == baseline_indices)}/{len(queries)}")
        # Shared edges/vertices can have several equally valid triangle indices.
        tied_boundary_matches = 0
        for row in np.flatnonzero(indices != baseline_indices):
            shared = np.intersect1d(mesh.triangles[indices[row]], mesh.triangles[baseline_indices[row]])
            if len(shared) == 1:
                boundary_point = mesh.vertices[shared[0]]
            elif len(shared) == 2:
                start, end = mesh.vertices[shared]
                edge = end - start
                fraction = np.clip((closest[row] - start) @ edge / (edge @ edge), 0, 1)
                boundary_point = start + fraction * edge
            else:
                raise AssertionError(f"Unexpected non-adjacent triangle mismatch for query {row}")
            np.testing.assert_allclose(closest[row], boundary_point, atol=1e-6, rtol=0)
            tied_boundary_matches += 1
        print(f"Different indices on a verified shared edge/vertex: {tied_boundary_matches}")
        evidence["ahmed_baseline"] = {
            "source_file": args.ahmed_source.name,
            "source_sha256": hashlib.sha256(args.ahmed_source.read_bytes()).hexdigest(),
            "seconds": elapsed, "repetitions": 1, "speedup_vs_batched_median": elapsed / median(times),
            "max_distance_difference_mm": float(np.max(np.abs(distances - baseline_distances))),
            "max_point_difference_mm": float(np.max(np.linalg.norm(closest - baseline_points, axis=1))),
            "matching_triangle_indices": int(np.count_nonzero(indices == baseline_indices)),
            "different_indices_on_shared_boundary": tied_boundary_matches,
        }

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
