"""Practice-only Step 8 validation; never imported by the graded workflow.

From assignment3: PYTHONPATH=. venv/bin/python experiments/validate_step8.py
Credentials are read locally from the notebook and never written to results.
"""

import argparse
import ast
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from src import RemoteSession, load_patch_boxes, load_sur
from src.student import icp, report, workflow


ROOT = Path(__file__).resolve().parents[1]


def outlier_metrics(samples, true_outliers):
    """Compare original placement indices; missing observations are not flags."""
    attempted = {row["sample_index"] for row in samples}
    if len(attempted) != len(samples):
        raise ValueError("duplicate original sample indices")
    observed = {row["sample_index"] for row in samples if row["d_ref"] is not None}
    flagged = {row["sample_index"] for row in samples
               if row["d_ref"] is not None and not row["used"]}
    truth = set(true_outliers)
    if truth - attempted:
        raise ValueError("truth includes placements outside this fresh run")
    positives = truth & observed
    tp, fp, fn = len(flagged & positives), len(flagged - positives), len(positives - flagged)
    tn = len(observed - positives - flagged)
    return {
        "population": "observed tracker samples; original placement indices",
        "true_outlier_indices": sorted(truth), "flagged_indices": sorted(flagged),
        "missing_indices": sorted(attempted - observed),
        "true_outliers_missing": sorted(truth - observed),
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "precision": tp / (tp + fp) if tp + fp else None,
        "recall": tp / (tp + fn) if tp + fn else None,
        "f1": 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else None,
    }


def validate_one(seed, stem, output, notebook, config, mesh, boxes):
    """Freeze the estimator output before querying any practice truth."""
    evidence = {
        "status": "NOT RUN", "seed": seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "validation": "fresh live practice session, unchanged headless workflow",
        "settings": {"n_patches": len(boxes), "samples_per_patch": 4,
                     "max_iter": 100, "tol": 1e-4, "reject": "mad"},
        "code_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [*(ROOT / "src/student").glob("*.py"),
                         ROOT / "data/ProximalFemurMeshFile.sur",
                         ROOT / "data/box-centroids-rotations-world.json"]},
    }
    session = None
    start = perf_counter()
    try:
        session = RemoteSession(config["SERVER_URL"], workflow.design_scene(), seed=seed,
                                token=config["COURSE_TOKEN"], timeout=15)
        if session.mode != "practice":
            raise RuntimeError("Step 8 requires a practice session; no truth queries were made")
        initial = session.initial_registration_guess()
        final = workflow.run_registration(session, mesh, boxes)
        # Save the full fit before any truth query. Scoring never changes this fit.
        final.to_json(output / f"{stem}_report.json")
        (output / f"{stem}_report.txt").write_text(final.to_text() + "\n")
        restored = report.RegistrationReport.from_json(output / f"{stem}_report.json")
        assert restored.to_dict() == final.to_dict()
        valid = [row for row in final.samples if row["d_ref"] is not None]
        namespace = {
            "session": session,
            "result": SimpleNamespace(F_reg=final.F_reg,
                                      inlier_mask=[row["used"] for row in valid]),
            "samples": SimpleNamespace(handles=[SimpleNamespace(sample_index=row["sample_index"])
                                                for row in valid]),
        }
        # Reuse the original teaching/evaluation cell without modifying the notebook.
        exec(compile("".join(notebook["cells"][19]["source"]), "original-step8-cell", "exec"),
             namespace)
        actual = namespace["tre"]
        metrics = outlier_metrics(final.samples, namespace["true_outliers"])
        assert set(metrics["flagged_indices"]) == namespace["flagged"]
        assert actual.n == mesh.n_vertices
        by_patch = {}
        for row in valid:
            by_patch.setdefault(row["patch_id"], []).append(row["d_ref"])
        coarse = icp.coarse_registration({k: np.asarray(v) for k, v in by_patch.items()}, boxes)
        # Diagnostic comparisons only, after the final estimate was frozen.
        initial_tre = asdict(session.evaluate_tre(initial))
        coarse_tre = asdict(session.evaluate_tre(coarse))
        c = np.asarray(final.eta_reg["C_reg"])
        rms = float(np.sqrt(np.mean([row["residual_mm"] ** 2 for row in valid if row["used"]])))
        evidence.update(
            status="VERIFIED", actual_tre_mm=asdict(actual),
            accuracy_rms_le_2mm="PASS" if actual.rms <= 2 else "FAIL",
            accuracy_all_vertices_le_2mm="PASS" if actual.max <= 2 else "FAIL",
            accuracy_rms_le_1mm="PASS" if actual.rms <= 1 else "FAIL",
            predicted_tre_mm=final.predicted_tre,
            actual_to_predicted_rms_ratio=actual.rms / final.predicted_tre["rms"],
            residual_rms_mm=rms, residual_stats=final.residual_stats,
            outliers=metrics, n_patches=len(final.patches), n_attempts=len(final.samples),
            n_valid=len(valid), n_missing=len(final.samples) - len(valid),
            n_used=sum(row["used"] for row in valid),
            converged=final.converged, n_icp_iterations=final.n_icp_iterations,
            t_simulated_seconds=final.t_simulated, t_wall_workflow_seconds=final.t_wall,
            workflow_wall_lt_600s=final.t_wall < 600,
            covariance_diagnostics=final.eta_reg["covariance_model"],
            covariance_eigenvalues=np.linalg.eigvalsh(c).tolist(),
            initial_tre_mm=initial_tre, coarse_tre_mm=coarse_tre,
            notebook_step8="VERIFIED unchanged evaluation cell",
            report_round_trip="VERIFIED",
        )
    except Exception as exc:
        evidence.update(status="FAILED", error=f"{type(exc).__name__}: {exc}")
    finally:
        if session is not None:
            try:
                session.close()
                evidence["session_closed"] = True
            except RuntimeError as exc:
                evidence["session_closed"] = False
                evidence["cleanup_error"] = str(exc)
        evidence["validation_wall_seconds"] = perf_counter() - start
        (output / f"{stem}_validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
    summary = {key: evidence.get(key) for key in
               ("seed", "status", "accuracy_rms_le_2mm", "actual_tre_mm", "predicted_tre_mm",
                "residual_rms_mm", "outliers", "t_wall_workflow_seconds", "error")}
    print(json.dumps(summary), flush=True)
    return evidence


def repeatability(runs, output):
    """Compare saved measurements, rather than assuming seed reproducibility."""
    comparisons = []
    for index, run in enumerate(runs):
        previous = next((r for r in runs[:index] if r["seed"] == run["seed"]), None)
        if previous is None or any(r["status"] != "VERIFIED" for r in (previous, run)):
            continue
        reports = [json.loads((output / f"{r['stem']}_report.json").read_text())
                   for r in (previous, run)]
        rows = [{r["sample_index"]: r for r in p["samples"] if r["d_ref"] is not None}
                for p in reports]
        shared = sorted(rows[0].keys() & rows[1].keys())
        same_plan = all(rows[0][i]["patch_id"] == rows[1][i]["patch_id"] for i in shared)
        delta = np.asarray([rows[1][i]["d_ref"] for i in shared]) - np.asarray(
            [rows[0][i]["d_ref"] for i in shared])
        comparisons.append({"seed": run["seed"], "runs": [previous["stem"], run["stem"]],
                            "common_observations": len(shared), "same_patch_assignments": same_plan,
                            "reference_tip_difference_rms_mm": float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))),
                            "same_true_outlier_indices": previous["outliers"]["true_outlier_indices"] ==
                            run["outliers"]["true_outlier_indices"]})
    return comparisons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3, 1])
    parser.add_argument("--tag", default=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    args = parser.parse_args()
    if not args.tag or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.tag):
        parser.error("tag must contain only letters, digits, underscores or hyphens")
    output = ROOT / "experiments/results/step8"
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / f"step8_{args.tag}_summary.json"
    if summary_path.exists() or list(output.glob(f"step8_{args.tag}_*_validation.json")):
        parser.error("this tag already has results; choose another to preserve evidence")
    notebook = json.loads((ROOT / "notebooks/assignment3_template.ipynb").read_text())
    config = {}
    for statement in ast.parse("".join(notebook["cells"][5]["source"])).body:
        if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
            name = getattr(statement.targets[0], "id", "")
            if name in {"SERVER_URL", "COURSE_TOKEN"}:
                config[name] = ast.literal_eval(statement.value)
    mesh = load_sur(ROOT / "data/ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(ROOT / "data/box-centroids-rotations-world.json")
    runs = []
    for i, seed in enumerate(args.seeds, start=1):
        stem = f"step8_{args.tag}_run{i}_seed{seed}"
        print(f"Starting fresh practice run {i}/{len(args.seeds)}, seed {seed}", flush=True)
        run = validate_one(seed, stem, output, notebook, config, mesh, boxes)
        run["stem"] = stem
        runs.append(run)
        summary_path.write_text(json.dumps({"runs": runs, "repeatability": repeatability(runs, output)},
                                           indent=2) + "\n")
    if any(r["status"] != "VERIFIED" or r["accuracy_rms_le_2mm"] != "PASS" or
           not r["converged"] or not r["workflow_wall_lt_600s"] or not r.get("session_closed")
           for r in runs):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
