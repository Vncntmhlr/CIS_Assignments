"""Validate the Step 7 report pipeline with replay transport or a fresh practice session.

No evaluate_tre/true_outliers calls: actual accuracy validation remains Step 8.
Run from assignment3 with PYTHONPATH=.; --live reads credentials from the notebook.
"""

import argparse
import ast
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from time import perf_counter
import warnings

import numpy as np

from src import RemoteSession, load_patch_boxes, load_sur
from src.student import closest_point, icp, report, uncertainty, workflow


ROOT = Path(__file__).resolve().parents[1]


def main():
    """Save the unmodified TA report, diagnostics, and exact/proxy comparison."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    stem = f"step7_practice_seed{args.seed}" if args.live else "step7_replay"
    output = ROOT / "experiments/results"
    output.mkdir(parents=True, exist_ok=True)
    notebook = json.loads((ROOT / "notebooks/assignment3_template.ipynb").read_text())
    mesh = load_sur(ROOT / "data/ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(ROOT / "data/box-centroids-rotations-world.json")
    evidence = {"status": "NOT RUN", "validation": "fresh practice session" if args.live else
                "mock tracker transport replaying saved measured reference tips",
                "seed": args.seed if args.live else None, "actual_tre": "NOT RUN (Step 8)",
                "true_outlier_labels": "NOT REQUESTED", "code_sha256": {
                    name: hashlib.sha256((ROOT / "src/student" / name).read_bytes()).hexdigest()
                    for name in ("icp.py", "uncertainty.py", "workflow.py")}}
    session = None
    start = perf_counter()
    try:
        if args.live:
            config = {}
            for statement in ast.parse("".join(notebook["cells"][5]["source"])).body:
                if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
                    name = getattr(statement.targets[0], "id", "")
                    if name in {"SERVER_URL", "COURSE_TOKEN"}:
                        config[name] = ast.literal_eval(statement.value)
            session = RemoteSession(config["SERVER_URL"], workflow.design_scene(), seed=args.seed,
                                    token=config["COURSE_TOKEN"], timeout=15)
            if session.mode != "practice":
                raise RuntimeError("expected a practice session")
            evidence["mu_reference"] = "original server initial_registration_guess"
        else:
            from experiments.test_step7_uncertainty import ReplaySession
            session = ReplaySession()
            evidence["mu_reference"] = "saved coarse pose used as mock F0; original server F0 unavailable"
        initial = session.initial_registration_guess()
        final = workflow.run_registration(session, mesh, boxes)
        final.to_json(output / f"{stem}_report.json")
        (output / f"{stem}_report.txt").write_text(final.to_text() + "\n")
        restored = report.RegistrationReport.from_json(output / f"{stem}_report.json")
        assert restored.to_dict() == final.to_dict()
        valid = [row for row in final.samples if row["d_ref"] is not None]
        d = np.array([row["d_ref"] for row in valid])
        x = d @ final.F_reg.R.matrix.T + final.F_reg.p.vec.ravel()
        q, triangles, residuals = closest_point.closest_points_on_mesh(x, mesh)
        mask = np.array([row["used"] for row in valid], dtype=bool)
        result = icp.IcpResult(final.F_reg, residuals, mask, q, mesh.face_normals[triangles],
                               final.n_icp_iterations, final.converged)
        result._transformed_points_ct = x.copy()
        namespace = {"uncertainty": uncertainty, "mesh": mesh, "result": result, "F0_reg": initial}
        # Execute the original teaching cell, without writing or restructuring it.
        exec(compile("".join(notebook["cells"][17]["source"]), "original-step7-cell", "exec"), namespace)
        np.testing.assert_allclose(namespace["C_reg"], final.eta_reg["C_reg"], rtol=1e-10, atol=1e-15)
        np.testing.assert_allclose(namespace["mu"], final.eta_reg["mu"], rtol=1e-10, atol=1e-12)
        np.testing.assert_allclose(list(namespace["pred"].values()), list(final.predicted_tre.values()), rtol=1e-10)
        legacy = replace(result)
        with warnings.catch_warnings(record=True) as notices:
            warnings.simplefilter("always")
            _, proxy_covariance = uncertainty.registration_covariance(legacy, initial)
        exact_covariance = np.asarray(final.eta_reg["C_reg"])
        proxy_prediction = uncertainty.predicted_tre(mesh, final.F_reg, proxy_covariance)
        inlier_residuals = np.array([row["residual_mm"] for row in valid if row["used"]])
        eigenvalues = np.linalg.eigvalsh(exact_covariance)
        evidence.update(status="VERIFIED", converged=final.converged, n_patches=len(final.patches),
                        n_attempts=len(final.samples), n_valid_observations=len(valid),
                        n_invisible=len(final.samples) - len(valid), n_used=int(mask.sum()),
                        n_icp_rejected=int((~mask).sum()), n_icp_iterations=final.n_icp_iterations,
                        residual_rms_mm=float(np.sqrt(np.mean(inlier_residuals ** 2))),
                        residual_stats=final.residual_stats, predicted_tre_mm=final.predicted_tre,
                        t_simulated_total=final.t_simulated, t_wall_workflow_seconds=final.t_wall,
                        covariance_diagnostics=final.eta_reg["covariance_model"],
                        covariance_eigenvalues=eigenvalues.tolist(),
                        notebook_step7="VERIFIED unchanged cell; agrees with headless report",
                        report_round_trip="VERIFIED",
                        compatibility_proxy={"warned": len(notices) > 0,
                            "relative_covariance_frobenius_difference": float(
                                np.linalg.norm(proxy_covariance - exact_covariance) / np.linalg.norm(exact_covariance)),
                            "predicted_tre_mm": proxy_prediction})
    except (RuntimeError, ValueError, AssertionError) as exc:
        evidence.update(status="FAILED", error=str(exc))
    finally:
        if args.live and session is not None:
            try:
                session.close()
            except RuntimeError as exc:
                evidence["cleanup_error"] = str(exc)
        evidence["validation_wall_seconds"] = perf_counter() - start
        (output / f"{stem}_validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({key: value for key, value in evidence.items() if key != "code_sha256"}, indent=2))
    if evidence["status"] != "VERIFIED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
