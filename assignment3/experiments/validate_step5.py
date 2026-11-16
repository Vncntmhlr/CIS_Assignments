"""Validate the original Step 5 notebook cell with mock or practice observations.

Run from assignment3 with PYTHONPATH=. . Credentials are read from the existing
notebook assignments only for --live, and are never written to the evidence.
No registration, true-outlier, or TRE methods are called.
"""

import argparse
import ast
from collections import Counter
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from src import RemoteSession, load_patch_boxes
from src.student import workflow


ROOT = Path(__file__).resolve().parents[1]


def main():
    """Write plan, collection, timing, and observation evidence without secrets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="Use the notebook's practice server settings")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or ROOT / "experiments" / "results" / (
        "step5_practice.json" if args.live else "step5_offline.json")
    notebook = json.loads((ROOT / "notebooks" / "assignment3_template.ipynb").read_text())
    boxes = load_patch_boxes(ROOT / "data" / "box-centroids-rotations-world.json")
    evidence = {"validation": "practice server" if args.live else "synthetic mock Session",
                "status": "NOT RUN", "n_candidates": 46, "samples_per_patch": 4,
                "sampling_rule": "Ahmed: consecutive normal axes separated by more than 50 degrees"}
    plan = workflow.plan_patches(boxes, 46, 4)
    evidence.update(selected_patch_ids=list(dict.fromkeys(plan)), n_attempts=len(plan),
                    budget_simulated_seconds=len(plan) * 1.03)
    session = None
    captured = io.StringIO()
    start = perf_counter()
    try:
        if args.live:
            config = {}
            tree = ast.parse("".join(notebook["cells"][5]["source"]))
            for statement in tree.body:
                if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
                    name = getattr(statement.targets[0], "id", "")
                    if name in {"SERVER_URL", "COURSE_TOKEN", "SEED"}:
                        config[name] = ast.literal_eval(statement.value)
            evidence["seed"] = config["SEED"]
            session = RemoteSession(config["SERVER_URL"], workflow.design_scene(),
                                    seed=config["SEED"], token=config["COURSE_TOKEN"], timeout=15.0)
            if session.mode != "practice":
                raise RuntimeError("Expected a practice session")
            design = session.design
            evidence["nominal_reference_visible"] = bool(session.is_visible([
                session.nominal_reference_pose() * p for p in design.ref_marker_positions]))
            evidence["nominal_invisible_selected_patches"] = [pid for pid in dict.fromkeys(plan)
                if not session.is_visible([session.nominal_pointer_pose(pid) * p
                                           for p in design.ptr_marker_positions])]
        else:
            from experiments.test_step5_sampling import MockSession, synthetic_observation_pair
            pair, expected = synthetic_observation_pair()
            session = MockSession([pair] * len(plan))

        namespace = {"workflow": workflow, "boxes": boxes, "session": session}
        cell = "".join(notebook["cells"][13]["source"])
        with redirect_stdout(captured):
            exec(compile(cell, "notebook-step5-cell", "exec"), namespace)
        samples = namespace["samples"]
        points = samples.as_array()
        evidence.update(status="VERIFIED", n_valid=len(samples), n_missing=len(plan) - len(samples),
                        t_collect_simulated_seconds=samples.t_collect,
                        valid_counts_by_patch=dict(Counter(samples.patch_ids)),
                        samples=[{"sample_index": h.sample_index, "patch_id": pid, "t_sim": h.t_sim,
                                  "d_ref_mm": p.tolist()}
                                 for h, pid, p in zip(samples.handles, samples.patch_ids, points)])
        if not args.live:
            evidence["max_tip_error_mm"] = float(np.linalg.norm(points - expected, axis=1).max())
            evidence["placement_calls"] = sum(call[0] == "place" for call in session.calls)
            evidence["tracker_calls"] = sum(call[0] == "sample" for call in session.calls)
    except (RuntimeError, KeyError, ValueError) as exc:
        evidence.update(status="FAILED", error=str(exc))
    finally:
        if args.live and session is not None:
            try:
                session.close()
            except RuntimeError as exc:
                evidence["cleanup_error"] = str(exc)
        evidence["wall_seconds"] = perf_counter() - start
        evidence["notebook_output"] = captured.getvalue().splitlines()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({k: v for k, v in evidence.items() if k not in {"samples"}}, indent=2))
    if evidence["status"] != "VERIFIED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
