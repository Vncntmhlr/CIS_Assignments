# Supplementary evaluation: infrastructure and measured pilot

Status: **pilot only**, Seeds 1 and 7. Seed 61 has not been calibrated or
evaluated. No changes to the calibrated implementation or official notebook.
The original Seed 1 Step 9 (729 poses) and Step 10 (40 targets) archives remain
unchanged. This experiment is separate from the normal submission execution.

## Reproduction and safeguards

From `assignment2/`, using the existing environment:

```bash
MPLCONFIGDIR=/tmp/cis_comprehensive_mpl venv/bin/python experiments/comprehensive_evaluation.py --stage pilot
venv/bin/python -m pytest experiments/test_comprehensive_evaluation.py -q
```

Cached pilot execution verifies model, calibration-data, protected-source and
report/model pairing hashes, then recomputes statistics/figures from raw attempts;
it does not acquire observations or fit again. New artifacts use the exclusive
`results/comprehensive_v1_*` prefix. Calibration/model files are never overwritten.
Do not delete caches to rerun: preserve their provenance first. A failed/timed-out
fit stops for review; fitting has a 300-second process limit.

The full command is deliberately gated by both `--stage full` and
`--approve-full`. **Do not run it until the user reviews this pilot.** It retains
the matrix below, loads frozen Seeds 1/7 and independently calibrates Seed 61.
Each new seed uses its own five homings, ten baseline readings, 765 training
observations at 69 configurations and the unchanged normal constrained fitter.
No cross-seed parameters, ground-truth calibration inputs or evaluation tuning.
Model/data SHA-256 hashes are saved before any pose-oracle evaluation.

Run the full experiment locally after approval:

```bash
cd /home/vincent/CIS_Assignments/assignment2
MPLCONFIGDIR=/tmp/cis_comprehensive_mpl ./venv/bin/python -u experiments/comprehensive_evaluation.py --stage full --approve-full
```

Only brief per-seed progress is printed. Results, fitted-model/data checkpoints
and all five figure groups save automatically under `results/`. Review
`comprehensive_v1_full_summary.json`; raw attempts stay in the separate seed
files. `comprehensive_v1_full_checkpoint.json` records completed/pending seeds
and figure completion. Rerunning the same command reuses completed seeds and
frozen models; an interrupted evaluation restarts its unfinished seed. JSON
summary/checkpoint updates are atomic. An interrupted or failed fit requires
review of its diagnostics rather than an automatic retry.

| Category | Full FK/seed | Full physical IK/model/seed | Pilot FK/seed | Pilot physical IK/model/seed |
| --- | ---: | ---: | ---: | ---: |
| Continuous joint-uniform random | 1000 | 80 | 40 | 8 |
| Joint limits | 120 | 20 | 24 | 4 |
| Workspace boundary | 104 | 20 | 16 | 4 |
| Wrist singularities | 96 | 20 | 16 | 4 |
| Total | 1320 | 140 | 96 | 20 |

The full numerical consistency check uses the same 140 design configurations
per model plus 64 alternate-initialization cases; pilot uses 20 plus 8. Single
tracker snapshots: 100/full seed, 8/pilot seed; no per-target averaging. Random
geometry is continuous, without filtering difficult poses. The ordinary subset
is tagged beforehand using 5 mm / 0.05 rad limit margins and 0.05 rad wrist-pole
clearance. Designed stress cohorts remain separate; pooled stress figures do
not replace their per-category/per-seed statistics.

Limits cover six joints, both endpoints, five inward distances and two random
backgrounds: linear distances 0/0.001/0.1/1/5 mm, angular distances
0/1e-5/1e-3/0.01/0.05 rad. Boundary uses 26 Cartesian boundary points times four
wrists. Singularity uses 84 configurations around the frozen model's predicted
poles (six backgrounds, two poles, seven offsets) plus 12 nominal-pole cases.
These are stage boundaries, not a claim of exhaustive end-effector-envelope
coverage. No target is selected by observed ground-truth error.

## Randomness and measurement conventions

- Geometry: `SeedSequence([20261025, robot_seed, category_id])`.
- Public robot/tracker initialization: Seeds 1, 7, 61. Tracker sample-noise streams
  are NumPy seeds 606 (historical Seed 1), 1607, 1661 for calibration, and
  `3100 + robot_seed` for evaluation. Global NumPy state is restored afterward.
- Robot noise follows its public seed-specific simulator stream, continuing
  after calibration motions. Cached scenes replay only the original calibration
  motions/homings to reconstruct public state and draw position, without tracker
  readings. This replay cost is reported separately. **No artificial burn-in**
  or private RNG manipulation is used. Nominal/improved physical executions use
  successive draws rather than the same realized motion perturbation.
- Seed 1 remains a historical development robot. Its new geometry is held out
  from calibration, but neither its model development nor potentially repeated
  post-calibration robot-noise draws are claimed to be blind. Seed 7 evaluation
  noise follows its new calibration draws. Replaying the pilot/full stream is
  reproducibility, not another independent noise realization; full extends the
  predetermined design. Full FK includes every pilot FK configuration; the
  pilot's short IK route includes one additional boundary edge in place of a
  full-protocol face target. Neither route is selected from measured errors.
- Physical FK: permitted pose oracle at commanded encoders, with no motion or
  tracker sampling; compares predictions to deterministic physical kinematics.
- Tracker prediction: command once, read matching encoders and one tracker
  snapshot, compare both models to that same observation. Missing observations
  remain attempted failures. Tracker noise is part of this metric.
- Numerical consistency: target from each model's own FK; score that model's
  reconstructed pose, without physical execution. Strict criteria are
  1e-6 mm / 1e-8 rad, separate from the physical accuracy targets.
- Physical IK: construct a reachable target with the permitted oracle, pass
  only the target through the original inverse, execute every returned command
  and score actual placement. The initial guess is the preselected generating
  design vector, matching the existing Step 10 approach: this is a favorable
  initialization, not evidence of arbitrary-start IK robustness. Additional
  numerical pole cases use center/perturbed guesses. There is no retry, clipping,
  solver replacement or rejection of inaccurate returned candidates.

All errors are `(rad, mm)`. Every attempted target remains in the simultaneous
pass denominator (`position <= 0.2 mm`, `orientation <= 0.001 rad`, legal joints).
RMS/median/p95/max condition on finite scored attempts and are explicitly labeled;
exceptions are not assigned fabricated errors. CDFs use the attempted denominator
and retain missing probability mass. Raw commands, initial guesses, solver
messages, limit flags and per-attempt timing are preserved in each pilot JSON.

## Measured costs and review gate

| Cost | Seed 1 | Seed 7 |
| --- | ---: | ---: |
| New calibration tracker readings | 0 | 775 (10 baseline + 765 training) |
| New calibration simulated time | 0 s | 554.046 s |
| State-reconstruction replay simulated time | 530.781 s | 0 s |
| Fitting wall time | reused historical fit | 64.415 s (64.898 s subprocess) |
| Pilot evaluation wall time | 2.992 s | 2.995 s |
| Pilot evaluation simulated time | 1296.664 s | 1222.593 s |

Seed 7 fit: five iterations, successful SLSQP termination, weighted cost
432760.085 -> 1134.716, physical-constraint normalized margin -3.11e-15
(roundoff), fixed gauge unchanged. It was frozen before evaluation.

Category-scaled extrapolation: full evaluation about 20.5 / 20.3 seconds wall and
9662 / 9150 seconds simulated per seed. For three seeds, reusing cached models
and adding one measured-cost Seed 61 fit plus a 45-second plotting allowance:
**171 s wall**; scaling the slowest observed calls instead gives **185 s**.
These are engineering estimates, not upper bounds. An unseen difficult seed
may take longer; the next fit stops at 300 s. Approximately **28985 s (8.05 h)**
simulated evaluation is projected, plus about 554 s new Seed 61 calibration and
1062 s cached Seeds 1/7 replay. Extensive testing is intentionally separate from
calibration/submission timing; the official PDF's ten-minute program execution
limit must not be confused with the simulator's elapsed acquisition/test clock.
Full results and Seed 61 remain pending regardless of this manageable wall cost.

## Artifacts and figures

- [Pilot projection](../results/comprehensive_v1_pilot_projection.json).
- Raw results: [Seed 1](../results/comprehensive_v1_s001_pilot.json),
  [Seed 7](../results/comprehensive_v1_s007_pilot.json).
- Each seed's `freeze.json` binds its model, data and protected-source hashes;
  the new Seed 7 calibration NPZ, acquisition JSON and fit diagnostics are saved
  beside the model. The original unconstrained/constrained archives remain intact.
- Five figure groups, each vector PDF and 220-dpi PNG, labeled **PILOT**:
  [CDFs](../results/comprehensive_v1_pilot_cdfs.pdf),
  [category distributions](../results/comprehensive_v1_pilot_categories.pdf),
  [workspace maps](../results/comprehensive_v1_pilot_workspace.pdf),
  [joint dependence](../results/comprehensive_v1_pilot_joints.pdf),
  [time breakdown](../results/comprehensive_v1_pilot_timing.pdf).

Figures pool Seeds 1/7 only for comparative display; raw statistics retain each
seed/category. Both models and all finite per-target observations are shown in
distributions. Workspace plots separate robots and use common color scales;
historical timing is hatched separately. Pilot figures are provisional report
assets, not results of the approved full three-seed experiment.

Local requirement references: Assignment 1 template Step 8 (single observation,
fixed evaluation volume, validation-only ground truth), Assignment 2 brief
pp. 6–7 (workspace accuracy, calibration/testing time, unit tests and report),
and the unchanged Assignment 2 notebook Steps 9–10. Assignment 1's original
handout is not present locally; its archived TA template supplies that protocol.
