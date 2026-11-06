# Calibration repeats versus accuracy

Runs the existing `assignment2_Draft_2.ipynb` pipeline with Step 7 repeats set to 1, 2, 3, 4, 5 and 6. Only the repeat argument changes in an in-memory copy. Notebook and source files are not edited. The current local notebook and workflow settings are used, not older committed versions.

From `assignment2/`, run:

```bash
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/cis-repeat-mpl /home/ahmed-cheruvattam/venv/bin/python -B experiments/repeat_sweep.py --output experiments/results/my_repeat_sweep
```

Use a new output directory for every sweep. The runner refuses to overwrite existing results. No seeds are selected based on performance.

Each run starts in a fresh Python process and executes the notebook setup, scene, baseline calibration, pose planning, fitting, forward evaluation and inverse evaluation. Robot/tracker seed remains 1 and target selection seed remains 0. Tracker measurement noise uses a separate global NumPy generator; the runner resets it (and Python's random generator) to the notebook's existing `SEED=1` before every run. This additional explicit reset makes results reproducible; earlier interactive notebook results without that reset need not match. More repeats consume more random samples, including before inverse testing.

The CSV and graph report the **improved model**. Both accuracy limits must hold for the same pose: position error <= 0.2 mm AND angular error <= 0.001 rad. Forward evaluation uses all 729 visible grid poses and inverse evaluation uses the same 40 selected targets. Nominal evaluations are also executed as in the notebook and retained in the logs. Inverse evaluation includes motion repeatability noise; forward evaluation uses deterministic ground truth. Evaluation targets are not used for fitting.

Calibration time is `t_calibration = clock.elapsed` at Step 8, including homing and baseline collection. The 900-second soft budget applies to this time. `testing_time_s` and actual `wall_time_s` are separate CSV fields. No partial run receives a pass percentage; failures are retained in the CSV and logs.

Outputs include `results.csv`, the PNG/SVG graph, per-run logs and JSON summaries, raw improved errors (`*_errors.npz`, columns angle rad and position mm), and settings for verifying identical scenes/plans/targets. `provenance.json`, `input_changes.patch` and `input_notebook.ipynb` record the source revision, local changes, input hashes and notebook snapshot.

One run is executed for each repeat count with fixed seeds. These points describe that comparison, not confidence intervals or guarantees across other seeds or the continuous workspace. Connecting lines are visual guides between measured configurations.
