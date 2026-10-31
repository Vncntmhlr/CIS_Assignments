# CIS I — Assignment 2: Kinematic Calibration

This assignment is driven from a Jupyter notebook, but — unlike assignment 1 —
your own code lives in Python modules under `src/student/` and is unit-tested
with `pytest`. You'll design the scene (tracker, robot, marker bodies),
collect calibration data against a simulated clock, fit an improved kinematic
model for the robot, and evaluate it over the work volume.

## What's in this folder

```
assignment2/
├── requirements.txt
├── src/
│   ├── robot.py                  -- the nominal robot (don't edit)
│   ├── simulation/               -- the handout's API: CreateRobot, MoveJoints,
│   │                                CreateTracker, SampleMarkerBodies, ... (don't edit)
│   └── student/                  <- YOUR code: metrics.py, registration.py,
│                                    workflow.py, kinematics.py (stubs to fill in)
├── tests/
│   ├── test_simulation.py        -- provided, passes as shipped; read it as an example
│   └── test_student.py           -- tests for your code; some complete, some TODO
└── notebooks/
    └── assignment2_template.ipynb   <- start here, this drives everything
```

The tracker and marker-body classes (`Tracker`, `MarkerBody`, `PlacedBody`,
`define_marker_body`, `place_marker_body`) are assignment 1's, imported from
`../assignment1/src` — so keep the whole course repository checked out, not
just this folder. You do not need to read or modify anything under `src/`
except `src/student/`. The handout's function names map to snake_case in
`src.simulation` (`CreateRobot` → `simulation.create_robot`,
`SampleMarkerBodies` → `simulation.sample_marker_bodies`, ...); every
docstring names the operation it implements.

## 1. Set up your environment

You need **Python 3.9 or newer**. Check with:

- Linux/macOS: `python3 --version`
- Windows: `python --version` (or `py --version`)

All commands below are run from the `assignment2/` folder (the one this
README is in).

### Linux / macOS

```bash
cd assignment2
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Windows

Command Prompt:

```bat
cd assignment2
py -m venv venv
venv\Scripts\activate.bat
pip install -r requirements.txt
```

PowerShell:

```powershell
cd assignment2
py -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

> If PowerShell refuses to run the activation script ("running scripts is
> disabled on this system"), run this once in an **admin** PowerShell, then
> retry: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

Once activated, your terminal prompt should show `(venv)`. Re-run the
activate command every time you open a new terminal for this assignment.

## 2. Run the tests, then launch the notebook

With the `venv` active:

```bash
python -m pytest tests -q
```

`tests/test_simulation.py` must pass out of the box. `tests/test_student.py`
will fail with `NotImplementedError` until you implement the stubs — that's
expected.

Then pick one:

**Jupyter, from the terminal**: `jupyter notebook` (or `jupyter lab`), open
`notebooks/assignment2_template.ipynb`.

**VS Code**: open the `assignment2` folder, open the notebook, and select the
interpreter inside your `venv` in the kernel picker (Linux/macOS:
`venv/bin/python`; Windows: `venv\Scripts\python.exe`).

Run the first code cell (the imports) first — if it fails with a
`ModuleNotFoundError`, the kernel isn't using the `venv` you installed
`requirements.txt` into.

## 3. What you have to do

Work through the notebook top to bottom; its sections match the handout.
Markdown cells explain each step; `# TODO` marks what's yours. In order:

1. **Design the scene** (Step 1): tracker and robot poses and the
   calibration body's layout (`||a*_k|| <= 75 mm`). Check visibility before
   spending simulated time. This handout's tracker has no jiggle, so the
   tracker frame is a fixed reference.
2. **Metrics** (`src/student/metrics.py`): `frame_error`, `summarize_errors`.
3. **Registration** (`src/student/registration.py`): your own point-set
   registration and frame averaging; marker-body refinement is optional
   (only needed if you add marker bodies of your own).
4. **Workflow** (`src/student/workflow.py`): measure the joint offsets at
   home with `HomeRobot()`, plan the calibration poses from the accuracy
   you want (the design decision that sets your accuracy *and* your time),
   collect the data, estimate the robot base in the tracker frame.
5. **Improved kinematics** (`src/student/kinematics.py`): a parameterized
   error model, `fwd_kins`, a least-squares `fit`, a Jacobian rank report
   that justifies your parameterization, and a `parameters_report` to
   compare with ground truth in debugging runs (`inv_kins` is provided — it
   iterates on your `fwd_kins`). The handout strongly urges "method 1"
   (estimate the kinematic parameters by moving along/about each axis from
   home) over "method 2" (map the space and interpolate), since your
   program's run time is limited when graded.
6. **Evaluate** (Steps 9–10) forward and inverse kinematics over the fixed
   test grid against the handout's targets (`||epsilon|| <= 0.2 mm`,
   `||alpha|| <= 1e-3 rad`, see `constants.TARGET_*`), and report the
   simulated time for calibration and for testing. If you cannot reach a
   target, report the accuracy achieved and how you went about it.
7. **Unit tests** (`tests/test_student.py`): make the complete ones pass and
   write the `TODO` ones. The handout requires you to describe them and
   report their results.

The notebook runs cleanly up to wherever your next unfilled TODO is, then
stops (with a "did you fill in the Step 1 TODOs?" error, or a
`NotImplementedError` from a stub) — that's expected, not a bug.

Ground truth (`rob.actual_ee_pose`, `rob.ground_truth_parameters`,
`simulation.actual_frame_of_body`, `PlacedBody.actual_placement`, anything
with `_truth` in its name) exists for validation, debugging and reporting
in Steps 8–10 only. Using it anywhere in your
calibration defeats the assignment; grading runs with different robot seeds.

**Don't forget**: the notebook and modules are only the code portion. The
handout also requires a written report (design rationale, error model and
identifiability discussion, pose-plan/time trade-off, unit tests, results
tables for more than one seed).

## Troubleshooting

- **`ModuleNotFoundError: No module named 'scipy'`** (or `numpy`, etc.) —
  your kernel isn't using the `venv` you installed `requirements.txt` into.
- **`ModuleNotFoundError: No module named 'src'`** — the notebook assumes it
  runs with `notebooks/` as the working directory (the default for Jupyter
  and VS Code); `pytest` assumes you run it from `assignment2/`.
- **`pip install` fails or `venv` module missing (Linux)** — install your
  distro's venv package first, e.g. `sudo apt install python3-venv`.
