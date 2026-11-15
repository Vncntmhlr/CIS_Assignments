# CIS I — Assignment 3: Surface Registration with a Tracked Pointer

You design a pointer and a reference body, touch the femur on quasi-planar
patches with the tracked pointer, and register the reference body to the CT
mesh with ICP (`F_reg`, reference body → CT) with a target registration error
(TRE) of at most 2 mm. As in assignment 2, your code lives in `src/student/`
and is unit-tested with `pytest`; the notebook drives everything.

Unlike assignments 1 and 2, the simulation does not run on your machine: the
femur's pose, the true registration, the pointer placement, the tracker and
the simulated clock live on the **course server**. `src/session.py` is the
client; its `RemoteSession` has the handout's operations as methods and that
is the only way your code interacts with the simulation. The server address
(and a token, if any) is announced on Piazza.

## What's in this folder

```
assignment3/
├── requirements.txt
├── data/
│   ├── ProximalFemurMeshFile.sur          -- the femur mesh (CT coordinates)
│   └── box-centroids-rotations-world.json -- F_box of the 46 quasi-planar patches
├── src/
│   ├── mesh.py                   -- Mesh / load_sur / PatchBox / load_patch_boxes (don't edit)
│   ├── session.py                -- Session interface + RemoteSession, the client of the
│   │                                course server: PlacePointer, SampleMarkerBodies, ... (don't edit)
│   └── student/                  <- YOUR code: closest_point.py, icp.py, uncertainty.py,
│                                    workflow.py (stubs); report.py is the output format (provided)
├── tests/
│   └── test_student.py           -- offline tests for your code; some complete, some TODO
└── notebooks/
    └── assignment3_template.ipynb   <- start here
```

The geometry types (`vct3`, `Rot`, `Frame`) and the tracker frustum come from
the course repository (`data_types/`, `../assignment1/src`), so keep the whole
repository checked out. The handout's function names map to `Session`
methods: `PlacePointer(patch)` → `session.place_pointer(patch)`,
`SampleMarkerBodies(trk)` → `session.sample_marker_bodies()` (returns
`{"ref": Observation|None, "ptr": Observation|None}`, each with `.frame`
relative to the tracker and `.markers`), the initial guess →
`session.initial_registration_guess()`, the clock → `session.elapsed()`.

## 1. Set up your environment

You need **Python 3.9 or newer**. All commands run from the `assignment3/`
folder.

Linux / macOS:

```bash
cd assignment3
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Windows (Command Prompt / PowerShell):

```bat
cd assignment3
py -m venv venv
venv\Scripts\activate.bat        (PowerShell: .\venv\Scripts\Activate.ps1)
pip install -r requirements.txt
```

## 2. Run the tests, then launch the notebook

```bash
python -m pytest tests -q
```

`tests/test_student.py` fails with `NotImplementedError` until you implement
the stubs (it runs offline; only `report` tests pass as shipped). Then open
`notebooks/assignment3_template.ipynb` in Jupyter or VS Code (kernel =
the `venv` interpreter).

## 3. Data files and conventions

- **Mesh** (`.sur`): `NV`, then `NV` lines `x y z`; `NT`, then `NT` lines whose
  first three integers are 0-based vertex indices (the remaining columns are
  unused). `src.mesh.load_sur` reads it and makes the triangle winding
  consistent and outward.
- **Patch boxes** (JSON list, patch id = list index): `centroid_world` is
  `p_box` and `rotation_local_to_world` is `R_box` stored row by row, so its
  *columns* are the box axes in CT coordinates (x = longest, z = shortest ≈
  surface normal). A box point `a` has CT coordinates `c = F_box a`, and
  `R_box z · (c − p_box)` is the distance of `c` from the box's XY plane.
- `F_reg` maps **reference-body coordinates to CT coordinates**
  (`c = F_reg b`). The handout's formulas use the inverse convention
  (`F*_ref F*_reg a*`, `R*_ref R*_reg R_box`); with ours they read
  `F*_ref F*_reg^-1 a*` and `R*_ref R*_reg^-1 R_box`.
- The pointer's tip is at `p_tip` in pointer coordinates and the pointer's
  **+z axis points away from the bone** when it touches the surface (the
  simulator aligns it with the box normal, plus a random tilt of up to 0.1 rad).
- Tracker noise: jiggle `σ_Ta = 0.01 rad`, `σ_Tp = 5 mm` (one draw per
  `sample_marker_bodies` call, shared by all bodies, so it cancels in
  `F_ref^-1 F_ptr`), `σ_b = 0.1 mm` per marker, `δ_sys = 0.03 mm`.
- Simulated time: `place_pointer` costs 1 s, `sample_marker_bodies` 0.03 s.
  Each call is one HTTP round trip (milliseconds); the simulated clock is
  per session and starts at 0.
- A **practice session** (what you open from the notebook) additionally
  answers `session.evaluate_tre(F_reg)` (TRE statistics only) and
  `session.true_outliers()`, for validation and your report. Passing a `seed`
  makes it reproducible. Graded sessions refuse those calls.

## 4. What you have to do

1. **Design** (Step 1 of the notebook): the tracker pose, the reference body
   and pointer marker layouts (manufactured exactly) and `p_tip`. The femur sits
   near the workspace origin; `session.nominal_reference_pose()`,
   `session.nominal_pointer_pose(patch)` and `session.is_visible(points)` let
   you check visibility locally before spending simulated time. The reference
   body is attached close to the bone surface — the simulator has no collision
   model, that is fine.
2. **Closest point** (`src/student/closest_point.py`): point-to-triangle and
   a vectorised closest-point-on-mesh. A per-triangle Python loop will not fit
   in the grading time limit (10 minutes wall clock for the whole program).
3. **Registration** (`src/student/icp.py`): your own point-set registration,
   a coarse initialisation from the patch boxes (the initial guess `F0_reg`
   from `session.initial_registration_guess()` is only rough — plain ICP from
   there is not guaranteed to converge), and
   a robust ICP with outlier rejection (10 % of the samples are outliers).
4. **Uncertainty** (`src/student/uncertainty.py`): estimate `η_reg ~ N(μ, C_reg)`
   from the final fit (left perturbation in CT coordinates) and predict the TRE
   distribution over the mesh vertices.
5. **Workflow** (`src/student/workflow.py`): `design_scene`, the patch plan,
   data collection and `run_registration`, which returns a
   `RegistrationReport` (`src/student/report.py`, the required output format:
   patches and sample counts, valid samples and outliers, residual statistics,
   simulated time, `η_reg`, predicted TRE). We call
   `workflow.design_scene()` and `workflow.run_registration(session, mesh, boxes)`
   headlessly when grading, with a graded in-process `Session` — keep those
   signatures and use only the `Session` methods.
6. **Unit tests** (`tests/test_student.py`): make the complete ones pass and
   write the `TODO` ones.

The hidden truth (`F_CT`, `F*_reg`, the sampled points, outlier flags) never
leaves the server. `session.evaluate_tre` / `session.true_outliers` are for the
validation section of the notebook and your report; a `run_registration`
that calls them is flagged by the grading harness, which also uses different
seeds and its own outlier probability.

**Don't forget**: the handout also requires a written report.

## Troubleshooting

- `ModuleNotFoundError: No module named 'scipy'` — the kernel is not using the venv.
- `ModuleNotFoundError: No module named 'src'` — the notebook assumes `notebooks/`
  is the working directory; `pytest` assumes you run it from `assignment3/`.
- `RuntimeError: cannot reach the server at ...` — wrong `SERVER_URL`, no network,
  or the server is down (ask on Piazza). `server returned 403` — wrong token;
  `404 unknown session` — the session expired (idle for hours): open a new one.
- `pip install` fails / `venv` missing (Linux) — `sudo apt install python3-venv`.
