# Assignment 3 report notes

## Step 1 — Scene geometry and frames

`src/student/workflow.py::design_scene()` takes no arguments and returns the TA's
`SceneDesign`. It constructs the nominal tracker-to-workspace frame, two
rigid marker layouts, and the pointer tip. The notebook's Step 1 code calls
the same function. Positions are in millimeters and angles are in radians.

| Marker | Reference-body coordinates (mm) | Pointer coordinates (mm) |
| --- | ---: | ---: |
| 1 | (−42, −25, 12) | (−34, −22, −8) |
| 2 | (36, −20, 18) | (33, −18, 2) |
| 3 | (−17, 34, 28) | (−15, 27, 16) |
| 4 | (23, 11, 10) | (16, 13, −10) |
| Centroid | (0, 0, 17) | (0, 0, 0) |

- **Tracker.** `F_tracker_approx = [I, (0, 0, −1600)]` maps tracker coordinates
  into workspace coordinates. Local +z aims at the workspace origin, which
  lies halfway between the frustum's 1,000 and 2,200 mm depth limits. We chose
  this alignment to leave equal depth allowance on either side of the origin.
- **Reference body.** Its 78 × 59 × 18 mm marker envelope gives spread in both
  lateral directions. All markers sit 10–28 mm above the local mounting
  plane, assuming the reference origin is on that plane.
- **Pointer.** The marker envelope is 67 × 49 × 26 mm. The tip is
  `p_tip = (0, 0, −110)` in the pointer frame. The marker cluster sits behind
  the tip along +z, which the simulator directs away from the bone at contact.
  The 110 mm axial distance leaves room for a handle while reducing the
  tip-to-centroid lever arm by 40 mm from the notebook's 150 mm example.
  The handle and mount are geometric design assumptions.

Both layouts have four asymmetric, noncoplanar markers. Three noncollinear
paired markers suffice to determine an ideal rigid pose. The fourth marker
adds an observation, and the lateral separation gives the pose fit more
leverage against marker position noise. The minimum pairwise separations
are 34.6 mm (reference) and 37.3 mm (pointer). The centered marker matrices
both have rank three.

For a workspace marker `p_W`, the camera-frame check transforms it to
`q = R_tracker^T (p_W − t_tracker)`. The tracker accepts every marker with
`1000 ≤ q_z ≤ 2200` and `|q_x|, |q_y| ≤ w(q_z)`, where
`w(z) = 500 + 500 (z − 1000) / 1200` mm is the frustum half-width. A small
pointer rotation error `α` in the pointer frame displaces its tip by
approximately `R_ptr (α × p_tip)`. This first-order adaptation of the
course frame-error model motivates keeping the lever arm short.

The layout and camera constraints come from Assignment 3 handout pp. 4–7,
TA README §§3–4, TA notebook Step 1, and TA Assignment 1 `Tracker.is_visible`.
The mathematical references are Taylor's `lecture2_frames.pdf` slides 21–22
(inverse and composition) and 117–118 (small frame errors),
`lecture3_02_localizers.pdf` slides 51 and 59 (optical triangulation and line
of sight), and `lecture3_rigid3d3dcalculations.pdf` slides 3–4 and 17 (rigid
paired-marker pose fitting). The dimensions and the mock mounting plane are
engineering choices.

**TA frame convention.** `F_reg` maps reference coordinates to CT coordinates,
as defined on handout p. 5 and in the TA README. A CT point maps into the
reference frame via `F_reg.inv()` in the handout p. 7 placement expression.
The same sources give different TRE targets (about 1 mm on p. 7 and at most
2 mm in the starter) and tracker noise values (0.01 mm per-marker and 0.003 mm
systematic on p. 7, versus 0.1 mm and 0.03 mm in the TA README).

## Step 1 — Offline verification and limitations

**VERIFIED.** Run `python -m pytest experiments/test_step1_scene.py -q -s`
from `assignment3/` with the interpreter in `SETUP.md`. Both tests passed
(0.37 s pytest elapsed time). They checked the headless return types, camera
inverse, rotation determinant, marker rank and asymmetry, and deterministic
serialization. The notebook imports and Step 1 code cells also ran in IPython
from `notebooks/`, loading 1,568 vertices, 3,135 triangles and 46 patch boxes.
The notebook and headless entry point serialized to the same scene.

The tests applied the official frustum check to all 46 real patch centers in
each of three **synthetic** workspace poses. These were the identity and
two translated, rotated poses (up to 50 mm per axis and 0.3 rad). The mock
reference origin was 15 mm outward from a surface vertex. Pointer origins
were computed so the tip stayed at each patch center under the specified
0 or ±0.1 rad tilts. All 138 mock pointer placements and all three reference
poses were visible. Across those checks, marker depths ranged from 1,425.3
to 1,837.8 mm. The minimum clearance was 362.2 mm in depth and 518.0 mm
laterally. These checks used zero simulated time.

**NOT RUN.** Server nominal poses, noisy tracker observations, collection,
registration, and TRE are later checkpoints. The offline frustum test
checks geometric coverage under the stated mock poses. Physical clearance,
hand occlusion, and actual pointing ergonomics require separate validation.

## Step 1 — Contribution and AI assistance

OpenAI Codex performed the starter setup, implemented `design_scene()` and
the notebook Step 1 cell, wrote and ran the offline checks, and drafted these
notes. The prompt asked it to read the project instructions and classroom
math reference, preserve existing work, import and verify the official
starter and dependencies, and implement only Step 1. The scene dimensions
and offline validation cases were selected during that work. Student and
teammate contributions beyond the supplied project instructions and math
reference have yet to be recorded.

## Step 2 — Practice connection and nominal visibility

The TA notebook creates a `RemoteSession` with the Step 1 design and practice
seed 1. The server supplies the initial Reference → CT registration guess and
the nominal reference and pointer poses in workspace coordinates. Step 2.1
transforms each body's marker positions into workspace coordinates and calls
`session.is_visible()` for the reference and for each patch's pointer pose.
The visibility calculation runs locally and consumes zero simulated seconds.

**VERIFIED OFFLINE (mock response).** Executed the original imports, Step 1,
Step 2, and Step 2.1 notebook cells from `notebooks/` using the Assignment 3
Python environment. A mock HTTP response supplied a practice session with
seed 1, the 46 supplied patch-box centers and orientations as synthetic
contacts, a synthetic reference pose, and a zero-second clock. The client
decoded the initial guess correctly. The reference and all 46 pointer poses
passed nominal visibility in the mock. The mock recorded session creation,
clock reads, and cleanup, with no placement or sampling calls.

**LIVE CHECK BLOCKED.** The notebook's course token field was empty. The
terminal failed to resolve the starter hostname, and opening it in the
browser returned `invalid course token`. The live session clock, initial
guess, and nominal visibility remain unverified. Use the current Piazza URL
and token in the existing Step 2 cell to run the live check.
