# Assignment 2 — Report Notes

Maintain concise, topic-organized material for the final report. Update after significant algorithm or experiment milestones; preserve links to relevant code, equations, figures, tables, and test results. Results so far cover nominal scene geometry and kinematic consistency; student calibration accuracy has not been evaluated.

## Scope and Report Structure
- Official brief: [2026_assignment_2.pdf](2026_assignment_2.pdf).
- Quality and structure reference: [Assignment 1 report](../assignment1/notebooks/CIS_I_Assignment1_Report_Final.pdf).
- Target approximately 20 pages, avoiding unnecessary length and meeting all official requirements.

## Design Decisions
- Tracker frame coincides with the workspace frame and looks along `+z`. The robot base has identity orientation and translation `(-800, 0, 1300)` mm, placing the center of its wrist-center travel box at `(0, 0, 1600)` mm in the camera volume. The earlier positive-x, negative-z placement put the robot behind the camera.
- Retain the four-marker tetrahedron `(s,s,s)`, `(s,-s,-s)`, `(-s,s,-s)`, `(-s,-s,s)`, with `s=37.5` mm. Each marker is `sqrt(3)*s = 64.952` mm from the EE origin, within the 75 mm limit.
- Design parameters and visibility sweep: [notebook, Steps 1–2.1](notebooks/assignment2_template.ipynb). Nominal geometry visualization: [function](vendor/functions/visualization.py), [figure](figures/workspace_design.png). The function uses no hidden truth and does not move the robot or spend simulated time.

## Mathematical Derivations and Key Equations
- In tracker coordinates, the nominal wrist center spans `x,y in [-300,300]` and `z in [1300,1900]` mm. This includes the nominal 500 mm base-to-wrist offset.
- The wrist-to-EE translation has length 100 mm. For any wrist rotation, every marker is within `rho = 100 + 64.952 = 164.952` mm of the wrist center by the triangle inequality. Hence a conservative marker envelope is `x,y in [-464.952,464.952]`, `z in [1135.048,2064.952]` mm. This is a bound, not the exact reachable volume.
- Camera half-width is `h(z) = 500 + (z-1000)*500/1200` mm for `1000 <= z <= 2200`. At the envelope's nearest depth, `h=556.270` mm exceeds its maximum lateral magnitude by approximately 91.3 mm. Near/far depth margins are approximately 135.0 mm. Thus the entire nominal envelope is inside the frustum, including wrist rotations between sampled poses.
- For kinematic round trips, compare `F_ref = FwdKins(theta)` with `F_rec = FwdKins(InvKins(F_ref))` using `e_p = ||p_ref-p_rec||` (mm) and `e_R = ||Log(R_ref^T R_rec)||` (rad). Near a wrist singularity, joint angles need not be unique: reconstruction of the pose is the appropriate correctness criterion.

## Algorithm Implementation
<!-- Explain implemented calibration, registration, forward/inverse kinematics, and estimation methods; link to relevant code in src/student/. -->

## Experimental Settings and Quantitative Results
<!-- Record experiment seeds, pose/sample settings, accuracy and simulated-time metrics with units, and links to figures and tables. -->

## Algorithmic Validation
- Scene geometry: the notebook's 81 corner/edge poses all keep every marker visible. A separate sweep of 3 values for each linear joint and 5 independently chosen values across each wrist joint's limits also passes: `3^3 * 5^3 = 3375/3375` nominal poses. No random robot realization or hidden truth was used to choose the placement.

### Nominal kinematics reference checks

Measured using the public nominal `fwd_kins` / `inv_kins` methods, without simulation truth. These are mathematical reference checks for later student validation, not evidence of improved calibration. The corresponding tests are in [test_simulation.py](tests/test_simulation.py).

| High-value check | Inputs and criterion | Observed result |
| --- | --- | --- |
| Forward kinematics at home | `theta=0`, robot-base coordinates | `p=(600,0,0)` mm; rotation exactly identity |
| Forward/inverse round trip | 100 poses, NumPy generator seed 0; linear joints across their full ranges, wrist joints 4–5 in `[-1.4,1.4]`, joint 6 in `[-pi,pi]`; original joints used as IK seed | Max `e_p=2.84e-14` mm; max `e_R=2.46e-16` rad. Max individual linear/wrist joint reconstruction errors: `1.14e-13` mm / `2.22e-16` rad |
| Wrist singularity | `theta=(300,0,300,0.3,pi/2,0.2)`; compare reconstructed pose rather than Euler angles | `e_p=7.11e-15` mm; `e_R=3.05e-17` rad |

### Student algorithm test criteria (pending implementation)

The former skipped tests now contain executable assertions in [test_student.py](tests/test_student.py). Their algorithm assertions have **not passed** yet because the corresponding student functions remain unimplemented.

- **Calibration fit and generalization:** generate 60 noiseless training poses and 20 independent held-out poses with seed 13 and nonzero model parameters (Gaussian perturbations of scale `1e-3`). Require held-out errors below `1e-3` mm and `1e-5` rad. Compare predicted poses, not raw fitted coefficients, to allow parameter gauge freedom; use SciPy rotation magnitude as an independent error oracle. These synthetic tolerances do not represent achieved simulator accuracy.
- **Pose-plan validity:** every planned joint vector must be finite, respect limits, keep every marker visible under nominal geometry, and excite all six joints. Planning must not move the robot or consume simulated time. This necessary check does not prove Jacobian identifiability or accuracy.
- **Optional marker refinement:** a controlled four-marker manufacturing deformation starts with proper-rigid-aligned marker RMSE `2.387` mm. Require at least 75% error reduction after 60 readings, update the nominal layout without changing physical truth, and account for exactly `60*0.03=1.8` s of sampling time. Centering and an independent SciPy proper-rotation alignment remove the arbitrary rigid-frame gauge while rejecting a reflected layout; truth is used only to construct and score this test fixture.

## Meaningful Limitations
- Coverage bounds and plots use nominal kinematics and placement. They do not establish accuracy or coverage under the simulator's unknown base, axis, tool, and joint errors, nor do they establish a feasible calibration-time budget. The 3375-pose sweep is a geometry check, not a calibration acquisition plan.
- Machine-precision consistency of the supplied nominal kinematics does not validate an error model or fitting procedure. Student metrics, registration, workflow, and improved-kinematics tests remain blocked on their implementations in later notebook steps. Synthetic parameter recovery will test optimizer consistency within the chosen model family; simulator accuracy and generalization across robot seeds still require separate experiments.
