# Assignment 2 — Report Notes

Maintain concise, topic-organized material for the final report. Update after significant algorithm or experiment milestones; preserve links to relevant code, equations, figures, tables, and test results. Results below are nominal geometry and synthetic numerical checks; student calibration accuracy has not been evaluated.

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

### Step 4: relative-frame error and angle extraction

- **[COURSE]** Use the TA ordering `deltaF = F_a^-1 F_b`: `deltaR = R_a^T R_b`, `epsilon = R_a^T(p_b-p_a)`. Report the principal axis-angle magnitude `theta in [0,pi]` in radians and `||epsilon||` in millimeters. Since `R_a` is orthogonal, `||epsilon|| = ||p_b-p_a||`. This is the translation of the relative Frame, not the translational component of a full SE(3) logarithm.
- **[COURSE]** Lecture 2, slide 45 defines the relative rotation and axis-angle error, with `deltaR ~= I + sk(alpha)` and `theta ~= ||alpha||` for small error. Its Cayley option uses `a = tan(theta/2)n`, so `theta = 2 atan(||a||)`; the Cayley vector is not the rotation vector `alpha=theta*n`.
- **[DERIVED]** For the unit quaternion convention on Lecture 3, slide 20, `q=[w,v]=[cos(theta/2), n sin(theta/2)]`, the principal angle is `theta = 2 atan2(||v||, |w|)`. Slides 23–24 connect matrix-to-quaternion conversion with axis-angle extraction. The scalar/vector ratio printed in the local slide 24 appears reversed relative to slide 20's definition; the expression here satisfies the identity and 180-degree limits.
- **[ENGINEERING]** Select the quaternion route using `Rotation.from_matrix(deltaR).magnitude()`, preserving the classroom error definition while evaluating the exact principal angle. Cayley requires solving with `I+deltaR`, which is singular at pi. The Rodrigues-derived trace formula `acos(clip((tr(deltaR)-1)/2,-1,1))` is mathematically correct but loses very small angles through cancellation. A first-order skew approximation alone returns `|sin(theta)|`, not the angle for general rotations.
- For `N` error pairs, compute angular and positional RMS separately: `rms_angle=sqrt(sum(theta_i^2)/N)`, `rms_pos=sqrt(sum(d_i^2)/N)`, plus the respective maxima and sample count. Do not combine radians and millimeters into an unscaled 6D norm.

Sources: [classroom-method reference](CIS_I_Assignment2_Classroom_Math_Methods.md), Lecture 2 slides 38/45 and Lecture 3 slides 20/23–24; [SciPy magnitude API](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.magnitude.html).

### Step 5: rigid registration and frame averaging

- **[COURSE]** Arun registration (Lecture 3, slide 17; [local method reference, §2.2](CIS_I_Assignment2_Classroom_Math_Methods.md#22-preferred-rigid-3-d3-d-solution-centering--svd-arun-slide-17)): minimize `sum ||R a_i+t-b_i||^2` over `R in SO(3)`. Center both clouds, set `H=A_centered^T B_centered=U S V^T`, then `R=V D U^T`, `t=b_bar-R a_bar`. **[DERIVED]** `D=diag(1,1,sign(det(V U^T)))` corrects reflections along the smallest-singular-value direction, yielding the best proper rigid fit without scaling.
- Require matching finite point sets with at least three noncollinear correspondences. Centered clouds and cross-covariance must have numerical rank at least two; three noncollinear or coplanar markers remain valid. Rank checks use relative floating-point tolerances, not a fixed length threshold. A reflected full-rank cloud generally has nonzero residual under any proper rigid transform; the solver must not hide that discrepancy by returning a reflection.
- **[DERIVED]** Equal-weight chordal frame mean: `p_bar=mean(p_i)`; project `M=mean(R_i)=U S V^T` onto `SO(3)` with `R_bar=U diag(1,1,sign(det(U V^T))) V^T`. This minimizes `sum ||R-R_i||_F^2`, since expanding the objective leaves maximizing `tr(R^T M)`. Orthogonality of `U,V,D` gives `R_bar^T R_bar=I`, and determinant correction gives `det(R_bar)=+1`. This extends the course's SVD constrained least-squares idea; it is a chordal mean, not an intrinsic/geodesic mean or an Euler-angle average.
- **Design decision:** leave `refine_marker_body` unimplemented for now. The special EE calibration body's local layout is exact (`a*_k=a_k`) and its frame is exactly `F_ee`, as stated in notebook Step 5 and [the calibration-body API](src/simulation/calibration_body.py). Re-estimating that known layout from noisy readings would introduce error. Refinement is relevant only if future designs add ordinary bodies with manufacturing uncertainty; repeated pose measurements can still reduce random observation noise for the exact body.

## Algorithm Implementation
- [metrics.py](src/student/metrics.py): `frame_error` uses the relative Frame and quaternion-based principal rotation angle; `summarize_errors` reports `n`, separate RMS values, and maxima. Empty, malformed, nonfinite, or negative error data raise `ValueError`, so missing observations cannot appear to give perfect calibration.
- [registration.py](src/student/registration.py): implements independent Arun/SVD point registration and projected chordal Frame averaging in the designated TODO bodies. Empty, nonfinite, and invalid-rotation frame inputs raise `ValueError`; no simulator solver or hidden geometry is used.

## Experimental Settings and Quantitative Results
<!-- Record experiment seeds, pose/sample settings, accuracy and simulated-time metrics with units, and links to figures and tables. -->

## Algorithmic Validation
- Scene geometry: the notebook's 81 corner/edge poses all keep every marker visible. A separate sweep of 3 values for each linear joint and 5 independently chosen values across each wrist joint's limits also passes: `3^3 * 5^3 = 3375/3375` nominal poses. No random robot realization or hidden truth was used to choose the placement.

### Step 4 metric validation

- All three original TA metric tests pass unchanged: known angular/translation errors, invariance to a common left transform, and RMS aggregation. The original notebook sanity example returns `(0.01 rad, 1.0 mm)`.
- [Student-added metric tests](tests/test_metrics_student.py) contain 27 passing parameter cases covering identity, `1e-12`/`1e-9` rad, the `1e-3` rad target scale, negative rotations, near/exact pi, principal-angle wrapping, relative-transform ordering, separate rad/mm summaries, and invalid/missing data. The angle oracle constructs matrices from known axis-angle values using Rodrigues' formula, independently of quaternion angle extraction.
- Additional deterministic sweep: 1064 known rotations, random axes from seed 42, with log-spaced angles near zero, a uniform interior range, and log-spaced distances from pi. Maximum absolute angle error: `8.88e-16` rad. For a known `1e-9` rad z rotation, direct trace/arccos returned `0`, while the selected method returned `1e-9` rad.
- Aggregation example: `(0.3 rad,3 mm)` and `(0.4 rad,4 mm)` give `rms_angle=0.3535533906 rad`, `max_angle=0.4 rad`, `rms_pos=3.5355339059 mm`, `max_pos=4 mm`, `n=2`. These are synthetic metric checks, not measured robot calibration performance.

### Nominal kinematics reference checks

Measured using the public nominal `fwd_kins` / `inv_kins` methods, without simulation truth. These are mathematical reference checks for later student validation, not evidence of improved calibration. The corresponding tests are in [test_simulation.py](tests/test_simulation.py).

| High-value check | Inputs and criterion | Observed result |
| --- | --- | --- |
| Forward kinematics at home | `theta=0`, robot-base coordinates | `p=(600,0,0)` mm; rotation exactly identity |
| Forward/inverse round trip | 100 poses, NumPy generator seed 0; linear joints across their full ranges, wrist joints 4–5 in `[-1.4,1.4]`, joint 6 in `[-pi,pi]`; original joints used as IK seed | Max `e_p=2.84e-14` mm; max `e_R=2.46e-16` rad. Max individual linear/wrist joint reconstruction errors: `1.14e-13` mm / `2.22e-16` rad |
| Wrist singularity | `theta=(300,0,300,0.3,pi/2,0.2)`; compare reconstructed pose rather than Euler angles | `e_p=7.11e-15` mm; `e_R=3.05e-17` rad |

### Step 5 registration and averaging validation

- All four original TA registration/averaging tests pass unchanged. [Student numerical tests](tests/test_registration_student.py) add 25 passing cases: rank-two planar recovery at scales `1e-6,1,1e6`, reflected data, noisy least squares (seed 51, 30 points, `0.1 mm` Gaussian noise), degenerate/nonfinite inputs, angle wrapping, translation averaging, common-left-frame equivariance, and proper-rotation validation.
- High-value reflection fixture: source points `(+/-6,0,0),(0,+/-3,0),(0,0,+/-1)` mm, reflected about x and translated by `(5,-2,4)` mm. The estimate is the proper rotation `diag(-1,1,-1)` with minimum squared residual `8 mm^2`; allowing a reflection would spuriously make the residual zero.
- High-value averaging fixtures: z rotations `179 deg` and `-179 deg` average to `180 deg`; `0.1` and `0.5 rad` average to `0.3 rad`, with translations averaged independently. Even when three orthogonal half-turns yield an improper raw mean `-I/3`, the returned matrix has determinant `+1` and the correct minimum proper-rotation chordal cost `16`.
- The unchanged notebook Step 5 sanity check recovers translation `(10,0,0)` mm. These checks validate the numerical helpers, not achieved robot calibration accuracy or reduction of systematic tracker bias.

### Student algorithm test criteria (pending implementation)

The former skipped tests now contain executable assertions in [test_student.py](tests/test_student.py). Their algorithm assertions have **not passed** yet because the corresponding student functions remain unimplemented.

- **Calibration fit and generalization:** generate 60 noiseless training poses and 20 independent held-out poses with seed 13 and nonzero model parameters (Gaussian perturbations of scale `1e-3`). Require held-out errors below `1e-3` mm and `1e-5` rad. Compare predicted poses, not raw fitted coefficients, to allow parameter gauge freedom; use SciPy rotation magnitude as an independent error oracle. These synthetic tolerances do not represent achieved simulator accuracy.
- **Pose-plan validity:** every planned joint vector must be finite, respect limits, keep every marker visible under nominal geometry, and excite all six joints. Planning must not move the robot or consume simulated time. This necessary check does not prove Jacobian identifiability or accuracy.
- **Optional marker refinement:** a controlled four-marker manufacturing deformation starts with proper-rigid-aligned marker RMSE `2.387` mm. Require at least 75% error reduction after 60 readings, update the nominal layout without changing physical truth, and account for exactly `60*0.03=1.8` s of sampling time. Centering and an independent SciPy proper-rotation alignment remove the arbitrary rigid-frame gauge while rejecting a reflected layout; truth is used only to construct and score this test fixture.

## Meaningful Limitations
- Coverage bounds and plots use nominal kinematics and placement. They do not establish accuracy or coverage under the simulator's unknown base, axis, tool, and joint errors, nor do they establish a feasible calibration-time budget. The 3375-pose sweep is a geometry check, not a calibration acquisition plan.
- Machine-precision consistency of the supplied nominal kinematics does not validate an error model or fitting procedure. Workflow, improved kinematics, and optional marker refinement remain unimplemented. Synthetic parameter recovery will test optimizer consistency within the chosen model family; simulator accuracy and generalization across robot seeds still require separate experiments.
- Metric inputs are assumed to be valid rigid Frames. SciPy's matrix conversion can orthogonalize small rotation-matrix drift; this metric is not a substitute for independently checking the SO(3) validity of a future fitted model. The scalar angle does not supply a signed rotation axis at pi, which Step 4 does not require.
- Registration numerical rank checks detect unobservable geometry, but do not guarantee good conditioning under measurement noise; spread-out markers remain important. The unweighted chordal frame mean assumes estimates of the same transform. Widely dispersed rotations can have nonunique means (SVD returns one minimizer); it does not account for unequal uncertainty or remove fixed systematic bias. Translation is averaged in the common target frame, independently of orientation, rather than with an SE(3) intrinsic mean.
