# CIS I — Classroom Mathematical Methods for Assignment 3

**Purpose:** A source-grounded, implementation-oriented reference for a coding agent working on **Fall 2026 Assignment 3: Surface Registration with a Tracked Pointer**. Follow the official TA interfaces and prefer the methods taught in CIS I rather than introducing unrelated algorithms. This is a **method-selection reference**, not completed code, validated numerical performance, or a claim of passing hidden grading.

**Suggested project path:** `assignment3/CIS_I_Assignment3_Classroom_Math_Methods.md`. Keep separate from `assignment3/REPORT_NOTES.md`, which should contain actual decisions, derivations used, verified numerical results, and limitations.

**Primary sources examined:** `Assignment3.pdf` (2026, pp. 1–8); `lecture2_frames.pdf`; `lecture3_02_localizers.pdf`; `lecture3_rigid3d3dcalculations.pdf`; `lecture4_mathreview_leastsquaresreview.pdf`; `lecture4_calibration.pdf`; `lecture6_02_finding_point-pairs.pdf`; `lecture6_registration_part_1.pdf`; `lecture7_registration_part_2.pdf`; `lecture7_02_coherent_point_drift_registration.pdf`; the TA's GitHub `LCSR-CIIS/CIS_I_Course/assignments/assignment3/README.md`, `notebooks/assignment3_template.ipynb`, `src/session.py`, `src/mesh.py`, `src/student/{closest_point,icp,uncertainty,workflow,report}.py`, and `tests/test_student.py`.

**Evidence labels:** **[COURSE]** = stated or developed in a lecture/assignment/official starter; **[DERIVED]** = algebraic consequence or an adaptation of a taught method; **[ENGINEERING]** = implementation/testing recommendation, **not** a stated course requirement. Each substantive mathematical derivation in the *final report* still needs a complete citation to the appropriate lecture and/or primary source.

**Important:** This document is specifically for Assignment 3. Do not carry over Assignment 2's robot kinematics, joint interpolation, or its strict marker-body and simulation assumptions indiscriminately.

---

## 0. What the assignment actually asks, and method priorities

### 0.1 Goal and binding constraints

- **[COURSE: Assignment 3, pp. 4–8]** The goal is rigid **reference-body-to-CT surface registration** from pointer-tip measurements on a physical/simulated femur. Find `F_reg` such that `c_CT ~= F_reg * d_ref`, where CT points lie on the supplied triangle mesh. The exact sampled CT points, true `F*_reg`, and outlier flags are hidden from the registration algorithm.
- **[COURSE: official TA README]** The grading entry points are `workflow.design_scene()` and `workflow.run_registration(session, mesh, boxes) -> RegistrationReport`. Student functions are in `src/student/`. Keep their signatures and return types. The grader uses a `Session` with the same interface, not necessarily `RemoteSession`.
- **[COURSE: official TA README]** The upstream `assignment3/` has a **1,568-vertex / 3,135-triangle** proximal femur `.sur` mesh and **46** quasi-planar patch boxes in JSON. These are **CT geometry**, not physical landmarks whose exact point correspondence is known to the student.
- **[COURSE: Assignment 3, p. 7]** Nominal outlier probability is `p_outlier=0.1`. A fresh `PlacePointer(patch)` call randomly chooses a new point within that patch (or an outlier); marker sampling is separate. **[COURSE: README]** Hidden grading can change the outlier probability: never hard-code a known outlier count or access true flags in the grading pipeline.
- **[COURSE: handout p. 8 and README]** Total grader **wall-clock runtime <= 10 minutes**, including the workflow. Simulated sampling time must be reasonable but is **not itself given a 600 s hard cap** for Assignment 3. Each `place_pointer()` costs approximately **1 simulated second** and each `sample_marker_bodies()` costs **0.03 s** in the TA simulator.
- **[SOURCE CONFLICT]** Handout p. 7 states a TRE goal near **1 mm**; the TA GitHub README/notebook state **<=2 mm**. Design for the stricter goal where feasible, and confirm grading interpretation with the TA; do not present them as identical.
- **[SOURCE CONFLICT]** Handout p. 7 discusses a calibrated tracker (`sigma_b=0.01 mm`, `delta_sys=0.003 mm`) after listing larger typical values; the README specifies the simulator configuration `sigma_b=0.1 mm`, `delta_sys=0.03 mm`, tracker-jiggle angle `sigma_Ta=0.01 rad`, translation `sigma_Tp=5 mm`. Distinguish **the PDF's stated assumption** from **the README's simulator configuration**; don't silently overwrite either.
- **[SOURCE CONFLICT]** The PDF p. 5 **explicitly defines** `F_reg: Reference -> CT`, matching the figure and GitHub code, but its p. 7 pointer-placement formula composes `F*_ref F*_reg a*_CT` as if `F*_reg: CT -> Reference`. Use the consistent GitHub convention `F*_ref (F*_reg)^-1 a*_CT`. The orientation equation similarly needs `R_box`, not a 4x4 `F_box`, when used as a rotation factor.
- **[COURSE: Assignment 3, pp. 1–3]** Written work must document equations and citations, code/algorithm reasoning, corner cases, call hierarchy, validation, measured performance, teammates' roles, and **AI tools/prompts**. The PDF's old submission mention of `assignment1/` is inconsistent with the provided `assignment3/` starter; follow current TA submission clarification rather than guessing.

### 0.2 Decision tree: preferred methods by component

| Task / function | Prefer: classroom-based baseline | Optional escalation | Avoid |
|---|---|---|---|
| `design_scene()` | Visible, widely spaced, asymmetric marker layouts; explicit frame/frustum checks [COURSE: Lectures 2/3, TA notebook] | Numerical marker-geometry comparison [ENGINEERING] | Invisible/degenerate markers; assuming camera jiggle cancels independent marker noise |
| `plan_patches()` | Many spatially distributed and diverse-normal patches; first benchmark uniform 46 x 4 [DERIVED/ENGINEERING] | Greedy farthest-point + normal-diversity selection; later adaptive sampling | All points on a tiny/near-planar region; choosing a large sample budget without runtime reasoning |
| `collect_samples()` | `d_ref = F_ref^-1 * F_ptr * p_tip`; preserve patch/sample IDs [COURSE: Lecture 2 and template] | Explicit retry / logging for invisible bodies | Treating the hidden CT point as a supplied pair; converting using unknown true registration |
| `closest_point_on_triangle()` | Planar least-squares/barycentric projection + exact edge/vertex restriction [COURSE: Finding Point-Pairs slides 11–16] | Equivalent Voronoi-region case dispatch | Returning the infinite-plane projection when it lies outside the triangle |
| `closest_points_on_mesh()` | Batched vectorized **exact triangle-distance** search [COURSE + ENGINEERING] | Conservative bounding-sphere/tree pruning (Lecture 6 Finding Point-Pairs) | Python loops over every triangle; vertex-only KD-tree as an exact surface answer |
| `register_points()` | Centered constrained LS **Arun/SVD** with reflection correction [COURSE: Lecture 3 slide 17] | Lecture 3 iterative SE(3) fit; quaternion/Horn | Unconstrained affine OLS matrix used as a final rotation |
| `coarse_registration()` | Robust per-patch representative points -> box centers -> Arun/SVD [DERIVED from lecture] | Patch-plane small-angle iterative LS; compare `F0_reg` candidates | Assuming patch-centroid and box center are exact point pairs |
| `icp()` | Closest triangle correspondences + point-to-point SVD updates + robust rejection [COURSE: Registration Part 1] | Point-to-plane Gauss-Newton near a good alignment [DERIVED from lecture LS/ICP] | Fixed correspondences forever; trusting local optimum without validation |
| Outliers inside `icp()` | MAD-based residual gate as starter's `reject="mad"` suggests [ENGINEERING; course teaches robust estimation] | Course robust M-estimator / IRLS; trimmed ICP as comparison | Assuming TA already implemented MAD; always deleting exactly 10% |
| `registration_covariance()` | Inlier Jacobian + local LS covariance, **left perturbation in CT** [DERIVED from course uncertainty/LS] | Weighted information matrix / mock-noise Monte Carlo | Confusing covariance with true error or mixing left/right Jacobians |
| `predicted_tre()` | Propagate 6D covariance to mesh vertices [COURSE foundation/DERIVED] | Monte Carlo SE(3) perturbations [ENGINEERING] | Equating FRE with actual TRE; conflating RMS with mean/percentiles |
| `run_registration()` / tests | Use provided API, explicit modules, numerical checks, multi-seed practice validation | Runtime and accuracy experiments | Grader calls to ground-truth oracles; changing protected support files |

**Overall baseline [ENGINEERING]:** valid scene -> patch plan -> measured `d_ref` -> robust patch-center coarse SVD -> point-to-point ICP with MAD -> Jacobian-based covariance -> predicted TRE -> report -> multi-seed practice evaluation. Add complexity only when verified failures indicate a specific need.

---

## 1. Lecture 2: coordinate frames, transforms, and small errors

**Sources:** `lecture2_frames.pdf`; Assignment 3 pp. 4–7; TA README's explicit `F_reg` convention.

### 1.1 Name frames and compose carefully

Use `T_A_B` to mean **maps coordinates expressed in B to coordinates expressed in A**:

`x_A = T_A_B x_B = R_A_B x_B + t_A_B`.

- **[COURSE]** `T_A_C = T_A_B T_B_C`; `T_A_B^-1=[R_A_B^T, -R_A_B^T t_A_B]`.
- **[COURSE]** Rotations are proper: `R^T R=I`, `det(R)=+1`. `sk(a)b=a x b`; `sk(a)^T=-sk(a)`. A frame acting on a position uses **translation added**, not multiplied by the position.
- **[COURSE/TA]** `T_CT_ref = F_reg`: `x_CT = F_reg * x_ref`. Inverse is `T_ref_CT=F_reg^-1`.
- **[COURSE/TA]** `T_tracker_ptr = F_ptr`, `T_tracker_ref = F_ref`, `p_tip^ptr` is the known tip point in pointer coordinates.
- **[COURSE/TA]** Physical-tip measurement in reference coordinates:

  `d_k^ref = F_ref,k^-1 F_ptr,k p_tip^ptr`.

- **[DERIVED]** True location of selected CT point in tracker/physical workspace (known *only to simulator*):

  `x_tracker* = F_ref* (F_reg*)^-1 a_CT*`.

- **[COURSE/TA]** Given a trial estimate `F_reg`, transform the observed point **to CT**: `x_k = F_reg d_k^ref`. Any closest-point-on-mesh query operates in CT coordinates.
- **[COURSE]** Patch box `F_box=[R_box,p_box]` maps **box -> CT**. `n_box = R_box e_z` is its approximate plane normal in CT; `n_box^T(c_CT-p_box)` is the signed distance to the box's XY plane, *not necessarily* to the exact curved mesh.

**Frame-order invariant:** Never multiply transformations without writing the start/end frames; matrices do not commute. In code, check `F * F.inv()` and known synthetic point chains.

### 1.2 Left perturbations and first-order point displacement

- **[COURSE: Lecture 2; Lecture 3, slides 14–16]** A small rotation vector `alpha` has `DeltaR ~= I+sk(alpha)`. This is a **linearization**, not an exact proper rotation. Recompose with `Rot(alpha)` / an SO(3) exponential.
- **[TA: `uncertainty.py`]** Use the **left/CT** residual error model near final registration:

  `F_reg* = Exp(delta) Fhat_reg`, `delta=[alpha; epsilon]`, with `alpha` (radians), `epsilon` (mm).

- **[DERIVED]** For a CT-frame point `x=Fhat_reg d_ref`, a left perturbation yields

  `delta x ~= alpha x x + epsilon = [-sk(x) | I] delta`.

- **[COURSE/DERIVED]** A **right** perturbation has a different Jacobian (`[-R sk(d) | R]`); never reuse it for left errors. Keep the convention consistent in ICP refinement, covariance, and predicted TRE.
- **[ENGINEERING]** `||alpha||` and `||epsilon||` have different units; never combine them in an unscaled Euclidean stopping threshold or condition-number calculation without considering units/coordinate origin.

---

## 2. Scene, tracker, noise, and observability (Steps 1–2)

**Sources:** Assignment 3 pp. 4–7; `lecture3_02_localizers.pdf`; `lecture3_rigid3d3dcalculations.pdf` slides 1–17; TA notebook Steps 1, 2, and 2.1.

### 2.1 What is designed vs what is hidden

- **[TA]** `SceneDesign` fields: `F_tracker_approx`, `ref_marker_positions`, `ptr_marker_positions`, `p_tip`. Reference and pointer marker locations are **manufactured exactly in the GitHub scenario**; do not apply the PDF's more general generic marker-manufacturing uncertainty to these two bodies without evidence.
- **[TA]** Tracker viewing frustum is along **local +z**, about **100–220 cm** from camera, expanding from **100 cm to 200 cm** squares. Nominal femur is near workspace origin. Check actual transformed markers using `session.nominal_reference_pose()`, `session.nominal_pointer_pose(patch_id)`, and `session.is_visible(points_world)`.
- **[TA]** Pointer local **+z points away from bone** when contacting; for a typical pointer, `p_tip` lies at negative local z relative to its marker cluster. Do not assume 0.1 rad tilt means the physician/controller knows exact contact position; the simulation places the pointer automatically using hidden truth.
- **[COURSE]** Optical pose estimation needs enough **visible, distinct, nondegenerate** markers; 3 noncollinear known point pairs determine an ideal proper rigid pose. Widely spaced, asymmetric marker layouts can improve conditioning/identifiability and reduce lever-arm amplification.

### 2.2 How noise enters, and what cancels

- **[TA]** `sample_marker_bodies()` supplies `Observation.frame` for both bodies (or `None` if invisible), plus individual marker observations. Body frames are **measured estimates**, not hidden true poses.
- **[DERIVED]** If the same left tracker jiggle `DeltaF` premultiplies both observed frames, `(DeltaF F_ref)^-1 (DeltaF F_ptr) = F_ref^-1 F_ptr` exactly. **Common-mode jiggle cancels** in the relative frame calculation.
- **[COURSE/DERIVED]** Independent per-marker measurement errors, occlusions, systematic spatial distortion, and relative body-localization errors **do not generally cancel**. Repeated observations cannot remove fixed bias merely by averaging.
- **[COURSE/DERIVED]** A pointer-tip lever arm `p_tip` amplifies pointer orientation uncertainty: `delta p_tip ~= -R_ptr sk(p_tip) alpha + epsilon`. Shorter effective lever arms and wider marker spread may reduce tip variance, subject to visibility.
- **[ENGINEERING]** Compare multiple feasible scene geometries with nominal visibility and finite-sample performance before choosing a final design; avoid putting markers behind the camera or losing the reference body on any planned patch.

---

## 3. Step 4 — Exact closest point on the mesh

**Sources:** `lecture6_02_finding_point-pairs.pdf`, especially **slides 11–20** (triangle point queries and bounding spheres), **slides 21–32** (hierarchical search), and **slides 79–100** (covariance trees and triangle meshes); TA `closest_point.py` / offline tests.

### 3.1 Closest point to ONE nondegenerate triangle

For triangle `A,B,C`, query `x`, let `u=B-A`, `v=C-A`, `w=x-A`.

- **[COURSE: Finding Point-Pairs, slides 11–15]** Seek planar projection `q=A+lambda*u+mu*v` by solving:

  `[ [u.u, u.v], [u.v, v.v] ] [lambda,mu]^T = [u.w,v.w]^T`.

- **[COURSE]** Projection belongs to the triangle iff `lambda>=0`, `mu>=0`, `lambda+mu<=1`. If true, `q` is the exact closest triangle point.
- **[COURSE]** Otherwise the exact closest point lies on an edge or vertex. Project onto each segment `P->Q` with

  `tau=clip( dot(x-P,Q-P)/dot(Q-P,Q-P), 0, 1 )`, `q_seg=P+tau(Q-P)`;

  choose the minimum squared distance over `AB`, `BC`, `CA`. Voronoi-region case classification is equivalent.
- **[ENGINEERING]** Handle degenerate/near-zero-area triangles: fall back to nonzero edges or isolated vertices, not inversion of a singular 2x2 matrix. Test face, all three edges, all three vertices, boundary ties, and zero-area input.
- **[ENGINEERING]** Use stable evaluation and absolute geometric tolerances appropriate for mm. The TA tests require ~`1e-6` agreement.

### 3.2 Closest point to the ENTIRE mesh

- **[COURSE]** `q*(x)=argmin_{q in mesh} ||x-q||`; exact answer may lie inside a triangle or edge, **not** at a mesh vertex.
- **[TA]** Return `(closest: (K,3), tri: (K,), dist: (K,))` for queries `(K,3)`, all expressed in CT coordinates; `dist` is Euclidean **distance**, not squared distance.
- **[ENGINEERING]** First implement batched **NumPy-vectorized exact closest-point-to-triangle** calculations and reduce to best triangle per query. Mesh has ~3,135 triangles; avoid per-triangle Python loops and unbounded `K x triangles x ...` temporary allocations.
- **[COURSE: Finding Point-Pairs, bounding-volume sections]** For triangle or subtree enclosed by a sphere of center `c_i` and radius `r_i`, a safe lower bound is `lb=max(0,||x-c_i||-r_i)`. If `lb>current_best`, the triangle or subtree can be safely skipped. A centroid is **not** an exact closest point, and centroid-nearest triangle alone is not an exact answer.
- **[COURSE]** Octrees / bounding-sphere hierarchies / covariance trees can prune many candidates. **[ENGINEERING]** Add an acceleration structure only if benchmarks of the vectorized baseline indicate a runtime bottleneck.
- **[TA]** Provided `Mesh` includes `tri_vertices`, `face_normals`, `areas`, `centroids`, and `bounding_sphere_radius`; use precomputed fields but do not edit `src/mesh.py`.

### 3.3 Step 4 tests and performance

Run the TA tests `test_closest_point_on_triangle_regions`, `test_closest_points_on_mesh_matches_brute_force`, `test_closest_points_on_surface_points_have_zero_distance`, plus deliberately degenerate cases. Benchmark repeated queries at the sample count expected by ICP (e.g. 200 points) and evaluate correct result **before** optimizing speed.

---

## 4. Step 5 — Patch planning and surface sampling

**Sources:** Assignment 3 pp. 5, 7–8; TA notebook Step 5; Lecture 6 surface-registration examples and Lecture 2 frame algebra. The *sampling optimization strategies* below are **engineering adaptations**, not prescribed lecture algorithms.

### 4.1 What patch IDs mean

- **[TA]** `boxes` are predefined CT quasi-planar regions; each has box centroid `p_i` and rotation `R_i`, hence approximate normal `n_i=R_i e_z`. The `plan_patches` output is **a list of integer patch IDs**, repeated according to desired sample counts.
- **[TA]** `place_pointer(i)` makes the server secretly choose one *new random point* on patch `i`, with optional outlier injection, and returns only a `PointerPlacement` handle (`patch_id`, `sample_index`, `t_sim`). The exact sampled CT point is **not exposed**.
- **[COURSE]** Patch IDs are region-level information; they are **not** exact 3D point-to-point correspondences.

### 4.2 Candidate patch plans and numerical criteria

1. **Uniform coverage [ENGINEERING baseline]:** choose all visible/usable 46 patches, e.g. 4 calls each (`N=184`). Simple, robust baseline for coverage and outlier redundancy. Not necessarily time-optimal.
2. **Greedy farthest-point sampling [ENGINEERING, derived from geometry]:** select next patch `i*=argmax_i min_{j in S} ||p_i-p_j||`. Choose initialization and stopping budget explicitly.
3. **Normal diversity [ENGINEERING]:** encourage `1-|n_i^T n_j|` across selected patches, in addition to spatial separation; use normalized distances and documented weights rather than adding mm to dimensionless angles unscaled.
4. **Adaptive sampling [COURSE: handout permits; ENGINEERING algorithm]:** sample initial patches -> register -> inspect inlier residuals / predicted uncertainty -> request more informative patches. More involved and can be misled by incorrect local minima; defer until fixed-plan baseline works.

**Why coverage matters [DERIVED]:** For point-to-plane residuals, each measurement gives a row `J_k=[-n_k^T sk(x_k), n_k^T]`. If chosen normals/positions do not span six independent directions, `J` is rank deficient and some rigid motions remain weakly constrained. Many samples on a single plane do **not** resolve every rigid DOF.

### 4.3 Collection math and output contract

- **[COURSE/TA]** For each `pid` in the plan: `handle=session.place_pointer(pid)`; `obs=session.sample_marker_bodies()`. If either body returns `None`, no valid pair exists; record a failed attempt and skip it for registration.
- **[COURSE/TA]** Compute `d_ref=F_ref.inv() * (F_ptr * session.design.p_tip)`. Expanded:

  `d_ref=R_ref^T (R_ptr p_tip+t_ptr-t_ref)`.

- **[TA]** `SampleSet` contains synchronized `handles`, `patch_ids`, `d_ref`, `t_collect`; its `as_array()` returns an `(N_valid,3)` NumPy array. Preserve original server `sample_index` for outlier comparison/report even when invisible attempts are skipped.
- **[ENGINEERING]** Make a mock `Session` to test frame chains, missing bodies and sample indexing without using a live server.
- **[TA]** With one placement and one tracker read per attempt, simulated time is approximately `T_sim=(1+0.03)N_attempts` seconds (excluding extra sampling). Example: 184 attempts -> **189.52 simulated s**. Wall time includes remote calls and ICP computation, and is different.

---

## 5. Step 6A — Rigid registration of KNOWN point pairs

**Sources:** `lecture3_rigid3d3dcalculations.pdf`, especially **slides 3–4, 12–17**, and quaternion alternatives on slides 18–29; `lecture4_mathreview_leastsquaresreview.pdf` (SVD/LS). TA `register_points(a,b)`.

### 5.1 Preferred constrained point-to-point LS: Arun/SVD

Given *known corresponding* pairs `(a_i,b_i)` expressed in source/target frames:

`min_{R,t} sum_i ||R a_i+t-b_i||^2`, subject to `R^T R=I`, `det R=+1`.

1. `abar=mean_i a_i`; `bbar=mean_i b_i`.
2. `a'_i=a_i-abar`; `b'_i=b_i-bbar`.
3. Cross-covariance `H=sum_i a'_i (b'_i)^T`.
4. SVD `H=U diag(s) V^T`.
5. `D=diag(1,1,det(V U^T))`, `R=V D U^T`.
6. `t=bbar-R abar`; return `Frame(R,t)`.

- **[COURSE]** Centering isolates translation; because `||R a'||^2=||a'||^2`, minimizing point-to-point squared error is equivalent to maximizing `tr(RH)`. SVD solves this **fixed-correspondence rigid objective directly**, without registration-level iteration.
- **[ENGINEERING]** Never solve unconstrained 12-variable affine OLS and use its `R` as a rotation. Apply determinant correction; test `R^T R~=I`, `det R~=1`, exact synthetic recovery, and collinearity/near-degeneracy.
- **[COURSE]** Other taught rotation estimators include small-angle iterative rigid LS and quaternion/Horn; valid alternatives, but no need to replace the direct Arun baseline.
- **[ENGINEERING]** `register_points` is for *known correspondences*, not for the unlabelled full surface sample set. It may be reused inside centroid coarse registration and point-to-point ICP fixed-match updates.

---

## 6. Step 6B — Coarse registration from patch information

**Sources:** TA notebook Step 6 / `icp.coarse_registration`; Lecture 3 rigid registration; Lecture 6 surface registration; Lecture 7 linearized least squares. The specific patch-based coarse objective is **[DERIVED]**, not a lecture-prescribed solver.

### 6.1 What is known, and why initialization is necessary

- **[TA]** `d_by_patch = {patch_id: array(n_i,3)}`; `boxes` supplies `p_box,i` and `n_box,i` in CT coordinates. Patch ID is known, but the exact sampled CT location inside a patch is hidden.
- **[COURSE: Registration Part 1, slide 61]** ICP has **local minima**, especially with poor initial alignment and partial overlap. The rough server `F0_reg` is **not guaranteed** to put standard ICP in the correct convergence basin.

### 6.2 Baseline: robust patch representative points + Arun

For each patch `i`, compute representative `m_i^ref` (initially mean; compare componentwise median/trimmed mean or robust mean if outliers distort it). Approximate correspondence: `m_i^ref <-> p_box,i`.

`F_coarse = argmin_{R,t} sum_i w_i ||R m_i^ref+t-p_box,i||^2`.

- **[DERIVED]** If weights are uniform, use `register_points()`. For nonuniform weights, use weighted centroids and weighted cross-covariance before SVD; no scale parameter.
- **[CRITICAL LIMITATION]** Uniform random sampled points need **not** have their centroid exactly at the box center. An outlier can badly skew a patch with only a few samples. This is **an approximation**, not exact correspondences. Assess geometric spread and sample count.
- **[ENGINEERING]** Use multiple, noncollinear, well-distributed patches and inspect covariance singular values and transformation quality; fail explicitly when usable patch geometry is insufficient.

### 6.3 Optional refinement: known patch planes + linearized 6D LS

Each sampled point `d_ik` belongs approximately to the plane at known box centroid `p_i` with normal `n_i`:

`e_ik(R,t)=n_i^T (R d_ik+t-p_i)`.

Minimize robustly `sum_{i,k} w_ik e_ik^2`. At current `F=[R,t]`, set `x_ik=F d_ik`, `xi=[alpha;epsilon]`, `DeltaF=Exp(xi)` (left/CT):

`e'_ik ~= e_ik + J_ik xi`, `J_ik=[-n_i^T sk(x_ik), n_i^T]`.

Solve `min_xi ||W^(1/2)(J xi+e)||^2` using `lstsq` / QR / SVD; update `F <- Exp(xi) F`; recompute residuals/J; stop on small changes or rank problems.

- **[DERIVED]** Standard Arun does **not** solve this general objective directly: after squaring the plane residual, terms involve `R^T n_i n_i^T R`, which do not simplify to `I` by orthogonality. Rotation constraints still matter. Linearization makes a **local LS subproblem**, hence the typical iteration.
- **[ENGINEERING]** Start the plane solver from the robust centroid estimate or another reasonable initialization. Use a damping/step-size safeguard when initial errors are not small. If normals lack diversity, the plane system is rank deficient.
- **[ENGINEERING]** `coarse_registration()` has no `F0_reg` input in its required signature. Comparing `F0_reg`, `F_coarse`, and any refinements must happen in `run_registration()` or other authorized orchestration, not by inventing hidden access to server truth.

---

## 7. Step 6C — ICP, outlier handling, and convergence

**Sources:** `lecture6_registration_part_1.pdf`, **slides 25–29** (surface registration), **51–61** (ICP), **63–69** (outlier-robust pose estimation), and **70–77** (practical ICP code); `lecture7_registration_part_2.pdf`, **slides 19–26** (robust estimation example); TA `icp.py` and tests.

### 7.1 Baseline: point-to-point ICP using Step 4 and Arun

Given current `F_j=F_ref->CT` and all input samples `d_k`:

1. Transform into CT: `x_k=F_j d_k`.
2. Match: `q_k=ClosestPointOnMesh(x_k, mesh)` (Step 4).
3. Compute **Euclidean** surface residuals `r_k=||x_k-q_k||`.
4. Build a robust inlier mask, re-evaluated as the pose improves.
5. With **fixed current correspondences**, solve `F_{j+1}=register_points(d_inlier,q_inlier)` (or an explicitly weighted variant).
6. Repeat; finally **recompute** `closest`, `normals`, `residuals`, and `inlier_mask` at `F_final`.

- **[COURSE]** ICP is iterative **because closest-point correspondences change**, even though its known-pair rigid SVD update is closed form.
- **[TA]** `IcpResult.residuals` must contain `||F_reg d_k-closest_k||` for **all** samples, even if the optimization uses signed point-to-plane distances internally. `inlier_mask` corresponds to each original valid sample; `history` is RMS **inlier Euclidean** residual per iteration.

### 7.2 Optional: point-to-plane ICP with incremental 6D LS

At current transformed point `x_k`, mesh closest point `q_k` and unit normal `n_k`, signed residual:

`e_k=n_k^T(x_k-q_k)`.

For a small left CT update `xi=[alpha;epsilon]`:

`e'_k ~= e_k+[-n_k^T sk(x_k) | n_k^T] xi`.

Stack rows into `J`, solve `min_xi sum_{inliers} (e_k+J_k xi)^2` (with weights if used); update `F <- Exp(xi)F`. Recompute correspondences and normals each ICP iteration.

- **[DERIVED]** The small-angle/Jacobian method follows Lecture 3 direct iterative registration and lecture LS; surface normals and closest points come from Step 4/mesh. The full point-to-plane update is **not** solved by one Arun SVD call.
- **[ENGINEERING]** Usually use after reasonably good initialization; may converge faster but can have unconstrained directions if the surface is approximately planar. Handle damping, conditioning, and exact rotation recomposition.

### 7.3 Robust outliers: implementation is YOUR responsibility

- **[TA]** `icp(..., reject: Optional[str]="mad")` declares a **string option**, not a provided MAD solver. Implement rejection **inside `icp()`** or in a student helper. `reject=None` should have defined no-rejection behavior.
- **[COURSE: Registration Part 1 robust pose estimation; Registration Part 2, slides 19–26]** Large outliers should be excluded or downweighted. The lectures introduce **robust pose estimation and M-estimators**; specific MAD cutoffs are an **engineering choice**.
- **[ENGINEERING baseline]** With Euclidean nonnegative residuals `r_k`, `m=median(r)`, `MAD=median(|r-m|)`, a heuristic scale `s=1.4826*MAD`, and threshold `tau=m+kappa*max(s,s_floor)`. Mark `r_k<=tau` as inliers. Choose/document `kappa` and floor; check early/late iteration behavior and avoid discarding nearly all samples.
- **[MATH CAUTION]** `1.4826` is calibrated for the MAD of **signed Gaussian** samples, not a universal exact standard deviation for **Euclidean 3D residual magnitudes**. State it as a robust heuristic if used here.
- **[ENGINEERING]** Other options: fixed threshold with mm noise justification, trimmed ICP (not automatically 10%), M-estimator/IRLS weights from course robustness discussions. Hidden grader can vary the outlier rate; never use `session.true_outliers()` in student registration code.
- **[ENGINEERING]** Hard rejection too early may classify genuine but initially misaligned samples as outliers. Consider a permissive initial gate/robust weights and revise masks after improved alignment. Enforce minimum inlier counts and spread/rank.

### 7.4 Termination and local minima

- **[COURSE: Registration Part 1 practical ICP outline]** Terminate using stable residual, small pose update, and maximum iterations. No single residual threshold proves correct registration.
- **[ENGINEERING]** Check `rms_{j-1}-rms_j`, rotation increment in **radians**, translation increment in **mm**, and `max_iter` (starter default 100, `tol=1e-4` with a documented interpretation). Guard oscillation and nonmonotonicity when masks change.
- **[ENGINEERING]** Compare multiple plausible initializations (`F_coarse`, optional `F0_reg`, plane-refined pose) using robust objectives and geometry. Do not choose based on forbidden ground-truth TRE in graded code.

---

## 8. Step 7A — Registration covariance (left CT perturbation)

**Sources:** `lecture2_frames.pdf` small error/covariance; `lecture3_rigid3d3dcalculations.pdf` slide 37 onward on probabilistic estimation; `lecture4_mathreview_leastsquaresreview.pdf` LS/linearization; `lecture6_registration_part_1.pdf` (probabilistic/uncertainty-aware registration); TA `uncertainty.py`, notebook Step 7.

### 8.1 Two distinct meanings: correction mean vs residual uncertainty

- **[TA]** `registration_covariance(result, F0_reg) -> (mu:(6,), C_reg:(6,6))`.
- **[TA]** Comment: `F*_reg=Exp(delta) . F_reg` and `eta_reg=[alpha;eps] relative to F0_reg`; keep **left perturbations expressed in CT**.
- **[DERIVED]** The estimated correction from initial `F0` to final `Fhat` can be represented as `mu ~= Log_SE3(Fhat F0^-1)` in 6D local coordinates. This is **not** an observed true error. For non-small correction, use an actual `SE(3)` log or a clearly documented convention/approximation.
- **[DERIVED]** The covariance `C_reg` represents the **remaining local uncertainty near Fhat** in `F*=Exp(delta)Fhat`, typically with residual perturbation mean zero by construction. These are conceptually different distributions/references; avoid claiming `mu` directly equals a known registration bias.

### 8.2 Preferred geometric local model: point-to-plane information

For final inlier `k`, let `x_k=Fhat d_k`, `q_k` closest mesh point, and `n_k` its unit normal:

`e_k=n_k^T(x_k-q_k)`, `J_k=[-n_k^T sk(x_k) | n_k^T]` (1x6).

Stack `e (M,)`, `J (M,6)`. Under **fixed/local correspondences** and independent homoscedastic signed residuals of variance `sigma^2`:

`C_reg ~= sigma^2 (J^T J)^-1`, `sigma_hat^2 ~= (sum_k e_k^2)/(M-6)` for full rank and `M>6`.

- **[COURSE/DERIVED]** This is the first-order least-squares covariance formula applied to a local surface residual model; **not an exact statement about ICP's full nonlinear/contact-selection uncertainty**.
- **[ENGINEERING]** Use SVD/QR or stable solves to assess rank and condition rather than explicitly computing a fragile inverse. If rank <6, the data do **not** identify a finite 6DOF covariance under this model. `pinv` alone can disguise missing information by assigning zero variance to a null direction.
- **[ENGINEERING]** If meaningful correlations/noise variances are known, use a weighted model `C ~= (J^T Sigma_e^-1 J)^-1`, applying whitening/stable solves; heuristically weighted robust objectives need additional statistical justification for calibrated covariance.
- **[CAUTION]** A point-to-plane covariance is useful even if final ICP used point-to-point, *but the objective mismatch must be explained*. Do not treat arbitrary point-to-point nearest-neighbor correspondences as known independent anatomical landmarks.

### 8.3 Strict starter-interface limitation: transformed samples are absent

- **[TA]** `IcpResult` fields are `F_reg`, `residuals` (**unsigned Euclidean norms**), `inlier_mask`, `closest`, `normals`, `n_iter`, `converged`, `history`. The result **does not contain** original `d_k` or final `x_k=Fhat d_k`.
- **[DERIVED/ENGINEERING, interface-compatible]** For small final residuals, approximate `x_k ~= q_k` **inside the Jacobian**:

  `J_k ~= [-n_k^T sk(q_k) | n_k^T]`.

  This respects the provided signature. The available unsigned `result.residuals` are **not exactly** the signed point-to-plane `e_k`; if using their squared magnitudes for the residual scale, label that a **proxy approximation** and document/validate its effects. A simple proxy is `sigma_hat^2 ~= sum(inlier r_k^2)/(M-6)` under near-surface errors, **not an exact derived noise estimator**.
- **[ALTERNATIVE]** Store exact final transformed points / signed plane residuals in a backward-compatible extension to `IcpResult` *only with explicit authorization to modify TA-provided non-TODO structures*, or design a permitted companion data path. Do not silently change public signatures.
- **[ENGINEERING]** If a point-to-point *fixed-match* covariance is instead used, stack full point residual Jacobians `J_k^3D=[-sk(x_k) | I]` (3x6), with `3M-6` residual DOF under independent 3D noise; note that nearest-mesh correspondences violate the naive fixed-pair statistical assumptions. This is a distinct, potentially overconfident model.

### 8.4 Numerical checks and physical interpretation

- Check `C_reg` is `6x6`, symmetric, positive semidefinite (up to floating-point tolerance), and has physically meaningful magnitude/units (rad^2, mm^2, rad*mm).
- Low sample **FRE** can coexist with high registration **TRE** due to a wrong local minimum or unobservable motion. Covariance quantifies **local sensitivity conditional on the assumed model**, not global correctness.
- **[TA tests]** Complete the TODO statistical-consistency test: with synthetic known truth and repeated trials, compare squared Mahalanobis error `delta_true^T C^-1 delta_true` to a `chi-square_6` benchmark **only if** Gaussian/full-rank assumptions are justified; one run does not have to equal six exactly.

---

## 9. Step 7B — Predicted TRE over CT vertices

**Sources:** Lecture 2 covariance propagation/pose perturbations; Assignment 3 pp. 7–8; TA `predicted_tre(mesh,F_reg,C_reg)` and report fields.

### 9.1 First-order covariance propagation

For a **CT-frame** target vertex `c_j` and small left CT pose error `delta=[alpha;epsilon]`:

`Delta c_j ~= J_j delta`, with `J_j=[-sk(c_j) | I]` (`3x6`).

`C_{c_j} ~= J_j C_reg J_j^T` (`3x3`).

For zero-mean local error, per-vertex predicted root-mean-square displacement:

`rho_j = sqrt( tr(C_{c_j}) )`.

Across `N_v` vertices, the exact RMS of **this first-order second-moment model** is:

`RMS_pred = sqrt( (1/N_v) sum_j tr(C_{c_j}) )`.

- **[COURSE/DERIVED]** This is ordinary covariance propagation from Lecture 2, applied to left-pose error; a rotation error generates positional uncertainty that increases with lever arm/distance from effective rotational center.
- **[MATH CAUTION]** The **mean of `rho_j`**, **p90 of `rho_j`**, and **max of `rho_j`** are summary statistics of *per-vertex RMS predictions*, not necessarily the **mean/p90/max realized TRE magnitude**. Explain exactly which statistic is being returned under keys `mean`, `p90`, `max`.
- If the pose-error model has nonzero residual mean `m_delta`, then `E||Delta c_j||^2 ~= tr(C_{c_j}) + ||J_j m_delta||^2`. Do **not** plug in the correction-from-`F0` `mu` as a residual mean automatically.

### 9.2 Optional Monte Carlo estimate

- Sample `delta^(s) ~ N(0,C_reg)` (with physically consistent mixed rad/mm units), construct a proper `DeltaF^(s)=Exp_SE3(delta^(s))`, apply it as a **left** perturbation to `F_reg`/CT vertices, and compute displacement norms.
- Compute mean/rms/p90/max with a **stated aggregation definition** (e.g., per-trial mesh-wide summary vs pooled vertex-trial values). Finite Monte Carlo maxima depend on the trial count; a maximum is not a guaranteed error bound.
- **[ENGINEERING]** This gives more directly comparable magnitude distributions and checks the small-angle approximation, at extra computation cost. Neither analytical nor Monte Carlo prediction corrects an incorrect `C_reg` or wrong ICP registration.

### 9.3 FRE versus TRE

- **[COURSE: handout p. 8]** Residuals between registered measured samples and the CT surface are often called **FRE** in this handout. Actual target error uses hidden true registration and **all mesh vertices**.
- **[TA]** Practice session: `session.evaluate_tre(F_reg)` gives true-TRE **statistics** only; it does **not** reveal `F*_reg`. `session.true_outliers()` provides true outlier sample indices for practice validation.
- **[TA]** Those two oracle calls are **for notebook Step 8 only**. Never call them in `run_registration()`, `icp()`, `uncertainty.py`, patch-planning logic in graded mode, or any grader-facing decision procedure.

---

## 10. Reporting / final integration with official starter

**Sources:** Assignment 3 pp. 1–3, 7–8; TA `src/student/report.py`, `src/student/workflow.py`; notebook Steps 7–8 and final cell.

### 10.1 Required program-level execution graph

```text
workflow.design_scene() -> SceneDesign
  -> session = RemoteSession(..., design, seed)     [not part of graded run_registration]
  -> workflow.run_registration(session, mesh, boxes)
       -> session.initial_registration_guess() [optional baseline candidate]
       -> workflow.plan_patches(boxes, n_patches, samples_per_patch)
       -> workflow.collect_samples(session, plan)
            -> session.place_pointer(patch_id)
            -> session.sample_marker_bodies()
            -> d_ref = F_ref^-1 F_ptr p_tip
       -> icp.coarse_registration(d_by_patch, boxes)
            -> icp.register_points(...) [if centroid/SVD baseline]
       -> icp.icp(d_ref, mesh, F_init, reject="mad")
            -> closest_point.closest_points_on_mesh(...)
            -> robust mask / rigid least-squares update (repeat)
       -> uncertainty.registration_covariance(result, F0_reg)
       -> uncertainty.predicted_tre(mesh, result.F_reg, C_reg)
       -> report.residual_statistics(result.residuals[result.inlier_mask])
       -> report.RegistrationReport(...)
  -> notebook Step 8: practice-only evaluate_tre/true_outliers [NOT in workflow]
```

`run_registration` must be a **headless** self-contained pipeline. Do not depend on notebook global variables or hidden server state. Keep `Session` protocol compatibility.

### 10.2 Expected output fields

- `RegistrationReport.F_reg` (Reference -> CT).
- `patches`: records of `patch_id`, `n_samples`, `n_used`, `n_rejected`.
- `samples`: original `sample_index`, `patch_id`, `t_sim`, `d_ref`, `residual_mm`, `used`, `reason`; handle invisible attempts carefully without corrupting accepted-sample indices.
- `residual_stats`: provided `report.residual_statistics(inlier residuals)` returns `n_used`, `mean`, `std`, `max`, `p90` on **Euclidean** residual magnitude.
- `t_simulated`, `t_wall`, `n_icp_iterations`, `converged`.
- `eta_reg = {mu, C_reg, convention:"left_ct"}` and `predicted_tre={rms,mean,max,p90}`; serialize arrays properly.
- **[TA]** `RegistrationReport.to_text()` and `.to_json()` are **already implemented**; do not rewrite the class.

### 10.3 Report duties (do not confuse this math reference with evidence)

- Cover sheet, teamwork/honor declaration, goal/performance, tracker/marker sketches and dimensions, design rationale/ergonomics.
- Mathematical derivations with variables, sources, corner cases: triangle projection; frame chain; SVD rigid fit; coarse initialization; ICP/matching; outlier rule; covariance and TRE propagation **if used**.
- Algorithmic explanation with pseudocode and an end-to-end caller/callee diagram; do not dump source code as a substitute.
- Quantitative experiments: sampling plans, points/inliers/outliers, Euclidean residual statistics, predicted and practice-measured TRE, simulated time and wall time, seed sensitivity.
- Cite all third-party/AI-generated code, disclose **tools and prompts** used for code and writing, and distinguish each team member's contributions.
- Document only **verified** experiments in `REPORT_NOTES.md`; keep environment setup in `SETUP.md` if needed. Do not claim a method or precision succeeded before testing.

---

## 11. Numerical tests, failure modes, and escalation criteria

### 11.1 Test at component boundaries, not only at the end

| Module | Priority numerical tests | Typical failure to detect |
|---|---|---|
| Scene / frames | tracker visibility for reference and pointer at planned patch nominal poses; `F F^-1`; synthetic tip frame chain | Invisible markers; reversed transforms |
| Triangle CP | face/edge/vertex/degenerate, cross-check exhaustive exact distances | Infinite-plane projection outside triangle; wrong distances |
| Mesh CP | match brute-force, on-surface distance ~0, repeated 200-query benchmark | Approximate nearest-vertex/centroid masquerading as exact; slow Python loops |
| Point registration | known noiseless `F_true` recovery; `det R=1`; collinear input | Reflections, inverted cross-covariance convention |
| Coarse alignment | simulated patch samples with known transform, outliers and noncentered patch sampling | Large bias from box-centroid mismatch, insufficient patches |
| ICP | provided synthetic recovery and injected-outlier tests, convergence traces | Local minima, overaggressive rejection, inconsistent masks |
| Covariance | shape/symmetry/PSD, rank, finite-difference Jacobian, Mahalanobis calibration across trials | False certainty, inconsistent left/right perturbations |
| TRE/report | propagate known synthetic covariance, verify RMS definitions, JSON roundtrip, complete summary | Equating predicted and actual error; invalid schema or missing sample indices |

- **[TA tests]** `test_student.py` already includes closest-point face/edge/vertex tests, exact mesh matching, known point-set registration, synthetic ICP recovery, synthetic outlier rejection, and report roundtrip. Two TODO tests cover covariance consistency and patch-plan visibility. Run early; extend only the student-designated TODOs or authorized additional student tests.
- **[ENGINEERING]** Use multiple held-out simulation seeds for final validation; compare *actual* `TRE_rms/mean/max/p90`, predicted TRE, residuals, inlier counts, patch coverage, and both clocks. Test outlier rates differing from 10% in offline synthetic tests.

### 11.2 Failure -> recommended response

| Symptom | First diagnose | Prefer before adding complexity |
|---|---|---|
| No valid samples | Camera layout / visibility / marker geometry | Fix `design_scene`, patch plan, visibility |
| Closest search wrong | Segment boundary cases / degenerate triangles / shapes/units | Exact geometry + tests |
| Closest search slow | Triangle loop / large allocation / repeated precomputation | Vectorization and batching; bounding sphere pruning |
| Coarse F very wrong | Patch IDs, box-frame direction, centroid bias, outliers, rank | Robust patch centroids and coverage; plane refinement |
| ICP nonconvergent | Initial alignment, match quality, outlier gate, termination | Fix init/outliers first; optional point-to-plane after good init |
| RMS surface residual low but TRE high | Wrong local minimum / weak geometry / symmetric regions | Better spatial+normal coverage, extra initializations, independent validation |
| Covariance implausibly tiny | Wrong residual DOF, neglecting unknown correspondences, rank deficiency, unmodeled bias | Revisit model and assumptions; test predicted-vs-actual calibration |
| Grader incompatibility | Wrong function signature / notebook globals / oracle usage / modified TA code | Restore official interface and headless integration |

### 11.3 Stability and ethical boundaries

1. Prefer `np.linalg.lstsq`, SVD, QR or a stable linear solve over explicitly constructing `(J^T J)^-1` for optimization; check singular values and effective rank.
2. Recompose exact rotations (`Rot` or SO(3) exponential) from small-angle increments. Check determinant and orthogonality after each registration.
3. Distinguish **fixed noise/correlated error** from fresh independent noise; don't claim systematic bias disappears with repeated samples.
4. Keep units consistent: CT/reference positions and residuals in **mm**, rotations in **rad**, time in **s**.
5. Never look up or hardcode hidden `F*_reg`, sampled CT coordinates or true outlier labels in the graded algorithm; use approved `Session` methods only.
6. Preserve TA starter and official tests unless their designated TODO areas explicitly authorize implementation; follow active repository `AGENTS.md` and get permission before changes to protected structures.
7. Do not claim a course-specific closed-form derivation when it is actually an engineering adaptation; use the evidence labels and verify calculations.

---

## 12. Source map / citations for implementation and report

| Source | Relevant sections | Intended use |
|---|---|---|
| `Assignment3.pdf` | pp. 1–3 instructions/report/AI; pp. 4–6 scenario/geometry/tracker; p. 7 sampled point/outliers/TRE; p. 8 residuals/TRE/time | Binding assumptions, error definitions, report requirements, conflicting PDF conventions |
| TA `assignment3/README.md` | data, session API, conventions, student modules, limitations | **Runtime/starter interface**, marker noise/config, headless grading, actual data files |
| TA `notebooks/assignment3_template.ipynb` | Steps 1–8, final `run_registration` cell | Pipeline checkpoints, per-step contracts, practice validation |
| `lecture2_frames.pdf` | frames, rotation/skew, small-error models, covariance | Frame chaining, left CT Jacobian, TRE propagation |
| `lecture3_02_localizers.pdf` | optical tracking, marker visibility/geometry | Design, body tracking, observation noise |
| `lecture3_rigid3d3dcalculations.pdf` | slides 3–4; 12–17 (especially **17 Arun/SVD**); 18–29 quaternions; 36–38 probabilistic registration | Rigid paired-point registration, incremental pose updates, uncertainty foundation |
| `lecture4_mathreview_leastsquaresreview.pdf` | linear LS, orthogonal decomposition, SVD, adjustment | Numerically stable LS, Gauss-Newton subproblems, covariance principles |
| `lecture4_calibration.pdf` | calibrated pointer / tracker lessons, parameter sensitivity | Design/measurement understanding; **not** an instruction to redo pivot calibration |
| `lecture6_02_finding_point-pairs.pdf` | slides 11–20 triangles/bounds; 21–32 tree; 79–100 covariance-tree families | Exact closest-triangle search and acceleration |
| `lecture6_registration_part_1.pdf` | slides 21–29 surface registration; 51–61 ICP; 63–69 robust estimation; 70–77 practical ICP; 79–80 generalized/uncertainty-aware registration | Core iterative surface-registration logic and limitations |
| `lecture7_registration_part_2.pdf` | slides 19–26 robust estimates; later ICP revisited, IMLP/anisotropic registration | Robust M-estimators and advanced alternatives |
| `lecture7_02_coherent_point_drift_registration.pdf` | CPD conceptual algorithm and outlier/partial-overlap comparisons | Optional probabilistic alternative; **not** required for baseline ICP |
| TA `src/student/{closest_point,icp,uncertainty,workflow,report}.py`, `tests/test_student.py` | required signatures, return fields, tests | Actual code contracts and integration invariants |

**Attribution discipline:** For final report equations, cite the underlying lecture/assignment directly with a precise slide/page reference (and cite any external method if added). This math file is a study and method-selection aid, not a replacement for those citations. Verify slide numbering against the locally shipped PDF before final LaTeX submission.

---

## 13. Short task instruction for your desktop coding agent

> Before implementing Assignment 3, inspect active `AGENTS.md`, the local `assignment3/` tree, the official TA Assignment 3 README/notebook/student stubs/tests and `assignment3/CIS_I_Assignment3_Classroom_Math_Methods.md`. If `assignment3/` is only a placeholder, bring in the official Assignment 3 starter **without overwriting local changes** and keep support modules read-only. Use `F_reg: Reference -> CT` consistently, and obey the `Session` API and `RegistrationReport` schema. Prefer lecture methods: exact barycentric/segment closest-triangle geometry, batched exact mesh search, Arun/SVD paired-point registration, robust patch-based coarse alignment, point-to-point ICP with an implemented MAD gate, and left-CT Jacobian covariance/TRE propagation. Benchmark and validate each module with offline tests before multi-seed practice sessions; use true TRE/outlier oracle calls **only in practice validation**, never in the graded workflow. If the baseline fails, diagnose geometry, initialization, and robustness before adding point-to-plane refinements or sophisticated data structures. Track mathematical decisions, genuinely measured performance, limitations, and AI assistance/prompts for the report; never claim hidden-grader performance, modify protected TA files, or commit/push without explicit authorization.

---

## 14. Final pre-implementation checklist

- [ ] Official Assignment 3 starter is present locally; Python package imports work; TA files and student TODOs identified.
- [ ] `F_reg` frame direction confirmed and tested on synthetic points; PDF p. 7 inverse-convention issue documented.
- [ ] Design marker geometry, viewing frustum, and tip/frame conventions are explicit.
- [ ] Patch-selection baseline and sample/time budget selected; failure and visibility bookkeeping planned.
- [ ] Exact closest-triangle implementation and runtime strategy agreed.
- [ ] SVD/Arun convention, reflection correction and degeneracy checks agreed.
- [ ] Patch-centroid coarse assumption and possible plane-refinement fallback agreed.
- [ ] ICP residual, MAD threshold, inlier-mask semantics and convergence criteria agreed.
- [ ] Covariance model, left CT convention, rank/units, and `IcpResult` data limitation agreed.
- [ ] Predicted TRE statistic definitions (especially `mean`, `p90`, `max`) documented.
- [ ] Practice-only evaluation and hidden-grader isolation explicitly enforced.
- [ ] Report citations, AI prompts, teammate ownership, numerical evidence and call tree will be recorded as work proceeds.
