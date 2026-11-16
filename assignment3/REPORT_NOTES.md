# Assignment 3 report notes

Status statements describe their development checkpoint; later sections
record subsequent implementation and validation.

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

## Step 3 — Offline unit tests and baseline

OpenAI Codex completed the visibility TODO in `tests/test_student.py`
at Vincent's request. The fixture uses the Step 1 scene, the supplied
patch boxes and a fixed test plan containing each of the 46 patch IDs once.
For the synthetic scene, CT coincides with workspace and the reference
mount matches the Step 1 offline setup. For each patch, the pointer uses
the box rotation and a workspace origin of `c_box - R_box p_tip` in mm.
This puts the transformed tip at the box center. The test checks the tip
location and transforms both bodies' markers from their local coordinates
to workspace for the TA tracker frustum check described in Step 1.

**VERIFIED OFFLINE.** The focused visibility test passed in 0.32 s of
pytest elapsed time. The reference pose and all 46 synthetic pointer
poses passed, with zero simulated seconds and no server calls. The TA
suite went from 1 passed, 6 failed and 2 skipped to 2 passed, 6 failed
and 1 skipped (0.34 s pytest elapsed time). All six failures are
`NotImplementedError` from the closest-point, rigid registration and
ICP stubs. From `assignment3/`, reproduce with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests -q`.
For the focused check, replace `tests` with
`tests/test_student.py::test_patch_plan_keeps_bodies_visible`.

**NOT RUN.** Step 5 will supply the actual patch planner and Step 7 will
supply the estimator required by the covariance TODO. Visibility under
live server poses remains a separate Step 2.1 check.

## Step 4 — Closest-point mathematics and interfaces

`src/student/closest_point.py` now implements `closest_point_on_triangle()`
and `closest_points_on_mesh()`. Mesh queries and returned points are in CT
coordinates, in mm. The mesh function maps `(K,3)` queries to closest points
`(K,3)`, triangle indices `(K,)`, and Euclidean distances `(K,)`. It applies
no registration transform; later ICP must first map reference points into
CT using `F_reg`. The single-triangle function accepts four `vct3` inputs
in one common frame and returns a `vct3` in that frame.

For triangle corners `A,B,C` and query `x`, let `u=B-A`, `v=C-A`, and
`w=x-A`. The plane projection minimizes `||A+lambda*u+mu*v-x||²`.
It is inside the triangle when `lambda>=0`, `mu>=0`, and `lambda+mu<=1`;
then it is the closest point. Otherwise, check the three boundary segments.
For an edge `P,Q`, calculate

`t=clip(((x-P)·(Q-P))/||Q-P||², 0, 1)`, `q_edge=P+t*(Q-P)`.

Clipping includes the vertices. Choose the edge candidate with minimum
squared distance to `x`. In code, with `n=u×v`, the plane coordinates are
`lambda=w·(v×n)/||n||²` and `mu=w·(n×u)/||n||²`. These solve the same
least-squares projection without subtracting nearly equal terms in a Gram
determinant; this is an algebraic adaptation of the classroom method.

**Engineering safeguards.** If `||n|| <= eps*max_edge_length²`, use only
the segments (`eps` is float64 machine precision). Zero-length segments
return their vertex. This covers collinear/repeated corners and numerically
collapsed faces. Empty queries return empty arrays; invalid query shapes
and empty meshes raise `ValueError`. Inputs are assumed finite and at
ordinary CT scales. Equal minima keep the first triangle, although roundoff
can change which adjacent triangle wins a boundary tie.

**Sources.** Checked locally: TA README §4 item 2, notebook Step 4, handout
pp. 2–3 (corner cases and algorithm explanation) and p. 8 (wall-clock limit).
The classroom selection guide §3 attributes projection/boundary cases to
`lecture6_02_finding_point-pairs.pdf`, slides 11–16.
**CITATION CHECK PENDING:** this lecture PDF was absent from the local course
materials searched during this continuation; its slide numbers must be
independently checked before final report use. Batching and numerical
safeguards are engineering choices, not quoted course requirements.

## Step 4 — Algorithm and design rationale

We search every triangle for each query. Triangle geometry is reused within
one call, and NumPy evaluates many query–triangle pairs together. Processing
32 queries per batch bounds temporary memory while preserving the full search.

```text
Precompute triangle edges and plane-projection coefficients.
For each batch of at most 32 queries:
    Project each query onto every triangle's plane.
    Keep face projections that lie inside their triangle.
    For outside/collapsed cases, choose the nearest of three segments.
    For each query, select the triangle with minimum squared distance.
    Return its point, triangle index, and square-root distance.
```

Both public functions call `_closest_points()`; the scalar function passes
one triangle, the mesh function passes all triangles. Work is `O(K*T)`
for `K` queries and `T` triangles; temporary memory is `O(32*T)`, apart
from input/output arrays. Broadcasting reduces Python overhead; batching
controls memory. Peak memory has not been measured.

Ahmed's comparison uses the same projection/boundary geometry, with `lstsq`
inside Python loops over queries and triangles. Our version reuses geometry
and evaluates pairs in NumPy; checking all three segments keeps boundary
logic simple. A search tree is unnecessary for the measured workload. The
fixed comparison copy and provenance are in `experiments/reference/` and
are not imported by the graded estimator.

**Requirement review (2026-11-16).** Vincent requested switching to Ahmed's
simpler implementation only if it meets the TA requirements. README §4
item 2 and notebook Step 4 explicitly require vectorized mesh search.
Ahmed's fixed comparison still uses Python loops over both queries and
triangles; calling NumPy `lstsq` for one pair does not vectorize that search.
Its tested geometric agreement therefore does not establish compliance with
this implementation requirement. The batched version remains active; no
fallback directory or code replacement was made. This review changed only
documentation, so numerical tests were not rerun.

## Step 4 — Offline verification and performance

**VERIFIED, 2026-11-16.** All three TA closest-point tests pass: triangle
regions, exhaustive mesh comparison, and zero distance for surface points.
All 13 supplementary tests in `experiments/test_step4_closest_point.py` pass.
They cover all edges/vertices, boundaries, collapsed and thin triangles,
empty queries, input errors, transformed coordinates, and reversed winding.
An independent scalar `lstsq` reference checks 67 synthetic queries against
17 random triangles (seed 42), across multiple batches. This avoids relying
solely on the scalar/vectorized functions' shared kernel.

For the TA's 150 known surface points (seed 0), the largest distance is
`1.07e-14 mm`. The original notebook imports/data cell and Step 4 cell ran
from `notebooks/` without a session and returned the required shapes. Its
Step 4 code and Markdown were left unchanged.

Benchmark: real mesh, 1,568 vertices and 3,135 triangles; 200 queries uniform
in `[-60,60] mm` per CT axis, seed 0, matching the notebook. Triangle data
is preloaded for both versions. The saved final run measured:

| Measurement | Result |
| --- | ---: |
| Batched NumPy median, five runs | 0.145901 s |
| Batched range | 0.142219–0.150320 s |
| Ahmed's fixed comparison, one run | 13.120711 s |
| Comparison time / batched median | 89.9× |
| Largest closest-point difference | 2.84e-14 mm |
| Largest distance difference | 7.11e-15 mm |
| Matching triangle indices | 174 / 200 |
| Different indices, verified on a shared edge/vertex | 26 / 200 |

All differing indices identify valid boundary ties: each answer was checked
against the common edge/vertex of both selected triangles. Points and
distances agree to floating-point precision. A preliminary run gave about
0.15 s versus 13.5 s; the final run added this explicit tie check. Timings
are machine/workload-specific. These are geometry checks, not TRE results;
simulated time is zero, with no server calls.

Reproduce from `assignment3/`:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests/test_student.py -k closest_point experiments/test_step4_closest_point.py -q
PYTHONPATH=. venv/bin/python experiments/benchmark_step4.py --ahmed-source experiments/reference/ahmed_closest_point_ca10a27.py --output experiments/results/step4_benchmark.json
```

Settings, individual runtimes, source hash and numerical differences are in
`experiments/results/step4_benchmark.json`. The full TA suite is **5 passed,
3 failed, 1 skipped**. Including Step 1 and Step 4 supplementary tests gives
**20 passed, 3 failed, 1 skipped** (16.73 s). All three failures remain
`NotImplementedError` in `register_points()` and `icp()` (Step 6); the
covariance TODO remains skipped (Step 7). The ordinary pytest command was
blocked before test collection; reusable environment instructions are in
`SETUP.md`. Full ICP/grader timing, Steps 5 onward, and live simulator
validation are **NOT RUN**.

## Step 4 — Contribution and AI assistance

At Vincent's request, OpenAI Codex continued the “Implement Assignment 3
Setup” discussion locally, reviewed and finalized the unfinished Step 4
implementation, extended offline checks, benchmarked the agreed batched
method, and wrote these notes. The prompt authorized Step 4 only and asked
for simple, concise report explanations. Ahmed783-pixel's uploaded
closest-point implementation was verified at commit
`ca10a27f8d71cfd64076b0b7610546c614517d98` and preserved as an offline
comparison. No other teammate contributions are inferred.

## Step 5 — Sampling plan and collection method

**IMPLEMENTED.** Notebook Step 5 now calls the implemented
`workflow.plan_patches()` and `workflow.collect_samples()` in
`src/student/workflow.py`. The notebook itself, `SampleSet`, the Step 1
scene, and the TA session/mesh/report APIs were unchanged in this step.
Following Vincent's explicit choice, both functions use Ahmed783-pixel's
[workflow implementation at commit ca10a27](https://github.com/Vncntmhlr/CIS_Assignments/blob/ca10a27f8d71cfd64076b0b7610546c614517d98/assignment3/src/student/workflow.py).
The 50-degree value is named as a constant and docstrings explain its behavior;
the selection and collection rules are unchanged.

**Plan.** Each patch has unit CT normal `n_i=R_box[:,2]`. Scan the first
`n_patches` boxes in their input order. Accept the first box, then accept a
new box only if `abs(n_i·n_previous) < cos(50 degrees)`, where `previous`
means the last accepted box. Repeat each accepted ID `samples_per_patch`
times. This avoids consecutive nearly parallel or antiparallel directions
using a simple rule. The threshold is Ahmed's engineering choice, not a
course-prescribed angle.

Here `n_patches` is a **candidate limit**, not a promise to return that many
distinct regions. This differs from the starter's short “how many patches
to use” description and is retained deliberately under Vincent's instruction
to follow Ahmed's method. The actual selected regions/counts must be reported.
For the supplied boxes, arguments `(46,4)` select these 33 regions:

`0,1,3,5,7,8,10,11,12,13,14,15,16,18,21,22,23,26,28,29,30,32,34,35,37,38,39,40,41,42,43,44,45`.

**Collection and frames.** For each ID, place the pointer afresh and take one
tracker reading. From measured pointer-to-tracker `F_ptr` and
reference-to-tracker `F_ref`, calculate

`d_ref = F_ref.inv() * F_ptr * p_tip`

`= R_ref.T @ (R_ptr @ p_tip + t_ptr - t_ref)`.

This expresses the tip in reference-body coordinates, in mm. It does not
apply `F_reg`, reveal the sampled CT point, or make the patch center an exact
correspondence. If either observation is `None`, print its original sample
index and patch ID and skip it. Otherwise append its handle, patch ID and
copied `(3,)` point together. `SampleSet.as_array()` returns `(N_valid,3)`.
The simulated collection time is `elapsed_after-elapsed_before`, including
invisible attempts. There are no automatic retries or outlier decisions.

```text
Scan candidate boxes; keep consecutive normal directions separated by >50°.
Repeat each accepted patch ID by its requested sampling count.
For each ID: place pointer, then read reference and pointer observations.
    If either is missing: log and skip this attempt.
    Otherwise: transform the tip to reference coordinates and save its handle.
Return valid samples and the simulated clock difference.
```

**Sources.** TA notebook Step 5 and README §§3–4 specify the frame chain,
session calls, return structures and time costs. Handout pp. 5, 7–8 describes
patch geometry, repeated fresh contacts and sampling/report requirements.
The local `lecture2_frames.pdf` was checked: printed slide **21** gives the
inverse frame and slide **22** gives composition (PDF pages 10–11, containing
multiple slides). Applying them to measured pointer/reference frames gives
the equation above. The 50-degree selection rule is attributed to Ahmed,
not to those lectures. The earlier all-46/farthest-center proposal was not
implemented, following Vincent's preference for Ahmed's method.

## Step 5 — Verification, measurements and limitations

**VERIFIED OFFLINE.** From `assignment3/`, run
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider experiments/test_step5_sampling.py -q`.
All **17 tests passed (0.33 s)**. They check the normal rule (including
opposite directions and comparison to the last accepted box), candidate and
repeat counts, the actual 33-region plan, invalid/empty inputs, a known
synthetic tip position under nontrivial rotations/translations, cancellation
of a common tracker jiggle, skipped reference/pointer observations, original
nonconsecutive sample indices, empty/all-invisible results, and clock deltas.
The actual selected patches also passed the TA frustum check in all three
Step 1 synthetic scenes: 99 pointer placements and three reference poses.

Direct comparisons with Ahmed's pinned functions gave identical plans for
**94 cases** (`n_patches=0..46`, repetitions 1 and 4), plus identical samples,
handles, times and skip messages in **three mock collection cases** (empty,
all visible, and missing observations). The comparison source's Git blob
is `9ed3bf803016ab7075542df7ec8b0b3bc1015d75`;
`experiments/results/step5_ahmed_equivalence.json` records these checks.

The original notebook Step 5 cell was executed unchanged with a labeled mock:
132 valid synthetic observations, 135.96 mock simulated seconds, and maximum
tip-coordinate error `3.41e-14 mm`. Reproduce using
`PYTHONPATH=. venv/bin/python experiments/validate_step5.py`;
evidence is `experiments/results/step5_offline.json`.

**VERIFIED ON PRACTICE SERVER, seed 1, 2026-11-16.** Reproduce using
`PYTHONPATH=. venv/bin/python experiments/validate_step5.py --live`, which
reads the existing notebook connection settings without saving credentials.
It checks nominal visibility and executes the original Step 5 cell. The
session is closed afterwards; no TRE or true-outlier oracle was called.

| Practice measurement | Result |
| --- | ---: |
| Selected regions / fresh placements / tracker readings | 33 / 132 / 132 |
| Nominal reference and selected pointer poses | All visible |
| Valid paired observations / invisible attempts | 132 / 0 |
| Valid observations per selected region | 4 |
| Simulated collection time | 135.96 s |
| Whole validation script wall time, including session creation/cleanup | 36.562 s |

Actual returned sample indices, patch IDs, times and measured reference-frame
tip coordinates are saved in `experiments/results/step5_practice.json`.
“Valid” here means that both tracker bodies were observed; it does **not**
mean the sample is a registration inlier. Outlier labels and TRE were not
requested. Earlier notes that live visibility/collection were unverified
are superseded by this practice check for seed 1 and the current scene.

The combined TA + Step 1/4/5 supplementary suite is **37 passed, 3 failed,
1 skipped (16.58 s)**. The same three `NotImplementedError` failures existed
before Step 5: rigid point registration and ICP remain Step 6 stubs. The
covariance TODO remains skipped. Original TA assertions were not changed.

**LIMITATIONS / NOT RUN.** The selected normals need not be globally diverse:
the rule compares only consecutive accepted regions and ignores center
separation. Input box rotations are assumed valid. Results depend on input
order and the uncalibrated 50-degree threshold. Invisible attempts are printed
and excluded from `SampleSet`; persistent full-report accounting of those
attempts must be considered at workflow/report integration. No live missing
observations occurred, so that branch is verified with mocks only. One seed
does not establish general visibility or registration performance. Coarse
alignment, ICP, residuals, uncertainty and TRE are **NOT RUN**; they belong
to later steps.

## Step 5 — Contribution and AI assistance

Ahmed783-pixel supplied the selected normal-angle planning rule and collection
logic in the pinned repository version. Vincent chose that method after the
comparison with the all-patch proposal. OpenAI Codex integrated only the two
Step 5 functions, clarified their docstrings, named the angle constant, wrote
the isolated tests/validation script, ran offline and seed-1 practice checks,
and drafted these notes. The instruction was to confirm Ahmed's method and
complete Step 5 using it; no additional teammate contributions are inferred.

## Step 6 — Rigid registration and coarse alignment

**IMPLEMENTED.** `src/student/icp.py` now supplies `register_points()`,
`coarse_registration()` and `icp()`. Signatures and the TA `IcpResult` fields
are unchanged. Distances are in mm; `F_reg` maps Reference → CT. These
functions work without notebook globals or simulator access. Step 7
uncertainty and `workflow.run_registration()` remain unimplemented.

**Known point pairs: Arun/SVD.** Given corresponding source points `a_i`
and destination points `b_i`, minimize `sum ||R*a_i+t-b_i||²` subject to a
proper rotation. Subtract their respective means to obtain `a'_i,b'_i`, then
form `H=sum a'_i*(b'_i).T` and compute `H=U*S*V.T`. Use

`R=V*diag(1,1,det(V*U.T))*U.T`, `t=mean(b)-R*mean(a)`.

Centering separates translation from rotation. The determinant correction
prevents a reflected solution. The implementation rejects fewer than three
pairs, nonfinite coordinates, collinear point sets, and numerically deficient
cross-covariance. Planar noncollinear points are allowed. The public function
accepts lists of `vct3`; its array helper is also used by coarse alignment
and ICP to avoid repeated small object conversions.

**Coarse Reference → CT alignment.** For each nonempty patch group, take
the median of each reference coordinate as its representative point and
pair it approximately with that patch's CT box center. Fit these pairs with
Arun/SVD, using equal weight per patch. Median representatives reduce the
influence of an isolated extreme contact compared with a mean. Unknown patch
IDs or insufficient usable geometry fail explicitly.

The representative and box center are **not exact anatomical correspondences**.
With only four contacts, a median can still be biased, and componentwise
medians depend on the coordinate axes. This coarse frame is only an ICP
starting estimate. Patch-plane refinement was not added because the agreed
baseline converged on the saved practice measurements.

**Sources.** Verified locally in `lecture3_rigid3d3dcalculations.pdf`:
printed slides **3–4** define the paired rigid objective, centering and
translation; slide **17** gives Arun's cross-covariance/SVD rotation and
determinant check. The explicit determinant correction is the standard
constrained-SVD completion, not quoted as a full formula on that slide.
TA README §4 item 3, notebook Step 6 and the student signatures specify
coarse initialization followed by ICP. Patch median-to-center pairing is
a lecture-derived application with an engineering robustness choice.

## Step 6 — ICP, rejection and stopping rule

ICP updates correspondences because the exact sampled CT points are unknown.
At the current Reference → CT estimate, map each original reference sample
`d_k` to `x_k=F_reg*d_k`. Step 4 returns its exact closest mesh point `q_k`
and Euclidean distance `r_k=||x_k-q_k||`. Fit the accepted original `d_k`
directly to their current `q_k` with Arun/SVD, obtaining a new complete
Reference → CT frame. This avoids mixing absolute fits with incremental
frame multiplication.

**MAD rejection.** At every pose, compute `m=median(r)` and
`MAD=median(abs(r-m))`. Keep samples satisfying

`r_k <= m + 3*max(1.4826*MAD, 0.01 mm)`.

The mask is recomputed and a previously rejected sample may re-enter.
`reject=None` retains every sample; unsupported modes raise `ValueError`.
The factor 1.4826 comes from signed Gaussian MAD scaling, but these residuals
are nonnegative 3D distances, so this is an engineering gate, not an exact
noise estimate. The algorithm never assumes exactly 10% outliers or accesses
server truth. Fewer than three usable, noncollinear pairs cannot define an
update and cause an explicit error.

**Stopping.** Require an unchanged mask, inlier RMS change ≤`tol`, and RMS
movement of all transformed samples ≤`10*tol`, both in mm. Default
`tol=1e-4` therefore requires RMS change ≤0.0001 mm and movement ≤0.001 mm.
RMS movement measures the combined rotation/translation effect on points,
so radians are not directly added to millimeters. Stop at `max_iter=100`
otherwise and return `converged=False`. `history` stores post-update inlier
Euclidean RMS; the returned matches, normals, residuals and mask all refer
to the final frame, including when the iteration cap is reached.

```text
Coarse: patch coordinate medians → approximate box-center pairs → Arun/SVD.
ICP: transform all original samples to CT → exact closest mesh matches.
     Calculate Euclidean residuals and refresh the MAD mask.
     Fit accepted original samples to their matches with Arun/SVD.
     Recompute matches, residuals and mask at the new frame; record RMS.
     Stop when the mask, residual and point movement stabilize, or hit the cap.
Return the final Reference → CT frame and per-sample results.
```

Call hierarchy: `coarse_registration → _rigid_fit`; public
`register_points → _rigid_fit`; `icp → closest_points_on_mesh / _inliers /
_rigid_fit`. The covariance/report APIs and notebook were not changed.
The classroom guide §7 links the ICP outline to Lecture 6 Registration Part 1;
its original PDF remains unavailable locally, so its suggested slide numbers
are **CITATION CHECK PENDING**. The MAD constants and stopping thresholds are
engineering choices, not attributed verbatim to the lectures.

**Measured revisions.** The initial 1e-6 mm MAD scale floor rejected 5 and
10 clean points in synthetic seeds 11 and 33 after near-exact alignment.
Raising the floor to 0.01 mm retained all clean points in these cases while
still rejecting all injected outliers. This sets a minimum added gate width
of 0.03 mm, below the README's 0.1 mm per-marker noise scale; it is not a
calibrated tip-noise model. Earlier, requiring point movement ≤`tol` left the
TA recovery test unflagged at 100 iterations despite a maximum paired error
of 0.0038 mm. The stated `10*tol` motion criterion supersedes that prototype;
the original test assertions and iteration cap were preserved.

## Step 6 — Verification and measured performance

**VERIFIED.** All three original TA registration tests pass: exact paired
transform recovery, ICP recovery from perturbed alignment, and injected-outlier
rejection. Supplementary checks in `experiments/test_step6_registration.py`
cover legal planar fits, reflection exclusion, invalid/degenerate inputs,
patch medians with controlled extreme outliers, 0/10/25% outlier fractions,
no-rejection mode, iteration-cap behavior, input preservation, and consistency
of final points/residuals/normals/history. Two regression cases specifically
check the measured near-zero-MAD failure above. Final combined-suite counts
are **64 passed, 1 skipped (34.73 s)**: eight checks in the TA test suite and all
Step 1/4/5/6 supplementary checks pass, including the 24 new Step 6 cases.
The only skip is the still-unimplemented Step 7 covariance TODO. Before
this step the TA baseline was five passed, three failed and one skipped;
all three former registration failures are resolved without changing tests.

Reproduce from `assignment3/`:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests experiments/test_step1_scene.py experiments/test_step4_closest_point.py experiments/test_step5_sampling.py experiments/test_step6_registration.py -q
PYTHONPATH=. venv/bin/python experiments/validate_step6.py
```

`experiments/results/step6_validation.json` records configuration, source hash,
transforms, RMS histories, rejection counts and wall times. The synthetic
cases use the actual CT mesh (1,568 vertices, 3,135 triangles), 120 exact
surface contacts, known rigid truth, and 12 random additive outliers with
20 mm standard deviation per axis. Initial alignment is perturbed by
0.04 rad about CT z and translation `(1,-1,0.5) mm`.

| Synthetic seed | Iterations | Injected outliers rejected | Clean points falsely rejected | Max clean paired error (mm) | Synthetic mesh displacement RMS (mm) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 11 | 55 | 12/12 | 0/108 | 0.003170 | 0.001843 |
| 22 | 41 | 12/12 | 0/108 | 0.002041 | 0.001281 |
| 33 | 46 | 12/12 | 0/108 | 0.002995 | 0.001719 |

All three converged. “Synthetic mesh displacement” compares the estimated
and known synthetic transformations at CT vertices; it is not a measured
practice-server TRE. These contacts are noise-free apart from the injected
outliers and do not reproduce all tracker errors.

**REPLAY OF ACTUAL MEASUREMENTS.** The unchanged Step 6 notebook cell was
also executed with the 132 saved Step 5 practice observations (collection
seed 1). No new placement, tracker reading, or server score was requested.
The final baseline selected **112/132** samples and converged in **39**
iterations. Inlier distances were: RMS **0.163200 mm**, mean **0.128322 mm**,
standard deviation **0.100836 mm**, maximum **0.463372 mm**, p90 **0.250742 mm**.
The coarse all-sample RMS was 5.494697 mm, which includes every outlier and
must not be treated as the same statistic as final inlier RMS. Replaying the
solver took **3.708 s** plus about **0.00061 s** for coarse alignment in the
recorded run. These per-call timings were observed alongside pytest and are
machine/load dependent, not isolated performance guarantees. The original
notebook-cell check is saved separately in
`experiments/results/step6_notebook_replay.json`.

**LIMITATIONS / NOT RUN.** The 20 rejected practice observations have not
been checked against hidden outlier labels. Low surface RMS and a convergence
flag do not establish correct global registration or ≤2 mm actual TRE.
The median-based initializer and hard MAD gate may fail for badly contaminated
patches, weak geometry or a wrong local minimum. Multiple independent practice
seeds, actual server TRE, uncertainty and the headless full workflow remain
unverified; only the three synthetic seeds and the saved seed-1 replay above
were used in this step. `IcpResult` still lacks original `d_ref`; an appropriate
data path must be agreed for Step 7 rather than silently extending the TA type.

## Step 6 — Contribution and AI assistance

Vincent approved the proposed Arun/SVD, patch coordinate-median initialization,
and point-to-point ICP with MAD. OpenAI Codex implemented the three functions,
added isolated numerical/regression checks and a replay script, verified lecture
formulas and wrote these notes. Ahmed's uploaded `icp.py` at commit `ca10a27`
contains only the three starter stubs, so no Step 6 implementation was copied
or attributed to him. His accepted Step 5 sampling method remains unchanged.
The prompts requested a Step 6 comparison followed by implementation after
approval, with concise, plain-English report explanations. No Step 7 functions
or protected TA files were edited.

## Step 7 — Data path and uncertainty model

**IMPLEMENTED.** `uncertainty.registration_covariance()` and `predicted_tre()`
now complete the notebook's Step 7 math. Vincent explicitly approved carrying
exact transformed CT sample points privately from ICP, keeping all TA public
fields and function signatures unchanged. `icp()` attaches
`result._transformed_points_ct` after its final match; the public `IcpResult`
dataclass definition remains untouched. There is no global cache or notebook
dependency, and dataclass serialization ignores this private companion data.

**Mean correction versus remaining uncertainty.** The returned six-vector is
`mu=Log_SE3(F_reg*F0_reg.inv())`, with rotation-vector entries in rad and
translation-tangent entries in mm. The matrix logarithm here means the six
parameters whose rigid exponential reproduces the correction. Its translation
tangent is not generally just the correction frame's translation at finite
rotation. The implementation uses a proper rotation vector and solves the
SE(3) left-Jacobian translation relation, with a small-angle series near zero.
This is a finite-pose bookkeeping choice consistent with the TA's `Exp`
notation, not a verbatim lecture algorithm. `mu` is an estimated correction
from the initial guess, **not measured true error or a residual bias**.

Covariance describes a small **left perturbation in CT coordinates** near
the final pose: `F_true ≈ Exp(delta)*F_reg`, `delta=[alpha;epsilon]`.
It is not claimed to be the exact covariance of the finite correction log
relative to the initial guess. Only final ICP inliers contribute.
Rotation, translation and cross blocks of `C_reg` have units rad², mm² and
rad·mm respectively.

For transformed sample `x_k`, closest mesh point `q_k` and unit normal `n_k`:

`e_k=n_k.T*(x_k-q_k)`, `J_k=[-n_k.T*sk(x_k) | n_k.T]`.

Here `sk(x)*a=x×a`. The Jacobian says how a small rotation/translation changes
distance along the local surface normal. For `M>6` inliers, the local model is

`sigma_hat²=sum(e_k²)/(M-6)`, `C_reg≈sigma_hat²*(J.T*J)^-1`.

This assumes independent, equal-variance, zero-mean signed residuals and fixed
local surface matches. Step 6 fits point-to-point distances; this covariance
uses a local point-to-plane model to describe surface sensitivity. That
objective difference, nearest-point changes, robust selection, correlated
tracker errors and systematic bias limit statistical calibration. This is a
conditional local uncertainty estimate, not proof of globally correct ICP.

**Numerics.** Center sample coordinates at their inlier mean `c` and use their
RMS radius `L` to scale rotational lever arms. Solve with
`A=[((x_k-c)/L)×n_k | n_k]`, so both parameter blocks have mm units. After SVD,
map covariance back to the original CT origin using
`delta=[beta_rot/L; beta_translation+sk(c)*beta_rot/L]`.
This transformation is checked by translating the whole CT coordinate system
and confirming invariant predicted displacements. No explicit matrix inverse
is formed in production. Require rank six and scaled condition ≤`1e8`;
otherwise report a geometry/conditioning error. Also check finite data,
unit normals, symmetry and positive semidefiniteness.

**Compatibility approximation.** An externally constructed original-format
`IcpResult` may lack transformed points. In that case issue a warning and
use `x≈q` in the Jacobian and squared Euclidean residual magnitudes for the
noise-scale proxy. This is explicitly approximate, not a recovered signed
residual. Private diagnostic metadata identifies the branch; normal pipeline
execution uses exact transformed points and signed residuals. Tests compare
both paths for pure-normal and tangential residual cases.

**Sources verified locally.** `lecture3_rigid3d3dcalculations.pdf` printed
slides **36–38** introduce left pose error, the point Jacobian `[-sk(Fb),I]`,
and zero-mean uncertainty after updating a pose. Projecting that Jacobian
onto the normal and using the scalar least-squares variance is our derived
surface-model application. `lecture2_frames.pdf` slide **114** shows the
small-rotation lever-arm term; slide **124** gives covariance propagation
`C_y=A*C_x*A.T`, and slide **125** discusses Mahalanobis testing. The scaled
SVD solve, finite-log bookkeeping and compatibility fallback are engineering
adaptations. TA `uncertainty.py`, notebook Step 7 and handout p. 8 establish
the expected convention, uncertainty outputs and comparison with TRE.

## Step 7 — Predicted TRE and headless report flow

For each already-CT mesh vertex `c_j`, use `G_j=[-sk(c_j) | I]`:

`C_j=G_j*C_reg*G_j.T`, `rho_j=sqrt(trace(C_j))`.

`rho_j` is the predicted RMS displacement at that vertex under the local
zero-mean model. Return `rms=sqrt(mean(rho_j²))`, plus `mean`, `max`, and
`p90` **over the per-vertex RMS values**. Those latter values are not the
mean/max/p90 of actual random TRE magnitudes; `max` is not an error bound.
Do not apply `F_reg` again to CT vertices or insert `mu` as a residual bias.
Production prediction is analytical; random trials are used only for testing.

`workflow.run_registration()` now performs this complete headless flow:

```text
plan_patches → collect samples and attempt records
    → coarse_registration → ICP (with private final CT points)
    → registration_covariance → predicted_tre → RegistrationReport
```

`collect_samples()` still returns the unchanged `SampleSet` and uses Ahmed's
unchanged planning/collection policy. Its shared private collector additionally
records each placement for the report. It preserves original sample indices
and distinguishes `invisible_ref`, `invisible_ptr`, `invisible_ref_and_ptr`,
`icp_mad`, and `ok`. Missing observations have `d_ref=None` and
`residual_mm=None`; they do not enter ICP. Each patch reports attempted,
used and unused counts. The TA text renderer groups all unused attempts under
“Rejected”; the per-sample reasons distinguish invisibility from ICP outliers.

The report uses the existing `RegistrationReport` without editing TA report
code. `eta_reg` includes the left-CT convention, correction definition,
covariance diagnostics, and predicted-statistic definition. Simulated time is
the session's total clock, including invisible attempts. Workflow wall time
includes sampling and computation but excludes session creation/cleanup.
Options are candidate count, repeats, iteration cap, tolerance and rejection;
invalid options are checked before spending sampling time. Only documented
`Session` methods are used, with no true-TRE or outlier-label calls.

## Step 7 — Verification and measured results

**VERIFIED OFFLINE.** The full TA and Step 1/4/5/6/7 supplementary suite passed
**93 tests, no skips (43.83 s)**. The former covariance TODO now tests a stated
fixed-Jacobian, independent Gaussian model; original non-TODO tests were not
changed. The 28 new Step 7 cases include finite-difference Jacobians, exact
left correction logs near zero and near pi, direct covariance comparison,
normal/shape/rank/conditioning checks, translation-of-CT invariance, known
translation/rotation TRE cases, private-data compatibility, original report
JSON round trips, and full Session-only workflow tests with invisible attempts.

The statistical test uses seed **73**, 100 scalar residuals, sigma **0.02 mm**,
and **400 trials**. Mean squared Mahalanobis distance was **6.0842**, compared
with **6.1304** expected when variance is estimated from the same fit; empirical
95% coverage was **0.953**. The finite-sample reference is `6*F(6,94)`, tending
to `chi-square_6` as residual degrees of freedom grow. This checks the local
linear model only; it is not a calibrated full-ICP or tracker experiment.

Reproduce from `assignment3/`:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests experiments/test_step1_scene.py experiments/test_step4_closest_point.py experiments/test_step5_sampling.py experiments/test_step6_registration.py experiments/test_step7_uncertainty.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests/test_student.py::test_registration_covariance_is_consistent -q -s
PYTHONPATH=. venv/bin/python experiments/validate_step7.py
PYTHONPATH=. venv/bin/python experiments/validate_step7.py --live --seed 1
```

The validation script preserves the notebook and executes its original Step 7
cell with the headless report's final data; `mu`, covariance and predictions
agree. It also writes and reloads the unmodified TA report format and compares
the exact-data and legacy-proxy covariance paths. Reports, text summaries and
validation metadata are under `experiments/results/step7_*`.

**Replay transport is a mock.** The default script manufactures tracker
frames reproducing saved actual reference-tip coordinates. It uses the saved
coarse pose as mock `F0`, since the old server initial guess was not saved.
Its `mu` therefore describes that coarse-to-final correction, not the old
server's correction. The full replay used 112/132 points and predicted RMS
**0.088877 mm**, with rank six and scaled condition **3.7934**. Its mock clock
starts at 17 s, so total reported simulated time is 152.96 s, of which 135.96 s
is mock collection. It is not a new server run.

**VERIFIED ON FRESH PRACTICE SESSIONS, seeds 1 and 2, 2026-11-16.** Both
independent sessions completed the full workflow and were closed afterwards.
The estimator used no notebook globals, practice TRE scoring, or true outlier
labels. Every covariance used exact transformed points, had rank six, and
passed symmetry/PSD checks. The unchanged Step 7 cell and report JSON round
trip agreed with the headless result for both sessions.

| Measurement | Seed 1 | Seed 2 |
| --- | ---: | ---: |
| Selected patches | 33 | 33 |
| Placements / tracker reads | 132 / 132 | 132 / 132 |
| Paired observations / invisible attempts | 132 / 0 | 132 / 0 |
| Used / ICP-rejected samples | 114 / 18 | 115 / 17 |
| ICP iterations, converged in both | 41 | 40 |
| Inlier surface RMS (mm) | 0.166429 | 0.144576 |
| Predicted mesh RMS (mm) | 0.090273 | 0.074975 |
| Mean per-vertex predicted RMS (mm) | 0.088181 | 0.073442 |
| Maximum per-vertex predicted RMS (mm) | 0.132868 | 0.108928 |
| p90 of per-vertex predicted RMS (mm) | 0.118326 | 0.096607 |
| Centered/scaled Jacobian condition | 3.8465 | 3.5783 |
| Total simulated time (s) | 135.96 | 135.96 |
| Workflow wall time (s) | 40.901 | 39.124 |
| Entire validation script wall time (s) | 41.704 | 39.840 |

Live workflow timings include remote sampling and computation; the separate
script total also includes session setup, compatibility checks and cleanup.
They are observed local timings, not a hidden-grader runtime guarantee.
Outputs are `experiments/results/step7_practice_seed{1,2}_report.json`, matching
`.txt` reports, and `step7_practice_seed{1,2}_validation.json`. Run the same
validation command with `--seed 2` for the second case.

In the exact/proxy comparison, relative covariance Frobenius differences were
0.0368% for the replay, approximately roundoff for fresh seed 1, and 0.1984%
for fresh seed 2. These small differences are specific to these near-surface
data; the synthetic tangential-residual test shows the compatibility proxy
can differ substantially. The exact-data path remains the default.

**Checks left open at Step 7 (see Step 8 outcomes below).** The new seed-1
session did not reproduce the older Step 5 snapshot exactly: the paired tip
coordinates differed by 0.543 mm RMS. The snapshots are stored separately;
the cause and repeatability across server state/versions have not been
established. Do not merge those measurements as one repeatable run.
The final corrections relative to the fresh server initial guesses had
rotation-vector norms about 0.550 and 0.506 rad. These are larger than the
handout's stated 0.2 rad initial-perturbation scale, so the initial-guess
assumption and actual registration accuracy need checking in Step 8. A large
estimated correction alone does not reveal the true error.

**NOT RUN at Step 7; superseded by the fresh Step 8 validation below.** Actual
practice-server TRE, correctness of the 18/17 outlier
decisions, and predicted-versus-actual calibration remain Step 8 work.
Conditional local covariance does not include systematic bias, incorrect
local minima, or all correlations introduced by nearest-point matching and
MAD selection. The 400-trial linear-model check cannot establish calibrated
confidence for full ICP. Both small predicted values and small surface
residuals therefore remain insufficient to claim the ≤2 mm actual TRE target.

## Step 7 — Contribution and AI assistance

Vincent approved the exact private-point companion path, an explicitly marked
legacy approximation, local left-CT covariance, analytic vertex propagation,
and full report integration. OpenAI Codex implemented these changes, completed
only the student-designated covariance TODO in TA tests, added 28 supplementary
cases and a replay/live validation script, ran 93 tests and the two practice
sessions, verified the cited lecture formulas, and drafted these notes. The
prompt requested explanation/approval before implementation and simple report
prose. Ahmed's uploaded uncertainty functions remain stubs at commit
`ca10a27`; no uncertainty code was copied from him. His chosen sampling rule
remains unchanged. Public `IcpResult`, `SampleSet`, `RegistrationReport`,
Session APIs, the whole notebook and TA non-TODO tests were preserved.

## Step 8 — Practice validation method

**IMPLEMENTED and VERIFIED.** The separate practice script
`experiments/validate_step8.py` runs the existing headless workflow, then
compares its saved estimate with the server's scores. Production sampling,
Arun/SVD coarse alignment, point-to-point ICP with MAD rejection, and the
Step 7 covariance are unchanged. In particular, Ahmed's agreed patch rule
remains in use; these measurements do not motivate a more complex algorithm.

**Goal and data flow.** Open a fresh practice session → run
`workflow.run_registration()` → save the full report → execute the notebook's
unchanged Step 8 evaluation cell → save actual TRE and outlier scores → close
the session. Repeat sequentially for seeds **1, 2, 3, 1**. The script checks
practice mode before any truth query. It uses original placement indices;
missing tracker observations are counted separately from ICP rejection.
There are no truth queries in student estimator modules, and no score is used
to choose a transform, threshold, patch, or stopping rule.

`F_reg` still maps Reference → CT. All errors below are in mm. For the 1,568
CT target vertices, the server gives actual displacement statistics; actual
TRE RMS is `sqrt(mean(target displacement²))`. Surface residual RMS instead
uses distances from the **used measured tips** to their closest mesh points.
Predicted TRE RMS comes from the local covariance model described in Step 7.
These answer different questions and are reported separately. For rejection,
TP is a correctly rejected true outlier, FP a rejected normal sample, and FN
a retained true outlier; precision is `TP/(TP+FP)` and recall `TP/(TP+FN)`.
Undefined ratios are saved as null, not a fabricated perfect score.

**Sources verified locally.** The TA README sections 3–4 and notebook Step 8
define the practice-only scoring calls and request comparison on more than
one seed. Handout pp. **7–8** define mesh vertices as the TRE targets and
require patch/sample lists, rejection, residual statistics and both clocks;
p. 8 gives the **10-minute wall-clock limit**. No new mathematical solver or
lecture formula was introduced in Step 8. The Step 7 Jacobian/covariance
derivation and its verified slide references still apply.

## Step 8 — Measured accuracy, rejection and timing

**VERIFIED on the live practice server, 2026-11-16.** All four sessions used
33 patches, four placements per patch, 132 valid observations, zero missing
bodies, and **135.96 simulated seconds**. Defaults were `max_iter=100`,
`tol=1e-4`, `reject="mad"`. All fits converged. Each covariance had rank six,
positive eigenvalues and scaled condition number between 3.29 and 3.74.

| Run / seed | Actual TRE RMS | Actual max | Actual p90 | Predicted RMS | Surface RMS | Workflow wall s | ICP iterations |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 / 1 | 0.0700 | 0.1277 | 0.0976 | 0.0855 | 0.1588 | 35.68 | 44 |
| 2 / 2 | 0.1143 | 0.2181 | 0.1845 | 0.0682 | 0.1299 | 33.21 | 41 |
| 3 / 3 | 0.0715 | 0.1138 | 0.0969 | 0.0656 | 0.1312 | 32.17 | 38 |
| 4 / 1 repeat | 0.1503 | 0.2976 | 0.2580 | 0.0920 | 0.1722 | 33.02 | 38 |

All four actual RMS values **and all vertex maxima** passed the TA's 2 mm
comparison. They were also below 1 mm, while the handout ~1 mm versus TA
2 mm specification discrepancy remains unresolved. Wall times include
collection/network and fitting; session creation, practice scoring and cleanup
are excluded. Including those operations, each validation took 33.18–36.97 s,
also below 600 s. This is measured practice performance, not a hidden-grader
guarantee. All four sessions were closed successfully.

| Run / seed | Used / observed | True outliers | Rejected | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 / 1 | 114 / 132 | 18 | 18 | 18 | 0 | 0 | 1.000 | 1.000 |
| 2 / 2 | 118 / 132 | 11 | 14 | 11 | 3 | 0 | 0.786 | 1.000 |
| 3 / 3 | 117 / 132 | 13 | 15 | 13 | 2 | 0 | 0.867 | 1.000 |
| 4 / 1 repeat | 114 / 132 | 18 | 18 | 18 | 0 | 0 | 1.000 | 1.000 |

The MAD gate found every true outlier in these sessions, but also rejected
three and two normal samples for seeds 2 and 3. Do not call its detection
perfect or assume the grader's outlier rate is exactly 10%.

**Initial-guess diagnostic.** Only after freezing the final report, the script
also scored the server initial guess and the reconstructed patch-median
coarse transform. Their actual RMS values were:

| Run / seed | Server initial guess | Patch coarse alignment | Final ICP |
|---|---:|---:|---:|
| 1 / 1 | 20.551 | 1.268 | 0.070 |
| 2 / 2 | 14.741 | 2.169 | 0.114 |
| 3 / 3 | 19.070 | 1.620 | 0.071 |
| 4 / 1 repeat | 20.551 | 1.253 | 0.150 |

The initial guesses really were rough under the server's scoring convention.
Coarse alignment followed by ICP corrected them successfully in these runs;
the large Step 7 correction vectors did not signal a failed final alignment.
This comparison supports retaining the coarse stage. The handout's initial
perturbation-scale discrepancy cannot be resolved from TRE summaries alone.

## Step 8 — Repeatability, limits and reproducible evidence

**FAILED: exact observation repeatability for the same seed.** Sequential
seed-1 sessions had identical patch assignments and true outlier indices,
but their 132 paired measured reference tips differed by **0.548186 mm RMS**.
Their initial-guess actual RMS values matched; their final actual RMS values
were 0.0700 and 0.1503 mm. Both were accurate. The cause of the measurement
difference is unverified; do not claim that a seed reproduces every tracker
reading, infer hidden server internals, or combine the repeats as one run.
This confirms the earlier Step 7 observation of differing seed-1 snapshots.

**NOT ESTABLISHED: full ICP uncertainty calibration.** Actual/predicted RMS
ratios were 0.819, 1.677, 1.090 and 1.634. A predicted RMS is a model's
second-moment estimate, not an upper bound for a single realized error.
Four sessions cannot establish calibrated confidence or a systematic bias.
The local model still omits incorrect local minima and some correlations
from contact selection, tracker noise and MAD selection. Its per-vertex
predicted p90/max also must not be interpreted as actual TRE p90/max bounds.

**VERIFIED offline.** Before Step 8 edits, the TA tests passed **9/9** in
25.53 s. The selected regression suite (TA tests plus Steps 4–8 supplementary
tests) passed **95/95** in 43.58 s; the two earlier scene tests passed **2/2**
in 0.28 s. Together all **97 distinct current tests** passed, with no skips.
The four new Step 8 cases use explicitly synthetic labels to check original
indices, missed observations, false positives/negatives, undefined ratios,
and invalid index sets. They do not stand in for the live server results.
Unchanged notebook Step 8 execution and report JSON round trips passed in
all four live sessions. Source hashes confirm no Step 8 changes to production
student algorithms, TA infrastructure, original TA tests or the notebook.

Reproduce the practice batch with a fresh output tag:

```bash
PYTHONPATH=. venv/bin/python experiments/validate_step8.py --seeds 1 2 3 1 --tag fresh_check
```

See `experiments/results/step8/README.md` for the tables and links to the
four full JSON/text reports, per-run scores and exact truth/flag indices.
`step8_practice_summary.json` stores all mean/std/max/p90 statistics, both
clocks, covariance diagnostics, settings, timestamps, source/data hashes and
repeatability comparison. `offline_checks.json` records test commands/results
and preservation checks. Credentials are not copied into result files.

**NOT RUN.** Hidden grading, live seeds outside 1–3, live alternative outlier
rates and statistically powered end-to-end uncertainty trials remain untested.
Zero missing observations here is not a universal visibility guarantee.

**Contribution and AI assistance.** Vincent requested completion of Step 8.
OpenAI Codex wrote the independent validation script and four bookkeeping
tests, ran the fresh practice sessions and regression checks, and prepared
these measured results and explanations. Previously agreed methods and
Ahmed's sampling policy were preserved. No production algorithm, protected
TA file, original notebook cell, or public API was changed in Step 8.

## Final code review and submission packaging — 2026-11-16

**VERIFIED.** Reviewed all four student implementations and reran the complete
offline suite: **97 passed**, no failures/skips, **42.73 s**. Production code,
the original notebook sources, TA infrastructure and data needed no edits.
There are no ground-truth scoring calls in the graded estimator. Existing
multi-seed results and covariance/repeatability limitations remain applicable.

An independent package was assembled with the required official course
modules. A separate temporary Python environment omitted the checkout's course
path configuration; module-origin checks confirmed all runtime course imports
came from the package. All ten non-test notebook code cells ran successfully.
The optional pytest cell alone was skipped externally because Vincent
explicitly excluded unit tests from the upload ZIP. Its source is unchanged,
and the ZIP README tells the TA which cell needs the full development checkout.

Fresh seed-1 package sessions gave actual stepwise TRE RMS **0.070219 mm**
(max **0.106577 mm**, 18 correctly rejected outliers and no false alarms or
misses) and fresh headless RMS **0.071 mm** (the original cell's rounded
practice score). The headless fit converged, predicted RMS was **0.079516 mm**,
wall time **34.485 s**, simulated time **135.96 s**. Both sessions closed.
Evidence is in `submission/verification/`; `submission/FINAL_REVIEW.md`
describes the review and `build_submission.py` reproduces the allowlisted ZIP.

Only the code package is prepared; the narrative report is to be submitted
separately. The ZIP contains the notebook, all Assignment 3 original sources,
data, requirements, necessary course source dependencies and a short README.
It excludes experiments, unit tests, environments, caches, sessions and these
development notes. Shared course modules use the verified snapshot/provenance
already described in SETUP; no TA source or public API is altered.

**Requested scope.** Vincent confirmed `main` + existing `dev/vincent`, code
ZIP only with the report separate, and preserving the original course token
and notebook code. Packaging helpers and verification evidence are separate
from the upload files. The final report and its disclosure are handled by
the student separately.
