# Assignment 2 pre-submission audit — 2026-10-31

**Status: NOT READY for final submission. No algorithms, report chapters, original tests, simulation, commits or remote branches were changed.**

Audited branch: `dev/vincent`, HEAD `603d02af32e9a69a416c9c0b58938240a6ceb443`, plus the current uncommitted final notebook-summary cell and comment-only registration/test edits. Remote refs were refreshed with `git fetch origin`; no checkout, merge, commit or push occurred.

## 1. Mathematical implementation and library use

| Classification | Functions/calls | Finding |
| --- | --- | --- |
| KEEP — own core methods | `metrics.frame_error`, `summarize_errors`; `registration.register_points`, `average_frames` | Explicit relative-frame angle/statistics, centered Arun/SVD with proper-rotation handling and degeneracy checks, and proper chordal rotation mean. No black-box registration/averaging substitutes in production. |
| KEEP — own core methods | `workflow.measure_home_offsets`, `plan_calibration_poses`, `collect_calibration_data`, `estimate_base_frame` | Correct additive home-offset sign; public APIs; copied synchronized encoder/frame pairs; missing observations excluded but time charged; correct observed-times-inverse-nominal order. Sampling/noise approximations are documented as estimates. |
| KEEP — own core methods | `ImprovedKinematics.initial_params`, `base_frame`, `fwd_kins`, `residuals`, `fit`, `jacobian_rank_report`, `parameters_report`; `_fit_constraints.distortion_bounds`, `solve_constrained` | Explicit joint maps, transformation order, signed/weighted residual assembly, gauge fixing, numerical derivatives and full-interval extrema/constraint construction. Scalar joint bounds do not prove unique physical recovery of gauge-dependent base/axis/tool coordinates. |
| KEEP — numerical primitives | NumPy matrix products/solve/norm/SVD/eigenvalues/rank; Polynomial evaluation/derivatives/roots; SciPy least_squares/SLSQP/nnls | Appropriate primitives. The TA fitter TODO explicitly permits scipy.optimize. SVD is the classroom Arun and rank-analysis primitive. NNLS is used for a convergence diagnostic, not hidden calibration truth. |
| KEEP — rotation-vector conversion | `Rotation.from_matrix(...).as_rotvec()` in Step 8 residuals; Rotation conversions in the gauge audit | Supplies the signed SO(3) logarithm, not a complete calibration solver. Keep it: no explicit self-implementation restriction was found. Replacing it with the course trace/sine logarithm introduces a near-pi stability concern; Step 4's scalar magnitude cannot replace a signed three-component residual. Explain the mathematics in the report; no equally robust replacement is justified here. |
| KEEP — TA utilities | Frame/Rot/vct3, public robot/tracker APIs, course exp_so3/exp_se3/log_se3; original inv_kins/log_so3 | Starter documentation explicitly points to course SE(3) utilities. The provided inverse remains unchanged; its numerical limitations must remain disclosed. |
| KEEP — independent test oracles | SciPy expm, rotation alignment/conversion and independently constructed analytic fixtures in tests/experiment helpers | Valid independent validation uses. They are not calibration inputs or production replacements. Retained experiment drivers are dependencies of regression tests, not normal notebook execution. |
| CONSIDER | Mathematical explanations and optional refinement-test treatment | Apply approved report changes through the owner/teammate workflow. Decide how to handle the disclosed optional failure without weakening original assertions. No mandatory library replacement was identified. |

## 2. Completion, protection and tests

- All required core student methods are implemented. `refine_marker_body()` remains the sole explicit `NotImplementedError`; the pinned official TA README calls it optional for the exact special calibration body.
- The final Achieved Performance cell is implemented and executed successfully in the fresh run. It uses current thresholds/SEED, complete IK diagnostic denominators, conditional executed-error statistics and measured-versus-derived timing labels. It performs no new calibration, movement, sampling or truth query.
- Step 8 still has the diagnostic `compare the rest (axes / tilts, F_we offset)` TODO comment. Existing parameters_report and saved post-freeze inter-axis comparisons cover much of its intent, but the notebook itself does not finish that comparison. Flag it for owner review rather than claiming every TODO comment is gone.
- All 13 original complete test functions/helpers match the official starter AST; three changed tests were explicitly student TODOs. The current uncommitted test edit only removes comments. The original `inv_kins()` AST is identical to the pinned starter.
- Every official notebook Markdown cell is unchanged; all original non-TODO code cells remain intact. There are 27 rather than 26 cells because of the previously authorized standalone visualization cell. TA robot/simulation/requirements/import files match retrieved starter text; all 26 vendor support checksums pass.
- Current working tree full suite: **253 passed, 3 xfailed, 1 failed**, 32.72 s. Breakdown: simulation 23/23 pass; official student test file 13 pass/1 fail; additional regressions 217 pass/3 xfail. No unexpected error or hidden ordinary skip.
- Final pruned candidate extraction: **253 passed, 3 xfailed, 1 failed**, 29.92 s. The sole failure is the student-written optional-refinement test; three strict xfails document unchanged IK limitations. See `current_tests.xml`, `extracted_tests.xml` and logs. A completed notebook does not make its pytest subprocess green.

## 3. Calibration integrity and performance

- Production calibration methods contain no ground-truth/private-truth queries. Notebook truth calls occur after fitting or in permitted evaluation. Regression oracles may construct/score known truth, independently of calibration inputs.
- Saved seeds 1/7/61 each have 765 paired observations and 69 distinct encoder configurations; values are finite and rotations proper to roundoff. The seed-1 tracker holdout shares zero configurations with training. Saved joint-map magnitudes and derivatives meet bounds to floating-point tolerance; minimum normalized margins are approximately -8.44e-15, -3.11e-15 and +7.77e-16. See `data_integrity.json`.
- Saved report inputs and source/PDF hashes match the report evidence summary. Historical three-seed figures/results were neither reacquired, refitted nor overwritten. They still show full FK success and serious stress-IK limitations; ordinary-target success is not full-workspace inverse robustness.
- One fresh normal Seed 1 notebook run (including original tests, visualization, calibration, rank check, fixed FK and physical IK evaluation) completed in **103.983 s wall time**; fitting took **67.610 s**. This is below 600 s on this machine, not a guarantee for unknown grading hardware.
- Fresh simulated calibration: **554.030569 s**; Step 10 testing: **1058.715910 s**. These simulated durations must not be compared directly with the ten-minute program wall-time limit. Measured component values and derived overhead are in `normal_workflow.json`.
- Fresh improved FK: **729/729**, RMS/max position **0.06218/0.08707 mm**, RMS/max angle **1.048e-4/1.829e-4 rad**. Fresh improved physical IK: **40/40**, **0.06976/0.10632 mm**, **2.220e-4/4.859e-4 rad**, no solver/movement exceptions. These are a new single-run audit, not replacements for historical report tables. The original sensor sampling is stochastic, so bitwise reproduction is not asserted.

## 4. Report blockers and ownership

1. The local PDF is still the old 21-page snapshot. v2 replacement snippets have not been applied/approved locally; commit naming is not report approval or Overleaf synchronization.
2. Name/JHED, signatures, team contributions and Claude Code disclosure remain blank. The Codex disclosure exists. Authors must supply true statements and the approved final PDF; do not infer their contributions/tool use.
3. The old report still describes quaternion scalar metrics and the older test count. CHG-001/002 are pending. Chapters 1–4 are teammate-owned: flag these discrepancies, do not edit them here. Chapter 5 may add the new normal-workflow runtime with explicit provenance after approval.
4. Decide whether the optional failing test and the remaining Step 8 debug comparison are acceptable/documented or warrant separately authorized completion. Do not suppress the failure, rewrite assertions, change the fixed grid or replace the TA inverse to produce a green submission.

## 5. Candidate manifest and extraction validation

`CIS_Assignment2_CANDIDATE.zip` contains **96 files**; `submission_manifest.json` lists every path, source and SHA-256. Package root contains `README.md` and `assignment2/`.

Included: current notebook source with fresh executed outputs; all src/ implementations and unchanged TA simulator; vendor/course support plus the two vendor/functions helpers and provenance/checksums; official tests; all student regression Python files and their imported experiment-helper dependencies; requirements/lock/configure_environment/pytest.ini; candidate-only setup documentation; current PDF and the two separately referenced supplementary PDF assets; **23** essential regression/evidence data files.

Excluded: environments, caches, Git metadata, other assignments, local archives, duplicate ZIPs/LaTeX drafts/intermediate reports, duplicated plots, pilot projections/plots, unused data, and the three large full-run per-attempt JSON exports. Two pilot JSON files remain because existing regression assertions read them; they are test fixtures, not additional experiments. No development file was deleted. Tests were rerun after pruning unused temporary fixtures.

The ZIP was extracted into an empty temporary directory. A new Python **3.13.11** venv installed the submitted requirements/pins; pip check passed. NumPy **2.5.4**, SciPy **1.18.1**. configure_environment created only extraction-local support paths/kernel. Imports resolved to packaged src, vendor/course/data_types, vendor/course/src/uncertainty_networks and vendor/course/assignments/assignment1/src, without the original repository or system site packages. All ZIP CRCs and manifest hashes pass. Python 3.9, Windows/macOS and the precise grading configuration were not reproduced.

The development notebook was not executed/overwritten. The package's notebook comes from `executed_assignment2_template.ipynb`; its cell types/source match the working notebook exactly. The extra audit query cell was removed before saving. Candidate-only README/SETUP templates are separate artifacts, not changes to development setup files.

## 6. Safe integration and final ZIP procedure (recommendations only)

- Refreshed origin/main is `4cce0669`; dev/vincent is 18 commits ahead/4 main-only commits relative to their split. Those four main commits have no net tree changes relative to the split. Local main is stale; do not use it as the integration baseline without updating.
- Ahmed's remote branch has 14 commits not on main (main also has four not on Ahmed). They include notes/rules and separate draft notebooks/course ZIPs, not edits to the current student model. Review them with Ahmed; do not blindly merge or overwrite them. Revisions is already an ancestor of main.
- First resolve/review the report and optional-test/debug-TODO decisions, obtain teammate review, and review the three existing uncommitted files. Stage only approved changes; do not blindly add audit ZIPs or experiment caches.
- After authorization, commit on dev/vincent, fetch and push normally, open a PR dev/vincent -> main, obtain teammate approval, then merge through GitHub. A normal merge preserves the main history; no force-push, reset or overwriting teammate work.
- After merge, audit the actual main checkout again, refresh the executed notebook/final PDF and candidate docs as needed, then rebuild and extract/test the final package. The builder intentionally refuses changed source hashes without `--accept-reviewed-changes` and refuses final packaging while its README still declares unresolved items.

```bash
git diff origin/main...origin/dev/ahmed -- assignment2 AGENTS.md
git diff -- assignment2/notebooks/assignment2_template.ipynb assignment2/src/student/registration.py assignment2/tests/test_student.py
# Only after approval: stage reviewed files, commit, then:
git fetch origin
git push origin dev/vincent
gh pr create --base main --head dev/vincent --title "Complete CIS I Assignment 2"
# Review and merge the PR in GitHub; do not force-push.
# After final report/source/docs are reviewed and fresh-extraction checks repeated:
python3 assignment2/results/submission_audit_2026-10-31/rebuild_candidate.py --accept-reviewed-changes --final-approved
```

The final command is a future approved action, not something run during this audit. It creates CIS_Assignment2_Final.zip only after the README gate is cleared. No ZIP label, script flag or passing numerical test substitutes for author approval of the unresolved items.
