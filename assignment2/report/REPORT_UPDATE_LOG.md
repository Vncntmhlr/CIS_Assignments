# Assignment 2 — Report Version and Change Tracking

Initialized: 2026-10-30 (Asia/Shanghai). Record report-relevant development changes here after substantive milestones. Updating this log does not authorize report edits, version approval, commits, pushes, or Overleaf synchronization.

## Current report baseline

| Field | Verified state |
| --- | --- |
| `current_report_version` | **v1** — the user's 2026-10-31 v2-preparation request explicitly identifies the current report as approved v1. |
| `report_baseline_commit` | `03ec9e61b3c1f3854a66a5accf224b18e7b5ec07` — preserves the current local v1 report source. |
| `report_baseline_hash` | SHA-256 of `main.tex`: `daaec4105fa1153975c03a67cfe167a91964065ecf09d717705eb62af5f214f9` |
| `report_status` | User identifies v1 as approved; local source/PDF preserved. Current Overleaf synchronization **unconfirmed**. |
| `next_report_version` | **v2 proposed** — incremental replacement instructions only; not applied or approved. |
| `pending_changes` | **CHG-001**: explicit Step 4 angle extraction and validation evidence; **CHG-002**: accessible explanation of Section 3.7's identifiability/gauge analysis and numerical results. Neither is incorporated yet. |

### Snapshot evidence

- Source: [main.tex](CIS_Assignment2_Overleaf/main.tex); compiled preview: [CIS_Assignment2_Report.pdf](CIS_Assignment2_Overleaf/CIS_Assignment2_Report.pdf); package: [CIS_Assignment2_Overleaf.zip](CIS_Assignment2_Overleaf.zip).
- PDF SHA-256: `cb11723019825c9cdcf381ff78b64bf0bfe4ca52373fa2176e8f9ba20980771b`.
- The working `main.tex` matches its committed content at the baseline above. Its source hash and the PDF hash match [evidence_summary.json](CIS_Assignment2_Overleaf/evidence_summary.json). ZIP copies of `main.tex` and the PDF match the corresponding local files. These checks establish artifact consistency, not a fresh compilation or content approval.
- Git branch at initialization: `dev/vincent`; HEAD equals the baseline commit. Before initialization, the only working-tree change was the user's updated root `AGENTS.md`. No remote fetch or Overleaf check was performed.
- The baseline commit contains both the report draft and the Step 4 code update. Sharing a commit does **not** establish that the report describes the updated implementation: its Section 3.1 still explicitly describes quaternion-based extraction.
- Cover name/JHED fields, team contributions and Claude Code disclosure remain intentionally blank, as requested by the user. Their completion is an author responsibility, not an inferred contribution or tool-use statement.

## Version history

| Snapshot/version | Parent | Approval and preservation |
| --- | --- | --- |
| Current unnumbered local draft | No approved parent established | Preserved in Git at `03ec9e61b3c1f3854a66a5accf224b18e7b5ec07`; source hash above. Approval and Overleaf synchronization unconfirmed. |
| v1 (identified by user on 2026-10-31) | No earlier approved version established | The same source snapshot is now identified as approved v1 by the user's v2-preparation request. This resolves the original unconfirmed designation; Overleaf synchronization remains unconfirmed. |

Do not infer numbered versions from commit messages such as `report1-2_2`. Once the user explicitly approves a version, record that approval, its parent, commit/hash and separate Overleaf status without deleting this initial snapshot record.

### Proposed v2 audit — 2026-10-31

- Reviewed all report chapters/appendices against the current source and changes since the baseline, including staged/unstaged/untracked state. `main.tex` still has the baseline hash. Notebook cell count/types/source are unchanged (27 cells); its Git changes do not introduce a new implementation or evaluation protocol. Model, experiment scripts, saved results and report notes have no additional changes relative to the baseline beyond the previously identified report/code mismatch.
- The five input-result hashes recorded in the report evidence summary still match the saved files, including the original Step 9/10 and full supplementary results. No acquisition, fitting or physical evaluation was rerun. Tables/figures need no numerical replacement; Chapter 5 needs a short provenance note identifying the archived evaluations as preceding the Step 4 implementation change.
- Fresh complete regression: **253 passed, 3 xfailed, 1 failed** in **32.12 s** (`venv/bin/python -m pytest tests experiments -q`). The sole failure remains `test_refine_marker_body_reduces_layout_error`, due to the existing unimplemented optional function. The prior 253-pass run excluded this test; do not conflate those run scopes. New output was inspected in a temporary log; no persistent result archive was created.
- Recomputed the existing deterministic Phase C fixtures: 69 distinct configurations; rank **33/34**, then **33/33**; reduced extrema **305.1249510932708 / 0.3611204259138441**, condition **844.9396079469273**; full null singular value **3.960197791045084e-8**, threshold **3.0512519912218782e-5**, physical-gauge/nullspace alignment **1.0**. Existing rounded report values are reproduced without calibration measurements or fitting.
- CHG-001 and CHG-002 were recorded before version approval was established; retain their original `based_on_report_version` history below. They are now reconciled against this same approved **v1** source and remain **PENDING**. Proposed replacements cover Sections 3.1, 3.7, 4.2 and a Chapter 5 provenance insertion. No report source/PDF/ZIP has been modified or compiled for v2; no change is marked incorporated.
- The v2 transition may include Chapters 1–4. After explicit v2 approval, obey the ownership boundary in `AGENTS.md`: Chapters 5–7 and explicitly assigned related appendices only; flag issues in Chapters 1–4 for the teammate.

## Pending changes

### CHG-001 — Explicit Rodrigues-derived Step 4 rotation angle

- `change_date`: 2026-10-30; backfilled when initializing this log.
- `based_on_report_version`: **UNCONFIRMED**, referring to the exact local report snapshot identified above; no approved version number is invented retrospectively.
- `status`: **PENDING**.
- `incorporated_in_version`: —
- **Previous → new behavior and reason:** `frame_error()` previously used SciPy `Rotation.from_matrix(...).magnitude()`. It now exposes the classroom mathematics using basic NumPy operations, at the user's request. For `deltaF = F_a.inv() * F_b`, let `R = deltaF.R`, `v = [R32-R23, R13-R31, R21-R12]/2` and `c = clip((tr(R)-1)/2, -1, 1)`. Rodrigues' formula gives `v = sin(theta)n`; compute the principal angle as `atan2(||v||, c)` in `[0, pi]`. Translation remains `||deltaF.p||` in mm. The error definition and public interface are unchanged. Unlike the former matrix conversion, this formula assumes a valid rotation and does not project invalid matrices onto SO(3).
- **Exact affected sources:** [src/student/metrics.py](../src/student/metrics.py), `frame_error()` TODO implementation; [experiments/test_metrics.py](../experiments/test_metrics.py), added `test_rotation_angle_across_random_axes_and_endpoint_scales()`; [REPORT_NOTES.md](../REPORT_NOTES.md), Step 4 derivation, metric implementation summary, validation and input-validity limitation. Recorded in commit `03ec9e61b3c1f3854a66a5accf224b18e7b5ec07`.
- **Verified evidence from the preceding implementation task:** 31 focused tests passed (28 student-owned metric cases and 3 unchanged TA metric tests). The retained sweep uses 1064 known rotations with random axes, seed 42, and angles near zero, in the interior, and near pi; maximum absolute angle error was `4.44e-16 rad`. The original notebook sanity expression returned `(0.01 rad, 1.0 mm)`; a `1e-9 rad` input retained `1e-9 rad`. Broader regression: **253 passed, 3 xfailed, 1 deselected** in **29.57 s**; the pre-existing unimplemented optional `refine_marker_body` test was excluded. These are previously measured results, not tests rerun during log initialization.
- **Reproduction and result references:** [metric tests](../experiments/test_metrics.py), the first three metric tests in [tests/test_student.py](../tests/test_student.py), and [Step 4 validation notes](../REPORT_NOTES.md#step-4-metric-validation). From `assignment2/`, run `venv/bin/python -m pytest experiments/test_metrics.py tests/test_student.py::test_frame_error_known_values tests/test_student.py::test_frame_error_is_invariant_to_common_left_factor tests/test_student.py::test_summarize_errors -q`; broader command: `venv/bin/python -m pytest tests experiments -q -k 'not refine_marker_body'`. No persistent machine-readable result file was created for this update; the regression summary above was checked against the retained temporary run log during initialization.
- **Affected report locations:** Section **3.1, Rigid Frames and Pose-Error Metrics**, especially the quaternion-extraction sentence following `eq:pose_error`; preserve the relative-frame equation and explain the explicit sine/cosine derivation. Section **4.2, Focused Unit Tests and Regression Evidence**, needs reconciliation of the new test evidence with the historical test count. The report's [evidence summary](CIS_Assignment2_Overleaf/evidence_summary.json) still records the earlier test run and should retain its historical meaning when new evidence is added. No new figure or calibration-performance table is justified by this code-only change.
- **Limits on claims:** No new calibration acquisition, model fitting, or complete three-seed physical evaluation was performed. Saved experimental results remain historical measurements; do not relabel them as rerun with the revised metric or claim bitwise equality without checking. Report source, PDF, ZIP, figures and saved evaluation outputs were not modified for this change or for log initialization.

### CHG-002 — Explain the purpose and interpretation of the rank/gauge audit

- `change_date`: 2026-10-31.
- `based_on_report_version`: **UNCONFIRMED**, using the current local draft snapshot identified above.
- `status`: **PENDING**.
- `incorporated_in_version`: —
- **Previous → proposed explanation:** Section 3.7 currently presents SVD/rank terminology and numerical values with insufficient introductory explanation. The user requests a more accessible explanation in the next report revision, without assuming prior knowledge of singular values or gauge freedom. Introduce the practical question (which calibration parameters the observations can distinguish), explain the Jacobian as a sensitivity table, and connect every reported number to that question before interpreting the result. Keep the explanation proportionate to the report rather than adding a general linear-algebra tutorial.
- **Required clarifications:** define identifiability, gauge freedom, base roll, singular-value extrema, numerical-null singular value, rank threshold, condition number, weak parameter combinations and local versus global uniqueness. Explain why the full rank is 33/34, why fixing base-twist x yields 33/33, and why arbitrary parameter fixing is not equivalent: for a one-dimensional nullspace with direction `v`, fixing coordinate `j` removes that local freedom only if `v[j] != 0`, with conditioning still requiring examination. Distinguish an unobservable expression convention from true base-roll recovery, and numerical rank from accurate recovery under noise. Explain parameter scaling/residual weighting, the two finite-difference steps, and why repeated identical configurations improve noise statistics but do not supply missing excitation. Qualify the square-root repetition scaling as the equivalent-Jacobian-block case.
- **Exact supporting sources/functions (unchanged):** [src/student/kinematics.py](../src/student/kinematics.py), `jacobian_rank_report()` and the fixed-coordinate convention in `fit()`; [Phase C regression tests](../experiments/test_kinematics.py), especially `test_expected_physical_gauge_explains_the_only_null_direction`, `test_actual_69_pose_design_has_no_additional_null_but_has_weak_modes`, and `test_nominal_analytic_gauge_and_explicit_parameters_do_not_mutate_the_model`; [Phase C mathematical notes](../REPORT_NOTES.md#step-8-phase-c-actual-model-rank-gauge-and-sensitivity).
- **Evidence and numerical provenance:** explain the existing 69-configuration nonzero-model audit: rank 33/34, reduced rank 33/33, reduced extrema `305.125` / `0.361120`, reduced condition number `844.94`, full numerical-null singular value `3.96e-8`, and rank threshold `3.05e-5`. These are historical Phase C results from the scaled, weighted Jacobian, not physical positioning errors. This entry records an editorial request and the preceding explanation; no new numerical experiment, test run, or result file was produced. Recheck the values against the current sources when updating the report.
- **Affected report locations:** Section **3.7, Identifiability and Gauge Freedom** (`sec:math`); clarify its finite-difference/SVD paragraph and physical interpretation, preserving valid results. Cross-reference the corrected model parameterization and Section 4.2's validation evidence where helpful. No new figure/table or changed numerical claim is currently requested.
- **Scope:** only this tracking log is updated now. Report source/PDF/ZIP, implementations, tests, simulation and saved results remain unchanged; no report version approval, commit, push or Overleaf synchronization is implied.

## Next report update

Only on explicit request: audit changes from the snapshot commit above (including uncommitted/untracked work), reconcile pending entries against the actual report text, update affected sections, compile and verify artifacts, and present a proposed version for review. Do not mark it approved or synchronized until each is explicitly confirmed. Subsequent report-relevant development changes receive the next unused change ID and remain based on approved v1 until a later report version is explicitly approved; apply the ownership restrictions after v2 approval.
