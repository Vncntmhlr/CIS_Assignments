# Assignment 2 — Report Version and Change Tracking

Initialized: 2026-10-30 (Asia/Shanghai). Record report-relevant development changes here after substantive milestones. Updating this log does not authorize report edits, version approval, commits, pushes, or Overleaf synchronization.

## Current report baseline

| Field | Verified state |
| --- | --- |
| `current_report_version` | **UNCONFIRMED** — no explicit approval of a numbered report version is established in the available conversation or repository. |
| `report_baseline_commit` | `03ec9e61b3c1f3854a66a5accf224b18e7b5ec07` — preserves the current local report snapshot; this is not evidence of version approval. |
| `report_baseline_hash` | SHA-256 of `main.tex`: `daaec4105fa1153975c03a67cfe167a91964065ecf09d717705eb62af5f214f9` |
| `report_status` | Local draft and compiled PDF available; numbered-version approval **unconfirmed**; Overleaf synchronization **unconfirmed**. |
| `next_report_version` | **v1 proposed**, subject to explicit user confirmation of the first approved version; no version has been assigned or advanced by this initialization. |
| `pending_changes` | **CHG-001**: explicit Step 4 angle extraction and updated validation evidence have not been incorporated into the report. |

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

Do not infer numbered versions from commit messages such as `report1-2_2`. Once the user explicitly approves a version, record that approval, its parent, commit/hash and separate Overleaf status without deleting this initial snapshot record.

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

## Next report update

Only on explicit request: audit changes from the snapshot commit above (including uncommitted/untracked work), reconcile pending entries against the actual report text, update affected sections, compile and verify artifacts, and present a proposed version for review. Do not mark it approved or synchronized until each is explicitly confirmed. Subsequent report-relevant development changes receive the next unused change ID and remain based on the current approved version, or this explicitly unconfirmed snapshot until approval is established.
