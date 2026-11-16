# CIS I Assignment 3 — Code submission

This archive contains the completed Assignment 3 notebook, all original
Assignment 3 `src/` files (including student implementations), mesh/patch
data, requirements, and the course source modules required by the TA imports.
The narrative report is submitted separately. Experiments, unit tests, virtual
environments, caches, local session records and development notes are excluded.

## Run

Use Python 3.9 or newer. From this archive's `CIS_Assignment3/` directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r assignments/assignment3/requirements.txt
cd assignments/assignment3/notebooks
python -m notebook
```

On Windows, activate with `.venv\Scripts\activate`.

Select this environment's Python kernel and run
`assignment3_template.ipynb` in order, except the optional pytest cell discussed
below. The notebook's original server/token configuration is preserved at the
student's request. Update `SERVER_URL` / `COURSE_TOKEN` from Piazza if needed.
Server access
is needed for practice observations and validation; the hidden-grader entry
points use the provided in-process `Session` and require no credentials.
The original pytest cell (under the unit-test section) needs `../tests/` in
the full development checkout. Skip that cell in this code-only ZIP: tests
are intentionally excluded at the student's request. Its source has not been
rewritten. All registration, uncertainty and practice-validation cells remain
executable with the course server. Final offline checks passed 97/97 tests
in the full checkout; those checks are not represented as having run inside
this test-free ZIP.

For headless grading, run from `assignments/assignment3/` and call the TA APIs:

```python
from src.student import workflow
design = workflow.design_scene()
# The grading harness provides session, mesh, and boxes.
result = workflow.run_registration(session, mesh, boxes)
```

Keep the archive's relative directory structure. The shared `data_types/`,
`src/uncertainty_networks/`, and `assignments/assignment1/src/` modules are
required course dependencies, not another student's Assignment 1 solution.
No machine-specific Python-path setup or files from the development checkout
are required. The handout's ZIP paragraph says `assignment1/`; this archive
uses the official starter's `assignments/assignment3/` course layout.

## Implementations and provenance

- `closest_point.py`: exact triangle projection/boundary cases and batched
  vectorized exhaustive mesh search.
- `icp.py`: Arun/SVD rigid registration, patch-median coarse alignment, and
  point-to-point ICP with MAD outlier rejection.
- `uncertainty.py`: left-CT local Jacobian covariance and first-order
  per-vertex RMS TRE prediction.
- `workflow.py`: device design, patch plan, tracker sampling and headless
  report assembly. The retained patch-selection rule was adapted from
  Ahmed783-pixel's `workflow.py`, commit
  `ca10a27f8d71cfd64076b0b7610546c614517d98`.

`F_reg` maps Reference → CT. The true-TRE/outlier APIs are used only by the
notebook's practice validation, never by the estimator. Predicted RMS is a
local model estimate, not a maximum-error guarantee.

TA support sources, data and public APIs are preserved. Course dependencies
are copied from the locally verified official snapshot of
`LCSR-CIIS/CIS_I_Course`, commit `d91ea42775ac16b6639cb7a4b0d7b8d157f795a0`.
OpenAI Codex assisted implementation, testing and documentation; the separate
written report must contain the detailed prompts and individual contributions.
