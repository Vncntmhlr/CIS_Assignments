# Assignment 3 code submission review — 2026-11-16

**VERIFIED: code submission ready on the checks below.** No algorithm change
was needed. A final narrative report is a separate deliverable and is not
included in the code ZIP, as Vincent requested. Hidden-grader success is not
claimed.

## Code and numerical checks

- All four student modules were reviewed for frame order, dimensions, proper
  rotations, exact/vectorized mesh search, SVD reflection correction, patch
  initialization, MAD rejection, convergence and covariance/TRE propagation.
- `F_reg` consistently maps Reference → CT. The estimator uses only the
  documented `Session` interface and never calls ground-truth scoring APIs.
- TA source/API/data protections and source hashes were checked. The final
  numerical implementations match the versions already measured in Step 8.
- Full offline suite: **97 passed, zero failures/skips, 42.73 s**, using
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests experiments -q --tb=short`.
  Coverage includes triangle cases, mesh exactness, known transforms, coarse
  alignment, ICP, outliers at 0/10/25%, missing observations, covariance
  rank/PSD, report serialization and original sample-index bookkeeping.
- Prior independent practice validation covers seeds 1, 2, 3 plus a repeat
  of seed 1: actual RMS 0.070–0.150 mm, max below 0.30 mm. The same-seed
  observation difference and the limited covariance calibration evidence
  remain documented in `REPORT_NOTES.md`.

## Standalone package execution

Required TA/course modules were copied into the official relative layout.
A fresh temporary environment reused only installed Python dependencies;
it did not load this checkout's course-path `.pth` configuration. Module
origins were checked: student code, `data_types`, tracker and
`uncertainty_networks` all loaded from the standalone package.

The notebook source was preserved, including the original course token, in
accordance with Vincent's explicit instruction. All **10** non-test code
cells ran successfully. Only the optional pytest cell was skipped externally:
the requested ZIP deliberately excludes `tests/`. The cell itself is still
unchanged and the ZIP README explains this limitation.

Two new practice sessions were executed from the package:

- Notebook's stepwise path: actual TRE RMS **0.070219 mm**, maximum
  **0.106577 mm**; 18 true outliers correctly flagged, no false alarms/misses.
- Fresh headless `run_registration`: actual TRE RMS **0.071 mm**, as rounded
  by the original notebook; predicted RMS **0.079516 mm**; converged;
  **34.485 s wall**, **135.96 s simulated**.

Both sessions were closed by the original notebook. Exact exported results
and the full headless report are in `verification/package_execution.json`
and `verification/headless_report.json`. The executed notebook is retained
locally for packaging, with its source checked against the original.

## ZIP and Git scope

`build_submission.py` uses an explicit file allowlist. The code ZIP includes
the notebook, all original Assignment 3 Python sources, two data files,
requirements, a concise run/provenance README, and required course modules.
It excludes experiments, unit tests, environments, caches, session logs,
handouts, development notes, build/verification tools, and the final report.
The local manifest records every ZIP member's size/hash and the source commit.

The main branch is updated from its existing head, changing only Assignment 3
paths. Other assignments and main's pre-existing extra files/history are
preserved. The existing `dev/vincent` branch carries the reviewed development
version. Neither branch needs a force push. The generated ZIP/manifest and
the local executed-notebook copy are not committed to GitHub.

## Rebuild

After the reviewed commit is saved, run from `assignment3/`:

```bash
venv/bin/python submission/build_submission.py --executed-notebook submission/verification/executed_assignment3_template.ipynb
```

The final upload file is `assignment3/CIS_Assignment3_SUBMISSION.zip`. The
external manifest is `submission/manifest.json`. Without the optional
executed-notebook argument, the same runtime files are packaged with cleared
notebook outputs; all code cells remain unchanged.
