# Assignment 3 setup

## Starter and course dependencies

- Official upstream [LCSR-CIIS/CIS_I_Course](https://github.com/LCSR-CIIS/CIS_I_Course), commit [`d91ea42775ac16b6639cb7a4b0d7b8d157f795a0`](https://github.com/LCSR-CIIS/CIS_I_Course/commit/d91ea42775ac16b6639cb7a4b0d7b8d157f795a0).
- Copied all 17 files from [`assignments/assignment3/`](https://github.com/LCSR-CIIS/CIS_I_Course/tree/d91ea42775ac16b6639cb7a4b0d7b8d157f795a0/assignments/assignment3) through a separate GitHub snapshot. Each file matched its upstream Git blob before Step 1 edits. The original local placeholder is preserved as [README.placeholder.md](README.placeholder.md).
- The starter imports `data_types`, `assignments.assignment1.src.tracker`, and transitively `uncertainty_networks`. The existing [course support snapshot](../assignment2/vendor/course/) matches the pinned commit's Python sources byte for byte for all three packages. The Assignment 1 code in this repository has an older tracker API, so the Assignment 3 client uses the compatible course snapshot.

## Use the Assignment 3 environment

Run commands from `assignment3/`. The Python 3.13.11 environment at `venv/` satisfies the TA's [requirements.txt](requirements.txt). Its `site-packages/cis_assignment3_course.pth` adds the course support snapshot to Python's import path when the interpreter starts, including when VS Code launches a notebook kernel. `python -m pip check` passed in this environment.

```bash
source venv/bin/activate
unset PYTHONPATH PYTHONHOME
python -m pytest tests -q
python -m pytest experiments/test_step1_scene.py -q -s
python -m notebook
```

The TA test suite starts with six `NotImplementedError` failures in the later-step closest-point and registration stubs, one passing report test, and two skipped student TODO tests.

In Jupyter or VS Code, select `assignment3/venv/bin/python` and open [assignment3_template.ipynb](notebooks/assignment3_template.ipynb) with `notebooks/` as its working directory. If the kernel was open when the `.pth` file was added, restart the kernel and rerun the imports/data cell followed by the Step 1 code cell. The imports/data cell loads the mesh (1,568 vertices, 3,135 triangles) and 46 patch boxes. Step 2 needs the course server details and token from Piazza.

The imports/data and Step 1 code cells passed in an in-process Jupyter kernel using `assignment3/venv/bin/python` with `PYTHONPATH` unset.

## Test collection on this workstation

If the local ROS `launch_testing` pytest plugin prevents test collection, disable
automatic third-party plugin loading for these offline tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests -q
```

This avoids the incompatible external plugin without changing the TA tests.

## Fresh environment

Use Python 3.9 or newer as specified by the TA. Run these commands from `assignment3/` when recreating the environment or after moving the checkout.

```bash
python3 -m venv venv
source venv/bin/activate
unset PYTHONPATH PYTHONHOME
python -m pip install -r requirements.txt
python - <<'PY'
from pathlib import Path
import sysconfig

course = Path("../assignment2/vendor/course").resolve(strict=True)
pth = Path(sysconfig.get_path("purelib")) / "cis_assignment3_course.pth"
pth.write_text(f"{course}\n{course / 'src'}\n", encoding="utf-8")
PY
python -m pip check
python -m pytest experiments/test_step1_scene.py -q -s
python -m notebook
```

## Step 8 practice validation

Run from `assignment3/` with a working course-server configuration in the
notebook's existing `SERVER_URL` and `COURSE_TOKEN` assignments:

```bash
PYTHONPATH=. venv/bin/python experiments/validate_step8.py --seeds 1 2 3 1 --tag fresh_check
```

Use a new `--tag` for each batch; existing evidence is never overwritten.
The script opens and closes a fresh practice session for each seed, saves the
headless report before asking for truth scores, and executes the notebook's
unchanged Step 8 evaluation cell. Credentials are not copied into results.
Results are in `experiments/results/step8/`; the saved `practice` batch is
summarized in `README.md`. Repeated seeds are measured independently because
identical seeds did not reproduce identical tracker observations in this batch.
The script exits nonzero on execution/cleanup failure, nonconvergence, a
workflow taking 600 s or more, or actual TRE RMS above 2 mm. It also records
the stricter all-vertices 2 mm and RMS 1 mm comparisons separately. This is
practice evidence, not a hidden-grader guarantee.

Offline validation, including the Step 8 scoring bookkeeping checks:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 venv/bin/python -m pytest -p no:cacheprovider tests experiments -q
```

## Submission layout

The final code-only upload is built with `submission/build_submission.py`.
See `submission/FINAL_REVIEW.md` for the final checks and
`submission/README_FOR_ZIP.md` for portable execution instructions. It includes
the required course Python sources, excluding development tests/experiments
and environments. Per Vincent's instruction, the notebook source/token are
preserved; skip its optional pytest cell when using the test-free ZIP.

The official course root contains `data_types/`, `assignments/assignment1/src/`,
and `src/uncertainty_networks/` alongside the path `assignments/assignment3/`.
If the TA requests a standalone ZIP, include those shared course sources in
the official relative layout so the starter's imports resolve after
extraction. Exclude the local virtual environment. The PDF on p. 3 calls
the submission folder `assignment1/`, while the TA starter uses
`assignments/assignment3/`. Follow the Assignment 3 submission instructions
issued by the TA for the actual upload.
