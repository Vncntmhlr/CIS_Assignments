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

## Submission layout

The official course root contains `data_types/`, `assignments/assignment1/src/`,
and `src/uncertainty_networks/` alongside the path `assignments/assignment3/`.
If the TA requests a standalone ZIP, include those shared course sources in
the official relative layout so the starter's imports resolve after
extraction. Exclude the local virtual environment. The PDF on p. 3 calls
the submission folder `assignment1/`, while the TA starter uses
`assignments/assignment3/`. Follow the Assignment 3 submission instructions
issued by the TA for the actual upload.
