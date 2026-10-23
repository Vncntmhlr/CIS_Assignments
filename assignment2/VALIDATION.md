# Setup validation

Setup baseline: 2026-10-24, Python 3.13.11 on Linux, with
[pinned dependencies](requirements-lock.txt). Commands run from `assignment2/`
with inherited `PYTHONPATH` and `PYTHONHOME` cleared.

| Check | Result |
| --- | --- |
| `venv/bin/python -m pip check` | No broken requirements |
| `venv/bin/python -m pytest tests/test_simulation.py -q` | 23 passed |
| `venv/bin/python -m pytest tests/test_student.py -q` | 11 expected `NotImplementedError` failures; 3 TODO skips |
| First code cell of `notebooks/assignment2_template.ipynb` | Pass in a fresh `cis-assignment2` Jupyter kernel, working directory `notebooks/` |

The TA starter's 19 files and vendored support's 26 files match upstream commit
`0723d0b6793c3288457b34acfaf2cd872914207e`; no TA source or assignment TODO was changed.
The PDF and completed Assignment 1 remain unchanged. Vendor file hashes are kept
in [SHA256SUMS.json](vendor/SHA256SUMS.json).

These checks validate setup and the provided simulator only. Student algorithms,
calibrated accuracy, and complete notebook execution remain unvalidated. Temporary
setup logs were removed; rerun the commands above and the first notebook cell
using [SETUP.md](SETUP.md) to reproduce these checks.
