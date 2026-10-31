# Code-only candidate setup

Tested on Linux, Python 3.13.11. The official minimum is Python 3.9, but the frozen
pins are validated on 3.13 only. Windows and the exact grader environment have
not been reproduced. Internet access is needed for dependency installation.

From the extracted assignment2/ directory:

```bash
unset PYTHONPATH PYTHONHOME
python3.13 -m venv venv
venv/bin/python -m pip install -r requirements.txt -c requirements-lock.txt
venv/bin/python configure_environment.py
venv/bin/python -m pip check
venv/bin/python -m pytest tests experiments -q
venv/bin/python -m notebook
```

Select the locally installed Python (CIS Assignment 2) kernel, restart it, and run
notebooks/assignment2_template.ipynb with notebooks/ as its working directory.
The notebook saves calibration and evaluation in memory; its final Achieved
Performance cell summarizes the current SEED without new robot measurements.
When saving outputs, retain the optional refinement failure shown by its test cell.

configure_environment.py installs the bundled vendor/course namespaces via a
venv-local .pth file and installs the kernel inside venv/share/jupyter. For a
nonstandard Jupyter installation, set JUPYTER_PATH to the absolute venv/share/jupyter
path. In VS Code choose this extraction's venv/bin/python. Recreate the environment
if the directory is moved. No other assignment checkout or global Python package
is required. pytest.ini excludes unrelated automatically loaded plugins.

Expected test outcomes: one optional refinement failure and three expected IK
failures; other tests should pass. The normal notebook is separate from the
expensive supplementary three-seed experiment and distinguishes simulated time
from actual wall-clock execution time.

Official TA README:
https://github.com/LCSR-CIIS/CIS_I_Course/blob/0723d0b6793c3288457b34acfaf2cd872914207e/assignments/assignment2/README.md
