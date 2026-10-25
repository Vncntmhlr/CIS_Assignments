# Assignment 2 setup

The [official starter](https://github.com/LCSR-CIIS/CIS_I_Course/tree/0723d0b6793c3288457b34acfaf2cd872914207e/assignments/assignment2)
is pinned to TA commit `0723d0b6793c3288457b34acfaf2cd872914207e`.
The TA simulator, vendor support snapshot, and `requirements.txt` are preserved;
student modules, tests, and notebook design cells are developed in this repository.
The [official TA README](https://github.com/LCSR-CIIS/CIS_I_Course/blob/0723d0b6793c3288457b34acfaf2cd872914207e/assignments/assignment2/README.md)
contains the assignment workflow guide.

## Fresh clone (Linux/macOS)

Use Python **3.13** for the tested dependency pins (validated on Linux with
Python 3.13.11). If cloning into a new directory:

```bash
git clone https://github.com/Vncntmhlr/CIS_Assignments.git
cd CIS_Assignments/assignment2
unset PYTHONPATH PYTHONHOME
python3.13 -m venv venv
venv/bin/python -m pip install -r requirements.txt -c requirements-lock.txt
venv/bin/python configure_environment.py
venv/bin/python -m pip check
venv/bin/python -m pytest tests/test_simulation.py -q
venv/bin/python -m notebook
```

For an existing clone, start in `assignment2/` and omit the clone and `cd` commands.
No upstream checkout, local archive, or globally installed course package is needed.
The TA minimum is Python 3.9, but these dependency pins target Python 3.13;
other Python versions may need a different dependency resolution.

Open `notebooks/assignment2_template.ipynb` and select **Python (CIS Assignment 2)**.
Run the first code cell to check imports. Keep `notebooks/` as the notebook working
directory. In VS Code, select `assignment2/venv/bin/python` if the named kernel
is not listed. Run pytest from `assignment2/`.

For later sessions:

```bash
source venv/bin/activate
unset PYTHONPATH PYTHONHOME
python -m notebook
```

Clearing these variables prevents inherited ROS or other shell import paths from
polluting the environment. The kernel also clears `PYTHONPATH` and disables user
site packages. It is registered inside `venv/share/jupyter/kernels/cis-assignment2/`,
so launch Jupyter from this venv. No global kernel installation is required.
Recreate the venv after moving the repository; its paths are absolute.

## Assignment 1 compatibility

The TA code imports `assignments.assignment1.src` and requires newer tracker APIs
and sampling helpers absent from the completed Assignment 1 saved here.
[vendor/course/](vendor/course/) contains the matching **unmodified TA** support
modules and shared geometry/uncertainty packages, preserving the official namespace.
`configure_environment.py` adds `vendor/course/` and `vendor/course/src/` to this
venv through `cis_assignment2_course.pth`. Keep the vendor snapshot in Git.
See [provenance](vendor/PROVENANCE.md) and [checksums](vendor/SHA256SUMS.json).

## Development

`pytest.ini` disables automatic loading of third-party plugins, preventing
inherited system ROS plugins from conflicting with this assignment's pytest.
Built-in plugins and `tests/conftest.py` still load. Keep the notebook's original
test command; no environment-variable workaround is needed in its code cell.
Load any intentionally needed third-party plugin explicitly with `-p plugin_name`.

Keep the original TA file lists in `src/student/` and `tests/`. Implement only
existing designated TODOs there. Student helpers live in `vendor/functions/`;
additional regression tests and retained experiment drivers live in `experiments/`.
Datasets, fitted parameters and essential evidence live in `results/`. The
protected notebook keeps its original test command (official `tests/` only);
use the commands below to include our additional regression coverage.

From `assignment2/`:

```bash
venv/bin/python -m pytest tests -q           # Complete original TA test files
venv/bin/python -m pytest experiments -q     # Student regression suite
venv/bin/python -m pytest -q                 # Both, via pytest.ini
venv/bin/python experiments/step8_phase_e.py --verify-fit
```

The complete TA suite includes a known failure in the intentionally unimplemented
`refine_marker_body()` TODO (the exact special marker body currently needs no
refinement). Three student tests explicitly document expected failures in the
unchanged provided IK near singularities/limits. Preserve these limitations;
do not alter original assertions to obtain a green summary.

The verification command reuses saved calibration/held-out measurements and checks
the normal constrained fit against the adopted model without acquiring data or
overwriting archives. See [experiment guide](experiments/README.md) and
[result index](results/README.md). Step 9 can be reproduced with
`venv/bin/python experiments/step9_forward.py` using the saved seed-1 fit;
Step 10 remains pending.
