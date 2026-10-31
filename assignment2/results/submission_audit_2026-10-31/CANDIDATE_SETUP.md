# Candidate setup (Linux/macOS)

Run inside the extracted assignment2/ folder. Use Python 3.13 for the tested pins.
The official TA minimum is 3.9; these pins are not validated for every version.

```bash
unset PYTHONPATH PYTHONHOME
python3.13 -m venv venv
venv/bin/python -m pip install -r requirements.txt -c requirements-lock.txt
venv/bin/python configure_environment.py
venv/bin/python -m pip check
venv/bin/python -m pytest tests -q
venv/bin/python -m pytest experiments -q
venv/bin/python -m notebook
```

Open notebooks/assignment2_template.ipynb, select Python (CIS Assignment 2),
restart the kernel and run all cells. Keep notebooks/ as the notebook working
directory. In VS Code select this extracted assignment2/venv/bin/python.
The kernel lives inside the venv; no global installation is required.

configure_environment.py installs a .pth file pointing only to the packaged
vendor/course and vendor/course/src directories. Recreate the venv after moving
the extraction. The vendor snapshot preserves the official Assignment 1 tracker,
geometry and uncertainty namespaces required by the Assignment 2 starter.

All TA simulation files, notebook Markdown and non-TODO code, original complete
test assertions and the provided inverse algorithm are preserved. The notebook
contains a previously authorized separate student visualization cell. Tests in
experiments/ are student-owned. pytest.ini prevents unrelated third-party plugins.

Known test outcome: optional marker-body refinement remains unimplemented and
has one failing student-written test; three student tests are expected failures
for the unchanged TA inverse. A passing notebook execution does not erase the
pytest subprocess's failing test. Report simulated time separately from wall time.
The full supplementary evaluation is intentionally not run by the notebook.

Official starter README:
https://github.com/LCSR-CIIS/CIS_I_Course/blob/0723d0b6793c3288457b34acfaf2cd872914207e/assignments/assignment2/README.md
