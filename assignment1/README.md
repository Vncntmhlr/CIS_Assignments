# CIS I — Assignment 1: Pivot Calibration

## Submission overview

This directory contains the final implementation and supporting source code for CIS I Assignment 1 (pivot calibration using a simulated optical tracker).

**Primary notebook:** `notebooks/assignment1_final.ipynb`

**Final report:** `notebooks/final_report.pdf` (added separately before final submission).

**For grading:** Execute the primary notebook from top to bottom in a fresh kernel. The `archive_notebooks/` directory is included only to document earlier development, experiments, and unit tests; those notebooks are not prerequisites for the final run and are not the authoritative implementation.

## Directory structure

```text
assignment1/
├── notebooks/
│   ├── assignment1_final.ipynb      # final executable notebook
│   └── final_report.pdf             # written report (added before submission)
├── archive_notebooks/              # intermediate versions and test notebooks
├── functions/                       # student-developed algorithm and plotting helpers
│   ├── calibration_pose.py
│   ├── marker_refinement.py
│   ├── tip_calibration.py
│   ├── tip_prediction.py
│   └── workspace_viz.py
├── src/                             # course-provided simulation and tracking code
├── data_types/                      # course-provided geometry and covariance types
├── uncertainty_networks/           # course-provided transformation/uncertainty utilities
├── requirements.txt
└── README.md
```

## Requirements

- Python **3.9 or newer**, as specified in the course setup instructions.
- Python packages are listed in `requirements.txt` (NumPy, SciPy, Matplotlib, ipykernel, and notebook).
- The local `src/`, `data_types/`, `uncertainty_networks/`, and `functions/` folders are required. They are included in this submission and should remain alongside `notebooks/`.

## Clean execution — Windows PowerShell

Open PowerShell in the extracted `assignment1/` directory:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m notebook
```

In Jupyter, open **`notebooks/assignment1_final.ipynb`**. Select the kernel using `assignment1/.venv/Scripts/python.exe`, then choose **Kernel → Restart & Run All**.

The notebook expects **`notebooks/` to be its working directory**. Running the notebook through Jupyter from its usual location maintains the relative imports used by its first code cell. Do not relocate the final notebook without adjusting those paths.

## Clean execution — Linux/macOS

From the extracted `assignment1/` directory:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python -m notebook
```

Open `notebooks/assignment1_final.ipynb` and select the environment's Python kernel. Restart the kernel and run all cells in order.

## Implementation outline

The final notebook implements the assignment workflow: defining marker layouts and approximate poses; refining marker-body geometry; collecting tracker and calibration-sensor observations; solving for the pointer-tip offset; and evaluating predicted tip positions on test poses. The separate `functions/` files contain reusable student-implemented functionality.

The simulation contains random measurement and placement effects, so numerical outcomes may vary between independent runs. The final report describes validation methods and achieved performance.

## Notes on archived notebooks

The `archive_notebooks/` folder preserves the project history, including intermediate drafts and experimental/unit-test notebooks. It is provided for transparency and reference; some archived notebooks may assume earlier file layouts or execution environments. **Please use `notebooks/assignment1_final.ipynb` as the entry point for evaluating the final system.**

## Troubleshooting

- **Missing NumPy/SciPy/Jupyter:** Confirm that the notebook's kernel is the Python interpreter in the newly created `.venv` and that `requirements.txt` was installed there.
- **`ModuleNotFoundError: src`, `data_types`, `uncertainty_networks`, or `functions`:** Confirm the full folder structure above is present and the notebook is executing with `notebooks/` as its working directory.
- **Notebook runs only after manually executing cells:** Restart the kernel and use **Run All** to verify execution order; the final submission should not rely on previously saved Python variables.

## Course code acknowledgement

The course-provided simulation and mathematical support modules are included to make the submission self-contained. Their upstream source is the [CIS I Course repository](https://github.com/LCSR-CIIS/CIS_I_Course). The `functions/` directory and final notebook contain the assignment-specific implementation.
