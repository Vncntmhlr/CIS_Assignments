# Step 8 practice validation — 2026-11-16

VERIFIED: three independent seeds and one repeated seed, with the unchanged
headless estimator. All four runs converged and passed both the RMS and
all-vertices 2 mm comparisons. These results do not establish hidden-grader
performance or calibrated uncertainty confidence.

Reproduce from `assignment3/` (use a new tag to keep existing evidence):

```bash
PYTHONPATH=. venv/bin/python experiments/validate_step8.py --seeds 1 2 3 1 --tag fresh_check
```

Each run uses 33 patches × 4 fresh placements = 132 observed samples,
135.96 s simulated time, and no missing tracker bodies. Times below include
sampling/network and fitting; session creation, scoring and cleanup are separate.

| Run / seed | Actual TRE RMS | Actual max | Actual p90 | Predicted RMS | Surface residual RMS | Workflow wall s | ICP iterations |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 / 1 | 0.069997 | 0.127692 | 0.097598 | 0.085502 | 0.158809 | 35.675 | 44 |
| 2 / 2 | 0.114338 | 0.218051 | 0.184488 | 0.068168 | 0.129896 | 33.211 | 41 |
| 3 / 3 | 0.071496 | 0.113840 | 0.096912 | 0.065598 | 0.131243 | 32.171 | 38 |
| 4 / 1 | 0.150320 | 0.297639 | 0.258004 | 0.092001 | 0.172160 | 33.016 | 38 |

Distances are mm. Actual scores cover 1,568 CT vertices. Predicted RMS is
a local covariance second moment, not an upper bound on realized error.

| Run / seed | Used | True outliers | Flagged | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 / 1 | 114 | 18 | 18 | 18 | 0 | 0 | 1.0000 | 1.0000 |
| 2 / 2 | 118 | 11 | 14 | 11 | 3 | 0 | 0.7857 | 1.0000 |
| 3 / 3 | 117 | 13 | 15 | 13 | 2 | 0 | 0.8667 | 1.0000 |
| 4 / 1 | 114 | 18 | 18 | 18 | 0 | 0 | 1.0000 | 1.0000 |

TP = correctly rejected outlier; FP = rejected normal sample; FN = retained
outlier. Missing observations would be listed separately, using the original
placement indices rather than indices in the compact ICP array.

| Run / seed | Initial guess actual RMS | Patch coarse actual RMS | Final actual RMS |
|---|---:|---:|---:|
| 1 / 1 | 20.550945 | 1.267537 | 0.069997 |
| 2 / 2 | 14.741474 | 2.168944 | 0.114338 |
| 3 / 3 | 19.069650 | 1.619797 | 0.071496 |
| 4 / 1 | 20.550945 | 1.253018 | 0.150320 |

All truth scores are queried after the final report is frozen. They are not
inputs to coarse alignment, ICP, covariance, sampling, or model selection.

## Repeatability and limits

The two seed-1 runs had the same patch assignments and true outlier indices,
but their 132 paired reference tips differed by 0.548186 mm RMS.
The server was run sequentially. The cause of the difference is unverified;
do not assume exact seed reproducibility or merge the repeats into one run.

Actual / predicted RMS ratios were 0.819, 1.677, 1.090, 1.634.
Four runs are insufficient to establish statistical calibration of the
fixed-local-correspondence covariance model. Normal-sample false rejection
occurred, and zero missing observations in these sessions does not prove
visibility for all future sessions. Other seeds/outlier rates remain untested
on the live server. The handout ~1 mm versus TA 2 mm discrepancy remains;
all four observed maxima were also below 1 mm.

## Evidence

- [Batch summary and code/data hashes](step8_practice_summary.json)
- [Offline checks and preservation audit](offline_checks.json)
- Run 1, seed 1: [scores](step8_practice_run1_seed1_validation.json), [full report JSON](step8_practice_run1_seed1_report.json), [full report text](step8_practice_run1_seed1_report.txt)
- Run 2, seed 2: [scores](step8_practice_run2_seed2_validation.json), [full report JSON](step8_practice_run2_seed2_report.json), [full report text](step8_practice_run2_seed2_report.txt)
- Run 3, seed 3: [scores](step8_practice_run3_seed3_validation.json), [full report JSON](step8_practice_run3_seed3_report.json), [full report text](step8_practice_run3_seed3_report.txt)
- Run 4, seed 1: [scores](step8_practice_run4_seed1_validation.json), [full report JSON](step8_practice_run4_seed1_report.json), [full report text](step8_practice_run4_seed1_report.txt)

The JSON scores include all mean/std/max/p90 statistics, exact truth/flag
indices, covariance rank/eigenvalues/conditioning, and both clocks. Reports
list every attempt and the per-patch counts. All four sessions were closed.
