# TA support snapshot

Source: https://github.com/LCSR-CIIS/CIS_I_Course

Commit: `0723d0b6793c3288457b34acfaf2cd872914207e` (main retrieved 2026-10-24).

Only the official Assignment 1 `src/`, shared `data_types/`, and
`src/uncertainty_networks/` are included under `course/`. Files are unmodified;
bytecode caches and Git metadata are excluded. These support the unchanged
Assignment 2 starter; they do not replace this repository's Assignment 1 work.

Assignment 2 imports `assignments.assignment1.src` and expects the newer TA
tracker API (frustum constants, `is_visible`, and sampling helpers). The saved
Assignment 1 implementation predates that API. Assignment 2's virtual environment
adds `course/` and `course/src/` to its import paths, preserving the TA namespace
and keeping the support version consistent. Do not edit this snapshot to
implement student algorithms.
