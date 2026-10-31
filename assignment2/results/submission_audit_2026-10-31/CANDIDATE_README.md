# CIS I Assignment 2 — CANDIDATE, NOT APPROVED FOR SUBMISSION

Source: dev/vincent, HEAD 603d02af32e9a69a416c9c0b58938240a6ceb443, plus the current uncommitted notebook and
comment-only registration/test edits. See the external manifest for exact hashes.

## Setup and normal execution
Follow assignment2/SETUP.md. The notebook must run with assignment2/notebooks/
as its working directory, after configuring vendor support in its new venv.
No outer Assignment 1 checkout, Codex directory, or original workspace is needed.

## Known unresolved items
- The bundled report is the existing local PDF, not an approved final v2.
  Author/JHED/signature/contribution/Claude fields are incomplete.
- Optional refine_marker_body is unimplemented. Its student-written test fails;
  three expected-failure regressions document unchanged TA IK limitations.
- Saved three-seed evaluation is historical; no extensive experiment was rerun.
- Obtain teammate review and approve the final report and merge before submitting.

The package retains all regression tests. Experiment drivers and two pilot JSON
files are retained only where they serve regression/reproducibility dependencies;
they are not part of normal notebook execution. Results include calibration
observations, fitted models and compact summaries, not duplicate plot exports
or large per-attempt full-run JSON records. The report embeds its main figures;
two separately referenced supplementary figure PDFs are included once.
