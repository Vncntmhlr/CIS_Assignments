"""Execute the standalone notebook in an isolated, externally configured kernel.

Only the optional pytest cell is skipped because the code ZIP excludes tests.
The notebook source is preserved. This verification tool is not shipped.
"""

import argparse
import json
from pathlib import Path
import re

import nbformat
from nbclient import NotebookClient


ROOT = Path(__file__).resolve().parents[1]


def main():
    """Save executed notebook and concise live evidence, without printing tokens."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("notebook", type=Path)
    parser.add_argument("--kernel", default="a3-submission-check")
    args = parser.parse_args()
    notebook = nbformat.read(args.notebook, as_version=4)
    original_sources = [(c.cell_type, c.source) for c in notebook.cells]
    for cell in notebook.cells:
        if cell.cell_type == "code":
            cell.outputs, cell.execution_count = [], None
    assert "pytest" in notebook.cells[9].source
    original_tags = notebook.cells[9].metadata.get("tags")
    notebook.cells[9].metadata["tags"] = [*(original_tags or []), "skip-execution"]
    # Read only already-computed results after the original cells have closed both sessions.
    notebook.cells.append(nbformat.v4.new_code_cell('''
import dataclasses, json
from pathlib import Path
import data_types.nominal_types as package_nominal
import uncertainty_networks as package_uncertainty
import assignments.assignment1.src.tracker as package_tracker
package_root = Path.cwd().parents[2]
for module in [workflow, package_nominal, package_uncertainty, package_tracker]:
    assert Path(module.__file__).is_relative_to(package_root), module.__file__
print("PACKAGE_EVIDENCE=" + json.dumps({
    "step8_actual_tre_mm": dataclasses.asdict(tre),
    "headless_converged": final.converged,
    "headless_predicted_tre_mm": final.predicted_tre,
    "headless_wall_seconds": final.t_wall,
    "headless_simulated_seconds": final.t_simulated,
    "step8_correct_outliers": len(true_outliers & flagged),
    "step8_false_alarms": len(flagged - true_outliers),
    "step8_missed_outliers": len(true_outliers - flagged),
    "course_modules_loaded_from_package": True,
}))
'''))
    client = NotebookClient(notebook, timeout=300, kernel_name=args.kernel,
                            resources={"metadata": {"path": str(args.notebook.parent)}})
    print("Executing original notebook from standalone package; optional test cell skipped.", flush=True)
    try:
        client.execute()
    finally:
        output = ROOT / "submission/verification"
        output.mkdir(parents=True, exist_ok=True)
        probe = notebook.cells.pop()
        if original_tags is None:
            notebook.cells[9].metadata.pop("tags", None)
        else:
            notebook.cells[9].metadata["tags"] = original_tags
        assert [(c.cell_type, c.source) for c in notebook.cells] == original_sources
        nbformat.write(notebook, output / "executed_assignment3_template.ipynb")
    probe_text = "".join(o.get("text", "") for o in probe.get("outputs", []))
    evidence_line = next(line for line in probe_text.splitlines() if line.startswith("PACKAGE_EVIDENCE="))
    evidence = json.loads(evidence_line.split("=", 1)[1])
    headless_text = "".join(o.get("text", "") for o in notebook.cells[21].get("outputs", []))
    match = re.search(r"TRE rms \(server, practice only\):\s*([0-9.]+)", headless_text)
    assert match, "headless practice score was not printed"
    evidence["headless_actual_tre_rms_mm_rounded"] = float(match.group(1))
    evidence.update(status="VERIFIED", code_cells_executed=sum(
        c.cell_type == "code" and i != 9 for i, c in enumerate(notebook.cells)),
                    optional_pytest_cell="SKIPPED: tests deliberately excluded from code ZIP",
                    notebook_sources="VERIFIED unchanged", isolated_kernel=args.kernel)
    assert evidence["headless_converged"] and evidence["headless_actual_tre_rms_mm_rounded"] < 2
    assert evidence["step8_actual_tre_mm"]["max"] < 2
    (output / "package_execution.json").write_text(json.dumps(evidence, indent=2) + "\n")
    report = args.notebook.parent / "registration_report.json"
    (output / "headless_report.json").write_bytes(report.read_bytes())
    print(json.dumps(evidence, indent=2), flush=True)


if __name__ == "__main__":
    main()
