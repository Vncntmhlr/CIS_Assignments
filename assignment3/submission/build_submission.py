"""Build the code-only upload ZIP from an explicit allowlist.

Run from any directory with Python 3.9+. No third-party modules are needed.
The development checkout, experiments and tests are never copied wholesale.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parents[1]
COURSE = ROOT.parent / "assignment2/vendor/course"
ARCHIVE_ROOT = "CIS_Assignment3"


def selected_files():
    """Return source-path/archive-path pairs for runtime code and required data."""
    files = [(ROOT / "submission/README_FOR_ZIP.md", f"{ARCHIVE_ROOT}/README.md")]
    for source in sorted((ROOT / "src").rglob("*.py")):
        files.append((source, f"{ARCHIVE_ROOT}/assignments/assignment3/{source.relative_to(ROOT)}"))
    for relative in ["requirements.txt", "data/ProximalFemurMeshFile.sur",
                     "data/box-centroids-rotations-world.json", "notebooks/assignment3_template.ipynb"]:
        files.append((ROOT / relative, f"{ARCHIVE_ROOT}/assignments/assignment3/{relative}"))
    for relative in ["data_types", "assignments/assignment1/src", "src/uncertainty_networks"]:
        for source in sorted((COURSE / relative).rglob("*.py")):
            # These two optional modules are not imported by the required API.
            if source.parent.name == "uncertainty_networks" and source.name in {"examples.py", "visualization.py"}:
                continue
            files.append((source, f"{ARCHIVE_ROOT}/{source.relative_to(COURSE)}"))
    return files


def main():
    """Write the ZIP plus an external manifest; optionally preserve verified outputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=ROOT / "CIS_Assignment3_SUBMISSION.zip")
    parser.add_argument("--executed-notebook", type=Path)
    args = parser.parse_args()
    source_notebook = ROOT / "notebooks/assignment3_template.ipynb"
    notebook = json.loads(source_notebook.read_text())
    if args.executed_notebook:
        executed = json.loads(args.executed_notebook.read_text())
        assert [(c["cell_type"], c["source"]) for c in executed["cells"]] == [
            (c["cell_type"], c["source"]) for c in notebook["cells"]], "executed source differs from reviewed notebook"
        assert not any(o.get("output_type") == "error" for c in executed["cells"]
                       for o in c.get("outputs", [])), "notebook contains execution errors"
        notebook = executed
    else:
        for cell in notebook["cells"]:
            if cell["cell_type"] == "code":
                cell["outputs"], cell["execution_count"] = [], None
    notebook["metadata"]["kernelspec"] = {"display_name": "Python 3", "language": "python", "name": "python3"}
    files = selected_files()
    names = [name for _, name in files]
    assert len(names) == len(set(names))
    manifest = []
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.destination, "w", compression=ZIP_DEFLATED, compresslevel=9) as archive:
        for source, name in files:
            data = ((json.dumps(notebook, indent=1, ensure_ascii=False) + "\n").encode()
                    if source == source_notebook else source.read_bytes())
            archive.writestr(name, data)
            manifest.append({"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    evidence = {"source_commit": subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "archive": args.destination.name, "archive_sha256": hashlib.sha256(args.destination.read_bytes()).hexdigest(),
        "file_count": len(manifest), "files": manifest,
        "excluded": ["experiments", "tests", "venv", "caches", "sessions", "development notes",
                     "assignment handout", "unneeded optional course examples/visualization", "build scripts"],
        "report": "Final narrative report is submitted separately; not included in this code ZIP."}
    (ROOT / "submission/manifest.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(f"Built {args.destination.name}: {len(manifest)} files, {args.destination.stat().st_size} bytes")


if __name__ == "__main__":
    main()
