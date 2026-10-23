"""Configure TA support imports and a Jupyter kernel in this assignment's venv."""

from pathlib import Path
import json
import os
import subprocess
import sys
import sysconfig


def main():
    assignment = Path(__file__).resolve().parent
    expected_venv = assignment / "venv"
    if Path(sys.prefix).resolve() != expected_venv.resolve() or sys.prefix == sys.base_prefix:
        raise SystemExit("Run with assignment2/venv/bin/python (Windows: venv/Scripts/python.exe).")
    course = assignment / "vendor" / "course"
    support_paths = (course, course / "src")
    if not all(path.is_dir() for path in support_paths):
        raise SystemExit("Missing vendor/course support snapshot.")
    pth = Path(sysconfig.get_path("purelib")) / "cis_assignment2_course.pth"
    pth.write_text("".join(f"{path}\n" for path in support_paths))
    clean_env = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME"):
        clean_env.pop(key, None)
    subprocess.run([
        sys.executable, "-m", "ipykernel", "install", "--prefix", sys.prefix,
        "--name", "cis-assignment2", "--display-name", "Python (CIS Assignment 2)",
    ], check=True, env=clean_env)
    kernel_file = expected_venv / "share/jupyter/kernels/cis-assignment2/kernel.json"
    kernel = json.loads(kernel_file.read_text())
    kernel.setdefault("env", {}).update({"PYTHONPATH": "", "PYTHONNOUSERSITE": "1"})
    kernel_file.write_text(json.dumps(kernel, indent=2) + "\n")
    print(f"Configured support imports: {pth}")


if __name__ == "__main__":
    main()
