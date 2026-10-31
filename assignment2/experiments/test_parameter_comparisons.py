"""Post-fit notebook diagnostics: gauge alignment must not change calibration."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.linalg import expm

from src import Frame, Rot, vct3
from src.robot import Robot
from src.student import metrics


@pytest.mark.parametrize("roll", [-0.31, 0.27])
def test_notebook_comparison_removes_only_common_roll(monkeypatch, roll):
    notebook = Path(__file__).resolve().parents[1] / "notebooks/assignment2_template.ipynb"
    cells = json.loads(notebook.read_text())["cells"]
    marker = "# Post-fit parameter comparisons: reporting only, never fed back into fitting."
    source = next("".join(c["source"]) for c in cells if marker in "".join(c["source"]))
    source = source[source.index(marker):]
    axes = np.array([axis.vec.ravel() for axis in Robot.AXES])
    gauge = Frame(Rot("x", roll), vct3())
    tracker = Frame(Rot("y", 0.13), vct3(3, -5, 7))
    base = Frame(Rot("z", -0.21), vct3(11, 23, -4))
    xi = np.array([0.003, -0.002, 0.001, 0.1, -0.2, 0.15])
    algebra = np.zeros((4, 4))
    x, y, z = xi[:3]
    algebra[:3, :3] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    algebra[:3, 3] = xi[3:]
    tool = Robot.F_WE * Frame(expm(algebra))  # Independent exponential oracle.
    estimated = dict(axes=axes @ gauge.R.matrix.T,
                     F_X=base * gauge.inv(), F_we=gauge * tool)
    truth = dict(axes=axes, F_rob_actual=tracker * base, F_we_twist=xi)
    before = copy.deepcopy((estimated, truth))
    displayed = []
    monkeypatch.setattr("IPython.display.display", displayed.append)
    namespace = dict(np=np, Frame=Frame, Rot=Rot, vct3=vct3, metrics=metrics,
                     estimated=estimated, truth=truth, rob=Robot,
                     trk=SimpleNamespace(nominal_pose=lambda: tracker))
    exec(compile(source, str(notebook), "exec"), namespace)
    assert np.max(namespace["_compare_axis_error_deg"]) < 1e-10
    assert max(namespace["_compare_base_error"]) < 1e-10
    assert max(namespace["_compare_tool_error"]) < 1e-10
    assert displayed
    for actual, saved in zip((estimated, truth), before):
        for key in actual:
            if isinstance(actual[key], np.ndarray):
                np.testing.assert_array_equal(actual[key], saved[key])
            else:
                assert max(metrics.frame_error(actual[key], saved[key])) < 1e-12

    # A common yaw is not an allowed gauge: do not silently align it away.
    estimated["axes"] = axes @ Rot("z", 0.02).matrix.T
    exec(compile(source, str(notebook), "exec"), namespace)
    assert np.max(namespace["_compare_axis_error_deg"]) > 1.0
