"""Student-added Step 4 checks; original TA tests are left unchanged."""

import numpy as np
import pytest

from src import Frame, Rot, vct3
from src.student.metrics import frame_error, summarize_errors


@pytest.mark.parametrize("axis", [[0, 0, 1], [1, -2, 3]])
@pytest.mark.parametrize("angle, expected", [
    (0.0, 0.0), (1e-12, 1e-12), (1e-9, 1e-9), (1e-3, 1e-3),
    (0.1, 0.1), (-0.3, 0.3), (np.pi - 1e-9, np.pi - 1e-9),
    (np.pi, np.pi), (1.5 * np.pi, 0.5 * np.pi),
])
def test_rotation_angle_matches_rodrigues_reference(axis, angle, expected):
    # Known axis-angle -> matrix is independent of the matrix -> angle routine.
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    measured, distance = frame_error(Frame.eye(), Frame(Rot(matrix), vct3()))
    assert np.isclose(measured, expected, rtol=1e-12, atol=1e-15)
    assert distance == 0.0


def test_frame_error_uses_reference_inverse_times_comparison():
    reference = Frame(Rot('z', 0.8), vct3(100, -200, 300))
    relative = Frame(Rot('y', 0.2), vct3(3, 4, 12))
    comparison = reference * relative
    np.testing.assert_allclose(frame_error(reference, comparison), (0.2, 13), atol=1e-12)
    np.testing.assert_allclose(frame_error(comparison, reference), (0.2, 13), atol=1e-12)


@pytest.mark.parametrize("errors, expected", [
    ([(0.3, 3), (0.4, 4)], (2, np.sqrt(0.125), 0.4, np.sqrt(12.5), 4)),
    ([(0.1, 2)], (1, 0.1, 0.1, 2, 2)),
])
def test_summary_retains_radians_millimeters_and_sample_count(errors, expected):
    result = summarize_errors(errors)
    keys = ("n", "rms_angle", "max_angle", "rms_pos", "max_pos")
    np.testing.assert_allclose([result[key] for key in keys], expected)
    assert isinstance(result["n"], int)


@pytest.mark.parametrize("errors", [
    [], [(0, 1, 2)], [(np.nan, 0)], [(0, np.inf)], [(-0.1, 0)], [(0, -1)],
])
def test_summary_rejects_missing_or_invalid_evidence(errors):
    # Empty data must not be reported as perfect calibration accuracy.
    with pytest.raises(ValueError):
        summarize_errors(errors)
