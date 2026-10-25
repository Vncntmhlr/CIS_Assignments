"""Student-owned numerical checks; the original TA tests stay unchanged."""

import numpy as np
import pytest

from src import Frame, Rot, vct3
from src.student.registration import average_frames, register_points


def points(rows):
    return [vct3(row) for row in rows]


def assert_proper(rotation):
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0)


@pytest.mark.parametrize("scale", [1e-6, 1.0, 1e6])
def test_register_three_noncollinear_planar_points(scale):
    source = points(scale * np.array([[1, 2, 0], [4, 2, 0], [1, 6, 0]]))
    truth = Frame(Rot('x', 0.4) * Rot('z', -0.7), vct3(10, -20, 30))
    result = register_points(source, [truth * p for p in source])
    assert np.allclose(result.R.matrix, truth.R.matrix, atol=1e-8, rtol=0)
    assert np.allclose(result.p.vec, truth.p.vec, atol=1e-8, rtol=0)
    assert_proper(result.R.matrix)


def test_register_reflection_is_best_proper_fit():
    # Distinct axis spreads: reflecting x then flipping the least-spread z
    # direction is the proper optimum, with unavoidable squared residual 8.
    source = np.array([[6, 0, 0], [-6, 0, 0], [0, 3, 0],
                       [0, -3, 0], [0, 0, 1], [0, 0, -1]], dtype=float)
    target = source @ np.diag([-1, 1, 1]) + [5, -2, 4]
    result = register_points(points(source), points(target))
    assert_proper(result.R.matrix)
    assert np.allclose(result.R.matrix, np.diag([-1, 1, -1]), atol=1e-12)
    predictions = np.array([(result * p).vec.ravel() for p in points(source)])
    assert np.isclose(np.sum((predictions - target) ** 2), 8.0)


def test_register_noisy_data_reduces_least_squares_objective_without_mutation():
    rng = np.random.default_rng(51)
    source = rng.normal(size=(30, 3)) * 30 + [100, -20, 50]
    truth = Frame(Rot(vct3(1, 2, -1), angle=0.7), vct3(5, -10, 25))
    noiseless = np.array([(truth * p).vec.ravel() for p in points(source)])
    target = noiseless + rng.normal(scale=0.1, size=source.shape)
    a, b = points(source), points(target)
    result = register_points(a, b)
    predictions = np.array([(result * p).vec.ravel() for p in a])
    assert np.sum((predictions - target) ** 2) <= np.sum((noiseless - target) ** 2)
    assert np.allclose(predictions.mean(axis=0), target.mean(axis=0), atol=1e-12)
    assert_proper(result.R.matrix)
    assert np.array_equal([p.vec.ravel() for p in a], source)
    assert np.array_equal([p.vec.ravel() for p in b], target)


TRIANGLE = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]


@pytest.mark.parametrize("a,b", [
    ([], []),
    (TRIANGLE[:2], TRIANGLE[:2]),
    (TRIANGLE, TRIANGLE[:2]),
    ([[1, 2, 3]] * 3, TRIANGLE),
    (TRIANGLE, [[1, 2, 3]] * 3),
    ([[0, 0, 0], [1, 0, 0], [2, 0, 0]], TRIANGLE),
    (TRIANGLE, [[0, 0, 0], [1, 0, 0], [2, 0, 0]]),
    ([[0, 0, 0], [1, 0, 0], [2, 1e-16, 0]], TRIANGLE),
    ([[np.nan, 0, 0], [1, 0, 0], [0, 1, 0]], TRIANGLE),
    (TRIANGLE, [[np.inf, 0, 0], [1, 0, 0], [0, 1, 0]]),
])
def test_register_rejects_invalid_or_degenerate_geometry(a, b):
    with pytest.raises(ValueError):
        register_points(points(a), points(b))


def test_register_rejects_uninformative_correspondences():
    # Each cloud spans a plane or 3D, but pairing destroys all covariance.
    source = [[1, 0, 0], [-1, 0, 0], [0, 1, 0],
              [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    target = [[1, 0, 0]] * 2 + [[0, 1, 0]] * 2 + [[0, 0, 1]] * 2
    with pytest.raises(ValueError):
        register_points(points(source), points(target))


def test_average_singleton_and_symmetric_noise_with_translation():
    frame = Frame(Rot('z', 0.3), vct3(4, 5, 6))
    singleton = average_frames([frame])
    assert np.allclose(singleton.R.matrix, frame.R.matrix, atol=1e-12)
    assert np.array_equal(singleton.p.vec, frame.p.vec)
    result = average_frames([Frame(Rot('z', 0.1), vct3(2, 3, 4)),
                             Frame(Rot('z', 0.5), vct3(6, 7, 8))])
    assert np.allclose(result.R.matrix, frame.R.matrix, atol=1e-12)
    assert np.allclose(result.p.vec, frame.p.vec, atol=1e-12)
    assert_proper(result.R.matrix)


def test_average_across_angle_wrap():
    result = average_frames([Frame(Rot('z', np.deg2rad(angle)), vct3())
                             for angle in [179, -179]])
    assert np.allclose(result.R.matrix, np.diag([-1, -1, 1]), atol=1e-12)
    assert_proper(result.R.matrix)


def test_average_is_equivariant_to_common_left_frame():
    transform = Frame(Rot('x', 0.7), vct3(10, 20, -30))
    frames = [Frame(Rot('z', angle), vct3(angle, 2, 3)) for angle in [-0.1, 0.2, 0.4]]
    expected = transform * average_frames(frames)
    result = average_frames([transform * frame for frame in frames])
    assert np.allclose(result.R.matrix, expected.R.matrix, atol=1e-12)
    assert np.allclose(result.p.vec, expected.p.vec, atol=1e-12)


def test_average_determinant_correction_for_dispersed_rotations():
    # The raw mean is -I/3. Its closest proper rotations are nonunique;
    # any returned SO(3) minimizer has chordal cost 16 (reflections cost 12).
    matrices = [np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])]
    result = average_frames([Frame(Rot(matrix=matrix), vct3()) for matrix in matrices])
    assert_proper(result.R.matrix)
    assert np.isclose(sum(np.sum((result.R.matrix - matrix) ** 2) for matrix in matrices), 16.0)


@pytest.mark.parametrize("frames", [
    [],
    [Frame(Rot(), vct3(np.nan, 0, 0))],
    [Frame(Rot(matrix=np.full((3, 3), np.inf)), vct3())],
    [Frame(Rot(matrix=np.diag([-1, 1, 1])), vct3())],
    [Frame(Rot(matrix=np.diag([2, 1, 1])), vct3())],
])
def test_average_rejects_empty_nonfinite_or_nonrotation_inputs(frames):
    with pytest.raises(ValueError):
        average_frames(frames)
