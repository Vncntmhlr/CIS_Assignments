"""Offline checks for projection regions and collapsed triangles."""

import numpy as np
import pytest

from src import vct3
from src.mesh import Mesh
from src.student.closest_point import closest_point_on_triangle, closest_points_on_mesh


@pytest.mark.parametrize(
    "query, expected",
    [
        ((2, 3, 4), (2, 3, 0)),    # face
        ((3, -2, 4), (3, 0, 0)),   # AB
        ((5, 5, 4), (3, 3, 0)),    # BC
        ((-2, 3, 4), (0, 3, 0)),   # CA
        ((-2, -2, 4), (0, 0, 0)),  # A
        ((9, -2, 4), (6, 0, 0)),   # B
        ((-2, 9, 4), (0, 6, 0)),   # C
        ((3, 3, 4), (3, 3, 0)),    # face/edge boundary
    ],
)
def test_triangle_projection_regions(query, expected):
    triangle = [vct3(0, 0, 0), vct3(6, 0, 0), vct3(0, 6, 0)]
    closest = closest_point_on_triangle(vct3(*query), *triangle)
    np.testing.assert_allclose(closest.vec.ravel(), expected, atol=1e-12)


def test_mesh_search_with_collapsed_triangles_across_batches():
    mesh = Mesh(
        vertices=np.array([[0, 0, 0], [6, 0, 0], [2, 0, 0], [0, 9, 0]], dtype=float),
        triangles=np.array([[0, 1, 2], [3, 3, 3], [0, 0, 1]]),
    )
    # Collinear, fully collapsed, and repeated vertices. Triangles 0 and 2
    # represent the same segment, so the earlier triangle wins the tie.
    queries = np.tile([[4, 1, 2], [0, 10, 0], [7, 0, 0]], (23, 1))
    closest, indices, distances = closest_points_on_mesh(queries, mesh)
    np.testing.assert_allclose(closest, np.tile([[4, 0, 0], [0, 9, 0], [6, 0, 0]], (23, 1)))
    np.testing.assert_array_equal(indices, np.tile([0, 1, 0], 23))
    np.testing.assert_allclose(distances, np.tile([np.sqrt(5), 1, 1], 23))

    empty = closest_points_on_mesh(np.empty((0, 3)), mesh)
    assert tuple(array.shape for array in empty) == ((0, 3), (0,), (0,))


def test_triangle_regions_in_a_rotated_translated_frame():
    """Analytic face/edge/vertex answers must survive changes of frame and winding."""
    from src import Rot

    rotation = Rot(vct3(1, 2, 3), angle=0.73).matrix
    translation = np.array([41, -29, 17])
    triangle = np.array([[0, 0, 0], [6, 0, 0], [0, 6, 0]]) @ rotation.T + translation
    queries = np.array([[2, 3, -4], [5, 5, -4], [-2, -2, -4]]) @ rotation.T + translation
    expected = np.array([[2, 3, 0], [3, 3, 0], [0, 0, 0]]) @ rotation.T + translation
    for order in ([0, 1, 2], [0, 2, 1]):
        mesh = Mesh(triangle, np.array([order]))
        closest, _, distances = closest_points_on_mesh(queries, mesh)
        np.testing.assert_allclose(closest, expected, atol=1e-12)
        np.testing.assert_allclose(distances, np.linalg.norm(queries - expected, axis=1), atol=1e-12)


def test_mesh_search_against_independent_least_squares():
    """Cross-check random triangles with an offline scalar LS reference, not our kernel."""
    rng = np.random.default_rng(42)
    triangles = rng.normal(size=(17, 3, 3)) * 20
    mesh = Mesh(triangles.reshape(-1, 3), np.arange(51).reshape(-1, 3))
    queries = rng.uniform(-40, 40, size=(67, 3))
    expected_points = []
    expected_indices = []
    for query in queries:
        triangle_points = []
        for a, b, c in triangles:
            u, v = b - a, c - a
            lam, mu = np.linalg.lstsq(np.column_stack([u, v]), query - a, rcond=None)[0]
            candidates = []
            if lam >= 0 and mu >= 0 and lam + mu <= 1:
                candidates.append(a + lam * u + mu * v)
            for start, end in ((a, b), (b, c), (c, a)):
                edge = end - start
                fraction = np.clip((query - start) @ edge / (edge @ edge), 0, 1)
                candidates.append(start + fraction * edge)
            triangle_points.append(min(candidates, key=lambda p: np.linalg.norm(query - p)))
        winner = np.argmin(np.linalg.norm(query - triangle_points, axis=1))
        expected_points.append(triangle_points[winner])
        expected_indices.append(winner)
    closest, indices, distances = closest_points_on_mesh(queries, mesh)
    np.testing.assert_allclose(closest, expected_points, atol=1e-10, rtol=0)
    np.testing.assert_array_equal(indices, expected_indices)
    np.testing.assert_allclose(distances, np.linalg.norm(queries - expected_points, axis=1), atol=1e-10)


def test_mesh_search_input_contract():
    mesh = Mesh(np.eye(3), np.array([[0, 1, 2]]))
    with pytest.raises(ValueError, match="shape"):
        closest_points_on_mesh(np.zeros(3), mesh)
    with pytest.raises(ValueError, match="at least one triangle"):
        closest_points_on_mesh(np.zeros((1, 3)), Mesh(np.empty((0, 3)), np.empty((0, 3), dtype=int)))


def test_thin_triangle_retains_its_face():
    # Its Gram determinant loses precision, but the cross-product area does not.
    triangle = [vct3(0, 0, 0), vct3(1, 0, 0), vct3(1, 1e-9, 0)]
    closest = closest_point_on_triangle(vct3(0.8, 2e-10, 1), *triangle)
    np.testing.assert_allclose(closest.vec.ravel(), [0.8, 2e-10, 0], atol=1e-15, rtol=0)
