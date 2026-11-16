"""Closest-point queries against the triangle mesh."""

from typing import Tuple

import numpy as np

from data_types.nominal_types import vct3
from ..mesh import Mesh


_QUERY_BATCH_SIZE = 32


def _dot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Dot product along the coordinate axis, broadcasting the other axes."""
    return np.einsum("...i,...i->...", a, b)


def _closest_points(points: np.ndarray, triangles: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map float queries (K,3) and triangles (T,3,3) to points, indices, distances.

    Inputs share a coordinate frame. Search all triangles in batches of queries;
    collapsed faces use their segments, and equal minima keep the first triangle.
    """
    anchors = triangles[:, 0]
    edges = np.roll(triangles, -1, axis=1) - triangles  # AB, BC, CA
    edge_lengths_squared = _dot(edges, edges)
    ab = edges[:, 0]
    ac = -edges[:, 2]
    normals = np.cross(ab, ac)
    normal_squared = _dot(normals, normals)

    # Compare the doubled area to the squared edge length to detect a
    # numerically collapsed face at the triangle's scale.
    area_tol = np.finfo(float).eps * edge_lengths_squared.max(axis=1)
    has_face = normal_squared > area_tol**2
    # These dot products give the two in-plane coordinates of P - A.
    # Using the cross product for the area avoids subtracting nearly equal
    # terms in the determinant of the 2x2 projection system.
    safe_normal_squared = np.where(has_face, normal_squared, 1.0)
    lambda_axis = np.cross(ac, normals) / safe_normal_squared[:, None]
    mu_axis = np.cross(normals, ab) / safe_normal_squared[:, None]
    safe_edge_lengths = np.where(edge_lengths_squared > 0.0, edge_lengths_squared, 1.0)
    segment_axes = edges / safe_edge_lengths[..., None]

    closest = np.empty_like(points)
    triangle_indices = np.empty(len(points), dtype=int)
    distances = np.empty(len(points))
    for start in range(0, len(points), _QUERY_BATCH_SIZE):
        batch = slice(start, start + _QUERY_BATCH_SIZE)
        queries = points[batch, None, :]
        offsets = queries - anchors
        lam = _dot(offsets, lambda_axis)
        mu = _dot(offsets, mu_axis)
        inside = has_face & (lam >= 0.0) & (mu >= 0.0) & (lam + mu <= 1.0)
        candidates = anchors + lam[..., None] * ab + mu[..., None] * ac
        delta = queries - candidates
        squared_distances = np.where(inside, _dot(delta, delta), np.inf)

        for side in range(3):
            edge_start = triangles[:, side]
            fraction = np.clip(_dot(queries - edge_start, segment_axes[:, side]), 0.0, 1.0)
            edge_points = edge_start + fraction[..., None] * edges[:, side]
            delta = queries - edge_points
            edge_distances = _dot(delta, delta)
            better = ~inside & (edge_distances < squared_distances)
            candidates[better] = edge_points[better]
            squared_distances[better] = edge_distances[better]

        rows = np.arange(len(queries))
        winners = np.argmin(squared_distances, axis=1)
        closest[batch] = candidates[rows, winners]
        triangle_indices[batch] = winners
        distances[batch] = np.sqrt(squared_distances[rows, winners])

    return closest, triangle_indices, distances


def closest_point_on_triangle(p: vct3, a: vct3, b: vct3, c: vct3) -> vct3:
    """Return a vct3 closest to query p on triangle (a,b,c), including its boundary.

    All four vct3 inputs use the same frame and mm. Collinear or repeated
    corners are treated as segments or an isolated point.
    """
    query = np.asarray(p.vec, dtype=float).reshape(1, 3)
    triangle = np.array([a.vec, b.vec, c.vec], dtype=float).reshape(1, 3, 3)
    closest, _, _ = _closest_points(query, triangle)
    return vct3(closest[0])


def closest_points_on_mesh(points: np.ndarray, mesh: Mesh) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Find exact surface matches for finite CT queries points (K,3), in mm.

    Return closest CT points (K,3), triangle indices (K,), and Euclidean
    distances (K,), in mm. An empty query array is allowed; an empty mesh
    raises ValueError. NumPy batches avoid a Python loop over triangles.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (K, 3)")
    if mesh.n_triangles == 0:
        raise ValueError("mesh must contain at least one triangle")
    return _closest_points(points, np.asarray(mesh.tri_vertices, dtype=float))
