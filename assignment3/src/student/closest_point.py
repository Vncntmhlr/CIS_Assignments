"""Closest-point queries against the triangle mesh."""

from typing import Tuple

import numpy as np

from data_types.nominal_types import vct3
from ..mesh import Mesh


def closest_point_on_triangle(a: vct3, p: vct3, q: vct3, r: vct3) -> vct3:
    """
    TODO: implement.
    input: query point a, triangle corners p , q, r
    output: the point of the triangle (face, edge or vertex) closest to a
    """
   
    """Return the point on triangle pqr closest to query point a."""
    a, p, q, r = [
        np.asarray(v.vec, dtype=float).reshape(3)
        for v in (a, p, q, r)
    ]

    def project_on_segment(c, p, q):
        direction = q - p
        denominator = np.dot(direction, direction)

        if denominator == 0:
            return p.copy()

        lam = np.dot(c - p, direction) / denominator
        lam_seg = max(0.0, min(lam, 1.0))

        return p + lam_seg * direction

    # a - p ≈ lambda*(q - p) + mu*(r - p)
    A = np.column_stack((q - p, r - p))
    coefficients, _, rank, _ = np.linalg.lstsq(
        A, a - p, rcond=None
    )
    lam, mu = coefficients

    c = p + lam * (q - p) + mu * (r - p)

    if rank < 2:
        # Collinear or repeated triangle vertices.
        candidates = [
            project_on_segment(a, p, q),
            project_on_segment(a, q, r),
            project_on_segment(a, r, p),
        ]
    elif lam >= 0 and mu >= 0 and lam + mu <= 1:
        return vct3(*c)
    else:
        candidates = []

        if lam < 0:
            candidates.append(project_on_segment(c, r, p))

        if mu < 0:
            candidates.append(project_on_segment(c, p, q))

        if lam + mu > 1:
            candidates.append(project_on_segment(c, q, r))

    closest = min(
        candidates,
        key=lambda point: np.dot(a - point, a - point),
    )

    return vct3(*closest)


def closest_points_on_mesh(points: np.ndarray, mesh: Mesh) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    TODO: implement (vectorised; a per-triangle Python loop is far too slow).
    input: points (K,3) in CT coordinates, mesh
    output: closest (K,3), triangle index (K,), distance (K,)
    """
    """
    Input:
        points: (K, 3) query points in CT coordinates
        mesh.vertices: (V, 3) vertex coordinates
        mesh.triangles: (T, 3) integer vertex indices

    Output:
        closest: (K, 3) closest surface points
        triangle_index: (K,) indices of the winning triangles
        distance: (K,) Euclidean distances
    """
    points = np.asarray(points, dtype=float)
    vertices = np.asarray(mesh.vertices, dtype=float)
    triangles = np.asarray(mesh.triangles)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (K, 3)")
    if len(triangles) == 0:
        raise ValueError("mesh must contain at least one triangle")

    K = len(points)
    closest = np.empty((K, 3), dtype=float)
    triangle_index = np.empty(K, dtype=int)
    distance = np.empty(K, dtype=float)

    # Convert vertices once for the existing vct3-based function.
    vertex_vectors = [vct3(*vertex) for vertex in vertices]

    for k, a_k in enumerate(points):
        a = vct3(*a_k)

        best_squared_distance = np.inf
        best_point = None
        best_triangle = -1

        for i, indices in enumerate(triangles):
            p, q, r = [vertex_vectors[index] for index in indices]

            # Existing function: query first, then triangle corners.
            candidate = closest_point_on_triangle(a, p, q, r)
            candidate = np.asarray(
                candidate.vec, dtype=float
            ).reshape(3)

            difference = a_k - candidate
            squared_distance = np.dot(difference, difference)

            if squared_distance < best_squared_distance:
                best_squared_distance = squared_distance
                best_point = candidate
                best_triangle = i

        if best_point is None:
            raise ValueError(
                f"No finite closest point found for query {k}"
            )

        closest[k] = best_point
        triangle_index[k] = best_triangle
        distance[k] = np.sqrt(best_squared_distance)

    return closest, triangle_index, distance