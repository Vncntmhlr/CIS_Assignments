"""
Triangle mesh and patch-box loaders shared by the simulator and the student code.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List
import json

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame


@dataclass
class Mesh:
    """Triangulated surface. vertices: (N,3) float, triangles: (M,3) int (0-based)."""
    vertices: np.ndarray
    triangles: np.ndarray
    _cache: dict = field(default_factory=dict, repr=False)

    @property
    def n_vertices(self) -> int:
        return self.vertices.shape[0]

    @property
    def n_triangles(self) -> int:
        return self.triangles.shape[0]

    @property
    def tri_vertices(self) -> np.ndarray:
        """(M,3,3): the three corner points of every triangle."""
        if "tri" not in self._cache:
            self._cache["tri"] = self.vertices[self.triangles]
        return self._cache["tri"]

    @property
    def face_normals(self) -> np.ndarray:
        """(M,3) unit normals following the triangle winding."""
        if "n" not in self._cache:
            P = self.tri_vertices
            n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
            self._cache["n"] = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-300)
        return self._cache["n"]

    @property
    def areas(self) -> np.ndarray:
        """(M,) triangle areas."""
        if "a" not in self._cache:
            P = self.tri_vertices
            self._cache["a"] = 0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1)
        return self._cache["a"]

    @property
    def centroids(self) -> np.ndarray:
        """(M,3) triangle centroids."""
        if "c" not in self._cache:
            self._cache["c"] = self.tri_vertices.mean(axis=1)
        return self._cache["c"]

    @property
    def bounding_sphere_radius(self) -> np.ndarray:
        """(M,) radius of the sphere about each centroid that contains the triangle."""
        if "r" not in self._cache:
            self._cache["r"] = np.linalg.norm(self.tri_vertices - self.centroids[:, None, :], axis=2).max(axis=1)
        return self._cache["r"]

    def signed_volume(self) -> float:
        """Volume enclosed by the surface, positive when the winding is outward."""
        P = self.tri_vertices
        return float(np.einsum("ij,ij->i", P[:, 0], np.cross(P[:, 1], P[:, 2])).sum() / 6.0)


def load_sur(path, orient: bool = True) -> Mesh:
    """
    Reads a .sur file (NV, NV vertex lines, NT, NT triangle lines whose first
    three ints are 0-based vertex indices; extra columns are ignored).
    orient=True makes the triangle winding consistent and outward.
    """
    lines = Path(path).read_text().split("\n")
    nv = int(lines[0])
    vertices = np.array([[float(x) for x in lines[1 + i].split()[:3]] for i in range(nv)])
    nt = int(lines[1 + nv])
    triangles = np.array([[int(x) for x in lines[2 + nv + i].split()[:3]] for i in range(nt)], dtype=int)
    mesh = Mesh(vertices, triangles)
    return orient_outward(mesh) if orient else mesh


def orient_outward(mesh: Mesh) -> Mesh:
    """
    Returns a copy of mesh with consistent winding (propagated across
    2-manifold edges) and outward normals (positive signed volume).
    """
    tris = mesh.triangles.copy()
    edge_map = {}
    for t, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            edge_map.setdefault((min(u, v), max(u, v)), []).append(t)
    visited = np.zeros(len(tris), dtype=bool)
    for seed in range(len(tris)):
        if visited[seed]:
            continue
        visited[seed] = True
        stack = [seed]
        while stack:
            t = stack.pop()
            a, b, c = tris[t]
            for u, v in ((a, b), (b, c), (c, a)):
                owners = edge_map[(min(u, v), max(u, v))]
                if len(owners) != 2:
                    continue
                s = owners[0] if owners[1] == t else owners[1]
                if visited[s]:
                    continue
                sa, sb, sc = tris[s]
                if (u, v) in ((sa, sb), (sb, sc), (sc, sa)):  # same direction -> inconsistent
                    tris[s] = [sa, sc, sb]
                visited[s] = True
                stack.append(s)
    out = Mesh(mesh.vertices.copy(), tris)
    if out.signed_volume() < 0:
        out = Mesh(mesh.vertices.copy(), tris[:, [0, 2, 1]])
    return out


@dataclass
class PatchBox:
    """Oriented bounding box of a quasi-planar patch: F_box maps box -> CT coordinates."""
    id: int
    F_box: Frame

    @property
    def R(self) -> np.ndarray:
        return self.F_box.R.matrix

    @property
    def p(self) -> np.ndarray:
        return self.F_box.p.vec.ravel()

    @property
    def normal(self) -> np.ndarray:
        """Box z axis (shortest axis, approximately the surface normal) in CT coordinates."""
        return self.R[:, 2]

    def distance_to_plane(self, c) -> float:
        """Signed distance of CT point c from the box's XY plane."""
        return float(self.normal @ (np.asarray(c, dtype=float).ravel() - self.p))


def load_patch_boxes(path) -> List[PatchBox]:
    """Reads the box JSON (list of {centroid_world, rotation_local_to_world}); patch id = list index."""
    entries = json.loads(Path(path).read_text())
    boxes = []
    for i, e in enumerate(entries):
        R = Rot(matrix=np.array(e["rotation_local_to_world"], dtype=float))
        boxes.append(PatchBox(i, Frame(R, vct3(*e["centroid_world"]))))
    return boxes
