"""
The student-side interface to the simulation. The simulation (femur pose,
registration truth, pointer placement, tracker) runs on the course server;
RemoteSession talks to it over HTTP. Grading uses an in-process object with
the same interface, so your code only ever sees a Session.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol
import json
import urllib.error
import urllib.request

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame
from assignments.assignment1.src.tracker import Tracker

DEFAULT_TIMEOUT_SEC = 60.0


@dataclass
class SceneDesign:
    """Your Step 1 design values."""
    F_tracker_approx: Frame
    ref_marker_positions: List[vct3]
    ptr_marker_positions: List[vct3]
    p_tip: vct3


@dataclass(frozen=True)
class PointerPlacement:
    """What PlacePointer returns: bookkeeping only."""
    patch_id: int
    sample_index: int
    t_sim: float


@dataclass
class Observation:
    """One marker body as seen by the tracker in one SampleMarkerBodies call."""
    frame: Frame               # body pose relative to the tracker
    markers: List[vct3]        # individual marker readings, tracker coordinates


@dataclass
class TreSummary:
    """TRE statistics over the mesh vertices (practice sessions only)."""
    n: int
    rms: float
    mean: float
    std: float
    max: float
    p90: float


class Session(Protocol):
    """The handout's API as seen by your code."""
    n_patches: int
    design: SceneDesign

    def initial_registration_guess(self) -> Frame: ...
    def nominal_reference_pose(self) -> Frame: ...
    def nominal_pointer_pose(self, patch_id: int) -> Frame: ...
    def is_visible(self, points_world: List[vct3]) -> bool: ...
    def place_pointer(self, patch_id: int) -> PointerPlacement: ...
    def sample_marker_bodies(self) -> Dict[str, Optional[Observation]]: ...
    def elapsed(self) -> float: ...


# ---------------------------------------------------------------- JSON helpers

def frame_to_json(F: Frame) -> list:
    M = np.eye(4)
    M[:3, :3] = F.R.matrix
    M[:3, 3] = F.p.vec.ravel()
    return M.tolist()


def frame_from_json(M) -> Frame:
    M = np.asarray(M, dtype=float)
    return Frame(Rot(matrix=M[:3, :3]), vct3(M[:3, 3]))


def vct3_to_json(p: vct3) -> list:
    return [float(x) for x in np.asarray(p.vec, dtype=float).ravel()]


def design_to_json(d: SceneDesign) -> dict:
    return {"F_tracker": frame_to_json(d.F_tracker_approx),
            "ref_markers": [vct3_to_json(a) for a in d.ref_marker_positions],
            "ptr_markers": [vct3_to_json(a) for a in d.ptr_marker_positions],
            "p_tip": vct3_to_json(d.p_tip)}


def observation_from_json(o) -> Optional[Observation]:
    if o is None:
        return None
    return Observation(frame_from_json(o["frame"]), [vct3(*m) for m in o["markers"]])


# ---------------------------------------------------------------- remote session

class RemoteSession:
    """
    A simulation session on the course server.
    input: server url, your SceneDesign, optional seed (practice sessions are
           reproducible per seed) and course token
    """

    def __init__(self, url: str, design: SceneDesign, seed: Optional[int] = None, token: str = "",
                 timeout: float = DEFAULT_TIMEOUT_SEC):
        self.url = url.rstrip("/")
        self.design = design
        self.timeout = timeout
        self._token = token
        self._frustum = Tracker("frustum", design.F_tracker_approx)
        info = self._post("/sessions", {"design": design_to_json(design), "seed": seed, "token": token})
        self.session_id: str = info["session_id"]
        self.mode: str = info["mode"]
        self.n_patches: int = info["n_patches"]
        self._F0 = frame_from_json(info["F0_reg"])
        self._ref_nominal = frame_from_json(info["nominal_reference_pose"])
        self._ptr_nominal = [frame_from_json(M) for M in info["nominal_pointer_poses"]]

    # -- the handout's API

    def initial_registration_guess(self) -> Frame:
        """F0_reg: approximate reference-body -> CT transform."""
        return Frame(self._F0.R, self._F0.p)

    def nominal_reference_pose(self) -> Frame:
        """Approximate workspace pose of the reference body (visibility planning)."""
        return self._ref_nominal

    def nominal_pointer_pose(self, patch_id: int) -> Frame:
        """Approximate workspace pose of the pointer with its tip at the centre of the patch."""
        return self._ptr_nominal[patch_id]

    def is_visible(self, points_world: List[vct3]) -> bool:
        """Frustum check with your nominal tracker pose (local, free)."""
        return self._frustum.is_visible(list(points_world))

    def place_pointer(self, patch_id: int) -> PointerPlacement:
        """PlacePointer(patch): costs 1 s of simulated time."""
        r = self._post(f"/sessions/{self.session_id}/place_pointer", {"patch_id": int(patch_id)})
        return PointerPlacement(r["patch_id"], r["sample_index"], r["t_sim"])

    def sample_marker_bodies(self) -> Dict[str, Optional[Observation]]:
        """SampleMarkerBodies(trk): {"ref": Observation|None, "ptr": Observation|None}; costs 0.03 s."""
        r = self._post(f"/sessions/{self.session_id}/sample", {})
        return {k: observation_from_json(v) for k, v in r["bodies"].items()}

    def elapsed(self) -> float:
        """Simulated time spent so far."""
        return float(self._get(f"/sessions/{self.session_id}/state")["t_sim"])

    def close(self) -> None:
        self._request("DELETE", f"/sessions/{self.session_id}", None)

    # -- practice-session oracles (refused on grading sessions)

    def evaluate_tre(self, F_reg: Frame) -> TreSummary:
        """GROUND TRUTH TRE statistics of your estimate over the mesh vertices."""
        r = self._post(f"/sessions/{self.session_id}/evaluate_tre", {"F_reg": frame_to_json(F_reg)})
        return TreSummary(**r)

    def true_outliers(self) -> List[int]:
        """GROUND TRUTH sample indices that were outliers."""
        return list(self._get(f"/sessions/{self.session_id}/true_outliers")["sample_indices"])

    # -- transport

    def _get(self, path):
        return self._request("GET", path, None)

    def _post(self, path, body):
        return self._request("POST", path, body)

    def _request(self, method, path, body):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json", "X-Token": self._token})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode() or "null")
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")
            raise RuntimeError(f"server returned {e.code} for {method} {path}: {detail}") from None
        except urllib.error.URLError as e:
            raise RuntimeError(f"cannot reach the server at {self.url}: {e.reason}") from None
