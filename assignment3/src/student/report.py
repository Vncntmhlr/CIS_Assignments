"""The required output format for the registration results (fully provided)."""

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List
import json

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame


@dataclass
class RegistrationReport:
    """Everything the handout asks you to report, in one record."""
    F_reg: Frame                       # reference body -> CT
    patches: List[dict]                # {patch_id, n_samples, n_used, n_rejected}
    samples: List[dict]                # {sample_index, patch_id, t_sim, d_ref, residual_mm, used, reason}
    residual_stats: dict               # {n_used, mean, std, max, p90} over used samples
    t_simulated: float
    t_wall: float
    n_icp_iterations: int
    converged: bool
    eta_reg: dict = field(default_factory=dict)        # {mu, C_reg, convention: "left_ct"}
    predicted_tre: dict = field(default_factory=dict)  # {rms, mean, max, p90}

    def to_dict(self) -> dict:
        d = asdict(self)
        d["F_reg"] = _frame_to_list(self.F_reg)
        return _plain(d)

    def to_json(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def from_json(cls, path) -> "RegistrationReport":
        d = json.loads(Path(path).read_text())
        d["F_reg"] = _frame_from_list(d["F_reg"])
        return cls(**d)

    def to_text(self) -> str:
        """Human-readable summary (the five blocks the handout requires)."""
        lines = ["F_reg (reference body -> CT):"]
        lines += ["  " + " ".join(f"{v:10.5f}" for v in row) for row in _frame_to_list(self.F_reg)]
        lines.append(f"\nPatches ({len(self.patches)}):  id  samples  used  rejected")
        lines += [f"  {p['patch_id']:3d}  {p['n_samples']:5d}  {p['n_used']:5d}  {p['n_rejected']:5d}" for p in self.patches]
        used = [s["sample_index"] for s in self.samples if s["used"]]
        rej = [s["sample_index"] for s in self.samples if not s["used"]]
        lines.append(f"\nValid samples ({len(used)}): {used}")
        lines.append(f"Rejected samples ({len(rej)}): {rej}")
        r = self.residual_stats
        lines.append(f"\nResiduals over {r.get('n_used', 0)} used samples (mm): "
                     f"mean {r.get('mean', float('nan')):.3f}  std {r.get('std', float('nan')):.3f}  "
                     f"max {r.get('max', float('nan')):.3f}  p90 {r.get('p90', float('nan')):.3f}")
        if self.predicted_tre:
            t = self.predicted_tre
            lines.append(f"Predicted TRE (mm): rms {t.get('rms', float('nan')):.3f}  max {t.get('max', float('nan')):.3f}")
        lines.append(f"\nICP: {self.n_icp_iterations} iterations, converged={self.converged}")
        lines.append(f"Simulated time {self.t_simulated:.2f} s, wall time {self.t_wall:.1f} s")
        return "\n".join(lines)


def residual_statistics(residuals: np.ndarray) -> dict:
    """{n_used, mean, std, max, p90} of the given residual norms."""
    r = np.asarray(residuals, dtype=float).ravel()
    if r.size == 0:
        return {"n_used": 0, "mean": float("nan"), "std": float("nan"), "max": float("nan"), "p90": float("nan")}
    return {"n_used": int(r.size), "mean": float(r.mean()), "std": float(r.std()), "max": float(r.max()),
            "p90": float(np.percentile(r, 90))}


def _frame_to_list(F: Frame) -> list:
    M = np.eye(4)
    M[:3, :3] = F.R.matrix
    M[:3, 3] = F.p.vec.ravel()
    return M.tolist()


def _frame_from_list(M) -> Frame:
    M = np.asarray(M, dtype=float)
    return Frame(Rot(matrix=M[:3, :3]), vct3(M[:3, 3]))


def _plain(x):
    if isinstance(x, dict):
        return {k: _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    return x
