"""
The simulated elapsed-time variable t_elap. Advanced by every robot move
and every SampleMarkerBodies call; nothing else costs time.
"""


class SimClock:
    def __init__(self):
        self._t = 0.0

    @property
    def elapsed(self) -> float:
        return self._t

    def advance(self, dt: float) -> float:
        if dt < 0:
            raise ValueError("SimClock.advance: dt must be non-negative")
        self._t += float(dt)
        return self._t

    def reset(self):
        self._t = 0.0

    def __repr__(self):
        return f"SimClock(elapsed={self._t:.3f}s)"


clock = SimClock()
