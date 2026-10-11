
import numpy as np
from src import simulation, vct3


def refine_marker_body(trk, placed, n_readings=30, passes=3):
    for _ in range(passes):
        reading_a_values = []

        for _ in range(n_readings):
            readings = simulation.sample_marker_bodies(
                trk, [placed]
            )

            observed = readings[placed.body.name]
            F_inv = observed.frame.inv()

            new_a_positions = []

            for marker in observed.markers:
                a = F_inv * marker.p
                new_a_positions.append([a.x, a.y, a.z])

            reading_a_values.append(new_a_positions)

        stacked = np.stack(reading_a_values)
        mean_a = np.mean(stacked, axis=0)

        mean_positions = [
            vct3(*row) for row in mean_a
        ]

        placed.body.set_marker_positions(mean_positions)
