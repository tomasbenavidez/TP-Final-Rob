"""Small state machines used by the Graph SLAM node."""

from __future__ import annotations


class LandmarkObservationGate:
    """Promote a landmark only after it has appeared several times."""

    def __init__(self, min_observations: int):
        self.min_observations = max(1, int(min_observations))
        self.counts: dict[int, int] = {}

    def register(self, landmark_id: int) -> tuple[bool, int]:
        landmark_id = int(landmark_id)
        count = self.counts.get(landmark_id, 0) + 1
        self.counts[landmark_id] = count
        return count >= self.min_observations, count

    def clear(self, landmark_id: int):
        self.counts.pop(int(landmark_id), None)
