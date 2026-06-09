from collections import deque
from dataclasses import dataclass

from tp_slam_aruco.visual_observability import FrameObservability

@dataclass(frozen=True)
class DetectionStatsSummary:
    frames: int
    avg_raw: float
    avg_valid: float
    max_valid: int


class DetectionStatsWindow:
    def __init__(self, window_size=50):
        self.window_size = max(1, int(window_size))
        self._raw_counts = deque(maxlen=self.window_size)
        self._valid_counts = deque(maxlen=self.window_size)

    def observe(self, raw_count=None, valid_ids=None, frame=None):
        if frame is not None:
            unique_valid = int(frame.valid_unique_count)
            raw_count = int(frame.raw_count)
        else:
            unique_valid = len(set(valid_ids or []))
            raw_count = int(raw_count or 0)
        self._raw_counts.append(raw_count)
        self._valid_counts.append(unique_valid)

    def summary(self):
        frames = len(self._raw_counts)
        if frames == 0:
            return DetectionStatsSummary(frames=0, avg_raw=0.0, avg_valid=0.0, max_valid=0)
        return DetectionStatsSummary(
            frames=frames,
            avg_raw=sum(self._raw_counts) / frames,
            avg_valid=sum(self._valid_counts) / frames,
            max_valid=max(self._valid_counts),
        )

    def should_warn_low_multilandmark(self):
        summary = self.summary()
        return summary.frames >= self.window_size and summary.max_valid <= 1
