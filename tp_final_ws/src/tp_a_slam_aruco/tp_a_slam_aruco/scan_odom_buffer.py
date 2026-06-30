"""Bounded timestamp synchronization for LaserScan and dense odometry."""

from bisect import bisect_left
from dataclasses import dataclass

from tp_a_slam_aruco.slam_geometry import se2_interpolate


@dataclass(frozen=True)
class ReadyScan:
    scan: object
    stamp: float
    odom_pose: tuple
    interpolation_gap_ms: float


def _bracket(samples, stamp):
    if not samples:
        return None
    stamps = [item[0] for item in samples]
    index = bisect_left(stamps, stamp)
    if index < len(samples) and samples[index][0] == stamp:
        return samples[index], samples[index]
    if index == 0 or index == len(samples):
        return None
    return samples[index - 1], samples[index]


def interpolate_bracketed(samples, stamp):
    """Interpolate only when samples contain data on both sides of stamp."""
    bracket = _bracket(samples, stamp)
    if bracket is None:
        return None
    lower, upper = bracket
    if lower[0] == upper[0]:
        return lower[1]
    alpha = (stamp - lower[0]) / (upper[0] - lower[0])
    return se2_interpolate(lower[1], upper[1], alpha)


class ScanOdomBuffer:
    def __init__(
        self,
        max_odom_samples=4000,
        max_pending_scans=500,
        max_wait_seconds=1.0,
    ):
        if max_odom_samples < 2:
            raise ValueError('max_odom_samples debe ser al menos 2')
        if max_pending_scans < 1:
            raise ValueError('max_pending_scans debe ser al menos 1')
        if max_wait_seconds <= 0.0:
            raise ValueError('max_wait_seconds debe ser positivo')
        self.max_odom_samples = int(max_odom_samples)
        self.max_pending_scans = int(max_pending_scans)
        self.max_wait_seconds = float(max_wait_seconds)
        self.odom_samples = []
        self._pending_scans = []
        self.integrated_count = 0
        self.dropped_at_end = 0
        self.dropped_excessive_wait = 0
        self.max_interpolation_gap_ms = 0.0

    @property
    def waiting_count(self):
        return len(self._pending_scans)

    def add_odom(self, stamp, pose):
        stamp = float(stamp)
        index = bisect_left([item[0] for item in self.odom_samples], stamp)
        sample = (stamp, tuple(pose))
        if (
            index < len(self.odom_samples)
            and self.odom_samples[index][0] == stamp
        ):
            self.odom_samples[index] = sample
        else:
            self.odom_samples.insert(index, sample)
        overflow = len(self.odom_samples) - self.max_odom_samples
        if overflow > 0:
            del self.odom_samples[:overflow]
        self._drop_expired_scans()

    def add_scan(self, scan, stamp):
        stamp = float(stamp)
        index = bisect_left(
            [item[0] for item in self._pending_scans],
            stamp,
        )
        self._pending_scans.insert(index, (stamp, scan))
        overflow = len(self._pending_scans) - self.max_pending_scans
        if overflow > 0:
            del self._pending_scans[:overflow]
            self.dropped_excessive_wait += overflow

    def pop_ready(self):
        ready = []
        waiting = []
        for stamp, scan in self._pending_scans:
            bracket = _bracket(self.odom_samples, stamp)
            if bracket is None:
                waiting.append((stamp, scan))
                continue
            lower, upper = bracket
            odom_pose = interpolate_bracketed(self.odom_samples, stamp)
            gap_ms = (upper[0] - lower[0]) * 1000.0
            self.max_interpolation_gap_ms = max(
                self.max_interpolation_gap_ms,
                gap_ms,
            )
            ready.append(ReadyScan(scan, stamp, odom_pose, gap_ms))
        self._pending_scans = waiting
        self.integrated_count += len(ready)
        return ready

    def _drop_expired_scans(self):
        if not self.odom_samples:
            return
        newest_odom_stamp = self.odom_samples[-1][0]
        retained = []
        dropped = 0
        for stamp, scan in self._pending_scans:
            if (
                newest_odom_stamp - stamp > self.max_wait_seconds
                and _bracket(self.odom_samples, stamp) is None
            ):
                dropped += 1
            else:
                retained.append((stamp, scan))
        self._pending_scans = retained
        self.dropped_excessive_wait += dropped

    def finalize(self):
        dropped = len(self._pending_scans)
        self._pending_scans.clear()
        self.dropped_at_end += dropped
        return dropped
