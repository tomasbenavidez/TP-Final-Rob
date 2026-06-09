from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class FrameObservability:
    stamp: float
    raw_count: int
    valid_count: int
    valid_unique_count: int
    rejected_area: int
    rejected_depth: int
    rejected_reprojection: int
    rejected_tf: int
    rejected_no_calibration: int
    raw_ids: tuple[int, ...] = ()
    valid_ids: tuple[int, ...] = ()
    rejected_area_ids: tuple[int, ...] = ()
    rejected_depth_ids: tuple[int, ...] = ()
    rejected_reprojection_ids: tuple[int, ...] = ()
    rejected_tf_ids: tuple[int, ...] = ()


class VisualObservabilityAggregator:
    def __init__(self):
        self._frames = []

    def observe(self, frame):
        self._frames.append(frame)

    def serialized_frames(self):
        return [asdict(frame) for frame in self._frames]

    def summary(self):
        frame_count = len(self._frames)
        valid_unique_values = [frame.valid_unique_count for frame in self._frames]
        if not valid_unique_values:
            return {
                'frame_count': 0,
                'max_valid_unique_ever': 0,
                'frames_with_0_valid_unique': 0,
                'frames_with_1_valid_unique': 0,
                'frames_with_2plus_valid_unique': 0,
                'rejections': {
                    'area': 0,
                    'depth': 0,
                    'reprojection': 0,
                    'tf': 0,
                    'no_calibration': 0,
                },
                'percentiles': {
                    'p50_valid_unique': 0,
                    'p90_valid_unique': 0,
                    'p100_valid_unique': 0,
                },
            }

        sorted_values = sorted(valid_unique_values)
        def percentile(q):
            if not sorted_values:
                return 0
            if q >= 1.0:
                return sorted_values[-1]
            index = int(round((len(sorted_values) - 1) * q))
            return sorted_values[index]

        return {
            'frame_count': frame_count,
            'max_valid_unique_ever': max(valid_unique_values),
            'frames_with_0_valid_unique': sum(v == 0 for v in valid_unique_values),
            'frames_with_1_valid_unique': sum(v == 1 for v in valid_unique_values),
            'frames_with_2plus_valid_unique': sum(v >= 2 for v in valid_unique_values),
            'rejections': {
                'area': sum(frame.rejected_area for frame in self._frames),
                'depth': sum(frame.rejected_depth for frame in self._frames),
                'reprojection': sum(frame.rejected_reprojection for frame in self._frames),
                'tf': sum(frame.rejected_tf for frame in self._frames),
                'no_calibration': sum(frame.rejected_no_calibration for frame in self._frames),
            },
            'percentiles': {
                'p50_valid_unique': percentile(0.50),
                'p90_valid_unique': percentile(0.90),
                'p100_valid_unique': percentile(1.00),
            },
        }
