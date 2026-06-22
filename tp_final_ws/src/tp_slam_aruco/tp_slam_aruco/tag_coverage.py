from collections import Counter, defaultdict


class TagCoverageTracker:
    def __init__(self, expected_tag_count=50):
        self.expected_tag_count = int(expected_tag_count)
        self._raw_ids = set()
        self._valid_ids = set()
        self._seeded_ids = set()
        self._confirmed_ids = set()
        self._per_id = defaultdict(self._blank_entry)

    @staticmethod
    def _blank_entry():
        return {
            'raw': 0,
            'valid': 0,
            'seeded': 0,
            'confirmed': 0,
            'detector_rejections': Counter(),
            'age_rejections': 0,
            'gating_rejections': 0,
        }

    def record_raw(self, landmark_id):
        self._raw_ids.add(int(landmark_id))
        self._per_id[str(int(landmark_id))]['raw'] += 1

    def record_valid(self, landmark_id):
        self._valid_ids.add(int(landmark_id))
        self._per_id[str(int(landmark_id))]['valid'] += 1

    def record_seeded(self, landmark_id):
        self._seeded_ids.add(int(landmark_id))
        self._per_id[str(int(landmark_id))]['seeded'] += 1

    def record_confirmed(self, landmark_id):
        self._confirmed_ids.add(int(landmark_id))
        self._per_id[str(int(landmark_id))]['confirmed'] += 1

    def record_detector_rejection(self, landmark_id, reason):
        self._per_id[str(int(landmark_id))]['detector_rejections'][str(reason)] += 1

    def record_age_rejection(self, landmark_id):
        self._per_id[str(int(landmark_id))]['age_rejections'] += 1

    def record_gating_rejection(self, landmark_id):
        self._per_id[str(int(landmark_id))]['gating_rejections'] += 1

    def summary(self):
        return {
            'expected_tag_count': self.expected_tag_count,
            'unique_raw_ids_seen': len(self._raw_ids),
            'unique_valid_ids_seen': len(self._valid_ids),
            'unique_ids_seeded_in_graph': len(self._seeded_ids),
            'unique_ids_confirmed': len(self._confirmed_ids),
            'missing_ids_estimate': max(0, self.expected_tag_count - len(self._valid_ids)),
            'per_id': {
                key: {
                    **value,
                    'detector_rejections': dict(value['detector_rejections']),
                }
                for key, value in self._per_id.items()
            },
        }

