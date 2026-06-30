from collections import Counter

from tp_a_slam_aruco.landmark_coherence import summarize_landmark_clearance
from tp_a_slam_aruco.tag_coverage import TagCoverageTracker
from tp_a_slam_aruco.visual_observability import VisualObservabilityAggregator


class GraphRunDiagnostics:
    def __init__(self):
        self.keyframes = 0
        self.seeded_landmarks = 0
        self.candidate_landmarks = 0
        self.loop_closures = 0
        self.gating_acceptances = 0
        self.gating_rejections = 0
        self.optimization_runs = 0
        self.last_error_before = None
        self.last_error_after = None
        self.max_maha_sq = 0.0
        self.rejected_landmarks = Counter()
        self.used_landmark_batches = 0
        self.total_landmarks_in_batches = 0
        self.visual_observability = VisualObservabilityAggregator()
        self.tag_coverage = TagCoverageTracker(expected_tag_count=50)
        self.gating_rejections_bearing_dominant = 0
        self.gating_rejections_range_dominant = 0
        self.gating_rejections_mixed = 0
        self.discarded_for_age_new_landmark = 0
        self.discarded_for_age_reobservation = 0
        self.landmark_coherence = {}

    def record_keyframe(self):
        self.keyframes += 1

    def record_candidate(self):
        self.candidate_landmarks += 1

    def record_seeded_landmark(self, landmark_id=None):
        self.seeded_landmarks += 1
        if landmark_id is not None:
            self.tag_coverage.record_seeded(landmark_id)

    def record_confirmed_landmark(self, landmark_id):
        self.tag_coverage.record_confirmed(landmark_id)

    def record_loop_closure(self):
        self.loop_closures += 1

    def record_gating_acceptance(self):
        self.gating_acceptances += 1

    def record_gating_rejection(self, landmark_id, maha_sq, classification='mixed'):
        self.gating_rejections += 1
        self.max_maha_sq = max(self.max_maha_sq, float(maha_sq))
        self.rejected_landmarks[str(int(landmark_id))] += 1
        self.tag_coverage.record_gating_rejection(landmark_id)
        if classification == 'bearing_dominant':
            self.gating_rejections_bearing_dominant += 1
        elif classification == 'range_dominant':
            self.gating_rejections_range_dominant += 1
        else:
            self.gating_rejections_mixed += 1

    def record_observation_batch(self, valid_landmark_count, used_landmark_count):
        self.total_landmarks_in_batches += int(valid_landmark_count)
        self.used_landmark_batches += int(used_landmark_count)

    def record_optimization(self, err0, err1):
        self.optimization_runs += 1
        self.last_error_before = float(err0)
        self.last_error_after = float(err1)

    def record_visual_observability(self, frame):
        self.visual_observability.observe(frame)
        for landmark_id in frame.raw_ids:
            self.tag_coverage.record_raw(landmark_id)
        for landmark_id in frame.valid_ids:
            self.tag_coverage.record_valid(landmark_id)
        for landmark_id in frame.rejected_area_ids:
            self.tag_coverage.record_detector_rejection(landmark_id, 'area')
        for landmark_id in frame.rejected_depth_ids:
            self.tag_coverage.record_detector_rejection(landmark_id, 'depth')
        for landmark_id in frame.rejected_reprojection_ids:
            self.tag_coverage.record_detector_rejection(landmark_id, 'reprojection')
        for landmark_id in frame.rejected_tf_ids:
            self.tag_coverage.record_detector_rejection(landmark_id, 'tf')

    def record_age_discard(self, landmark_id, for_new_landmark):
        self.tag_coverage.record_age_rejection(landmark_id)
        if for_new_landmark:
            self.discarded_for_age_new_landmark += 1
        else:
            self.discarded_for_age_reobservation += 1

    def compute_landmark_coherence(self, trajectory, landmarks, min_landmark_clearance_m):
        self.landmark_coherence = summarize_landmark_clearance(
            trajectory=trajectory,
            landmarks=landmarks,
            min_landmark_clearance_m=min_landmark_clearance_m,
        )

    def summary(self):
        return {
            'keyframes': self.keyframes,
            'seeded_landmarks': self.seeded_landmarks,
            'candidate_landmarks': self.candidate_landmarks,
            'loop_closures': self.loop_closures,
            'gating_acceptances': self.gating_acceptances,
            'gating_rejections': self.gating_rejections,
            'optimization_runs': self.optimization_runs,
            'last_error_before': self.last_error_before,
            'last_error_after': self.last_error_after,
            'max_maha_sq': self.max_maha_sq,
            'gating_rejections_bearing_dominant': self.gating_rejections_bearing_dominant,
            'gating_rejections_range_dominant': self.gating_rejections_range_dominant,
            'gating_rejections_mixed': self.gating_rejections_mixed,
            'rejected_landmarks': dict(self.rejected_landmarks),
            'total_landmarks_in_batches': self.total_landmarks_in_batches,
            'used_landmark_batches': self.used_landmark_batches,
            'discarded_for_age_new_landmark': self.discarded_for_age_new_landmark,
            'discarded_for_age_reobservation': self.discarded_for_age_reobservation,
            'tag_coverage': self.tag_coverage.summary(),
            'landmark_coherence': self.landmark_coherence,
            'visual_observability': {
                'summary': self.visual_observability.summary(),
                'frames': self.visual_observability.serialized_frames(),
            },
        }
