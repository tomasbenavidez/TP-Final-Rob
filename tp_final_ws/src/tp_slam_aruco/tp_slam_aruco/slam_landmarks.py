from dataclasses import dataclass


@dataclass(frozen=True)
class LandmarkCandidate:
    landmark_id: int
    projected_x: float
    projected_y: float
    observation_count: int
    mean_reprojection_error_px: float


class LandmarkCandidateManager:
    def __init__(
        self,
        min_candidate_observations=2,
        min_candidate_parallax_m=0.20,
        max_candidate_reprojection_error_px=4.0,
        max_candidate_buffer_observations=10,
        candidate_reset_on_bad_observation=True,
        min_candidate_pose_separation_m=0.15,
    ):
        self.min_candidate_observations = int(min_candidate_observations)
        self.min_candidate_parallax_m = float(min_candidate_parallax_m)
        self.max_candidate_reprojection_error_px = float(
            max_candidate_reprojection_error_px
        )
        self.max_candidate_buffer_observations = int(max_candidate_buffer_observations)
        self.candidate_reset_on_bad_observation = bool(candidate_reset_on_bad_observation)
        self.min_candidate_pose_separation_m = float(min_candidate_pose_separation_m)
        self._candidates = {}
        self._confirmed = {}
        self._stats = {
            'discarded_buffer_exhausted': 0,
            'discarded_age_dominated': 0,
            'discarded_bad_quality': 0,
            'discarded_no_parallax': 0,
        }

    def is_candidate(self, landmark_id):
        return landmark_id in self._candidates

    def is_confirmed(self, landmark_id):
        return landmark_id in self._confirmed

    def observe(
        self,
        landmark_id,
        pose_xy,
        projected_xy,
        reprojection_error_px,
        observation_age_s=0.0,
        max_observation_age_s=None,
    ):
        if landmark_id in self._confirmed:
            return self._confirmed[landmark_id]

        if (
            max_observation_age_s is not None
            and float(observation_age_s) > float(max_observation_age_s)
        ):
            self._stats['discarded_age_dominated'] += 1
            return None

        state = self._candidates.get(landmark_id)
        if state is None:
            self._candidates[landmark_id] = {
                'poses': [tuple(pose_xy)],
                'points': [tuple(projected_xy)],
                'errors': [float(reprojection_error_px)],
            }
            return None

        if reprojection_error_px > self.max_candidate_reprojection_error_px:
            self._stats['discarded_bad_quality'] += 1
            if self.candidate_reset_on_bad_observation:
                self._candidates.pop(landmark_id, None)
            return None

        state['poses'].append(tuple(pose_xy))
        state['points'].append(tuple(projected_xy))
        state['errors'].append(float(reprojection_error_px))
        if len(state['poses']) >= self.max_candidate_buffer_observations:
            self._stats['discarded_buffer_exhausted'] += 1
            self._candidates.pop(landmark_id, None)
            return None

        first_pose = state['poses'][0]
        last_pose = state['poses'][-1]
        parallax = ((last_pose[0] - first_pose[0]) ** 2 + (last_pose[1] - first_pose[1]) ** 2) ** 0.5
        if len(state['poses']) < self.min_candidate_observations:
            return None
        if parallax < self.min_candidate_parallax_m:
            self._stats['discarded_no_parallax'] += 1
            return None
        if self._min_pose_separation(state['poses']) < self.min_candidate_pose_separation_m:
            return None

        mean_x = sum(point[0] for point in state['points']) / len(state['points'])
        mean_y = sum(point[1] for point in state['points']) / len(state['points'])
        mean_error = sum(state['errors']) / len(state['errors'])
        if mean_error > self.max_candidate_reprojection_error_px:
            return None

        candidate = LandmarkCandidate(
            landmark_id=int(landmark_id),
            projected_x=mean_x,
            projected_y=mean_y,
            observation_count=len(state['points']),
            mean_reprojection_error_px=mean_error,
        )
        self._confirmed[landmark_id] = candidate
        self._candidates.pop(landmark_id, None)
        return candidate

    @staticmethod
    def _min_pose_separation(poses):
        if len(poses) < 2:
            return 0.0
        first = poses[0]
        last = poses[-1]
        return ((last[0] - first[0]) ** 2 + (last[1] - first[1]) ** 2) ** 0.5

    def stats(self):
        return dict(self._stats)
