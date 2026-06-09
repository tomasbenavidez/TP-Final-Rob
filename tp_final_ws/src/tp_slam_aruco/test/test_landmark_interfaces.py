from tp_slam_interfaces.msg import (
    LandmarkObservation,
    LandmarkObservationArray,
    VisualObservability,
)


def test_landmark_observation_message_contains_required_fields():
    obs_fields = LandmarkObservation.get_fields_and_field_types()
    array_fields = LandmarkObservationArray.get_fields_and_field_types()
    observability_fields = VisualObservability.get_fields_and_field_types()

    assert 'header' in obs_fields
    assert 'landmark_id' in obs_fields
    assert 'range_m' in obs_fields
    assert 'bearing_rad' in obs_fields
    assert 'x_base' in obs_fields
    assert 'y_base' in obs_fields
    assert 'depth_m' in obs_fields
    assert 'reprojection_error_px' in obs_fields
    assert 'used_fallback_tf' in obs_fields
    assert 'source_frame' in obs_fields
    assert 'observations' in array_fields
    assert 'header' in observability_fields
    assert 'raw_count' in observability_fields
    assert 'valid_count' in observability_fields
    assert 'valid_unique_count' in observability_fields
    assert 'rejected_area' in observability_fields
    assert 'rejected_depth' in observability_fields
    assert 'rejected_reprojection' in observability_fields
    assert 'rejected_tf' in observability_fields
    assert 'rejected_no_calibration' in observability_fields
    assert 'raw_ids' in observability_fields
    assert 'valid_ids' in observability_fields
    assert 'rejected_area_ids' in observability_fields
    assert 'rejected_depth_ids' in observability_fields
    assert 'rejected_reprojection_ids' in observability_fields
    assert 'rejected_tf_ids' in observability_fields
