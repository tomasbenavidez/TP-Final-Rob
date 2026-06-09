from tp_slam_aruco.visual_observability import (
    FrameObservability,
    VisualObservabilityAggregator,
)


def test_visual_observability_counts_frames_by_valid_unique_bucket():
    aggregator = VisualObservabilityAggregator()

    aggregator.observe(
        FrameObservability(
            stamp=1.0,
            raw_count=2,
            valid_count=2,
            valid_unique_count=2,
            rejected_area=0,
            rejected_depth=0,
            rejected_reprojection=0,
            rejected_tf=0,
            rejected_no_calibration=0,
        )
    )
    aggregator.observe(
        FrameObservability(
            stamp=2.0,
            raw_count=1,
            valid_count=1,
            valid_unique_count=1,
            rejected_area=0,
            rejected_depth=0,
            rejected_reprojection=0,
            rejected_tf=0,
            rejected_no_calibration=0,
        )
    )
    aggregator.observe(
        FrameObservability(
            stamp=3.0,
            raw_count=1,
            valid_count=0,
            valid_unique_count=0,
            rejected_area=0,
            rejected_depth=0,
            rejected_reprojection=1,
            rejected_tf=0,
            rejected_no_calibration=0,
        )
    )

    summary = aggregator.summary()

    assert summary['frame_count'] == 3
    assert summary['max_valid_unique_ever'] == 2
    assert summary['frames_with_0_valid_unique'] == 1
    assert summary['frames_with_1_valid_unique'] == 1
    assert summary['frames_with_2plus_valid_unique'] == 1


def test_visual_observability_tracks_rejection_causes_and_percentiles():
    aggregator = VisualObservabilityAggregator()

    for idx, valid_unique in enumerate([0, 1, 1, 2, 3], start=1):
        aggregator.observe(
            FrameObservability(
                stamp=float(idx),
                raw_count=3,
                valid_count=valid_unique,
                valid_unique_count=valid_unique,
                rejected_area=1 if idx == 1 else 0,
                rejected_depth=1 if idx == 2 else 0,
                rejected_reprojection=1 if idx == 3 else 0,
                rejected_tf=1 if idx == 4 else 0,
                rejected_no_calibration=1 if idx == 5 else 0,
            )
        )

    summary = aggregator.summary()

    assert summary['frames_with_2plus_valid_unique'] == 2
    assert summary['rejections']['area'] == 1
    assert summary['rejections']['depth'] == 1
    assert summary['rejections']['reprojection'] == 1
    assert summary['rejections']['tf'] == 1
    assert summary['rejections']['no_calibration'] == 1
    assert summary['percentiles']['p50_valid_unique'] == 1
    assert summary['percentiles']['p100_valid_unique'] == 3


def test_visual_observability_preserves_frame_records_for_persistence():
    aggregator = VisualObservabilityAggregator()

    aggregator.observe(
        FrameObservability(
            stamp=42.5,
            raw_count=2,
            valid_count=1,
            valid_unique_count=1,
            rejected_area=0,
            rejected_depth=0,
            rejected_reprojection=1,
            rejected_tf=0,
            rejected_no_calibration=0,
            raw_ids=(4, 9),
            valid_ids=(4,),
            rejected_area_ids=(),
            rejected_depth_ids=(),
            rejected_reprojection_ids=(9,),
            rejected_tf_ids=(),
        )
    )

    frames = aggregator.serialized_frames()

    assert len(frames) == 1
    assert frames[0]['stamp'] == 42.5
    assert frames[0]['raw_count'] == 2
    assert frames[0]['valid_unique_count'] == 1
    assert frames[0]['rejected_reprojection'] == 1
    assert frames[0]['raw_ids'] == (4, 9)
    assert frames[0]['valid_ids'] == (4,)
    assert frames[0]['rejected_reprojection_ids'] == (9,)


def test_frame_observability_accepts_full_graph_callback_payload():
    frame = FrameObservability(
        stamp=10.0,
        raw_count=3,
        valid_count=2,
        valid_unique_count=2,
        rejected_area=1,
        rejected_depth=0,
        rejected_reprojection=0,
        rejected_tf=1,
        rejected_no_calibration=0,
        raw_ids=(1, 2, 3),
        valid_ids=(1, 3),
        rejected_area_ids=(2,),
        rejected_depth_ids=(),
        rejected_reprojection_ids=(),
        rejected_tf_ids=(2,),
    )

    assert frame.raw_ids == (1, 2, 3)
    assert frame.valid_ids == (1, 3)
    assert frame.rejected_area_ids == (2,)
    assert frame.rejected_tf_ids == (2,)
