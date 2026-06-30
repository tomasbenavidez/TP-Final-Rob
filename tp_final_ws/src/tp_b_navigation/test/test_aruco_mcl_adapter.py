from types import SimpleNamespace

import rclpy
from builtin_interfaces.msg import Time


def _identity_transform():
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _marker(stamp):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp, frame_id='camera'),
        pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0, z=2.0)),
    )


def test_transform_marker_uses_measurement_timestamp():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []
    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.allow_latest_tf_fallback = False
    adapter.tf_buffer = SimpleNamespace(
        lookup_transform=lambda target, source, stamp: (
            requested.append(stamp) or _identity_transform()
        ),
    )
    stamp = Time(sec=12, nanosec=300_000_000)

    point, used_fallback = adapter.transform_marker(_marker(stamp))

    assert requested == [rclpy.time.Time.from_msg(stamp)]
    assert point == (1.0, 0.0, 2.0)
    assert not used_fallback


def test_latest_tf_fallback_is_explicit_and_reported():
    from tp_b_navigation.aruco_mcl_adapter import ArucoMclAdapter

    requested = []

    def lookup(_target, _source, stamp):
        requested.append(stamp)
        if stamp.nanoseconds:
            raise RuntimeError('timestamp unavailable')
        return _identity_transform()

    adapter = ArucoMclAdapter.__new__(ArucoMclAdapter)
    adapter.base_frame = 'base_link'
    adapter.allow_latest_tf_fallback = True
    adapter.tf_buffer = SimpleNamespace(lookup_transform=lookup)

    point, used_fallback = adapter.transform_marker(
        _marker(Time(sec=12, nanosec=0)),
    )

    assert len(requested) == 2
    assert requested[0].nanoseconds == 12_000_000_000
    assert requested[1].nanoseconds == 0
    assert point == (1.0, 0.0, 2.0)
    assert used_fallback
