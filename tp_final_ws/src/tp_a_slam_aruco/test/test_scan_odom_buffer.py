from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FakeScan:
    stamp: float


def test_scan_waits_until_odom_brackets_timestamp():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer()
    buffer.add_odom(10.00, (0.0, 0.0, 0.0))
    buffer.add_scan(FakeScan(stamp=10.03), 10.03)
    assert buffer.pop_ready() == []

    buffer.add_odom(10.05, (0.1, 0.0, 0.0))
    ready = buffer.pop_ready()

    assert len(ready) == 1
    assert ready[0].scan.stamp == 10.03
    assert np.allclose(ready[0].odom_pose, (0.06, 0.0, 0.0))
    assert np.isclose(ready[0].interpolation_gap_ms, 50.0)


def test_interpolation_does_not_clamp_to_latest_odom():
    from tp_a_slam_aruco.scan_odom_buffer import interpolate_bracketed

    assert interpolate_bracketed(
        [(10.00, (0.0, 0.0, 0.0))],
        10.03,
    ) is None


def test_out_of_order_odom_and_scans_are_returned_in_scan_time_order():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer()
    buffer.add_odom(10.10, (0.2, 0.0, 0.0))
    buffer.add_odom(10.00, (0.0, 0.0, 0.0))
    buffer.add_scan(FakeScan(stamp=10.08), 10.08)
    buffer.add_scan(FakeScan(stamp=10.02), 10.02)

    ready = buffer.pop_ready()

    assert [item.scan.stamp for item in ready] == [10.02, 10.08]


def test_duplicate_odom_timestamp_replaces_previous_pose():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer()
    buffer.add_odom(10.00, (0.0, 0.0, 0.0))
    buffer.add_odom(10.00, (0.4, 0.0, 0.0))
    buffer.add_odom(10.10, (0.6, 0.0, 0.0))
    buffer.add_scan(FakeScan(stamp=10.05), 10.05)

    ready = buffer.pop_ready()

    assert len(buffer.odom_samples) == 2
    assert np.allclose(ready[0].odom_pose, (0.5, 0.0, 0.0))


def test_buffers_are_bounded_and_report_evicted_scans():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer(max_odom_samples=3, max_pending_scans=2)
    for index in range(5):
        buffer.add_odom(float(index), (float(index), 0.0, 0.0))
    for stamp in (5.0, 6.0, 7.0):
        buffer.add_scan(FakeScan(stamp=stamp), stamp)

    assert len(buffer.odom_samples) == 3
    assert buffer.waiting_count == 2
    assert buffer.dropped_excessive_wait == 1


def test_scan_without_an_older_bracket_is_dropped_after_wait_limit():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer(max_wait_seconds=0.10)
    buffer.add_scan(FakeScan(stamp=10.00), 10.00)
    buffer.add_odom(10.05, (0.0, 0.0, 0.0))
    assert buffer.waiting_count == 1

    buffer.add_odom(10.11, (0.1, 0.0, 0.0))

    assert buffer.waiting_count == 0
    assert buffer.dropped_excessive_wait == 1


def test_finalize_drops_unbracketed_end_of_bag_scans_without_clamping():
    from tp_a_slam_aruco.scan_odom_buffer import ScanOdomBuffer

    buffer = ScanOdomBuffer()
    buffer.add_odom(10.00, (0.0, 0.0, 0.0))
    buffer.add_scan(FakeScan(stamp=10.03), 10.03)

    dropped = buffer.finalize()

    assert dropped == 1
    assert buffer.waiting_count == 0
    assert buffer.dropped_at_end == 1
    assert buffer.integrated_count == 0
