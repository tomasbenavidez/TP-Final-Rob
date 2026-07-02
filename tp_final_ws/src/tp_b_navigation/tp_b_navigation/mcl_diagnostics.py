"""CSV diagnostics for MCL measurement updates."""

import csv
import math
import os


FIELDNAMES = [
    'stamp_sec',
    'measurement_stamp_sec',
    'delay_sec',
    'event',
    'used',
    'n_eff_before',
    'n_eff_after',
    'resampled',
    'reset_weights',
    'estimate_x_before',
    'estimate_y_before',
    'estimate_yaw_before',
    'estimate_x_after',
    'estimate_y_after',
    'estimate_yaw_after',
    'delta_xy',
    'delta_yaw',
    'cov_x',
    'cov_y',
    'cov_yaw',
    'weight_max',
    'log_likelihood_min',
    'log_likelihood_max',
    'accum_d',
    'accum_a',
    'laser_accum_d',
    'laser_accum_a',
    'detail',
]


def _format_float(value):
    if value is None:
        return ''
    number = float(value)
    if not math.isfinite(number):
        return ''
    return f'{number:.9g}'


def _format_time(value):
    if value is None:
        return ''
    number = float(value)
    if not math.isfinite(number):
        return ''
    return f'{number:.9f}'


def _pose_columns(prefix, pose):
    if pose is None:
        return {
            f'estimate_x_{prefix}': '',
            f'estimate_y_{prefix}': '',
            f'estimate_yaw_{prefix}': '',
        }
    return {
        f'estimate_x_{prefix}': _format_float(pose[0]),
        f'estimate_y_{prefix}': _format_float(pose[1]),
        f'estimate_yaw_{prefix}': _format_float(pose[2]),
    }


class MclDiagnosticsCsv:
    """Append-only CSV writer for MCL correction diagnostics."""

    def __init__(self, path):
        self.path = os.path.expanduser(str(path))
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        needs_header = (
            not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        )
        self._file = open(self.path, 'a', newline='', buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDNAMES)
        if needs_header:
            self._writer.writeheader()

    def write_update(
        self,
        *,
        stamp_sec,
        measurement_stamp_sec=None,
        event,
        used,
        n_eff_before,
        n_eff_after,
        resampled,
        reset_weights,
        estimate_before,
        estimate_after,
        covariance_diag,
        weight_max,
        log_likelihood_min,
        log_likelihood_max,
        accum_d,
        accum_a,
        laser_accum_d,
        laser_accum_a,
        detail='',
    ):
        delay_sec = None
        if measurement_stamp_sec is not None:
            delay_sec = float(stamp_sec) - float(measurement_stamp_sec)
        delta_xy = ''
        delta_yaw = ''
        if estimate_before is not None and estimate_after is not None:
            dx = estimate_after[0] - estimate_before[0]
            dy = estimate_after[1] - estimate_before[1]
            delta_xy = math.hypot(dx, dy)
            delta_yaw = math.atan2(
                math.sin(estimate_after[2] - estimate_before[2]),
                math.cos(estimate_after[2] - estimate_before[2]),
            )
        cov_x, cov_y, cov_yaw = covariance_diag
        row = {
            'stamp_sec': _format_time(stamp_sec),
            'measurement_stamp_sec': _format_time(measurement_stamp_sec),
            'delay_sec': _format_float(delay_sec),
            'event': str(event),
            'used': int(used),
            'n_eff_before': _format_float(n_eff_before),
            'n_eff_after': _format_float(n_eff_after),
            'resampled': int(bool(resampled)),
            'reset_weights': int(bool(reset_weights)),
            **_pose_columns('before', estimate_before),
            **_pose_columns('after', estimate_after),
            'delta_xy': _format_float(delta_xy),
            'delta_yaw': _format_float(delta_yaw),
            'cov_x': _format_float(cov_x),
            'cov_y': _format_float(cov_y),
            'cov_yaw': _format_float(cov_yaw),
            'weight_max': _format_float(weight_max),
            'log_likelihood_min': _format_float(log_likelihood_min),
            'log_likelihood_max': _format_float(log_likelihood_max),
            'accum_d': _format_float(accum_d),
            'accum_a': _format_float(accum_a),
            'laser_accum_d': _format_float(laser_accum_d),
            'laser_accum_a': _format_float(laser_accum_a),
            'detail': str(detail),
        }
        self._writer.writerow(row)

    def close(self):
        if not self._file.closed:
            self._file.close()
