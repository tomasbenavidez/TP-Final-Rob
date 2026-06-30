import os
from pathlib import Path
import shutil
import subprocess

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def test_colcon_can_topologically_order_workspace_packages(tmp_path):
    colcon = shutil.which('colcon')
    if colcon is None:
        pytest.skip('colcon is not available in this Python environment')

    env = os.environ.copy()
    env['COLCON_LOG_PATH'] = str(tmp_path / 'log')
    result = subprocess.run(
        [colcon, 'list', '--names-only'],
        cwd=WORKSPACE_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    packages = set(result.stdout.splitlines())
    assert {'tp_b_navigation', 'tp_c_mission'} <= packages
