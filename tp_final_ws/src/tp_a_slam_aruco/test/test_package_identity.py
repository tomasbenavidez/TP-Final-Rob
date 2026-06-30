import ast
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PACKAGE_NAME = 'tp_a_slam_aruco'


def _setup_package_name():
    tree = ast.parse((PACKAGE_ROOT / 'setup.py').read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(
            isinstance(target, ast.Name) and target.id == 'package_name'
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError('setup.py must assign package_name')


def test_parte_a_package_identity_is_consistent():
    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    setup_cfg = (PACKAGE_ROOT / 'setup.cfg').read_text()

    assert PACKAGE_ROOT.name == EXPECTED_PACKAGE_NAME
    assert manifest.findtext('name') == EXPECTED_PACKAGE_NAME
    assert _setup_package_name() == EXPECTED_PACKAGE_NAME
    assert f'lib/{EXPECTED_PACKAGE_NAME}' in setup_cfg
    assert (PACKAGE_ROOT / EXPECTED_PACKAGE_NAME / '__init__.py').is_file()
    assert (PACKAGE_ROOT / 'resource' / EXPECTED_PACKAGE_NAME).is_file()
