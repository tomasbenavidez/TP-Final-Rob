from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PackageContractTest(unittest.TestCase):
    def test_manifest_declares_ament_cmake_build_type(self):
        manifest = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        build_type = manifest.find("./export/build_type")

        self.assertIsNotNone(build_type)
        self.assertEqual("ament_cmake", build_type.text.strip())


if __name__ == "__main__":
    unittest.main()
