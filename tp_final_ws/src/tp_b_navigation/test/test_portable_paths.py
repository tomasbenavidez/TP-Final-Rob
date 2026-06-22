from pathlib import Path
import os
import re
import runpy
import unittest
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]


class PortableRuntimePathsTest(unittest.TestCase):
    def test_runtime_files_do_not_contain_personal_checkout_paths(self):
        files = [
            PACKAGE_ROOT / "launch" / "parte_b.launch.py",
            PACKAGE_ROOT / "launch" / "parte_b_localization.launch.py",
            PACKAGE_ROOT / "tp_b_navigation" / "map_loader.py",
        ]
        for path in files:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path):
                self.assertNotIn("/Users/", text)
                self.assertNotIn("Documents/GitHub", text)

    def test_setup_installs_map_assets_in_package_share(self):
        setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
        self.assertIn("os.path.join('share', package_name, 'maps')", setup)
        self.assertIn("mapas/map.*", setup)

    def test_setup_data_file_sources_are_relative_for_colcon(self):
        captured = {}

        def capture_setup(**kwargs):
            captured.update(kwargs)

        previous_cwd = Path.cwd()
        try:
            os.chdir(PACKAGE_ROOT)
            with patch("setuptools.setup", capture_setup):
                runpy.run_path(str(PACKAGE_ROOT / "setup.py"), run_name="__main__")
        finally:
            os.chdir(previous_cwd)

        map_sources = []
        for destination, sources in captured["data_files"]:
            if destination.endswith("/maps"):
                map_sources.extend(sources)

        self.assertEqual({"map.pgm", "map.yaml"}, {
            Path(source).name for source in map_sources
        })
        for source in map_sources:
            with self.subTest(source=source):
                self.assertFalse(os.path.isabs(source))

    def test_runtime_defaults_resolve_package_share(self):
        files = [
            PACKAGE_ROOT / "launch" / "parte_b.launch.py",
            PACKAGE_ROOT / "launch" / "parte_b_localization.launch.py",
            PACKAGE_ROOT / "tp_b_navigation" / "map_loader.py",
        ]
        for path in files:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path):
                self.assertIn("get_package_share_directory", text)
                self.assertIn("'maps', 'map.yaml'", text)

    def test_package_declares_ament_index_runtime_dependency(self):
        manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
        self.assertIn("<exec_depend>ament_index_python</exec_depend>", manifest)


class PortableDocumentationPathsTest(unittest.TestCase):
    def test_scripts_and_documentation_have_no_personal_checkout_paths(self):
        files = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "docs" / "parte_b" / "01_implementacion.md",
            REPO_ROOT / "docs" / "parte_b" / "02_guia_ejecucion.md",
            REPO_ROOT / "docs" / "parte_b" / "scripts" / "setup_parte_b.sh",
            REPO_ROOT / "docs" / "parte_b" / "scripts" / "gen_landmarks.py",
        ]
        personal_path = re.compile(r"/Users/|Documents.{0,20}GitHub|~/Documents")
        for path in files:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path):
                self.assertIsNone(personal_path.search(text))

    def test_agent_instructions_use_standard_plural_filename(self):
        self.assertTrue((REPO_ROOT / "AGENTS.md").is_file())
        self.assertFalse((REPO_ROOT / "AGENT.md").exists())

    def test_scripts_derive_repository_root(self):
        shell_setup = (
            REPO_ROOT / "docs" / "parte_b" / "scripts" / "setup_parte_b.sh"
        ).read_text(encoding="utf-8")
        landmark_generator = (
            REPO_ROOT / "docs" / "parte_b" / "scripts" / "gen_landmarks.py"
        ).read_text(encoding="utf-8")
        self.assertIn("REPO_ROOT", shell_setup)
        self.assertIn("Path(__file__).resolve()", landmark_generator)


if __name__ == "__main__":
    unittest.main()
