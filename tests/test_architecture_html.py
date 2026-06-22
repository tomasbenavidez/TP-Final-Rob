from html.parser import HTMLParser
from pathlib import Path
from collections import Counter
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
HTML_PATH = ROOT / "docs" / "arquitectura_sistema_completo.html"


class ArchitectureParser(HTMLParser):
    VOID_ELEMENTS = {
        "area",
        "base",
        "br",
        "col",
        "embed",
        "hr",
        "img",
        "input",
        "link",
        "meta",
        "param",
        "source",
        "track",
        "wbr",
    }

    def __init__(self):
        super().__init__()
        self.nodes = set()
        self.modes = set()
        self.mode_controls = []
        self.tours = set()
        self.tour_controls = []
        self.capabilities = set()
        self.capability_occurrences = []
        self.future_sections = 0
        self.future_nodes = set()
        self.future_text = []
        self._future_tag_stack = []
        self.invariants = []
        self.buttons_without_type = []
        self.aria_labels = set()
        self.remote_resources = []
        self.has_detail_region = False
        self.live_detail_regions = 0

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if "data-node" in attributes:
            self.nodes.add(attributes["data-node"])
        if "data-mode" in attributes:
            self.modes.add(attributes["data-mode"])
            self.mode_controls.append(
                (attributes["data-mode"], tag, attributes.get("type"))
            )
        if "data-tour" in attributes:
            self.tours.add(attributes["data-tour"])
            self.tour_controls.append(
                (attributes["data-tour"], tag, attributes.get("type"))
            )
        if "data-capability" in attributes:
            self.capabilities.add(attributes["data-capability"])
            self.capability_occurrences.append(attributes["data-capability"])
        starts_future_section = (
            tag == "section" and "future" in attributes.get("class", "").split()
        )
        if starts_future_section:
            self.future_sections += 1
            if not self._future_tag_stack:
                self._future_tag_stack.append(tag)
        elif self._future_tag_stack and tag not in self.VOID_ELEMENTS:
            self._future_tag_stack.append(tag)
        if "data-node" in attributes and self._future_tag_stack:
            self.future_nodes.add(attributes["data-node"])
        if "data-invariant" in attributes:
            self.invariants.append(attributes["data-invariant"])
        if tag == "button" and attributes.get("type") != "button":
            self.buttons_without_type.append(attrs)
        if "aria-label" in attributes:
            self.aria_labels.add(attributes["aria-label"])
        aria_live = attributes.get("aria-live")
        if aria_live in {"polite", "assertive"}:
            self.has_detail_region = True
            if "data-detail-region" in attributes:
                self.live_detail_regions += 1
        for name in ("href", "src", "poster"):
            value = attributes.get(name, "")
            if value.startswith(("http://", "https://", "//")):
                self.remote_resources.append(value)
        for candidate in attributes.get("srcset", "").split(","):
            value = candidate.strip().split(maxsplit=1)[0] if candidate.strip() else ""
            if value.startswith(("http://", "https://", "//")):
                self.remote_resources.append(value)

    def handle_endtag(self, tag):
        if tag in self._future_tag_stack:
            last_match = len(self._future_tag_stack) - 1 - self._future_tag_stack[::-1].index(tag)
            del self._future_tag_stack[last_match:]

    def handle_data(self, data):
        if self._future_tag_stack:
            self.future_text.append(data)


class ArchitectureHtmlTest(unittest.TestCase):
    def setUp(self):
        self.html = HTML_PATH.read_text(encoding="utf-8")
        self.parser = ArchitectureParser()
        self.parser.feed(self.html)

    def test_contains_all_runtime_nodes(self):
        expected = {
            "tf_bridge_node",
            "aruco_detector_node",
            "graph_slam_node",
            "occupancy_grid_node",
            "map_loader",
            "landmark_publisher",
            "landmark_sensor",
            "mcl_localization",
            "global_planner",
            "obstacle_monitor",
            "state_machine",
            "sim_mapper",
        }
        self.assertEqual(expected, self.parser.nodes)

    def test_documents_critical_interfaces(self):
        required_text = (
            "/aruco_detections",
            "/trajectory_optimized",
            "trayectoria.json",
            "map.pgm",
            "map.yaml",
            "map_sim.pgm",
            "map_sim.yaml",
            "/tmp/map_sim",
            "/observed_landmarks",
            "/initialpose",
            "/goal_pose",
            "/plan_request",
            "/plan_status",
            "/obstacle_detected",
            "/cmd_vel",
            "map → odom",
        )
        for interface in required_text:
            with self.subTest(interface=interface):
                self.assertIn(interface, self.html)

    def test_has_two_reading_modes_and_four_capabilities(self):
        expected_modes = {"concept", "ros"}
        self.assertEqual(expected_modes, self.parser.modes)
        self.assertEqual(
            Counter({mode: 1 for mode in expected_modes}),
            Counter(mode for mode, _, _ in self.parser.mode_controls),
        )
        self.assertTrue(
            all(tag == "button" and type_ == "button" for _, tag, type_ in self.parser.mode_controls)
        )
        expected_capabilities = {"perceive", "estimate", "decide", "act"}
        self.assertEqual(expected_capabilities, self.parser.capabilities)
        self.assertEqual(
            Counter({capability: 1 for capability in expected_capabilities}),
            Counter(self.parser.capability_occurrences),
        )

    def test_has_four_guided_tours(self):
        expected_tours = {"build-map", "localize", "navigate", "new-obstacle"}
        self.assertEqual(expected_tours, self.parser.tours)
        self.assertEqual(
            Counter({tour: 1 for tour in expected_tours}),
            Counter(tour for tour, _, _ in self.parser.tour_controls),
        )
        self.assertTrue(
            all(tag == "button" and type_ == "button" for _, tag, type_ in self.parser.tour_controls)
        )

    def test_part_c_is_explicitly_future_and_has_no_invented_nodes(self):
        self.assertEqual(1, self.parser.future_sections)
        self.assertEqual(set(), self.parser.future_nodes)
        future_text = " ".join(" ".join(self.parser.future_text).split())
        self.assertIn("NO IMPLEMENTADO", future_text)
        self.assertIn("Parte C", future_text)

    def test_accessibility_contract(self):
        self.assertTrue(self.parser.has_detail_region)
        self.assertEqual(1, self.parser.live_detail_regions)
        self.assertEqual([], self.parser.buttons_without_type)
        self.assertIn("Modo de lectura", self.parser.aria_labels)
        self.assertIn("Recorridos guiados", self.parser.aria_labels)
        self.assertIn("prefers-reduced-motion", self.html)

    def test_documents_execution_invariants(self):
        expected_invariants = {
            "execution-separation",
            "alternate-map-odom",
            "landmark-contracts",
            "frame-contracts",
            "dense-odometry",
            "cmd-vel-owner",
            "landmark-truth-frame",
            "map-handoff",
            "simulation-remapping",
        }
        self.assertEqual(expected_invariants, set(self.parser.invariants))
        self.assertEqual(
            Counter({invariant: 1 for invariant in expected_invariants}),
            Counter(self.parser.invariants),
        )

        required_text = (
            "no se ejecutan simultáneamente",
            "productores alternativos",
            "MarkerArray",
            "PoseArray",
            "base_link",
            "base_footprint",
            "odometría densa",
            "único productor de /cmd_vel",
            "usa odom como verdad simulada",
            "único artefacto persistente compartido",
            "remapeo del entorno simulado",
        )
        for invariant in required_text:
            with self.subTest(invariant=invariant):
                self.assertIn(invariant, self.html)

    def test_is_self_contained(self):
        self.assertEqual([], self.parser.remote_resources)
        self.assertIsNone(re.search(r"<(?:script|link)[^>]+(?:src|href)=[\"']https?://", self.html))
        self.assertIsNone(
            re.search(
                r"@import\s+(?:url\(\s*)?[\"']?(?:https?:)?//",
                self.html,
                flags=re.IGNORECASE,
            )
        )
        self.assertIsNone(
            re.search(
                r"url\(\s*[\"']?(?:https?:)?//",
                self.html,
                flags=re.IGNORECASE,
            )
        )


if __name__ == "__main__":
    unittest.main()
