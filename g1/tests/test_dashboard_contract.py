"""Pruebas del contrato visual que necesita el operador de la demo."""
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
import unittest


DASHBOARD_HTML = (
    Path(__file__).resolve().parents[1] / "dashboard" / "index.html"
).read_text(encoding="utf-8")


class IdCollector(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        values = dict(attrs)
        if values.get("id"):
            self.ids.append(values["id"])


class DashboardContractTest(unittest.TestCase):
    def test_every_dynamic_target_has_a_unique_id(self):
        parser = IdCollector()
        parser.feed(DASHBOARD_HTML)
        duplicates = [
            item
            for item, count in Counter(parser.ids).items()
            if count > 1
        ]
        self.assertEqual(duplicates, [])

    def test_history_is_with_camera_and_map_is_below_models(self):
        self.assertLess(
            DASHBOARD_HTML.index('id="mission-timeline"'),
            DASHBOARD_HTML.index('class="card brain-card"'),
        )
        self.assertGreater(
            DASHBOARD_HTML.index('id="map"'),
            DASHBOARD_HTML.index('class="card brain-card"'),
        )
        self.assertIn(
            "Ver trazabilidad técnica literal",
            DASHBOARD_HTML,
        )

    def test_live_view_explains_timestamp_matched_boxes(self):
        self.assertIn("nunca se pegan cajas viejas", DASHBOARD_HTML)
        self.assertIn("state.live_boxes_active", DASHBOARD_HTML)


if __name__ == "__main__":
    unittest.main()
