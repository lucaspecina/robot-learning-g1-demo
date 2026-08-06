"""Pruebas del contrato de búsqueda visual puntual."""
import json
import unittest

from open_vocabulary_core import make_search_request, parse_search_request


class OpenVocabularyCoreTest(unittest.TestCase):
    def test_builds_and_parses_a_known_target(self):
        request = make_search_request("red_table", request_id="request-1")
        result = parse_search_request(json.dumps(request))
        self.assertEqual(
            result,
            ("request-1", "red_table", ["a table"]),
        )

    def test_rejects_free_text_as_a_target(self):
        with self.assertRaises(ValueError):
            make_search_request("cualquier cosa")

    def test_clock_uses_a_bounded_open_vocabulary_label(self):
        request = make_search_request("clock", request_id="clock-1")

        self.assertEqual(
            parse_search_request(json.dumps(request)),
            ("clock-1", "clock", ["a digital wall clock"]),
        )

    def test_rejects_missing_request_id(self):
        with self.assertRaises(ValueError):
            parse_search_request(json.dumps({"target": "blue_table"}))


if __name__ == "__main__":
    unittest.main()
