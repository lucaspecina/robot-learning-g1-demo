import json
import unittest
import urllib.error

from g1.agent.intelligence_client import (
    CircuitOpenError,
    IntelligenceClient,
    RemoteIntelligenceError,
)


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read(self):
        return json.dumps(self.payload).encode("utf-8")


class IntelligenceClientTests(unittest.TestCase):
    def test_reads_clock_and_preserves_request_id(self):
        def opener(request, timeout):
            request_payload = json.loads(request.data)
            self.assertTrue(request_payload["image_base64"])
            request_id = request.headers["X-request-id"]
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "reading": {
                        "readable": True,
                        "hour": 9,
                        "minute": 0,
                        "text": "09:00",
                    },
                    "model": "vision-test",
                    "raw_output": (
                        '{"readable":true,"hour":9,'
                        '"minute":0,"text":"09:00"}'
                    ),
                    "elapsed_s": 2.1,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        result = client.read_clock(b"jpeg")

        self.assertEqual(result["text"], "09:00")
        self.assertEqual(result["model"], "vision-test")
        self.assertEqual(
            result["raw_output"],
            '{"readable":true,"hour":9,"minute":0,"text":"09:00"}',
        )
        self.assertEqual(client.consecutive_failures, 0)

    def test_opens_circuit_after_three_failures(self):
        now = [100.0]

        def failing_opener(request, timeout):
            raise urllib.error.URLError("corte")

        client = IntelligenceClient(
            server_url="http://server",
            opener=failing_opener,
            monotonic=lambda: now[0],
        )
        for _ in range(3):
            with self.assertRaises(RemoteIntelligenceError):
                client.read_clock(b"jpeg")

        with self.assertRaises(CircuitOpenError):
            client.read_clock(b"jpeg")

        now[0] += 31.0
        with self.assertRaises(RemoteIntelligenceError):
            client.read_clock(b"jpeg")

    def test_rejects_inconsistent_reading(self):
        def opener(request, timeout):
            request_id = request.headers["X-request-id"]
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "reading": {
                        "readable": True,
                        "hour": 9,
                        "minute": 0,
                        "text": "19:00",
                    },
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        with self.assertRaises(RemoteIntelligenceError):
            client.read_clock(b"jpeg")
        self.assertEqual(client.consecutive_failures, 1)

    def test_reads_bounded_object_detections(self):
        def opener(request, timeout):
            request_payload = json.loads(request.data)
            self.assertEqual(request_payload["labels"], ["a red table"])
            request_id = request.headers["X-request-id"]
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "detections": [
                        {
                            "label": "a red table",
                            "confidence": 0.62,
                            "box": [10.0, 20.0, 100.0, 120.0],
                        }
                    ],
                    "image_width": 640,
                    "image_height": 480,
                    "model": "grounding-dino-test",
                    "inference_s": 1.2,
                    "elapsed_s": 1.3,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        result = client.detect_objects(b"jpeg", ["a red table"])

        self.assertEqual(result["detections"][0]["confidence"], 0.62)
        self.assertEqual(result["image_width"], 640)
        self.assertEqual(client.consecutive_failures, 0)

    def test_rejects_detection_box_outside_image(self):
        def opener(request, timeout):
            request_id = request.headers["X-request-id"]
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "detections": [
                        {
                            "label": "a red table",
                            "confidence": 0.62,
                            "box": [-1.0, 20.0, 100.0, 120.0],
                        }
                    ],
                    "image_width": 640,
                    "image_height": 480,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        with self.assertRaises(RemoteIntelligenceError):
            client.detect_objects(b"jpeg", ["a red table"])


if __name__ == "__main__":
    unittest.main()
