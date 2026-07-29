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
                    "elapsed_s": 2.1,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        result = client.read_clock(b"jpeg")

        self.assertEqual(result["text"], "09:00")
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


if __name__ == "__main__":
    unittest.main()
