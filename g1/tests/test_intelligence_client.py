import base64
import json
import unittest
import urllib.error

from g1.agent.intelligence_client import (
    CircuitOpenError,
    IntelligenceClient,
    RemoteIntelligenceError,
    validate_review,
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
    def test_requests_a_plan_with_the_described_skills(self):
        catalog = [
            {
                "name": "remember_home",
                "description": "Guarda el inicio.",
                "availability": "ready",
                "variants": [
                    {
                        "argument": None,
                        "argument_description": "Sin argumento.",
                        "preconditions": ["robot_pose_known"],
                        "effects": ["home_saved"],
                    }
                ],
            }
        ]

        def opener(request, timeout):
            request_payload = json.loads(request.data)
            self.assertEqual(request_payload["command"], "Recordá el inicio")
            self.assertEqual(
                request_payload["skill_catalog"][0]["description"],
                "Guarda el inicio.",
            )
            request_id = request.headers["X-request-id"]
            raw_output = (
                '{"steps":[{"id":"remember_home",'
                '"skill":"remember_home","argument":null,'
                '"label":"Guardar el inicio"}]}'
            )
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "plan": json.loads(raw_output),
                    "raw_output": raw_output,
                    "model_input": {
                        "messages": [
                            {"role": "user", "content": "Recordá el inicio"}
                        ]
                    },
                    "model": "planner-test",
                    "elapsed_s": 0.5,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        result = client.plan_mission(
            "Recordá el inicio",
            catalog,
            ["robot_pose_known"],
        )

        self.assertEqual(result["steps"][0]["skill"], "remember_home")
        self.assertEqual(result["model"], "planner-test")
        self.assertEqual(
            result["model_input"]["messages"][0]["content"],
            "Recordá el inicio",
        )

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

    def test_reads_a_traced_step_review(self):
        def opener(request, timeout):
            request_id = request.headers["X-request-id"]
            self.assertTrue(request.full_url.endswith("/v1/review-step"))
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "review": {
                        "decision": "continue",
                        "reason": "El paso tuvo éxito.",
                        "revised_steps": [],
                        "question": None,
                    },
                    "raw_output": '{"decision":"continue"}',
                    "model_input": {"messages": []},
                    "model": "planner-test",
                    "elapsed_s": 0.4,
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        review = client.review_step(
            command="Recordá home",
            skill_catalog=[],
            world_facts=["home_saved"],
            completed_steps=[],
            last_step={"id": "remember_home"},
            outcome={"state": "succeeded", "message": "ok"},
            pending_steps=[],
            review_count=1,
        )

        self.assertEqual(review["decision"], "continue")
        self.assertEqual(review["model"], "planner-test")

    def test_attaches_one_declared_visual_evidence_to_review(self):
        jpeg = b"\xff\xd8" + b"visual" * 20 + b"\xff\xd9"

        def opener(request, timeout):
            sent = json.loads(request.data)
            evidence = sent["visual_evidence"]
            self.assertEqual(evidence["purpose"], "confirmar el reloj")
            self.assertEqual(evidence["detail"], "high")
            self.assertEqual(
                base64.b64decode(evidence["image_base64"]),
                jpeg,
            )
            self.assertNotIn("input_ref", evidence)
            request_id = request.headers["X-request-id"]
            return FakeResponse(
                {
                    "ok": True,
                    "request_id": request_id,
                    "review": {
                        "decision": "continue",
                        "reason": "La imagen coincide.",
                        "revised_steps": [],
                        "question": None,
                    },
                    "raw_output": '{"decision":"continue"}',
                    "model_input": {"messages": []},
                }
            )

        client = IntelligenceClient(
            server_url="http://server",
            opener=opener,
        )
        review = client.review_step(
            command="Mirá el reloj",
            skill_catalog=[],
            world_facts=["clock_confirmed"],
            completed_steps=[],
            last_step={"id": "look_at_clock"},
            outcome={"state": "succeeded", "message": "confirmado"},
            pending_steps=[],
            review_count=1,
            visual_evidence={
                "purpose": "confirmar el reloj",
                "image": jpeg,
                "input_ref": {
                    "topic": "/g1/perception/evidence/compressed",
                    "sec": 1,
                    "nanosec": 2,
                },
                "detail": "high",
            },
        )

        self.assertEqual(review["decision"], "continue")

    def test_rejects_truncated_visual_evidence_before_network(self):
        client = IntelligenceClient(
            server_url="http://server",
            opener=lambda *_args, **_kwargs: self.fail(
                "no debía abrir la red"
            ),
        )

        with self.assertRaisesRegex(
            RemoteIntelligenceError,
            "JPEG completo",
        ):
            client.review_step(
                command="Mirá",
                skill_catalog=[],
                world_facts=[],
                completed_steps=[],
                last_step={"id": "look"},
                outcome={"state": "succeeded", "message": "ok"},
                pending_steps=[],
                review_count=1,
                visual_evidence={
                    "purpose": "cuadro",
                    "image": b"\xff\xd8" + b"x" * 120,
                    "input_ref": {"sec": 1, "nanosec": 2},
                    "detail": "low",
                },
            )

    def test_rejects_continue_after_failed_step(self):
        with self.assertRaises(RemoteIntelligenceError):
            validate_review(
                {
                    "decision": "continue",
                    "reason": "Seguir.",
                    "revised_steps": [],
                    "question": None,
                },
                "failed",
            )

    def test_accepts_complete_only_after_terminal_success(self):
        review = {
            "decision": "complete",
            "reason": "La misión quedó cumplida.",
            "revised_steps": [],
            "question": None,
        }

        self.assertEqual(
            validate_review(
                review,
                "succeeded",
                pending_steps=[],
            ),
            review,
        )
        with self.assertRaisesRegex(
            RemoteIntelligenceError,
            "pasos pendientes",
        ):
            validate_review(
                review,
                "succeeded",
                pending_steps=[{"id": "pending"}],
            )

    def test_rejects_repair_without_the_missing_skill(self):
        with self.assertRaisesRegex(
            RemoteIntelligenceError,
            "skill faltante",
        ):
            validate_review(
                {
                    "decision": "retry",
                    "reason": "Intentar nuevamente.",
                    "revised_steps": [],
                    "question": None,
                },
                {
                    "state": "blocked",
                    "message": "falta barrer la habitación",
                    "blocker": {
                        "type": "missing_skill",
                        "skill": "scan_for_table",
                    },
                },
                [],
            )


if __name__ == "__main__":
    unittest.main()
