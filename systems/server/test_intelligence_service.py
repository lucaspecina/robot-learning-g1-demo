import base64
import json
import unittest
from types import SimpleNamespace

from systems.server.intelligence_service import (
    ClockReader,
    IntelligenceService,
    InvalidImageError,
    InvalidModelResponseError,
    MissionPlanner,
    decode_image,
    validate_generated_plan,
    validate_generated_review,
    validate_reading,
)
from systems.server.open_vocabulary_detector import (
    InvalidDetectionRequestError,
    bounded_coordinates,
    validate_labels,
)


class FakeCompletions:
    def __init__(self, payload):
        self.payload = payload
        self.last_kwargs = None

    def create(self, **kwargs):
        self.last_kwargs = kwargs
        return SimpleNamespace(
            choices=[
                SimpleNamespace(
                    message=SimpleNamespace(
                        content=json.dumps(self.payload)
                    )
                )
            ]
        )


class FakeClient:
    def __init__(self, payload):
        self.chat = SimpleNamespace(
            completions=FakeCompletions(payload)
        )


class FakeObjectDetector:
    def __init__(self):
        self.last_image = None
        self.last_labels = None

    def detect(self, image, labels):
        self.last_image = image
        self.last_labels = labels
        return {
            "detections": [
                {
                    "label": "a red table",
                    "confidence": 0.62,
                    "box": [10.0, 20.0, 100.0, 120.0],
                }
            ],
            "image_width": 640,
            "image_height": 480,
            "model": "test-detector",
            "inference_s": 1.2,
        }


class IntelligenceServiceTests(unittest.TestCase):
    def test_mission_planner_receives_skill_meaning_and_returns_literal_text(self):
        catalog = [
            {
                "name": "remember_home",
                "description": "Guarda la pose actual para regresar.",
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
        payload = {
            "steps": [
                {
                    "id": "remember_home",
                    "skill": "remember_home",
                    "argument": None,
                    "label": "Guardar el punto de inicio",
                }
            ]
        }
        fake_client = FakeClient(payload)
        planner = MissionPlanner(
            client=fake_client,
            deployment="planner-test",
        )

        result = planner.plan(
            "Recordá dónde empezaste",
            catalog,
            ["robot_pose_known"],
        )

        self.assertEqual(result["plan"], payload)
        self.assertEqual(json.loads(result["raw_output"]), payload)
        self.assertEqual(result["model"], "planner-test")
        request = fake_client.chat.completions.last_kwargs
        self.assertIn(
            "Guarda la pose actual para regresar.",
            request["messages"][0]["content"],
        )
        self.assertEqual(
            request["response_format"]["type"],
            "json_schema",
        )
        self.assertEqual(result["model_input"]["messages"], request["messages"])

    def test_step_reviewer_receives_measurements_and_continues(self):
        catalog = [
            {
                "name": "remember_home",
                "description": "Guarda la pose actual para regresar.",
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
        payload = {
            "decision": "continue",
            "reason": "El paso terminó correctamente.",
            "revised_steps": [],
            "question": None,
        }
        fake_client = FakeClient(payload)
        planner = MissionPlanner(
            client=fake_client,
            deployment="planner-test",
        )
        outcome = {
            "state": "succeeded",
            "message": "home guardado",
            "measurements": {"x": 0.0, "y": 0.0},
        }

        result = planner.review(
            "Recordá dónde empezaste",
            catalog,
            ["robot_pose_known", "home_saved"],
            [{"id": "remember_home", "state": "succeeded"}],
            {
                "id": "remember_home",
                "skill": "remember_home",
                "argument": None,
                "label": "Guardar home",
            },
            outcome,
            [],
            1,
        )

        self.assertEqual(result["review"], payload)
        request = fake_client.chat.completions.last_kwargs
        self.assertIn(
            '"measurements": {',
            request["messages"][1]["content"],
        )
        self.assertEqual(
            request["response_format"]["json_schema"]["name"],
            "robot_step_review",
        )

    def test_reviewer_rejects_continue_after_failure(self):
        with self.assertRaisesRegex(
            InvalidModelResponseError,
            "continuar",
        ):
            validate_generated_review(
                {
                    "decision": "continue",
                    "reason": "Seguir.",
                    "revised_steps": [],
                    "question": None,
                },
                [],
                [],
                "failed",
            )

    def test_external_validator_rejects_missing_preconditions(self):
        catalog = [
            {
                "name": "read_clock",
                "description": "Lee el reloj.",
                "availability": "ready",
                "variants": [
                    {
                        "argument": None,
                        "argument_description": "Sin argumento.",
                        "preconditions": ["clock_confirmed"],
                        "effects": ["clock_reading_known"],
                    }
                ],
            }
        ]

        with self.assertRaisesRegex(
            InvalidModelResponseError,
            "clock_confirmed",
        ):
            validate_generated_plan(
                {
                    "steps": [
                        {
                            "id": "read_clock",
                            "skill": "read_clock",
                            "argument": None,
                            "label": "Leer la hora",
                        }
                    ]
                },
                catalog,
                [],
            )

    def test_validates_consistent_reading(self):
        reading = {
            "readable": True,
            "hour": 9,
            "minute": 0,
            "text": "09:00",
        }
        self.assertEqual(validate_reading(reading), reading)

    def test_rejects_inconsistent_text(self):
        with self.assertRaises(InvalidModelResponseError):
            validate_reading(
                {
                    "readable": True,
                    "hour": 15,
                    "minute": 0,
                    "text": "05:00",
                }
            )

    def test_rejects_invalid_image(self):
        with self.assertRaises(InvalidImageError):
            decode_image("no-es-base64")

    def test_rejects_non_jpeg_bytes(self):
        encoded = base64.b64encode(b"x" * 200).decode("ascii")
        with self.assertRaises(InvalidImageError):
            decode_image(encoded)

    def test_clock_reader_sends_image_and_schema(self):
        payload = {
            "readable": True,
            "hour": 15,
            "minute": 30,
            "text": "15:30",
        }
        fake_client = FakeClient(payload)
        reader = ClockReader(
            client=fake_client,
            deployment="vision-test",
        )

        result = reader.read(b"\xff\xd8" + b"x" * 200 + b"\xff\xd9")

        self.assertEqual(result["reading"], payload)
        self.assertEqual(
            json.loads(result["raw_output"]),
            payload,
        )
        self.assertEqual(result["model"], "vision-test")
        request = fake_client.chat.completions.last_kwargs
        self.assertEqual(request["model"], "vision-test")
        self.assertEqual(
            request["response_format"]["type"],
            "json_schema",
        )
        image_url = request["messages"][1]["content"][1]["image_url"]["url"]
        encoded = image_url.split(",", 1)[1]
        self.assertTrue(base64.b64decode(encoded).startswith(b"\xff\xd8"))

    def test_validates_open_vocabulary_labels(self):
        self.assertEqual(
            validate_labels(["  A red   table ", "a blue table"]),
            ["a red table", "a blue table"],
        )
        with self.assertRaises(InvalidDetectionRequestError):
            validate_labels([])

    def test_clips_detection_box_to_the_image(self):
        self.assertEqual(
            bounded_coordinates([-5, 10, 700, 500], 640, 480),
            [0.0, 10.0, 640.0, 480.0],
        )

    def test_object_detection_is_a_separate_service(self):
        detector = FakeObjectDetector()
        service = IntelligenceService(
            clock_reader=object(),
            object_detector=detector,
        )
        result = service.detect_objects(b"jpeg", ["a red table"])

        self.assertTrue(result["ok"])
        self.assertEqual(result["detections"][0]["label"], "a red table")
        self.assertEqual(detector.last_labels, ["a red table"])


if __name__ == "__main__":
    unittest.main()
