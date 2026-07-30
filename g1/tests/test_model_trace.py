"""Pruebas de que el tablero recibe la realidad devuelta por los modelos."""
import unittest

from model_trace import build_model_event


class ModelTraceTest(unittest.TestCase):
    def test_preserves_literal_output_byte_for_byte_as_text(self):
        raw_output = '{\n  "text": "09:00", "explicación": "visible"\n}'
        event = build_model_event(
            event_id="event-1",
            task="read_clock",
            state="succeeded",
            input_summary="recorte JPEG del reloj",
            input_ref={
                "topic": "/g1/clock_crop/compressed",
                "sec": 12,
                "nanosec": 34,
            },
            model="vision-test",
            raw_output=raw_output,
            validated_output={"text": "09:00"},
        )

        self.assertEqual(event["raw_output"], raw_output)
        self.assertEqual(event["validated_output"], {"text": "09:00"})
        self.assertEqual(event["input_ref"]["nanosec"], 34)

    def test_does_not_present_a_success_without_raw_output(self):
        with self.assertRaises(ValueError):
            build_model_event(
                task="plan_mission",
                state="succeeded",
                input_summary="misión y catálogo de skills",
            )


if __name__ == "__main__":
    unittest.main()
