import unittest
from types import SimpleNamespace

from visual_evidence import (
    VISUAL_EVIDENCE_TOPIC,
    VisualEvidence,
    VisualEvidenceCache,
    image_ref,
    image_ref_key,
    is_complete_jpeg,
    validate_jpeg,
)


def jpeg_bytes(size=120):
    return b"\xff\xd8" + b"x" * (size - 4) + b"\xff\xd9"


class VisualEvidenceTests(unittest.TestCase):
    def test_reference_keeps_topic_and_sensor_time(self):
        header = SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=345),
        )

        reference = image_ref(VISUAL_EVIDENCE_TOPIC, header)

        self.assertEqual(reference["topic"], VISUAL_EVIDENCE_TOPIC)
        self.assertEqual(
            image_ref_key(reference),
            (VISUAL_EVIDENCE_TOPIC, 12, 345),
        )

    def test_rejects_incomplete_reference(self):
        with self.assertRaises(ValueError):
            image_ref_key({"sec": 1, "nanosec": 2})

    def test_only_accepts_complete_jpeg(self):
        self.assertTrue(
            is_complete_jpeg(jpeg_bytes())
        )
        self.assertFalse(is_complete_jpeg(b"\xff\xd8cortado"))

    def make_evidence(self, sec=4, received_at=10.0):
        return VisualEvidence(
            jpeg=jpeg_bytes(),
            sec=sec,
            nanosec=25,
            received_at=received_at,
            source_topic=VISUAL_EVIDENCE_TOPIC,
            kind="scene",
            description="cuadro exacto de la detección",
        )

    def test_rejects_truncated_jpeg(self):
        with self.assertRaisesRegex(ValueError, "JPEG completo"):
            validate_jpeg(b"\xff\xd8" + b"x" * 120)

    def test_returns_exact_recent_evidence(self):
        cache = VisualEvidenceCache(max_items=2)
        evidence = self.make_evidence()
        cache.add(evidence)

        result = cache.get(
            {
                "topic": VISUAL_EVIDENCE_TOPIC,
                "sec": 4,
                "nanosec": 25,
            },
            now=11.5,
            max_age_s=2.0,
        )

        self.assertIs(result, evidence)
        self.assertEqual(result.reference()["bytes"], 120)

    def test_never_returns_stale_evidence(self):
        cache = VisualEvidenceCache()
        cache.add(self.make_evidence(received_at=10.0))

        self.assertIsNone(
            cache.get(
                {
                    "topic": VISUAL_EVIDENCE_TOPIC,
                    "sec": 4,
                    "nanosec": 25,
                },
                now=12.1,
                max_age_s=2.0,
            )
        )

    def test_never_matches_another_topic_with_same_timestamp(self):
        cache = VisualEvidenceCache()
        cache.add(self.make_evidence())

        self.assertIsNone(
            cache.get(
                {"topic": "/other/image", "sec": 4, "nanosec": 25},
                now=10.1,
                max_age_s=2.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
