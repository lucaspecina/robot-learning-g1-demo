"""Pruebas de las métricas que cuantifican el balanceo del torso."""
import math
import unittest

from motion_quality import motion_quality_metrics


class MotionQualityTest(unittest.TestCase):
    def test_constant_pose_has_no_sway_or_bounce(self):
        samples = [
            {
                "roll_rad": 0.1,
                "pitch_rad": -0.2,
                "height_m": 0.74,
                "angular_x_radps": 0.3,
                "angular_y_radps": 0.4,
            }
            for _ in range(10)
        ]

        metrics = motion_quality_metrics(samples)

        self.assertAlmostEqual(metrics["roll_p90_span_deg"], 0.0)
        self.assertAlmostEqual(metrics["pitch_p90_span_deg"], 0.0)
        self.assertAlmostEqual(metrics["height_p90_span_m"], 0.0)
        self.assertAlmostEqual(metrics["tilt_p95_deg"], math.degrees(0.2))
        self.assertAlmostEqual(metrics["angular_speed_rms_radps"], 0.5)

    def test_reports_central_range_instead_of_only_extremes(self):
        samples = []
        for value in range(-10, 11):
            samples.append({
                "roll_rad": value / 100.0,
                "pitch_rad": value / 200.0,
                "height_m": 0.74 + value / 1000.0,
                "angular_x_radps": 0.0,
                "angular_y_radps": 0.0,
            })

        metrics = motion_quality_metrics(samples)

        self.assertAlmostEqual(
            metrics["roll_p90_span_deg"],
            math.degrees(0.18),
        )
        self.assertAlmostEqual(metrics["height_p90_span_m"], 0.018)


if __name__ == "__main__":
    unittest.main()
