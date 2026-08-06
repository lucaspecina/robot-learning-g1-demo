#!/usr/bin/env python3
"""Pruebas del vigilante común de capacidades."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from execution_core import ExecutionState, FeedbackWatchdog


class FeedbackWatchdogTest(unittest.TestCase):
    def test_feedback_keeps_execution_alive(self):
        watchdog = FeedbackWatchdog(deadline_s=30.0, silence_timeout_s=5.0)
        watchdog.start(10.0)
        watchdog.record_feedback(14.0)
        self.assertIsNone(watchdog.check(18.0))

    def test_missing_feedback_reports_unresponsive(self):
        watchdog = FeedbackWatchdog(deadline_s=30.0, silence_timeout_s=5.0)
        watchdog.start(10.0)
        decision = watchdog.check(15.1)
        self.assertEqual(decision.state, ExecutionState.UNRESPONSIVE)

    def test_total_deadline_has_priority(self):
        watchdog = FeedbackWatchdog(deadline_s=20.0, silence_timeout_s=8.0)
        watchdog.start(10.0)
        watchdog.record_feedback(29.5)
        decision = watchdog.check(30.1)
        self.assertEqual(decision.state, ExecutionState.TIMED_OUT)


if __name__ == "__main__":
    unittest.main()
