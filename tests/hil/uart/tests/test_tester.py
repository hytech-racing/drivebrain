"""Coordinator validation/report checks; no SSH or serial hardware required."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "tester"))
spec = importlib.util.spec_from_file_location("uart_bench", root / "tester/uart_bench.py")
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


class TesterTest(unittest.TestCase):
    def test_live_display_supports_terminal_and_redirected_output(self):
        from live_display import LiveDisplay
        class Output(io.StringIO):
            def __init__(self, tty):
                super().__init__()
                self.tty = tty
            def isatty(self):
                return self.tty
        stats = {"elapsed": 1, "verified": 2, "accepted": 2, "tx_frames": 2, "rtt_p99_ms": 1.25}
        for tty in (True, False):
            out = Output(tty)
            display = LiveDisplay(out)
            display.update("sample", 2, stats, {}, 2)
            display.update("sample", 2, stats, {}, 2)
            self.assertIn("verified" if not tty else "Verified", out.getvalue())
            self.assertEqual("\x1b[15A" in out.getvalue(), tty)

    def test_pi_network_errors_offer_ethernet_recovery(self):
        from pi_connection import guard_pi_connection, PiConnectionError
        for error in (TimeoutError("timed out"), ConnectionResetError("reset")):
            @guard_pi_connection
            def fail():
                raise error
            with self.subTest(error=error), self.assertRaisesRegex(PiConnectionError, "reconnect Ethernet"):
                fail()

    def test_unrelated_errors_are_not_reported_as_network_errors(self):
        from pi_connection import guard_pi_connection
        @guard_pi_connection
        def fail():
            raise PermissionError("access denied")
        with self.assertRaises(PermissionError):
            fail()

    def test_default_duration(self):
        profile = bench.validate_profile(json.loads((root / "profiles/default.json").read_text()))
        self.assertEqual(sum(p["seconds"] for p in profile["phases"]), 400)
        self.assertEqual(bench.DB.name, "drivebrain")

    def test_invalid_profiles(self):
        for field, value in (("rate", 0), ("seconds", float("nan")), ("pattern_bytes", 1025), ("cpu_workers", 5)):
            phase = {"name": "case", "kind": "command", "rate": 250, "seconds": 1, field: value}
            with self.subTest(field=field), self.assertRaises(ValueError):
                bench.validate_profile({"phases": [phase]})

    def test_normal_correctness_is_strict(self):
        phase = {"kind": "command"}
        pi = {"event": "result", "errors": 0, "verified": 10}
        firmware = {"commands": 10}
        self.assertEqual(bench.phase_errors(phase, pi, firmware), [])
        self.assertTrue(bench.phase_errors(phase, {**pi, "errors": 1}, firmware))
        self.assertTrue(bench.phase_errors(phase, pi, {**firmware, "crc_errors": 1}))
        self.assertTrue(bench.phase_errors(phase, pi, {"commands": 9}))

    def test_faults_require_observed_rejections(self):
        phase = {"kind": "faults"}
        pi = {"event": "result", "errors": 0, "verified": 10, "injected": 12}
        firmware = {k: 1 for k in ("cobs_errors", "crc_errors", "oversize_errors", "protobuf_errors", "invalid_commands")}
        self.assertEqual(bench.phase_errors(phase, pi, firmware), [])
        self.assertTrue(bench.phase_errors(phase, pi, {**firmware, "crc_errors": 0}))

    def test_overload_requires_backpressure(self):
        phase = {"kind": "overload"}
        pi = {"event": "result", "errors": 0, "verified": 10, "queue_rejected": 4}
        self.assertEqual(bench.phase_errors(phase, pi, {}), [])
        self.assertTrue(bench.phase_errors(phase, {**pi, "queue_rejected": 0}, {}))

    def test_partial_report_is_saved_and_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            report = bench.Report(Path(temp) / "report", {"seed": 1})
            report.event("test", {"event": "startup"})
            report.data["errors"].append("connection lost")
            report.save()
            report.close()
            result = json.loads((report.output / "report.json").read_text())
            self.assertFalse(result["passed"])
            self.assertFalse(result["complete"])
            self.assertIn("connection lost", (report.output / "report.md").read_text())

    def test_schema_mismatch_rejected(self):
        with self.assertRaises(RuntimeError):
            bench.identity_check({"version": 1, "schema_sha256": "wrong"}, "expected")


if __name__ == "__main__":
    unittest.main()
