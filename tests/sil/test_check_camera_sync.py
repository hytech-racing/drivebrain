"""Regression checks using actual protobuf MCAPs; requires requirements.txt."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from mcap_protobuf.writer import Writer


class CameraSyncRecordingTest(unittest.TestCase):
    def check_recording(self, left, right):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cameras.mcap"
            with Writer(str(path)) as writer:
                for camera, timestamps in (("left", left), ("right", right)):
                    for timestamp in timestamps:
                        image = RawImage()
                        image.timestamp.seconds, image.timestamp.nanos = divmod(timestamp, 1_000_000_000)
                        # Deliberately unrelated host envelope times. Only capture
                        # timestamps can establish alignment between these frames.
                        writer.write_message(f"blackfly_{camera}/perception", image,
                                             log_time=123456789, publish_time=123456789)
            script = Path(__file__).resolve().parents[2] / "scripts/check_camera_sync.py"
            return subprocess.run([sys.executable, str(script), str(path)],
                                  capture_output=True, text=True)

    def test_aligned_captures_with_different_arrival_times(self):
        result = self.check_recording([1_000_000_000, 1_066_666_667],
                                      [1_000_050_000, 1_066_716_667])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Matched pairs: 2; unmatched frames: 0", result.stdout)
        self.assertIn("max 50.0 us", result.stdout)

    def test_missing_frame(self):
        result = self.check_recording([1_000_000_000, 1_066_666_667], [1_000_000_000])
        self.assertEqual(result.returncode, 1)
        self.assertIn("unmatched frames: 1", result.stdout)

    def test_free_running_cameras_fail(self):
        result = self.check_recording([1_000_000_000], [1_010_000_000])
        self.assertEqual(result.returncode, 1)
        self.assertIn("Matched pairs: 0", result.stdout)

    def test_duplicate_frame_is_not_reused(self):
        result = self.check_recording([1_000_000_000, 1_000_000_000], [1_000_000_000])
        self.assertEqual(result.returncode, 1)
        self.assertIn("Matched pairs: 1; unmatched frames: 1", result.stdout)

    def test_missing_camera_topic(self):
        result = self.check_recording([1_000_000_000], [])
        self.assertEqual(result.returncode, 1)
        self.assertIn("Expected two camera", result.stderr)


if __name__ == "__main__":
    unittest.main()
