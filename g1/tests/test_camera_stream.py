import unittest
from types import SimpleNamespace

import numpy as np

from camera_stream import (
    SynchronizedCameraFrames,
    color_array,
    depth_array,
)


def message(stamp, encoding, array, row_padding=0):
    height, width = array.shape[:2]
    channels = 3 if encoding == "rgb8" else 1
    item_size = array.dtype.itemsize
    row_bytes = width * channels * item_size
    padded = np.zeros((height, row_bytes + row_padding), dtype=np.uint8)
    padded[:, :row_bytes] = array.view(np.uint8).reshape(height, row_bytes)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp, nanosec=stamp),
        ),
        encoding=encoding,
        height=height,
        width=width,
        step=row_bytes + row_padding,
        data=padded.tobytes(),
    )


class CameraStreamTests(unittest.TestCase):
    def test_decodes_rows_with_padding(self):
        color = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        depth = np.arange(6, dtype=np.float32).reshape(2, 3)
        np.testing.assert_array_equal(
            color_array(message(1, "rgb8", color, row_padding=2)),
            color,
        )
        np.testing.assert_array_equal(
            depth_array(message(1, "32FC1", depth, row_padding=4)),
            depth,
        )

    def test_requires_exact_stamp_and_evicts_old_frames(self):
        cache = SynchronizedCameraFrames(max_frames=1)
        color = np.zeros((1, 1, 3), dtype=np.uint8)
        depth = np.ones((1, 1), dtype=np.float32)
        first = message(1, "rgb8", color)
        cache.add("color", first)
        cache.add("depth", message(1, "32FC1", depth))
        self.assertIsNone(cache.complete(first))
        cache.add("info", message(1, "rgb8", color))
        self.assertIsNotNone(cache.complete(first))
        self.assertIs(cache.latest_complete(), cache.complete(first))
        second = message(2, "rgb8", color)
        cache.add("color", second)
        self.assertIsNone(cache.complete(first))


if __name__ == "__main__":
    unittest.main()
