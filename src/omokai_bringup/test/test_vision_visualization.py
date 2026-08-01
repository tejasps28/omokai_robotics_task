import unittest

import numpy as np

from omokai_bringup.vision_depth import depth_preview


class VisionVisualizationTest(unittest.TestCase):
    def test_depth_preview_marks_invalid_black_and_near_brighter(self) -> None:
        depth = np.array(
            [np.nan, np.inf, 0.12, 1.0, 4.0, 8.0, 9.0],
            dtype=np.float32,
        )

        preview = np.frombuffer(depth_preview(depth.tobytes()), dtype=np.uint8)

        self.assertEqual(0, preview[0])
        self.assertEqual(0, preview[1])
        self.assertEqual(255, preview[2])
        self.assertGreater(preview[3], preview[4])
        self.assertGreater(preview[4], preview[5])
        self.assertEqual(0, preview[6])


if __name__ == '__main__':
    unittest.main()
