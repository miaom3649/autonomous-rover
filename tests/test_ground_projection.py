import math
import unittest

from rover_navigation.ground_projection import pixel_to_ground


class GroundProjectionTest(unittest.TestCase):
    def test_downward_camera_center_hits_below_camera(self) -> None:
        point = pixel_to_ground(
            320, 240, fx=500, fy=500, cx=320, cy=240,
            camera_height_m=1.0, camera_pitch_down_deg=90.0,
        )
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point[0], 0.0, places=6)
        self.assertAlmostEqual(point[1], 0.0, places=6)

    def test_level_camera_pixel_below_horizon_hits_ground(self) -> None:
        point = pixel_to_ground(
            320, 340, fx=500, fy=500, cx=320, cy=240,
            camera_height_m=0.2,
        )
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point[0], 1.0, places=6)
        self.assertAlmostEqual(point[1], 0.0, places=6)

    def test_pixel_above_horizon_has_no_ground_intersection(self) -> None:
        point = pixel_to_ground(
            320, 140, fx=500, fy=500, cx=320, cy=240,
            camera_height_m=0.2,
        )
        self.assertIsNone(point)

    def test_yaw_rotates_result_left(self) -> None:
        point = pixel_to_ground(
            320, 340, fx=500, fy=500, cx=320, cy=240,
            camera_height_m=0.2, camera_yaw_left_deg=90.0,
        )
        self.assertAlmostEqual(point[0], 0.0, places=6)
        self.assertAlmostEqual(point[1], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
