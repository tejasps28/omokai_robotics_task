import tempfile
import unittest
from pathlib import Path

from omokai_bringup.test_zone_map import (
    FREE,
    OCCUPIED,
    RESOLUTION_M,
    generate_test_zone_map,
)
from omokai_fleet.scenario import DOCKING_POSES, ROOM_TARGETS


MODEL_DIRECTORY = Path(__file__).resolve().parents[1] / 'models' / 'test_zone'


class TestZoneMapTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.output = Path(self.temporary_directory.name)

    def test_generates_map_from_vendored_collision_mesh(self) -> None:
        yaml_path = Path(
            generate_test_zone_map(
                MODEL_DIRECTORY / 'model.sdf',
                self.output,
            )
        )
        metadata = yaml_path.read_text(encoding='utf-8')

        self.assertTrue((self.output / 'test_zone.pgm').is_file())
        self.assertIn(f'resolution: {RESOLUTION_M}', metadata)
        self.assertIn('mode: trinary', metadata)

    def test_dock_and_room_targets_are_free_and_walls_are_occupied(self) -> None:
        generate_test_zone_map(MODEL_DIRECTORY / 'model.sdf', self.output)
        width, height, pixels = self._read_image()
        origin_x, origin_y = -8.95, -9.1

        poses = DOCKING_POSES + tuple(point.pose for point in ROOM_TARGETS)
        for pose in poses:
            with self.subTest(pose=pose):
                self.assertEqual(
                    FREE,
                    self._pixel(
                        pixels,
                        width,
                        height,
                        origin_x,
                        origin_y,
                        pose.x,
                        pose.y,
                    ),
                )
        self.assertEqual(
            OCCUPIED,
            self._pixel(
                pixels,
                width,
                height,
                origin_x,
                origin_y,
                -6.2,
                4.0,
            ),
        )

    def test_vendored_asset_records_source_revision_and_license(self) -> None:
        source = (MODEL_DIRECTORY / 'SOURCE.md').read_text(encoding='utf-8')

        self.assertIn('cce115b82691b7c529a02f47e5efa391145a4ca1', source)
        self.assertTrue((MODEL_DIRECTORY / 'LICENSE.GPL-3.0').is_file())

    def test_missing_model_is_rejected(self) -> None:
        with self.assertRaises(FileNotFoundError):
            generate_test_zone_map(self.output / 'missing.sdf', self.output)

    def _read_image(self) -> tuple[int, int, bytes]:
        raw = (self.output / 'test_zone.pgm').read_bytes()
        header_end = raw.index(b'\n255\n') + len(b'\n255\n')
        header = raw[:header_end].decode('ascii').split()
        return int(header[1]), int(header[2]), raw[header_end:]

    @staticmethod
    def _pixel(
        pixels: bytes,
        width: int,
        height: int,
        origin_x: float,
        origin_y: float,
        world_x: float,
        world_y: float,
    ) -> int:
        cell_x = int((world_x - origin_x) / RESOLUTION_M)
        cell_y = int((world_y - origin_y) / RESOLUTION_M)
        image_row = height - 1 - cell_y
        return pixels[image_row * width + cell_x]


if __name__ == '__main__':
    unittest.main()
