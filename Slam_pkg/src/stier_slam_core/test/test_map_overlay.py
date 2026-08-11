import math
from pathlib import Path
import tempfile
import unittest

import yaml

from stier_slam_core.grid_map import (
    GridMap,
    MapFormatError,
    grid_map_from_pgm,
    load_ros_map,
    write_ros_map,
)
from stier_slam_core.map_overlay import OverlayValidationError, apply_overlay


class GridMapTest(unittest.TestCase):
    def test_map_to_cell_uses_inclusive_lower_and_exclusive_upper_bounds(self):
        """Rounding or accepting the upper edge would address the wrong ROS cell."""
        grid = GridMap(10, 10, 0.1, -0.2, 0.3, [-1] * 100)

        self.assertEqual(grid.map_to_cell(-0.2, 0.3), (0, 0))
        self.assertEqual(grid.map_to_cell(0.799999999, 1.299999999), (9, 9))
        for point in ((-0.200000001, 0.3), (-0.2, 0.299999999), (0.8, 0.3), (-0.2, 1.3)):
            with self.subTest(point=point), self.assertRaises(IndexError):
                grid.map_to_cell(*point)

    def test_map_to_cell_snaps_decimal_internal_boundary_to_next_cell(self):
        """Raw floor on 0.3 / 0.1 would incorrectly return the prior cell."""
        grid = GridMap(10, 10, 0.1, 0.0, 0.0, [-1] * 100)

        self.assertEqual(grid.map_to_cell(0.3, 0.3), (3, 3))

    def test_map_to_cell_rejects_decimal_exclusive_upper_boundary(self):
        """A rounded-up max extent must not make x == width * resolution valid."""
        grid = GridMap(3, 3, 0.1, 0.0, 0.0, [-1] * 9)

        with self.assertRaises(IndexError):
            grid.map_to_cell(0.3, 0.1)
        with self.assertRaises(IndexError):
            grid.map_to_cell(0.1, 0.3)

    def test_grid_rejects_invalid_dimensions_metadata_and_cells(self):
        """Malformed map metadata must not create an ambiguous ROS occupancy grid."""
        invalid_arguments = (
            (0, 1, 0.1, 0.0, 0.0, []),
            (1, True, 0.1, 0.0, 0.0, [-1]),
            (1, 1, 0.0, 0.0, 0.0, [-1]),
            (1, 1, math.nan, 0.0, 0.0, [-1]),
            (1, 1, 0.1, math.inf, 0.0, [-1]),
            (1, 1, 0.1, 10 ** 1000, 0.0, [-1]),
            (2, 1, 0.1, 0.0, 0.0, [-1]),
            (1, 1, 0.1, 0.0, 0.0, [50]),
        )
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                GridMap(*arguments)

    def test_map_to_cell_rejects_non_finite_coordinates(self):
        """NaN or infinite map coordinates must not leak into integer indexing."""
        grid = GridMap(1, 1, 0.1, 0.0, 0.0, [-1])

        for point in ((math.nan, 0.0), (0.0, math.inf)):
            with self.subTest(point=point), self.assertRaises(ValueError):
                grid.map_to_cell(*point)


class RosMapIoTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary_directory.name)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_grid_map_from_pgm_validates_direct_metadata_call(self):
        """Direct callers must not bypass the ROS map YAML schema validator."""
        document = {
            "image": "map.pgm",
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0.0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        with self.assertRaises(MapFormatError):
            grid_map_from_pgm(document, b"P5\n1 1\n255\n\xfe")

    def _write_yaml(self, image_name="map.pgm", extra=""):
        yaml_path = self.directory / "map.yaml"
        yaml_path.write_text(
            "image: {}\n"
            "resolution: 0.1\n"
            "origin: [-1.0, 2.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n{}".format(image_name, extra),
            encoding="utf-8",
        )
        return yaml_path

    def test_loads_literal_p2_and_p5_with_ros_vertical_row_conversion(self):
        """Reading image rows as ROS rows without a vertical flip would invert the map."""
        fixtures = (
            b"P2\n# literal trinary fixture\n3 2\n255\n0 205 254\n254 89 206\n",
            b"P5\n3 2\n255\n" + bytes((0, 205, 254, 254, 89, 206)),
        )
        for fixture in fixtures:
            with self.subTest(magic=fixture[:2]):
                (self.directory / "map.pgm").write_bytes(fixture)

                grid = load_ros_map(self._write_yaml())

                self.assertEqual((grid.width, grid.height), (3, 2))
                self.assertEqual((grid.resolution, grid.origin_x, grid.origin_y), (0.1, -1.0, 2.0))
                self.assertEqual(grid.cells, (0, 100, 0, 100, -1, 0))

    def test_threshold_boundary_pixels_remain_unknown(self):
        """Using inclusive threshold comparisons would misclassify trinary boundary pixels."""
        (self.directory / "map.pgm").write_bytes(b"P2\n2 1\n1000\n350 804\n")

        grid = load_ros_map(self._write_yaml())

        self.assertEqual(grid.cells, (-1, -1))

    def test_write_emits_literal_p5_map_server_pixels_and_yaml(self):
        """Wrong grayscale values or row order would change map_server occupancy on reload."""
        grid = GridMap(3, 2, 0.1, -1.0, 2.0, [0, 100, -1, -1, 100, 0])
        pgm_path = self.directory / "released.pgm"
        yaml_path = self.directory / "released.yaml"

        write_ros_map(grid, pgm_path, yaml_path)

        self.assertEqual(
            pgm_path.read_bytes(),
            b"P5\n3 2\n255\n" + bytes((205, 0, 254, 254, 0, 205)),
        )
        self.assertEqual(
            yaml.safe_load(yaml_path.read_text(encoding="utf-8")),
            {
                "image": "released.pgm",
                "resolution": 0.1,
                "origin": [-1.0, 2.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
        )
        self.assertEqual(load_ros_map(yaml_path), grid)

    def test_write_rejects_pgm_and_yaml_in_different_directories(self):
        """A basename-only YAML image reference cannot load a PGM from another parent."""
        grid = GridMap(1, 1, 0.1, 0.0, 0.0, [0])
        pgm_directory = self.directory / "images"
        yaml_directory = self.directory / "metadata"
        pgm_directory.mkdir()
        yaml_directory.mkdir()

        with self.assertRaises(ValueError):
            write_ros_map(
                grid,
                pgm_directory / "map.pgm",
                yaml_directory / "map.yaml",
            )

        self.assertEqual(list(pgm_directory.iterdir()), [])
        self.assertEqual(list(yaml_directory.iterdir()), [])

    def test_rejects_truncated_or_surplus_pgm_raster_data(self):
        """Accepting a partial or surplus raster would hide a corrupt map artifact."""
        fixtures = (
            b"P5\n2 1\n255\n\x00",
            b"P5\n2 1\n255\n\x00\xff\x7f",
            b"P2\n2 1\n255\n0\n",
            b"P2\n2 1\n255\n0 255 127\n",
        )
        for fixture in fixtures:
            with self.subTest(fixture=fixture):
                (self.directory / "map.pgm").write_bytes(fixture)
                with self.assertRaises(MapFormatError):
                    load_ros_map(self._write_yaml())

    def test_rejects_truncated_pgm_header_with_typed_format_error(self):
        """Leaking EOFError would bypass callers that safely handle corrupt map formats."""
        (self.directory / "map.pgm").write_bytes(b"P5\n1")

        with self.assertRaises(MapFormatError):
            load_ros_map(self._write_yaml())

    def test_rejects_duplicate_yaml_keys(self):
        """Last-value-wins YAML parsing would conceal ambiguous map metadata."""
        (self.directory / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
        yaml_path = self.directory / "map.yaml"
        yaml_path.write_text(
            "image: missing.pgm\nimage: map.pgm\nresolution: 0.1\n"
            "origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )

        with self.assertRaises(MapFormatError):
            load_ros_map(yaml_path)

    def test_rejects_unsupported_yaml_and_pgm_metadata(self):
        """Unsupported rotation, inversion, thresholds, or fields must fail closed."""
        (self.directory / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
        invalid_yaml_documents = (
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 1\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.7\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n",
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0.1]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: .nan\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: true\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: 0.1\norigin: [false, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0.0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            "image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n",
        )
        for document in invalid_yaml_documents:
            with self.subTest(document=document):
                yaml_path = self.directory / "map.yaml"
                yaml_path.write_text(document, encoding="utf-8")
                with self.assertRaises(MapFormatError):
                    load_ros_map(yaml_path)

        for pgm in (b"P6\n1 1\n255\n\x00\x00\x00", b"P5\n1 1\n0\n\x00", b"P5\n1 1\n65536\n\x00"):
            with self.subTest(pgm=pgm):
                (self.directory / "map.pgm").write_bytes(pgm)
                with self.assertRaises(MapFormatError):
                    load_ros_map(self._write_yaml())

    def test_rejects_image_path_traversal(self):
        """A map YAML image path must not escape its containing artifact directory."""
        outside = self.directory.parent / "outside.pgm"
        outside.write_bytes(b"P5\n1 1\n255\n\xff")
        try:
            with self.assertRaises(MapFormatError):
                load_ros_map(self._write_yaml("../outside.pgm"))
        finally:
            outside.unlink()


class MapOverlayTest(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap(10, 10, 0.1, 0.0, 0.0, [-1] * 100)

    def test_ordered_free_polygon_overrides_occupied_without_mutating_source(self):
        """Ignoring operation order or editing in place would corrupt the workspace source."""
        source_cells = self.grid.cells
        overlay = {
            "schema_version": 1,
            "operations": [
                {
                    "operation": "occupied",
                    "polygon": [[0.2, 0.2], [0.5, 0.2], [0.5, 0.5], [0.2, 0.5]],
                },
                {
                    "operation": "free",
                    "polygon": [[0.3, 0.3], [0.5, 0.3], [0.5, 0.5], [0.3, 0.5]],
                },
            ],
        }

        edited = apply_overlay(self.grid, overlay)

        self.assertIsNot(edited, self.grid)
        self.assertEqual(self.grid.cells, source_cells)
        self.assertEqual(edited.cell(2, 2), 100)
        self.assertEqual(edited.cell(3, 3), 0)
        self.assertEqual(edited.cell(4, 4), 0)
        self.assertEqual(edited.cell(5, 5), -1)

    def test_polygon_on_closed_map_extent_edits_last_cell(self):
        """Rejecting the geometric upper edge would prevent editing the last map cell."""
        overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "occupied",
                "polygon": [[0.9, 0.9], [1.0, 0.9], [1.0, 1.0], [0.9, 1.0]],
            }],
        }

        edited = apply_overlay(self.grid, overlay)

        self.assertEqual(edited.cell(9, 9), 100)
        self.assertEqual(sum(value == 100 for value in edited.cells), 1)

    def test_rejects_unknown_schema_fields_and_operations(self):
        """Silently accepting schema drift would produce editor/release disagreement."""
        invalid_documents = (
            {"schema_version": 2, "operations": []},
            {"schema_version": 1.0, "operations": []},
            {"schema_version": 1, "operations": [], "extra": True},
            {"schema_version": 1, "operations": "occupied"},
            {"schema_version": 1, "operations": [{"operation": "erase", "polygon": [[0, 0], [1, 0], [0, 1]]}]},
            {"schema_version": 1, "operations": [{"operation": "free", "polygon": [[0, 0], [1, 0], [0, 1]], "extra": 1}]},
        )
        for document in invalid_documents:
            with self.subTest(document=document), self.assertRaises(OverlayValidationError):
                apply_overlay(self.grid, document)

    def test_rejects_too_small_degenerate_and_self_intersecting_polygons(self):
        """Invalid polygon topology must not rasterize accidental occupied cells."""
        invalid_polygons = (
            [[0.1, 0.1], [0.2, 0.1]],
            [[0.1, 0.1], [0.2, 0.1], [0.3, 0.1]],
            [[0.1, 0.1], [0.4, 0.4], [0.1, 0.4], [0.4, 0.1]],
        )
        for polygon in invalid_polygons:
            document = {"schema_version": 1, "operations": [{"operation": "occupied", "polygon": polygon}]}
            with self.subTest(polygon=polygon), self.assertRaises(OverlayValidationError):
                apply_overlay(self.grid, document)

    def test_rejects_non_finite_malformed_and_out_of_bounds_coordinates(self):
        """Invalid map-meter coordinates must fail before any cell is changed."""
        invalid_polygons = (
            [[0.1, 0.1], [math.nan, 0.1], [0.1, 0.2]],
            [[0.1, 0.1], [0.2], [0.1, 0.2]],
            [[True, 0.1], [0.2, 0.1], [0.1, 0.2]],
            [[10 ** 1000, 0.1], [0.2, 0.1], [0.1, 0.2]],
            [[-0.01, 0.1], [0.2, 0.1], [0.1, 0.2]],
            [[0.9, 0.9], [1.01, 0.9], [0.9, 1.0]],
        )
        for polygon in invalid_polygons:
            document = {"schema_version": 1, "operations": [{"operation": "occupied", "polygon": polygon}]}
            with self.subTest(polygon=polygon), self.assertRaises(OverlayValidationError):
                apply_overlay(self.grid, document)


if __name__ == "__main__":
    unittest.main()
