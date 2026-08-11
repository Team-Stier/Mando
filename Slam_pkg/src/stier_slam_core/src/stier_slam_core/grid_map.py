"""ROS runtime import 없이 ROS map-server PGM/YAML을 엄격하게 변환한다."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile

import yaml


OCCUPIED_THRESHOLD = 0.65
FREE_THRESHOLD = 0.196
_CELL_TO_PIXEL = {-1: 205, 0: 254, 100: 0}
# cell 좌표가 이 값보다 정수에 가까우면 해당 경계값으로 처리한다.
# 한 cell보다 9자리 작아 binary float 반올림 오차만 흡수한다.
_CELL_BOUNDARY_TOLERANCE = 1.0e-9
_YAML_KEYS = {
    "image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"
}


class _UniqueKeyLoader(yaml.SafeLoader):
    """역할:
    ROS map YAML을 중복 key 없이 안전하게 읽는 loader를 제공한다.

    동작 방식:
    ``yaml.SafeLoader``의 mapping 생성을 확장해 같은 key가 두 번 나오면 모호한 값을
    선택하지 않고 즉시 ``ConstructorError``를 발생시킨다.
    """


def _construct_unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as error:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark,
                "found an unhashable key", key_node.start_mark,
            ) from error
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark,
                "found duplicate key {!r}".format(key), key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


class MapFormatError(ValueError):
    """지원하는 ROS 계약으로 map을 해석할 수 없을 때 발생한다."""


@dataclass(frozen=True)
class GridMap:
    """역할:
    ROS 2D occupancy grid의 크기, 해상도, 원점과 cell 값을 불변 형태로 표현한다.

    동작 방식:
    bottom-row-first row-major 순서의 ``-1/0/100`` cell만 허용하고, map frame의 미터
    좌표를 bounded grid index로 변환하거나 지정 cell 값을 조회한다.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    cells: tuple

    def __init__(self, width, height, resolution, origin_x, origin_y, cells):
        if (isinstance(width, bool) or not isinstance(width, int) or width <= 0
                or isinstance(height, bool) or not isinstance(height, int) or height <= 0):
            raise ValueError("width and height must be positive integers")
        resolution = _finite_number(resolution, "resolution")
        origin_x = _finite_number(origin_x, "origin_x")
        origin_y = _finite_number(origin_y, "origin_y")
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        try:
            immutable_cells = tuple(cells)
        except TypeError as error:
            raise ValueError("cells must be an iterable") from error
        if len(immutable_cells) != width * height:
            raise ValueError("cell count does not match width times height")
        if any(value not in (-1, 0, 100) or isinstance(value, bool)
               for value in immutable_cells):
            raise ValueError("cells must contain only -1, 0, or 100")
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)
        object.__setattr__(self, "resolution", resolution)
        object.__setattr__(self, "origin_x", origin_x)
        object.__setattr__(self, "origin_y", origin_y)
        object.__setattr__(self, "cells", immutable_cells)

    @property
    def max_x(self):
        return self.origin_x + self.width * self.resolution

    @property
    def max_y(self):
        return self.origin_y + self.height * self.resolution

    def map_to_cell(self, x, y):
        """map frame의 미터 좌표를 범위가 제한된 ``(column, row)`` cell로 변환한다."""
        x = _finite_number(x, "x")
        y = _finite_number(y, "y")
        normalized_x = _normalized_cell_coordinate(x, self.origin_x, self.resolution)
        normalized_y = _normalized_cell_coordinate(y, self.origin_y, self.resolution)
        if (normalized_x < 0.0 or normalized_x >= self.width
                or normalized_y < 0.0 or normalized_y >= self.height):
            raise IndexError("map coordinate is outside the grid")
        column = int(math.floor(normalized_x))
        row = int(math.floor(normalized_y))
        return column, row

    def cell(self, column, row):
        """엄격한 정수 범위 검사 후 cell 값을 반환한다."""
        if (isinstance(column, bool) or not isinstance(column, int)
                or isinstance(row, bool) or not isinstance(row, int)):
            raise TypeError("column and row must be integers")
        if column < 0 or column >= self.width or row < 0 or row >= self.height:
            raise IndexError("cell is outside the grid")
        return self.cells[row * self.width + column]


def load_ros_map(yaml_path):
    """ROS map-server map 중 지원하는 평면 trinary subset을 불러온다."""
    yaml_path = Path(yaml_path)
    document = _load_map_yaml(yaml_path)
    image_path = _resolve_image_path(yaml_path, document["image"])
    try:
        image_bytes = image_path.read_bytes()
    except OSError as error:
        raise MapFormatError("unable to read map image") from error
    return grid_map_from_pgm(document, image_bytes)


def parse_ros_map_yaml(yaml_bytes):
    """정확한 YAML byte snapshot 하나를 지원되는 map metadata로 해석한다."""
    if not isinstance(yaml_bytes, bytes):
        raise TypeError("yaml_bytes must be bytes")
    try:
        document = yaml.load(yaml_bytes.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeError, yaml.YAMLError) as error:
        raise MapFormatError("unable to parse map YAML") from error
    return _validate_map_yaml_document(document)


def grid_map_from_pgm(document, image_bytes):
    """검증된 YAML metadata와 정확한 PGM snapshot 하나로 grid를 만든다."""
    document = _validate_map_yaml_document(document)
    if not isinstance(image_bytes, bytes):
        raise TypeError("image_bytes must be bytes")
    width, height, max_value, samples = _parse_pgm(image_bytes)
    cells = []
    for image_row in range(height - 1, -1, -1):
        offset = image_row * width
        for sample in samples[offset:offset + width]:
            occupied_probability = (max_value - sample) / float(max_value)
            if occupied_probability > OCCUPIED_THRESHOLD:
                cells.append(100)
            elif occupied_probability < FREE_THRESHOLD:
                cells.append(0)
            else:
                cells.append(-1)
    return GridMap(
        width,
        height,
        document["resolution"],
        document["origin"][0],
        document["origin"][1],
        cells,
    )


def write_ros_map(grid, pgm_path, yaml_path):
    """map_saver 호환 P5 pixel과 평면 trinary YAML을 기록한다."""
    if not isinstance(grid, GridMap):
        raise TypeError("grid must be a GridMap")
    pgm_path = Path(pgm_path)
    yaml_path = Path(yaml_path)
    if pgm_path.resolve() == yaml_path.resolve():
        raise ValueError("PGM and YAML paths must differ")
    if pgm_path.parent.resolve() != yaml_path.parent.resolve():
        raise ValueError("PGM and YAML paths must share the same directory")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    raster = bytearray()
    for row in range(grid.height - 1, -1, -1):
        offset = row * grid.width
        raster.extend(_CELL_TO_PIXEL[value] for value in grid.cells[offset:offset + grid.width])
    pgm = "P5\n{} {}\n255\n".format(grid.width, grid.height).encode("ascii") + bytes(raster)
    yaml_text = (
        "image: {}\n"
        "resolution: {}\n"
        "origin: [{}, {}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    ).format(
        json.dumps(pgm_path.name),
        repr(grid.resolution),
        repr(grid.origin_x),
        repr(grid.origin_y),
    ).encode("utf-8")
    _atomic_write_bytes(pgm_path, pgm)
    _atomic_write_bytes(yaml_path, yaml_text)


def _load_map_yaml(yaml_path):
    try:
        yaml_bytes = yaml_path.read_bytes()
    except OSError as error:
        raise MapFormatError("unable to read map YAML") from error
    return parse_ros_map_yaml(yaml_bytes)


def _validate_map_yaml_document(document):
    if not isinstance(document, dict) or set(document) != _YAML_KEYS:
        raise MapFormatError("map YAML fields do not match the supported schema")
    image = document["image"]
    if not isinstance(image, str) or not image or "\x00" in image:
        raise MapFormatError("image must be a non-empty path string")
    try:
        resolution = _finite_number(document["resolution"], "resolution")
    except ValueError as error:
        raise MapFormatError(str(error)) from error
    if resolution <= 0.0:
        raise MapFormatError("resolution must be positive")
    origin = document["origin"]
    if not isinstance(origin, list) or len(origin) != 3:
        raise MapFormatError("origin must contain x, y, and yaw")
    try:
        origin = [_finite_number(value, "origin") for value in origin]
    except ValueError as error:
        raise MapFormatError(str(error)) from error
    if origin[2] != 0.0:
        raise MapFormatError("rotated map origins are unsupported")
    if type(document["negate"]) is not int or document["negate"] != 0:
        raise MapFormatError("only negate 0 is supported")
    if not _exact_number(document["occupied_thresh"], OCCUPIED_THRESHOLD):
        raise MapFormatError("only occupied_thresh 0.65 is supported")
    if not _exact_number(document["free_thresh"], FREE_THRESHOLD):
        raise MapFormatError("only free_thresh 0.196 is supported")
    return {
        "image": image,
        "resolution": resolution,
        "origin": origin,
        "negate": 0,
        "occupied_thresh": OCCUPIED_THRESHOLD,
        "free_thresh": FREE_THRESHOLD,
    }


def _resolve_image_path(yaml_path, image):
    relative = Path(image)
    if relative.is_absolute() or any(part == ".." for part in relative.parts):
        raise MapFormatError("image path must remain below the YAML directory")
    parent = yaml_path.parent.resolve()
    resolved = (parent / relative).resolve()
    try:
        resolved.relative_to(parent)
    except ValueError as error:
        raise MapFormatError("image path must remain below the YAML directory") from error
    if not resolved.is_file():
        raise MapFormatError("map image does not exist")
    return resolved


def _parse_pgm(data):
    position = 0
    tokens = []
    try:
        for _ in range(4):
            token, position = _next_pgm_token(data, position)
            tokens.append(token)
    except EOFError as error:
        raise MapFormatError("PGM header is truncated") from error
    magic = tokens[0]
    if magic not in (b"P2", b"P5"):
        raise MapFormatError("only P2 and P5 PGM images are supported")
    try:
        width, height, max_value = (int(token) for token in tokens[1:])
    except ValueError as error:
        raise MapFormatError("PGM dimensions and max value must be integers") from error
    if width <= 0 or height <= 0:
        raise MapFormatError("PGM dimensions must be positive")
    if max_value <= 0 or max_value > 65535:
        raise MapFormatError("PGM max value must be between 1 and 65535")
    sample_count = width * height
    if magic == b"P2":
        samples = []
        while True:
            try:
                token, position = _next_pgm_token(data, position)
            except EOFError:
                break
            try:
                samples.append(int(token))
            except ValueError as error:
                raise MapFormatError("P2 samples must be integers") from error
        if len(samples) != sample_count:
            raise MapFormatError("P2 raster length does not match dimensions")
    else:
        if position >= len(data) or data[position] not in b" \t\r\n\v\f":
            raise MapFormatError("P5 header must end with whitespace")
        if data[position:position + 2] == b"\r\n":
            position += 2
        else:
            position += 1
        bytes_per_sample = 1 if max_value < 256 else 2
        raster = data[position:]
        if len(raster) != sample_count * bytes_per_sample:
            raise MapFormatError("P5 raster length does not match dimensions")
        if bytes_per_sample == 1:
            samples = list(raster)
        else:
            samples = [raster[index] * 256 + raster[index + 1]
                       for index in range(0, len(raster), 2)]
    if any(sample < 0 or sample > max_value for sample in samples):
        raise MapFormatError("PGM sample is outside the declared range")
    return width, height, max_value, tuple(samples)


def _next_pgm_token(data, position):
    while position < len(data):
        if data[position] in b" \t\r\n\v\f":
            position += 1
            continue
        if data[position] == ord("#"):
            newline = data.find(b"\n", position)
            position = len(data) if newline < 0 else newline + 1
            continue
        break
    if position >= len(data):
        raise EOFError
    end = position
    while end < len(data) and data[end] not in b" \t\r\n\v\f#":
        end += 1
    return data[position:end], end


def _atomic_write_bytes(path, content):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}.".format(path.name), dir=str(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary_path), str(path))
        _fsync_directory(path.parent)
    except BaseException:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
        raise


def _fsync_directory(directory):
    descriptor = os.open(str(directory), os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number".format(name))
    try:
        normalized = float(value)
    except OverflowError as error:
        raise ValueError("{} must be a finite number".format(name)) from error
    if not math.isfinite(normalized):
        raise ValueError("{} must be a finite number".format(name))
    return normalized


def _normalized_cell_coordinate(coordinate, origin, resolution):
    normalized = (coordinate - origin) / resolution
    nearest_integer = round(normalized)
    if abs(normalized - nearest_integer) <= _CELL_BOUNDARY_TOLERANCE:
        return float(nearest_integer)
    return normalized


def _exact_number(value, expected):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        normalized = float(value)
    except OverflowError:
        return False
    return math.isfinite(normalized) and normalized == expected
