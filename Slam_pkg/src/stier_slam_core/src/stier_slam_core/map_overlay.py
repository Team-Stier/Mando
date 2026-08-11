"""static map overlay를 검증하고 순서대로 polygon rasterization을 수행한다."""

import math

from stier_slam_core.grid_map import GridMap


class OverlayValidationError(ValueError):
    """overlay document를 안전하게 rasterize할 수 없을 때 발생한다."""


def apply_overlay(grid, overlay_document):
    """순서가 있는 occupied/free polygon을 적용한 새 grid를 반환한다."""
    if not isinstance(grid, GridMap):
        raise TypeError("grid must be a GridMap")
    operations = _validate_overlay(grid, overlay_document)
    cells = list(grid.cells)
    for operation, polygon in operations:
        value = 100 if operation == "occupied" else 0
        min_x = min(point[0] for point in polygon)
        max_x = max(point[0] for point in polygon)
        min_y = min(point[1] for point in polygon)
        max_y = max(point[1] for point in polygon)
        first_column = max(0, int(math.floor((min_x - grid.origin_x) / grid.resolution)))
        last_column = min(
            grid.width - 1,
            int(math.floor((max_x - grid.origin_x) / grid.resolution)),
        )
        first_row = max(0, int(math.floor((min_y - grid.origin_y) / grid.resolution)))
        last_row = min(grid.height - 1, int(math.floor((max_y - grid.origin_y) / grid.resolution)))
        for row in range(first_row, last_row + 1):
            center_y = grid.origin_y + (row + 0.5) * grid.resolution
            for column in range(first_column, last_column + 1):
                center_x = grid.origin_x + (column + 0.5) * grid.resolution
                if _point_in_polygon(center_x, center_y, polygon):
                    cells[row * grid.width + column] = value
    return GridMap(
        grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y, cells
    )


def validate_overlay(grid, overlay_document):
    """overlay를 적용하지 않고 유효성만 검사한다."""
    _validate_overlay(grid, overlay_document)


def _validate_overlay(grid, document):
    if not isinstance(document, dict) or set(document) != {"schema_version", "operations"}:
        raise OverlayValidationError("overlay fields do not match schema version 1")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise OverlayValidationError("overlay schema_version must be 1")
    raw_operations = document["operations"]
    if not isinstance(raw_operations, list):
        raise OverlayValidationError("operations must be a list")
    validated = []
    for raw_operation in raw_operations:
        if (not isinstance(raw_operation, dict)
                or set(raw_operation) != {"operation", "polygon"}):
            raise OverlayValidationError("operation fields do not match schema version 1")
        operation = raw_operation["operation"]
        if operation not in ("occupied", "free"):
            raise OverlayValidationError("operation must be occupied or free")
        polygon = _validate_polygon(grid, raw_operation["polygon"])
        validated.append((operation, polygon))
    return tuple(validated)


def _validate_polygon(grid, raw_polygon):
    if not isinstance(raw_polygon, list) or len(raw_polygon) < 3:
        raise OverlayValidationError("polygon must contain at least three points")
    polygon = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, list) or len(raw_point) != 2:
            raise OverlayValidationError("polygon points must be [x, y] lists")
        x, y = raw_point
        point = (_finite_coordinate(x), _finite_coordinate(y))
        if (point[0] < grid.origin_x or point[0] > grid.max_x
                or point[1] < grid.origin_y or point[1] > grid.max_y):
            raise OverlayValidationError("polygon coordinate is outside the map extent")
        polygon.append(point)
    if len(set(polygon)) != len(polygon):
        raise OverlayValidationError("polygon vertices must be distinct")
    signed_double_area = sum(
        polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
        - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
        for index in range(len(polygon))
    )
    if abs(signed_double_area) <= grid.resolution * grid.resolution * 1.0e-9:
        raise OverlayValidationError("polygon area is too small")
    if _has_self_intersection(polygon):
        raise OverlayValidationError("polygon must not self-intersect")
    return tuple(polygon)


def _has_self_intersection(polygon):
    count = len(polygon)
    for first in range(count):
        first_next = (first + 1) % count
        for second in range(first + 1, count):
            second_next = (second + 1) % count
            if first == second or first_next == second or second_next == first:
                continue
            if _segments_intersect(
                    polygon[first], polygon[first_next], polygon[second], polygon[second_next]):
                return True
    return False


def _segments_intersect(first_a, first_b, second_a, second_b):
    orientations = (
        _orientation(first_a, first_b, second_a),
        _orientation(first_a, first_b, second_b),
        _orientation(second_a, second_b, first_a),
        _orientation(second_a, second_b, first_b),
    )
    if orientations[0] * orientations[1] < 0.0 and orientations[2] * orientations[3] < 0.0:
        return True
    return (
        (orientations[0] == 0.0 and _on_segment(first_a, second_a, first_b))
        or (orientations[1] == 0.0 and _on_segment(first_a, second_b, first_b))
        or (orientations[2] == 0.0 and _on_segment(second_a, first_a, second_b))
        or (orientations[3] == 0.0 and _on_segment(second_a, first_b, second_b))
    )


def _orientation(first, second, third):
    return ((second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0]))


def _on_segment(first, point, second):
    return (min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
            and min(first[1], second[1]) <= point[1] <= max(first[1], second[1]))


def _point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (_orientation(previous, current, (x, y)) == 0.0
                and _on_segment(previous, (x, y), current)):
            return True
        if ((current[1] > y) != (previous[1] > y)):
            crossing_x = ((previous[0] - current[0]) * (y - current[1])
                          / (previous[1] - current[1]) + current[0])
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def _finite_coordinate(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OverlayValidationError("polygon coordinates must be finite numbers")
    try:
        normalized = float(value)
    except OverflowError as error:
        raise OverlayValidationError("polygon coordinates must be finite numbers") from error
    if not math.isfinite(normalized):
        raise OverlayValidationError("polygon coordinates must be finite numbers")
    return normalized
