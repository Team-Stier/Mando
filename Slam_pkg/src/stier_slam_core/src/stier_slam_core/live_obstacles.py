"""release map을 변경하지 않고 수명이 짧은 LiDAR 장애물을 분류한다."""

import math

from stier_slam_core.grid_map import GridMap


class LiveObstacleTracker:
    """역할:
    불변 release map 위에 최근 관측된 임시 장애물 cell만 별도로 유지한다.

    동작 방식:
    map 좌표의 point를 base grid cell로 바꾸고 원본이 free인 cell만 timestamp와 함께
    저장하며, snapshot 시 TTL이 지난 cell을 제거해 비영속 obstacle layer를 반환한다.
    """

    def __init__(
            self,
            base_grid,
            ttl_sec,
            occupied_threshold,
            max_input_points=10000,
            max_tracked_cells=5000,
    ):
        if not isinstance(base_grid, GridMap):
            raise TypeError("base_grid must be a GridMap")
        self._base_grid = base_grid
        self._ttl_sec = _finite_positive_number(ttl_sec, "ttl_sec")
        self._occupied_threshold = _finite_number(occupied_threshold, "occupied_threshold")
        if self._occupied_threshold <= 0.0 or self._occupied_threshold > 100.0:
            raise ValueError("occupied_threshold must be greater than zero and at most 100")
        self._max_input_points = _positive_integer(max_input_points, "max_input_points")
        self._max_tracked_cells = _positive_integer(max_tracked_cells, "max_tracked_cells")
        self._cells = {}
        self._last_observation_stamp = None
        self._last_snapshot_stamp = None
        self.last_reset_reason = None

    def update(self, stamp, points_map):
        """``stamp``에 관측된 base map의 free cell을 기록하고 live snapshot을 반환한다."""
        stamp = _finite_number(stamp, "stamp")
        points = _bounded_points(points_map, self._max_input_points)
        observed = self._free_observed_cells(points)
        if (self._last_observation_stamp is not None
                and stamp < self._last_observation_stamp):
            if len(observed) > self._max_tracked_cells:
                raise ValueError("tracked obstacle cache exceeds max_tracked_cells")
            self._cells = {cell: stamp for cell in observed}
            self._last_observation_stamp = stamp
            self._last_snapshot_stamp = stamp
            self.last_reset_reason = "time_reversal"
            return set(self._cells)

        effective_stamp = max(
            stamp,
            self._last_snapshot_stamp if self._last_snapshot_stamp is not None else stamp,
        )
        retained = self._unexpired_cells(effective_stamp)
        live_observed = {
            cell for cell in observed if stamp + self._ttl_sec > effective_stamp
        }
        if len(set(retained).union(live_observed)) > self._max_tracked_cells:
            raise ValueError("tracked obstacle cache exceeds max_tracked_cells")
        retained.update({cell: stamp for cell in live_observed})
        self._cells = retained
        self._last_observation_stamp = stamp
        return set(self._cells)

    def snapshot(self, stamp):
        """정확한 TTL 경계에서 만료 처리한 뒤 ``stamp`` 시점의 live cell을 반환한다."""
        stamp = _finite_number(stamp, "stamp")
        if self._reset_for_snapshot_time_reversal(stamp):
            return set()
        self._cells = self._unexpired_cells(stamp)
        self._last_snapshot_stamp = stamp
        return set(self._cells)

    def _reset_for_snapshot_time_reversal(self, stamp):
        reference_stamp = self._last_snapshot_stamp
        if reference_stamp is None:
            reference_stamp = self._last_observation_stamp
        if reference_stamp is None or stamp >= reference_stamp:
            return False
        self._cells = {}
        self._last_observation_stamp = None
        self._last_snapshot_stamp = stamp
        self.last_reset_reason = "time_reversal"
        return True

    def _unexpired_cells(self, stamp):
        return {
            cell: last_seen
            for cell, last_seen in self._cells.items()
            if stamp < last_seen + self._ttl_sec
        }

    def _free_observed_cells(self, points):
        observed = set()
        for point in points:
            cell = self._free_cell_for_point(point)
            if cell is not None:
                observed.add(cell)
        return observed

    def _free_cell_for_point(self, point):
        try:
            x, y = point[0], point[1]
            x = _finite_number(x, "x")
            y = _finite_number(y, "y")
            cell = self._base_grid.map_to_cell(x, y)
        except (IndexError, KeyError, TypeError, ValueError):
            return None
        value = self._base_grid.cell(*cell)
        if value == -1 or value >= self._occupied_threshold:
            return None
        return cell


def _bounded_points(points_map, maximum):
    try:
        iterator = iter(points_map)
    except TypeError as error:
        raise TypeError("points_map must be iterable") from error
    points = []
    for _ in range(maximum + 1):
        try:
            points.append(next(iterator))
        except StopIteration:
            return points
    raise ValueError("points_map exceeds max_input_points")


def _finite_positive_number(value, name):
    value = _finite_number(value, name)
    if value <= 0.0:
        raise ValueError("{} must be positive".format(name))
    return value


def _finite_number(value, name):
    if isinstance(value, bool):
        raise ValueError("{} must be finite".format(name))
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("{} must be finite".format(name)) from error
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("{} must be a positive integer".format(name))
    return value
