#!/usr/bin/env python3
"""release map 위에 범위가 제한되고 만료되는 LiDAR obstacle layer를 발행한다."""

import math
import struct
import threading

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from stier_slam_core.grid_map import GridMap
from stier_slam_core.live_obstacles import LiveObstacleTracker


MAX_LOCAL_GRID_CELLS = 100000


class LiveObstacleNode:
    """역할:
    release map을 수정하지 않고 leveled LiDAR에서 임시 장애물 layer를 생성한다.

    동작 방식:
    측정 시각의 TF로 cloud를 map frame에 변환해 ``LiveObstacleTracker``에 넣고, TTL이
    남은 점과 bounded local occupancy grid를 별도 live-obstacle topic으로 발행한다.
    """

    def __init__(self):
        leveled_points_topic = rospy.get_param("/stier_slam/topics/processed/leveled_points")
        released_map_topic = rospy.get_param("/stier_slam/topics/outputs/released_map")
        live_points_topic = rospy.get_param("/stier_slam/topics/outputs/live_obstacle_points")
        live_grid_topic = rospy.get_param("/stier_slam/topics/outputs/live_obstacle_grid")
        self._map_frame = rospy.get_param("~map_frame")
        self._ttl_sec = rospy.get_param("~ttl_sec")
        self._occupied_threshold = rospy.get_param("~occupied_threshold")
        self._local_grid_width = rospy.get_param("~local_grid_width")
        self._local_grid_height = rospy.get_param("~local_grid_height")
        self._local_grid_resolution = rospy.get_param("~local_grid_resolution")
        self._max_input_points = rospy.get_param("~max_input_points")
        self._max_tracked_cells = rospy.get_param("~max_tracked_cells")
        self._publish_rate_hz = rospy.get_param("~publish_rate_hz")
        self._validate_configuration()
        self._state_lock = threading.RLock()
        self._base_grid = None
        self._tracker = None
        self._map_generation = 0
        self._layer_revision = 0
        self._cloud_sequence = 0
        self._has_transformed_cloud = False
        self._window_center = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._points_publisher = rospy.Publisher(
            live_points_topic, PointCloud2, queue_size=10
        )
        self._grid_publisher = rospy.Publisher(
            live_grid_topic, OccupancyGrid, queue_size=10
        )
        rospy.Subscriber(released_map_topic, OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber(leveled_points_topic, PointCloud2, self._on_cloud, queue_size=10)
        self._expiry_timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate_hz), self._on_timer, reset=True
        )

    def _validate_configuration(self):
        if (not isinstance(self._map_frame, str) or not self._map_frame.strip()
                or self._map_frame.startswith("/")):
            raise ValueError("~map_frame must be a non-empty relative frame name")
        self._ttl_sec = _finite_positive_number(self._ttl_sec, "~ttl_sec")
        self._occupied_threshold = _finite_number(
            self._occupied_threshold, "~occupied_threshold"
        )
        if self._occupied_threshold <= 0.0 or self._occupied_threshold > 100.0:
            raise ValueError("~occupied_threshold must be greater than zero and at most 100")
        self._local_grid_width = _positive_integer(
            self._local_grid_width, "~local_grid_width"
        )
        self._local_grid_height = _positive_integer(
            self._local_grid_height, "~local_grid_height"
        )
        if self._local_grid_width * self._local_grid_height > MAX_LOCAL_GRID_CELLS:
            raise ValueError("local grid exceeds MAX_LOCAL_GRID_CELLS")
        self._local_grid_resolution = _finite_positive_number(
            self._local_grid_resolution, "~local_grid_resolution"
        )
        self._max_input_points = _positive_integer(self._max_input_points, "~max_input_points")
        self._max_tracked_cells = _positive_integer(
            self._max_tracked_cells, "~max_tracked_cells"
        )
        self._publish_rate_hz = _finite_positive_number(
            self._publish_rate_hz, "~publish_rate_hz"
        )

    def _on_map(self, message):
        try:
            base_grid = _grid_map_from_message(
                message, self._map_frame, self._occupied_threshold
            )
            if self._covers_entire_base_grid(base_grid):
                raise ValueError("configured local grid must be smaller than the released map")
        except (AttributeError, TypeError, ValueError) as error:
            with self._state_lock:
                self._base_grid = None
                self._tracker = None
                self._has_transformed_cloud = False
                self._window_center = None
                self._map_generation += 1
                self._layer_revision += 1
            rospy.logwarn_throttle(5.0, "rejecting released map for live obstacles: %s", error)
            return
        with self._state_lock:
            self._base_grid = base_grid
            self._tracker = LiveObstacleTracker(
                base_grid,
                self._ttl_sec,
                self._occupied_threshold,
                max_input_points=self._max_input_points,
                max_tracked_cells=self._max_tracked_cells,
            )
            self._has_transformed_cloud = False
            self._window_center = None
            self._map_generation += 1
            self._layer_revision += 1

    def _covers_entire_base_grid(self, base_grid):
        return (
            self._local_grid_width * self._local_grid_resolution >= base_grid.width * base_grid.resolution
            and self._local_grid_height * self._local_grid_resolution
            >= base_grid.height * base_grid.resolution
        )

    def _on_cloud(self, message):
        with self._state_lock:
            if self._tracker is None:
                return
            tracker = self._tracker
            base_grid = self._base_grid
            map_generation = self._map_generation
            start_layer_revision = self._layer_revision
            # 새 callback이 이후 malformed로 버려지더라도 이전 준비 결과보다 우선한다.
            self._cloud_sequence += 1
            cloud_sequence = self._cloud_sequence
        try:
            stamp = message.header.stamp
            stamp_sec = _stamp_seconds(stamp)
            if stamp_sec <= 0.0:
                raise ValueError("cloud measurement stamp must be positive")
            _validate_cloud_fields(message)
            cloud_frame = _frame_id(message.header.frame_id, "cloud frame_id")
            transform = self._tf_buffer.lookup_transform(self._map_frame, cloud_frame, stamp)
            window_center = _transform_xyz((0.0, 0.0, 0.0), transform)[:2]
            points_map = self._finite_transformed_points(message, transform)
        except (AttributeError, TypeError, ValueError, struct.error) as error:
            rospy.logwarn_throttle(5.0, "dropping live obstacle cloud: %s", error)
            return
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, "waiting for live obstacle transform: %s", error)
            return
        with self._state_lock:
            if (map_generation != self._map_generation or tracker is not self._tracker
                    or base_grid is not self._base_grid
                    or start_layer_revision != self._layer_revision
                    or cloud_sequence != self._cloud_sequence):
                rospy.logwarn_throttle(5.0, "dropping superseded live obstacle cloud")
                return
            try:
                cells = tracker.update(stamp_sec, points_map)
            except ValueError as error:
                rospy.logwarn_throttle(5.0, "dropping live obstacle cloud: %s", error)
                return
            self._has_transformed_cloud = True
            self._window_center = window_center
            self._layer_revision += 1
            layer_revision = self._layer_revision
        self._prepare_and_publish(
            cells, stamp, base_grid, window_center, tracker, map_generation, layer_revision
        )

    def _on_timer(self, event):
        with self._state_lock:
            if (self._tracker is None or not self._has_transformed_cloud
                    or self._window_center is None):
                return
            tracker = self._tracker
            base_grid = self._base_grid
            window_center = self._window_center
            try:
                stamp = event.current_real
                cells = tracker.snapshot(_stamp_seconds(stamp))
            except (AttributeError, TypeError, ValueError) as error:
                rospy.logwarn_throttle(5.0, "dropping live obstacle expiry update: %s", error)
                return
            if tracker is not self._tracker or base_grid is not self._base_grid:
                return
            map_generation = self._map_generation
            self._layer_revision += 1
            layer_revision = self._layer_revision
        self._prepare_and_publish(
            cells, stamp, base_grid, window_center, tracker, map_generation, layer_revision
        )

    def _finite_transformed_points(self, message, transform):
        points = []
        for index, point in enumerate(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=False)):
            if index >= self._max_input_points:
                raise ValueError("cloud exceeds ~max_input_points")
            try:
                x, y, z = point[0], point[1], point[2]
                if not all(_is_finite_number(value) for value in (x, y, z)):
                    continue
                mapped = _transform_xyz((x, y, z), transform)
            except (IndexError, TypeError, ValueError):
                continue
            if all(_is_finite_number(value) for value in mapped):
                points.append(mapped)
        return points

    def _prepare_and_publish(
            self, cells, stamp, base_grid, window_center, tracker, map_generation, layer_revision):
        points = [_cell_center(base_grid, cell) for cell in sorted(cells)]
        header = Header()
        header.stamp = stamp
        header.frame_id = self._map_frame
        points_message = point_cloud2.create_cloud_xyz32(header, points)
        grid = self._local_grid(header, points, window_center)
        with self._state_lock:
            if (tracker is not self._tracker or base_grid is not self._base_grid
                    or map_generation != self._map_generation
                    or layer_revision != self._layer_revision):
                return
            self._points_publisher.publish(points_message)
            self._grid_publisher.publish(grid)

    def _local_grid(self, header, points, window_center):
        grid = OccupancyGrid()
        grid.header = header
        grid.info.resolution = self._local_grid_resolution
        grid.info.width = self._local_grid_width
        grid.info.height = self._local_grid_height
        center_x, center_y = window_center
        grid.info.origin.position.x = center_x - self._local_grid_width * self._local_grid_resolution * 0.5
        grid.info.origin.position.y = center_y - self._local_grid_height * self._local_grid_resolution * 0.5
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1] * (self._local_grid_width * self._local_grid_height)
        omitted = 0
        for x, y, _ in points:
            column = int(math.floor((x - grid.info.origin.position.x) / grid.info.resolution))
            row = int(math.floor((y - grid.info.origin.position.y) / grid.info.resolution))
            if 0 <= column < grid.info.width and 0 <= row < grid.info.height:
                grid.data[row * grid.info.width + column] = 100
            else:
                omitted += 1
        if omitted:
            rospy.logwarn_throttle(
                5.0, "omitted %d live obstacle cells outside bounded local grid", omitted
            )
        return grid


def _grid_map_from_message(message, expected_frame, occupied_threshold):
    if _frame_id(message.header.frame_id, "released map frame_id") != expected_frame:
        raise ValueError("released map frame_id does not match ~map_frame")
    origin = message.info.origin
    if not all(_is_finite_number(value) for value in (
            origin.position.x, origin.position.y, origin.position.z,
            origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w,
    )):
        raise ValueError("released map origin must be finite")
    if (origin.position.z != 0.0 or origin.orientation.x != 0.0 or origin.orientation.y != 0.0
            or origin.orientation.z != 0.0 or abs(origin.orientation.w) != 1.0):
        raise ValueError("released map must have a planar identity origin orientation")
    cells = []
    for value in message.data:
        if isinstance(value, bool) or not isinstance(value, int) or value < -1 or value > 100:
            raise ValueError("released map occupancy values must be integers from -1 through 100")
        if value == -1:
            cells.append(-1)
        elif value >= occupied_threshold:
            cells.append(100)
        else:
            cells.append(0)
    return GridMap(
        message.info.width,
        message.info.height,
        message.info.resolution,
        origin.position.x,
        origin.position.y,
        cells,
    )


def _validate_cloud_fields(message):
    try:
        fields = tuple(message.fields)
    except (AttributeError, TypeError) as error:
        raise ValueError("PointCloud2 fields must be iterable") from error
    by_name = {}
    for field in fields:
        try:
            name = field.name
            datatype = field.datatype
            offset = field.offset
            count = field.count
        except AttributeError as error:
            raise ValueError("PointCloud2 field metadata is incomplete") from error
        if name not in ("x", "y", "z"):
            continue
        if name in by_name:
            raise ValueError("PointCloud2 field names must be unique")
        if (not isinstance(name, str) or isinstance(datatype, bool) or not isinstance(datatype, int)
                or datatype not in (7, 8)
                or isinstance(offset, bool) or not isinstance(offset, int) or offset < 0
                or isinstance(count, bool) or not isinstance(count, int) or count != 1):
            raise ValueError("PointCloud2 must provide scalar float x, y, and z fields")
        by_name[name] = field
    if set(by_name) != {"x", "y", "z"}:
        raise ValueError("PointCloud2 must provide x, y, and z fields")


def _transform_xyz(point, transform):
    try:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        tx, ty, tz = translation.x, translation.y, translation.z
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    except AttributeError as error:
        raise ValueError("transform has no finite translation and rotation") from error
    if not all(_is_finite_number(value) for value in (tx, ty, tz, qx, qy, qz, qw)):
        raise ValueError("transform has non-finite translation or rotation")
    squared_magnitude = qx * qx + qy * qy + qz * qz + qw * qw
    if not math.isfinite(squared_magnitude) or squared_magnitude <= 1.0e-24:
        raise ValueError("transform rotation norm must be finite and above tolerance")
    magnitude = math.sqrt(squared_magnitude)
    qx, qy, qz, qw = (qx / magnitude, qy / magnitude, qz / magnitude, qw / magnitude)
    x, y, z = point
    twice_cross_x = 2.0 * (qy * z - qz * y)
    twice_cross_y = 2.0 * (qz * x - qx * z)
    twice_cross_z = 2.0 * (qx * y - qy * x)
    rotated_x = x + qw * twice_cross_x + (qy * twice_cross_z - qz * twice_cross_y)
    rotated_y = y + qw * twice_cross_y + (qz * twice_cross_x - qx * twice_cross_z)
    rotated_z = z + qw * twice_cross_z + (qx * twice_cross_y - qy * twice_cross_x)
    return rotated_x + tx, rotated_y + ty, rotated_z + tz


def _cell_center(base_grid, cell):
    column, row = cell
    return (
        base_grid.origin_x + (column + 0.5) * base_grid.resolution,
        base_grid.origin_y + (row + 0.5) * base_grid.resolution,
        0.0,
    )


def _stamp_seconds(stamp):
    try:
        return _finite_number(stamp.to_sec(), "cloud measurement stamp")
    except AttributeError as error:
        raise ValueError("cloud measurement stamp must provide to_sec()") from error


def _frame_id(value, name):
    if not isinstance(value, str) or not value.strip() or value.startswith("/"):
        raise ValueError("{} must be a non-empty relative frame name".format(name))
    return value


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("{} must be a positive integer".format(name))
    return value


def _finite_positive_number(value, name):
    value = _finite_number(value, name)
    if value <= 0.0:
        raise ValueError("{} must be positive".format(name))
    return value


def _finite_number(value, name):
    if not _is_finite_number(value):
        raise ValueError("{} must be finite".format(name))
    return float(value)


def _is_finite_number(value):
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except TypeError:
        return False


def main():
    rospy.init_node("live_obstacle")
    LiveObstacleNode()
    rospy.spin()


if __name__ == "__main__":
    main()
