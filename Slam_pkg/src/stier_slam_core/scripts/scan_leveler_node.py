#!/usr/bin/env python3
"""normalized LaserScan/IMU data를 동기화해 수평 보정된 base_link cloud로 만든다."""

import math

import message_filters
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Header

from stier_slam_core.scan_leveling import ScanValidationError, level_scan, validate_max_abs_tilt


class ScanLevelerNode:
    """역할:
    2D ``LaserScan``을 IMU roll/pitch로 보정한 ``PointCloud2``로 변환한다.

    동작 방식:
    scan과 IMU를 approximate-time 동기화하고 timestamp, frame, quaternion 및 tilt를
    검증한 뒤 mount transform과 leveling을 적용해 `/slam/scan/leveled_points`로 발행한다.
    안전하지 않은 입력 쌍은 보간하거나 publish하지 않고 버린다.
    """

    def __init__(self):
        scan_raw_topic = rospy.get_param("/stier_slam/topics/inputs/scan_raw")
        imu_topic = rospy.get_param("/stier_slam/topics/inputs/imu")
        leveled_points_topic = rospy.get_param("/stier_slam/topics/processed/leveled_points")
        self._queue_size = rospy.get_param("~queue_size")
        self._sync_slop_sec = rospy.get_param("~sync_slop_sec")
        self._max_abs_tilt_rad = rospy.get_param("~max_abs_tilt_rad")
        self._mount_xyz_rpy = tuple(rospy.get_param("~mount_xyz_rpy"))
        self._min_z_m = rospy.get_param("~min_z_m")
        self._max_z_m = rospy.get_param("~max_z_m")
        self._validate_configuration()

        self._publisher = rospy.Publisher(
            leveled_points_topic, PointCloud2, queue_size=10
        )
        self._scan_subscriber = message_filters.Subscriber(scan_raw_topic, LaserScan)
        self._imu_subscriber = message_filters.Subscriber(imu_topic, Imu)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._scan_subscriber, self._imu_subscriber],
            self._queue_size,
            self._sync_slop_sec,
        )
        self._synchronizer.registerCallback(self._on_synced)

    def _validate_configuration(self):
        if (isinstance(self._queue_size, bool) or not isinstance(self._queue_size, int)
                or self._queue_size <= 0):
            raise ValueError("~queue_size must be a positive integer")
        for name, value in (
            ("~sync_slop_sec", self._sync_slop_sec),
            ("~max_abs_tilt_rad", self._max_abs_tilt_rad),
            ("~min_z_m", self._min_z_m),
            ("~max_z_m", self._max_z_m),
        ):
            if not _is_finite_number(value):
                raise ValueError("{} must be finite".format(name))
        if self._sync_slop_sec < 0.0:
            raise ValueError("~sync_slop_sec must be non-negative")
        if self._max_abs_tilt_rad < 0.0:
            raise ValueError("~max_abs_tilt_rad must be non-negative")
        if self._max_z_m < self._min_z_m:
            raise ValueError("~max_z_m must be greater than or equal to ~min_z_m")
        if len(self._mount_xyz_rpy) != 6 or not all(
                _is_finite_number(value) for value in self._mount_xyz_rpy):
            raise ValueError("~mount_xyz_rpy must contain six finite values")

    def _on_synced(self, scan, imu):
        try:
            self._validate_scan_source_metadata(scan)
            orientation_covariance = imu.orientation_covariance
            if len(orientation_covariance) == 0 or orientation_covariance[0] == -1:
                raise ScanValidationError("IMU orientation is unavailable")
            quaternion = (
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            )
            validate_max_abs_tilt(quaternion, self._max_abs_tilt_rad)
            points = level_scan(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                quaternion,
                self._mount_xyz_rpy,
                self._min_z_m,
                self._max_z_m,
            )
            header = Header()
            header.stamp = scan.header.stamp
            header.frame_id = "base_link"
        except (AttributeError, TypeError, ValueError, ScanValidationError) as error:
            rospy.logerr_throttle(5.0, "dropping leveled scan: %s", error)
            return
        self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    @staticmethod
    def _validate_scan_source_metadata(scan):
        source_stamp = scan.header.stamp
        source_time = source_stamp.to_sec() if hasattr(source_stamp, "to_sec") else source_stamp
        if not _is_finite_number(source_time):
            raise ScanValidationError("scan source stamp must be finite")
        frame_id = scan.header.frame_id
        if (not isinstance(frame_id, str) or not frame_id.strip()
                or frame_id.startswith("/")):
            raise ScanValidationError("scan frame_id must be a non-empty relative frame name")


def _is_finite_number(value):
    try:
        return math.isfinite(value)
    except TypeError:
        return False


def main():
    rospy.init_node("scan_leveler")
    ScanLevelerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
