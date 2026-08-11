#!/usr/bin/env python3
"""Wheel encoder ROS adapter 계약."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "wheel_encoder_adapter_node.py"


class _Stamp:
    def __init__(self, seconds):
        self._seconds = seconds

    def to_sec(self):
        return self._seconds


class _JointState:
    def __init__(self, stamp, position=None, velocity=None):
        self.header = types.SimpleNamespace(stamp=_Stamp(stamp))
        self.name = ["rear_left_wheel", "rear_right_wheel"]
        self.position = [] if position is None else position
        self.velocity = [] if velocity is None else velocity


class _TwistWithCovarianceStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0),
                angular=types.SimpleNamespace(),
            ),
            covariance=[],
        )


class _Publisher:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Ros:
    def __init__(self):
        self.params = {
            "/stier_slam/topics/inputs/wheel_encoder": "/slam/input/wheel_encoder",
            "/stier_slam/topics/inputs/vehicle_speed": "/slam/input/vehicle_speed",
            "~wheel_radius_m": 0.2,
            "~left_joint_name": "rear_left_wheel",
            "~right_joint_name": "rear_right_wheel",
            "~left_sign": 1.0,
            "~right_sign": 1.0,
            "~max_abs_wheel_speed_rad_s": 20.0,
            "~output_frame": "base_link",
            "~twist_covariance": [0.25] * 36,
            "~real_mode": False,
            "~calibration_required": True,
        }
        self.publishers = []
        self.subscriptions = []
        self.warnings = []

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, topic, message_type, **_kwargs):
        publisher = _Publisher(topic, message_type)
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic, message_type, callback, **_kwargs):
        self.subscriptions.append((topic, message_type, callback))

    def logwarn(self, *arguments):
        self.warnings.append(arguments)


class WheelEncoderAdapterNodeTest(unittest.TestCase):
    def setUp(self):
        module_names = (
            "rospy", "sensor_msgs", "sensor_msgs.msg", "geometry_msgs",
            "geometry_msgs.msg", "wheel_encoder_adapter_node_under_test",
        )
        self._saved_modules = {name: sys.modules.get(name) for name in module_names}
        self.ros = _Ros()
        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.Subscriber = self.ros.Subscriber
        rospy.logwarn = self.ros.logwarn
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.JointState = _JointState
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TwistWithCovarianceStamped = _TwistWithCovarianceStamped
        sys.modules.update({
            "rospy": rospy,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location(
            "wheel_encoder_adapter_node_under_test", SCRIPT_PATH
        )
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.WheelEncoderAdapterNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def test_subscribes_and_publishes_using_central_topic_parameters(self):
        self.assertEqual(self.ros.subscriptions[0][:2], (
            "/slam/input/wheel_encoder", _JointState,
        ))
        self.assertEqual(self.ros.publishers[0].topic, "/slam/input/vehicle_speed")
        self.assertIs(self.ros.publishers[0].message_type, _TwistWithCovarianceStamped)

    def test_publishes_velocity_with_source_stamp_frame_and_covariance(self):
        source = _JointState(42.5, velocity=[2.0, 4.0])

        self.node._on_encoder(source)

        output = self.ros.publishers[0].messages[0]
        self.assertIs(output.header.stamp, source.header.stamp)
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertAlmostEqual(output.twist.twist.linear.x, 0.6)
        self.assertEqual(output.twist.covariance, [0.25] * 36)

    def test_position_baseline_does_not_publish_until_second_sample(self):
        self.node._on_encoder(_JointState(1.0, position=[0.0, 0.0]))
        self.assertEqual(self.ros.publishers[0].messages, [])

        self.node._on_encoder(_JointState(2.0, position=[1.0, 1.0]))
        self.assertAlmostEqual(
            self.ros.publishers[0].messages[0].twist.twist.linear.x, 0.2
        )

    def test_real_mode_requires_completed_calibration(self):
        self.ros.params["~real_mode"] = True

        with self.assertRaises(ValueError):
            self.module.WheelEncoderAdapterNode()


if __name__ == "__main__":
    unittest.main()
