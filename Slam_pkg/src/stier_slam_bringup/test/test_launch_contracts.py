#!/usr/bin/env python3
"""Semantic contracts for the hardware-independent SLAM bringup launches."""

import configparser
import os
import shlex
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LAUNCH_ROOT = os.path.join(PACKAGE_ROOT, "launch")
CONFIG_ROOT = os.path.join(PACKAGE_ROOT, "config")
TEST_SUPPORT_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", "stier_slam_test_support"))


def launch_tree(name):
    return ET.parse(os.path.join(LAUNCH_ROOT, name)).getroot()


def topic_contract():
    with open(os.path.join(CONFIG_ROOT, "topics.yaml"), encoding="utf-8") as stream:
        return yaml.safe_load(stream)["stier_slam"]["topics"]


def nodes(root):
    return root.findall(".//node")


def node_by_name(root, name):
    matching = [node for node in nodes(root) if node.attrib.get("name") == name]
    if len(matching) != 1:
        raise AssertionError("expected exactly one node named {!r}, got {}".format(name, len(matching)))
    return matching[0]


def remaps(node):
    return {
        remap.attrib["from"]: remap.attrib["to"]
        for remap in node.findall("remap")
    }


def params(node):
    return {
        param.attrib["name"]: param.attrib.get("value", param.attrib.get("textfile"))
        for param in node.findall("param")
    }


def param_attributes(node):
    return {param.attrib["name"]: param.attrib for param in node.findall("param")}


def launch_arguments(root):
    return {argument.attrib["name"]: argument.attrib for argument in root.findall("arg")}


class LaunchContractTest(unittest.TestCase):
    def test_central_topic_config_defines_complete_normalized_contract(self):
        document = {"stier_slam": {"topics": topic_contract()}}

        self.assertEqual(document, {
            "stier_slam": {"topics": {
                "inputs": {
                    "wheel_encoder": "/slam/input/wheel_encoder",
                    "scan_raw": "/slam/input/scan_raw",
                    "imu": "/slam/input/imu",
                    "gnss_fix": "/slam/input/gnss/fix",
                    "gnss_rtk_status": "/slam/input/gnss/rtk_status",
                    "vehicle_speed": "/slam/input/vehicle_speed",
                    "steering_angle": "/slam/input/steering_angle",
                    "camera_front_image": "/slam/input/camera/front/image_raw",
                },
                "processed": {
                    "wheel_odometry": "/slam/odometry/wheel",
                    "local_odometry": "/slam/odometry/local",
                    "leveled_points": "/slam/scan/leveled_points",
                    "accepted_gnss": "/slam/gnss/fix_accepted",
                },
                "outputs": {
                    "localization_pose": "/slam/localization/pose",
                    "released_map": "/slam/map/released",
                    "diagnostics": "/slam/diagnostics",
                    "rtk_diagnostics": "/slam/diagnostics/rtk_gate",
                    "live_obstacle_points": "/slam/live_obstacles/points",
                    "live_obstacle_grid": "/slam/live_obstacles/grid",
                },
            }},
        })

        expected_file = "$(find stier_slam_bringup)/config/topics.yaml"
        for launch_name in ("mapping.launch", "localization.launch", "synthetic_demo.launch"):
            loaded = launch_tree(launch_name).findall("rosparam")
            self.assertIn(expected_file, [item.attrib.get("file") for item in loaded])

        direct_fixture = ET.parse(
            os.path.join(TEST_SUPPORT_ROOT, "test", "synthetic_topics.test")
        ).getroot()
        loaded = direct_fixture.findall("rosparam")
        self.assertIn(expected_file, [item.attrib.get("file") for item in loaded])

    def test_required_launch_files_are_present_and_parseable(self):
        for launch_name in (
                "record.launch", "mapping.launch", "localization.launch", "editor.launch",
                "synthetic_demo.launch"):
            root = launch_tree(launch_name)
            self.assertEqual(root.tag, "launch")

    def test_synthetic_demo_is_hardware_free_and_non_real_by_default(self):
        root = launch_tree("synthetic_demo.launch")
        arguments = launch_arguments(root)
        self.assertEqual(arguments["real_mode"].get("default"), "false")
        self.assertEqual(arguments["use_sim_time"].get("default"), "true")
        self.assertEqual(arguments["publish_rate_hz"].get("default"), "20.0")
        self.assertEqual(arguments["cycle_samples"].get("default"), "40")
        self.assertEqual(arguments["obstacle_start_sample"].get("default"), "10")
        self.assertEqual(arguments["obstacle_end_sample"].get("default"), "20")

        demo_nodes = nodes(root)
        self.assertEqual(len(demo_nodes), 1)
        synthetic = node_by_name(root, "synthetic_sensor")
        self.assertEqual(synthetic.attrib.get("pkg"), "stier_slam_test_support")
        self.assertEqual(synthetic.attrib.get("type"), "synthetic_sensor_node.py")
        self.assertEqual(synthetic.attrib.get("required"), "true")
        self.assertEqual(params(synthetic).get("real_mode"), "$(arg real_mode)")
        self.assertEqual(params(synthetic).get("publish_clock"), "$(arg use_sim_time)")
        self.assertEqual(params(synthetic).get("gnss_frame"), "gps_link")
        self.assertEqual(params(synthetic).get("camera_frame"), "camera_link")

    def test_editor_launch_requires_explicit_workspace_and_localhost_by_default(self):
        root = launch_tree("editor.launch")
        arguments = launch_arguments(root)
        for name in ("course_dir", "workspace_id"):
            self.assertIn(name, arguments)
            self.assertNotIn("default", arguments[name])
        self.assertEqual(arguments["host"].get("default"), "127.0.0.1")
        self.assertEqual(arguments["port"].get("default"), "8765")
        self.assertEqual(arguments["unsafe_allow_nonlocal"].get("default"), "false")
        self.assertEqual(
            arguments["web_root"].get("default"), "$(find stier_slam_core)/web"
        )
        editor = node_by_name(root, "map_editor")
        self.assertEqual(editor.attrib.get("pkg"), "stier_slam_core")
        self.assertEqual(editor.attrib.get("type"), "map_editor_server.py")
        self.assertEqual(editor.attrib.get("required"), "true")
        self.assertEqual(
            shlex.split(editor.attrib.get("args", "")),
            [
                "--course-dir", "$(arg course_dir)",
                "--workspace-id", "$(arg workspace_id)",
                "--web-root", "$(arg web_root)",
                "--host", "$(arg host)",
                "--port", "$(arg port)",
                "--unsafe-allow-nonlocal", "$(arg unsafe_allow_nonlocal)",
            ],
        )

    def test_recording_starts_only_rosbag_with_exact_replay_safe_allowlist(self):
        root = launch_tree("record.launch")
        recorders = nodes(root)
        self.assertEqual(len(recorders), 1)
        recorder = recorders[0]
        self.assertEqual(recorder.attrib.get("pkg"), "rosbag")
        self.assertEqual(recorder.attrib.get("type"), "record")
        tokens = shlex.split(recorder.attrib.get("args", ""))
        topics = topic_contract()
        self.assertEqual(set(token for token in tokens if token.startswith("/")),
                         set(topics["inputs"].values()) | {
                             "/tf", "/tf_static", topics["outputs"]["diagnostics"],
                             "/diagnostics", "/clock",
                         })
        self.assertEqual(len(nodes(root)), 1)
        for forbidden in (
                "/slam/odometry/wheel", "/slam/odometry/local", "/slam/scan/leveled_points",
                "/slam/gnss/fix_accepted", "/slam/localization/pose"):
            self.assertNotIn(forbidden, tokens)

    def test_mapping_uses_incremental_rtabmap_and_unique_tf_owners(self):
        root = launch_tree("mapping.launch")
        rtabmap = node_by_name(root, "rtabmap")
        rtabmap_params = params(rtabmap)
        self.assertEqual(rtabmap.attrib.get("pkg"), "rtabmap_slam")
        self.assertEqual(rtabmap_params.get("config_path"), "$(arg mapping_session_dir)/rtabmap.ini")
        self.assertEqual(rtabmap_params.get("database_path"), "$(arg mapping_session_dir)/stier_slam_mapping.db")
        self.assertEqual(rtabmap_params.get("publish_tf"), "true")
        self.assertEqual(rtabmap_params.get("frame_id"), "base_link")
        self.assertEqual(rtabmap_params.get("map_frame_id"), "map")
        self.assertEqual(rtabmap_params.get("odom_frame_id"), "")
        self.assertEqual(param_attributes(rtabmap)["odom_frame_id"].get("type"), "string")
        self.assertEqual({
            name: rtabmap_params.get(name) for name in (
                "subscribe_depth", "subscribe_rgb", "subscribe_rgbd", "subscribe_stereo",
                "subscribe_scan", "subscribe_scan_cloud")
        }, {
            "subscribe_depth": "false", "subscribe_rgb": "false", "subscribe_rgbd": "false",
            "subscribe_stereo": "false", "subscribe_scan": "false", "subscribe_scan_cloud": "true",
        })
        self.assertEqual(rtabmap_params.get("scan_cloud_is_2d"), "true")
        self.assertEqual(rtabmap_params.get("Reg/Force3DoF"), "true")
        self.assertEqual(rtabmap_params.get("Mem/IncrementalMemory"), "true")
        for name in (
                "subscribe_depth", "subscribe_rgb", "subscribe_rgbd", "subscribe_stereo",
                "subscribe_scan", "subscribe_scan_cloud"):
            self.assertEqual(param_attributes(rtabmap)[name].get("type"), "bool")
        for name in ("Mem/IncrementalMemory", "Reg/Force3DoF"):
            self.assertEqual(param_attributes(rtabmap)[name].get("type"), "string")
        self.assertEqual(
            shlex.split(rtabmap.attrib.get("launch-prefix", "")),
            [
                "rosrun", "stier_slam_core", "rtabmap_runtime_guard.py",
                "--source-config", "$(find stier_slam_bringup)/config/rtabmap_mapping.ini",
                "--runtime-config", "$(arg mapping_session_dir)/rtabmap.ini",
                "--mapping-database", "$(arg mapping_session_dir)/stier_slam_mapping.db",
                "--mapping-runtime-root", "$(arg mapping_runtime_root)",
                "--mapping-session-dir", "$(arg mapping_session_dir)", "--",
            ],
        )
        topics = topic_contract()
        self.assertEqual(remaps(rtabmap), {
            "scan_cloud": topics["processed"]["leveled_points"],
            "odom": topics["processed"]["local_odometry"],
            "imu": topics["inputs"]["imu"],
            "gps/fix": topics["processed"]["accepted_gnss"],
            "localization_pose": topics["outputs"]["localization_pose"],
        })
        ekf = node_by_name(root, "ekf_local")
        self.assertEqual(params(ekf).get("publish_tf"), "true")
        self.assertEqual(remaps(ekf).get("odometry/filtered"), "/slam/odometry/local")
        self.assertEqual(odometry_node_names_with_publish_tf(root), {"ekf_local", "rtabmap"})
        self.assertNotIn("released_map_server", [node.attrib.get("name") for node in nodes(root)])
        self.assertNotIn("live_obstacles", [node.attrib.get("name") for node in nodes(root)])

    def test_encoder_adapter_is_required_only_for_real_mapping_and_localization(self):
        expected_config = "$(find stier_slam_bringup)/config/encoder.yaml"
        for launch_name in ("mapping.launch", "localization.launch"):
            root = launch_tree(launch_name)
            adapter = node_by_name(root, "wheel_encoder_adapter")
            self.assertEqual(adapter.attrib.get("pkg"), "stier_slam_core")
            self.assertEqual(adapter.attrib.get("type"), "wheel_encoder_adapter_node.py")
            self.assertEqual(adapter.attrib.get("required"), "true")
            self.assertEqual(params(adapter).get("real_mode"), "$(arg real_mode)")
            loaded = adapter.findall("rosparam")
            self.assertEqual([item.attrib.get("file") for item in loaded], [expected_config])

        synthetic = launch_tree("synthetic_demo.launch")
        self.assertNotIn(
            "wheel_encoder_adapter",
            [node.attrib.get("name") for node in nodes(synthetic)],
        )

    def test_localization_uses_read_only_rtabmap_and_released_map(self):
        root = launch_tree("localization.launch")
        map_server = node_by_name(root, "released_map_server")
        self.assertEqual(map_server.attrib.get("pkg"), "map_server")
        self.assertEqual(map_server.attrib.get("args"), '"$(arg runtime_bundle_dir)/map.yaml"')
        self.assertEqual(
            shlex.split(map_server.attrib.get("args")), ["$(arg runtime_bundle_dir)/map.yaml"]
        )
        self.assertEqual(
            shlex.split(map_server.attrib.get("launch-prefix", "")),
            [
                "rosrun", "stier_slam_core", "rtabmap_runtime_guard.py",
                "--release-manifest", "$(arg release_manifest)",
                "--map-yaml", "$(arg map_yaml)",
                "--source-database", "$(arg database_path)",
                "--source-config", "$(find stier_slam_bringup)/config/rtabmap_localization.ini",
                "--runtime-root", "$(arg runtime_root_dir)",
                "--runtime-bundle", "$(arg runtime_bundle_dir)",
                "--bundle-role", "map-server", "--",
            ],
        )
        self.assertEqual(remaps(map_server), {"map": "/slam/map/released"})
        rtabmap = node_by_name(root, "rtabmap")
        self.assertEqual(
            params(rtabmap).get("config_path"),
            "$(arg runtime_bundle_dir)/rtabmap.ini",
        )
        self.assertEqual(params(rtabmap).get("Mem/IncrementalMemory"), "false")
        self.assertEqual(params(rtabmap).get("publish_tf"), "true")
        self.assertEqual(params(rtabmap).get("odom_frame_id"), "")
        self.assertEqual(param_attributes(rtabmap)["odom_frame_id"].get("type"), "string")
        self.assertEqual({
            name: params(rtabmap).get(name) for name in (
                "subscribe_depth", "subscribe_rgb", "subscribe_rgbd", "subscribe_stereo",
                "subscribe_scan", "subscribe_scan_cloud")
        }, {
            "subscribe_depth": "false", "subscribe_rgb": "false", "subscribe_rgbd": "false",
            "subscribe_stereo": "false", "subscribe_scan": "false", "subscribe_scan_cloud": "true",
        })
        self.assertEqual(remaps(rtabmap).get("odom"), "/slam/odometry/local")
        self.assertEqual(params(rtabmap).get("database_path"), "$(arg runtime_bundle_dir)/map.db")
        self.assertEqual(
            shlex.split(rtabmap.attrib.get("launch-prefix", "")),
            [
                "rosrun", "stier_slam_core", "rtabmap_runtime_guard.py",
                "--release-manifest", "$(arg release_manifest)",
                "--map-yaml", "$(arg map_yaml)",
                "--source-database", "$(arg database_path)",
                "--source-config", "$(find stier_slam_bringup)/config/rtabmap_localization.ini",
                "--runtime-root", "$(arg runtime_root_dir)",
                "--runtime-bundle", "$(arg runtime_bundle_dir)",
                "--bundle-role", "rtabmap", "--",
            ],
        )
        self.assertEqual(
            remaps(rtabmap).get("localization_pose"), "/slam/localization/pose"
        )
        self.assertIn("live_obstacles", [node.attrib.get("name") for node in nodes(root)])
        self.assertEqual(odometry_node_names_with_publish_tf(root), {"ekf_local", "rtabmap"})
        for name in (
                "subscribe_depth", "subscribe_rgb", "subscribe_rgbd", "subscribe_stereo",
                "subscribe_scan", "subscribe_scan_cloud"):
            self.assertEqual(param_attributes(rtabmap)[name].get("type"), "bool")
        for name in ("Mem/IncrementalMemory", "Reg/Force3DoF"):
            self.assertEqual(param_attributes(rtabmap)[name].get("type"), "string")

    def test_localization_requires_immutable_release_inputs_and_critical_nodes(self):
        root = launch_tree("localization.launch")
        arguments = launch_arguments(root)
        for name in ("map_yaml", "database_path", "release_manifest"):
            self.assertIn(name, arguments)
            self.assertNotIn("default", arguments[name])
        self.assertEqual(
            arguments.get("runtime_root_dir", {}).get("default"),
            "$(env HOME)/.ros/stier_slam_runtime",
        )
        self.assertEqual(
            arguments["runtime_bundle_dir"].get("default"),
            "$(arg runtime_root_dir)/$(anon localization)",
        )
        self.assertNotIn("runtime_database_path", arguments)
        self.assertNotIn("runtime_config_path", arguments)
        for name in ("released_map_server", "wheel_odometry", "scan_leveler", "ekf_local", "rtabmap"):
            self.assertEqual(node_by_name(root, name).attrib.get("required"), "true")
        self.assertEqual(node_by_name(root, "released_map_server").attrib.get("required"), "true")

    def test_core_node_remaps_and_calibration_guard_are_explicit(self):
        root = launch_tree("mapping.launch")
        odometry = node_by_name(root, "wheel_odometry")
        self.assertEqual(odometry.attrib.get("type"), "vehicle_odometry_node.py")
        self.assertEqual(params(odometry).get("real_mode"), "$(arg real_mode)")
        self.assertEqual(params(odometry).get("calibration_required"), "$(arg calibration_required)")
        self.assertEqual(remaps(odometry), {})
        self.assertEqual(remaps(node_by_name(root, "scan_leveler")), {})

    def test_mapping_critical_pipeline_nodes_are_required(self):
        root = launch_tree("mapping.launch")
        self.assertEqual(
            launch_arguments(root).get("mapping_runtime_root", {}).get("default"),
            "$(env HOME)/.ros/stier_slam_mapping",
        )
        self.assertEqual(
            launch_arguments(root).get("mapping_session_dir", {}).get("default"),
            "$(arg mapping_runtime_root)/$(anon mapping)",
        )
        for name in ("wheel_odometry", "scan_leveler", "ekf_local", "rtabmap"):
            self.assertEqual(node_by_name(root, name).attrib.get("required"), "true")

    def test_local_ekf_only_fuses_wheel_forward_velocity_and_imu_yaw_rate(self):
        import yaml

        with open(os.path.join(CONFIG_ROOT, "ekf_local.yaml"), "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertTrue(config["two_d_mode"])
        self.assertEqual(config["world_frame"], "odom")
        topics = topic_contract()
        self.assertEqual(config["odom0"], topics["processed"]["wheel_odometry"])
        self.assertEqual(config["imu0"], topics["inputs"]["imu"])
        self.assertEqual(
            config["odom0_config"],
            [False, False, False, False, False, False, True, False, False,
             False, False, False, False, False, False],
        )
        self.assertEqual(
            config["imu0_config"],
            [False, False, False, False, False, True, False, False, False,
             False, False, True, False, False, False],
        )
        self.assertNotIn("gps0", config)

    def test_vehicle_pairing_timeout_is_finite_and_bounded(self):
        import math
        import yaml

        with open(os.path.join(CONFIG_ROOT, "vehicle.yaml"), "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        timeout_sec = config["control_pair_timeout_sec"]
        self.assertTrue(math.isfinite(timeout_sec))
        self.assertGreater(timeout_sec, 0.0)
        self.assertLessEqual(timeout_sec, 1.0)

    def test_rtabmap_ini_contracts_select_2d_icp_and_mapping_mode(self):
        for filename, incremental in (
                ("rtabmap_mapping.ini", "true"),
                ("rtabmap_localization.ini", "false")):
            parser = configparser.ConfigParser()
            parser.optionxform = str
            parser.read(os.path.join(CONFIG_ROOT, filename), encoding="utf-8")
            self.assertEqual(parser["Core"]["Mem/IncrementalMemory"], incremental)
            self.assertEqual(parser["Core"]["Reg/Strategy"], "1")
            self.assertEqual(parser["Core"]["Reg/Force3DoF"], "true")
            self.assertEqual(parser["Core"]["Icp/Strategy"], "1")
            self.assertEqual(parser["Core"]["Grid/Sensor"], "0")
            self.assertNotIn("Icp/PM", parser["Core"])
        parser = configparser.ConfigParser()
        parser.optionxform = str
        parser.read(os.path.join(CONFIG_ROOT, "rtabmap_mapping.ini"), encoding="utf-8")
        self.assertEqual(parser["Core"]["RGBD/CreateOccupancyGrid"], "true")

    def test_rviz_uses_pose_with_covariance_for_localization_pose(self):
        import yaml

        with open(os.path.join(PACKAGE_ROOT, "rviz", "slam.rviz"), "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        displays = document["Visualization Manager"]["Displays"]
        matching = [
            display for display in displays
            if display.get("Topic") == "/slam/localization/pose"
        ]
        self.assertEqual(len(matching), 1)
        self.assertEqual(matching[0].get("Class"), "rviz/PoseWithCovariance")

    def test_no_bringup_launch_contains_a_vendor_device_path(self):
        for filename in (
                "record.launch", "mapping.launch", "localization.launch",
                "synthetic_demo.launch"):
            with open(os.path.join(LAUNCH_ROOT, filename), "r", encoding="utf-8") as stream:
                self.assertNotIn("/dev/", stream.read())


def odometry_node_names_with_publish_tf(root):
    return {
        node.attrib.get("name")
        for node in nodes(root)
        if params(node).get("publish_tf") == "true"
    }


if __name__ == "__main__":
    import rostest
    rostest.rosrun("stier_slam_bringup", "launch_contracts", LaunchContractTest)
