#!/usr/bin/env python3
"""운영 설정과 문서가 합의된 Localization 계약을 유지하는지 검사한다."""

import pathlib
import struct
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


def load_yaml(name):
    with (PACKAGE / "config" / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class ConfigurationContractTest(unittest.TestCase):
    def test_encoder_message_contract(self):
        message_path = PACKAGE.parent / "erp42_msgs" / "msg" / "SerialFeedBack.msg"
        lines = [
            line.split("#", 1)[0].strip()
            for line in message_path.read_text(encoding="utf-8").splitlines()
        ]
        fields = [line for line in lines if line]
        self.assertEqual(
            [
                "uint8 MorA",
                "uint8 EStop",
                "uint8 Gear",
                "float64 speed",
                "float64 steer",
                "int16 brake",
                "int32 encoder",
                "uint8 alive",
            ],
            fields,
        )
        drive_path = PACKAGE.parent / "erp42_msgs" / "msg" / "DriveCmd.msg"
        drive_fields = [
            line.split("#", 1)[0].strip()
            for line in drive_path.read_text(encoding="utf-8").splitlines()
            if line.split("#", 1)[0].strip()
        ]
        self.assertEqual(["uint16 KPH", "int16 Deg", "uint8 brake"], drive_fields)

    def test_encoder_rosserial_driver_is_wired_to_the_connected_uno(self):
        driver = load_yaml("encoder_driver.yaml")
        self.assertEqual(
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_"
            "11254501101131313365-if00",
            driver["port"],
        )
        self.assertEqual(57600, driver["baud"])
        self.assertEqual("11254501101131313365", driver["hardware"]["usb_serial"])
        self.assertEqual("2341:0043", driver["hardware"]["usb_vid_pid"])

        sensors = ET.parse(PACKAGE / "launch" / "sensors.launch").getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in sensors.findall("arg")
        }
        self.assertEqual("true", arguments["start_encoder_driver"])
        encoder_node = sensors.find(".//node[@pkg='rosserial_python']")
        self.assertIsNotNone(encoder_node)
        self.assertEqual("serial_node.py", encoder_node.attrib["type"])
        self.assertEqual(
            "$(arg encoder_driver_node_name)", encoder_node.attrib["name"]
        )
        self.assertEqual(
            "$(arg encoder_driver_config)",
            encoder_node.find("rosparam").attrib["file"],
        )

        bringup = (PACKAGE / "launch" / "bringup.launch").read_text(encoding="utf-8")
        self.assertIn('name="start_encoder_driver" default="true"', bringup)
        self.assertIn(
            'name="encoder_driver_config" value="$(arg encoder_driver_config)"',
            bringup,
        )
        dependencies = {
            element.text
            for element in ET.parse(PACKAGE / "package.xml").getroot().findall("exec_depend")
        }
        self.assertIn("rosserial_python", dependencies)

    def test_public_interfaces_and_units(self):
        config = load_yaml("localization_interfaces.yaml")
        self.assertEqual(1, config["schema_version"])
        self.assertEqual("/molit/localization/odometry", config["topics"]["output_odometry"])
        self.assertEqual("map", config["frames"]["map"])
        self.assertEqual("base_link", config["frames"]["base_link"])
        self.assertEqual("mando_encoder_serial", config["nodes"]["encoder_driver"])
        self.assertEqual("/erp42_serial/feedback", config["topics"]["encoder_state"])
        self.assertEqual("erp42_msgs/SerialFeedBack", config["message_types"]["encoder_state"])
        self.assertEqual("m/s", config["contracts"]["speed_unit"])
        self.assertEqual("rad/s", config["contracts"]["angular_velocity_unit"])
        public_topics = list(config["topics"].values())
        self.assertEqual(len(public_topics), len(set(public_topics)))
        internal_topics = list(config["internal_topics"].values())
        self.assertEqual(len(internal_topics), len(set(internal_topics)))
        self.assertFalse(set(public_topics) & set(internal_topics))
        self.assertEqual("localization_supervisor", config["nodes"]["supervisor"])
        self.assertEqual(
            "mando_localization/GpsGateReanchor",
            config["message_types"]["gps_gate_reanchor"],
        )
        self.assertNotEqual(
            config["services"]["local_ekf_set_pose"],
            config["services"]["global_ekf_set_pose"],
        )

    def test_external_nodes_use_fixed_internal_topics(self):
        adapter = (PACKAGE / "src" / "common" / "localization_interface_adapter.cpp").read_text(
            encoding="utf-8"
        )
        sensors = (PACKAGE / "launch" / "sensors.launch").read_text(encoding="utf-8")
        lidar = (PACKAGE / "launch" / "lidar_localization.launch").read_text(
            encoding="utf-8"
        )
        for topic in (
            "/mando_localization/internal/driver/gps_navpvt",
            "/mando_localization/internal/amcl/map",
            "/mando_localization/internal/amcl/initialpose",
        ):
            self.assertIn(topic, adapter)
        self.assertIn('from="~fix"', sensors)
        self.assertIn('from="~navpvt"', sensors)
        self.assertIn("/mando_localization/internal/driver/gps_navpvt", sensors)
        for topic in (
            "/mando_localization/internal/amcl/map",
            "/mando_localization/internal/amcl/scan",
            "/mando_localization/internal/amcl/pose",
            "/mando_localization/internal/amcl/initialpose",
        ):
            self.assertIn(topic, lidar)

    def test_tf_has_single_dynamic_owners_and_unmeasured_static_sensors(self):
        config = load_yaml("tf_configuration.yaml")
        reference = config["base_link_reference"]
        self.assertEqual("rear_axle_center", reference["origin"])
        self.assertEqual("forward", reference["x_axis"])
        self.assertEqual("left", reference["y_axis"])
        self.assertEqual("up", reference["z_axis"])
        dynamic = {
            (item["parent_frame"], item["child_frame"]): item["owner_node"]
            for item in config["dynamic_transforms"]
        }
        self.assertEqual("odometry_gps_lidar_global_ekf", dynamic[("map", "odom")])
        self.assertEqual("imu_encoder_local_ekf", dynamic[("odom", "base_link")])
        children = set()
        for transform in config["static_transforms"]:
            self.assertNotIn(transform["child_frame"], children)
            children.add(transform["child_frame"])
            self.assertFalse(transform["enabled"])
            self.assertEqual("unmeasured", transform["calibration_state"])
        static = {item["child_frame"]: item for item in config["static_transforms"]}
        self.assertEqual([0.25, 0.0, 0.0], static["imu_link"]["translation_m"])
        self.assertEqual([1.05, 0.0, 0.0], static["laser_link"]["translation_m"])

    def test_filters_and_amcl_obey_tf_contract(self):
        local = load_yaml("ekf_local.yaml")
        global_filter = load_yaml("ekf_global.yaml")
        amcl = load_yaml("lidar_localization.yaml")
        self.assertEqual("odom", local["world_frame"])
        self.assertTrue(local["publish_tf"])
        self.assertEqual("map", global_filter["world_frame"])
        self.assertTrue(global_filter["publish_tf"])
        self.assertFalse(amcl["tf_broadcast"])
        self.assertNotIn("odom0", global_filter)

    def test_measured_imu_identity_and_fail_closed_covariance(self):
        imu = load_yaml("imu_driver.yaml")
        self.assertEqual("/dev/imu", imu["port"])
        self.assertEqual("03889250", imu["device_id"])
        self.assertEqual(115200, imu["baudrate"])
        self.assertTrue(imu["covariance_override"]["enabled"])
        self.assertEqual("measured", imu["covariance_override"]["calibration_state"])
        self.assertTrue(imu["covariance_override"]["measured_at"])
        self.assertIn("imu_stationary_20260831", imu["covariance_override"]["source"])
        self.assertTrue(imu["imu"]["require_positive_covariance_diagonal"])

    def test_gps_driver_forces_volatile_ubx_navpvt(self):
        gps = load_yaml("gps_driver.yaml")
        self.assertEqual("/dev/mando_gps", gps["device"])
        self.assertEqual(460800, gps["uart1"]["baudrate"])
        self.assertEqual(1, gps["uart1"]["in"])
        self.assertEqual(1, gps["uart1"]["out"])
        self.assertTrue(gps["config_on_startup"])
        self.assertFalse(gps["save_on_shutdown"])
        self.assertEqual("both", gps["fix_mode"])
        self.assertTrue(gps["publish"]["nav"]["pvt"])
        self.assertFalse(gps["publish"]["nav"]["relposned"])

    def test_operational_datum_is_not_faked(self):
        reference = load_yaml("gps_reference.yaml")["reference"]
        self.assertEqual("first_fix", reference["mode"])
        self.assertFalse(reference["measured"])
        self.assertEqual("", reference["measured_at"])
        self.assertEqual("", reference["source"])
        self.assertEqual(
            2.0,
            load_yaml("gps_reference.yaml")["quality"]["max_reanchor_candidate_distance_m"],
        )

    def test_dead_reckoning_is_bounded(self):
        policy = load_yaml("status_policy.yaml")
        self.assertEqual(2.0, policy["dead_reckoning"]["max_duration_sec"])
        self.assertEqual(10.0, policy["dead_reckoning"]["max_distance_m"])
        self.assertEqual("first_exceeded", policy["dead_reckoning"]["limit_policy"])
        self.assertFalse(policy["output_gate"]["publish_last_pose_when_invalid"])

    def test_long_outage_recovery_is_bounded_and_gps_only_is_disabled(self):
        policy = load_yaml("relocalization_policy.yaml")["relocalization"]
        self.assertEqual(2.0, policy["long_outage_sec"])
        self.assertEqual(3, policy["lidar_assisted"]["required_consecutive_candidates"])
        self.assertEqual(5.0, policy["lidar_assisted"]["max_gps_lidar_distance_m"])
        self.assertEqual(0.20, policy["lidar_assisted"]["max_timestamp_skew_sec"])
        self.assertEqual(1.0, policy["reanchor_ack_timeout_sec"])
        self.assertFalse(policy["gps_only"]["automatic_reset_enabled"])
        self.assertTrue(policy["gps_only"]["require_measured_datum"])
        self.assertEqual(5, policy["gps_only"]["required_consecutive_candidates"])
        self.assertEqual(3, policy["gps_only"]["required_global_confirmations"])

    def test_supervisor_owns_public_status_and_ekf_reset_services_are_split(self):
        safety = (PACKAGE / "launch" / "safety_and_tf.launch").read_text(
            encoding="utf-8"
        )
        local = (PACKAGE / "launch" / "local_fusion.launch").read_text(encoding="utf-8")
        global_fusion = (PACKAGE / "launch" / "global_fusion.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('type="localization_supervisor_node"', safety)
        self.assertIn('name="start_supervisor"', safety)
        self.assertIn('if="$(arg start_supervisor)"', safety)
        self.assertIn("relocalization_policy_config", safety)
        self.assertNotIn('type="localization_status_manager_node"', safety)
        self.assertIn("/mando_localization/internal/ekf/local_set_pose", local)
        self.assertIn("/mando_localization/internal/ekf/global_set_pose", global_fusion)
        self.assertTrue((PACKAGE / "msg" / "GpsGateReanchor.msg").is_file())

    def test_lidar_is_disabled_without_real_map(self):
        launch = ET.parse(PACKAGE / "launch" / "lidar_localization.launch").getroot()
        argument = next(item for item in launch.findall("arg") if item.attrib["name"] == "enable_lidar_localization")
        self.assertEqual("false", argument.attrib["default"])
        self.assertFalse((PACKAGE / "maps" / "map.yaml").exists())

    def test_launch_files_are_valid_xml(self):
        launch_files = list((PACKAGE / "launch").glob("*.launch"))
        self.assertGreaterEqual(len(launch_files), 5)
        for launch_file in launch_files:
            with self.subTest(launch=launch_file.name):
                self.assertEqual("launch", ET.parse(launch_file).getroot().tag)

    def test_navsat_transform_is_not_a_second_gps_path(self):
        self.assertFalse((PACKAGE / "config" / "navsat_transform.yaml").exists())
        launch_and_config = "\n".join(
            path.read_text(encoding="utf-8")
            for folder in ("launch", "config")
            for path in (PACKAGE / folder).glob("*")
            if path.is_file()
        )
        self.assertNotIn("navsat_transform", launch_and_config)

    def test_required_document_set_exists(self):
        for relative_path in (
            "docs/README.md",
            "docs/architecture.md",
            "docs/configuration.md",
            "docs/tf_frames.md",
            "docs/gps_quality_and_recovery.md",
            "docs/status_and_recovery.md",
        ):
            with self.subTest(path=relative_path):
                self.assertGreater((PACKAGE / relative_path).stat().st_size, 200)
        gps_document = (PACKAGE / "docs" / "gps_quality_and_recovery.md").read_text(
            encoding="utf-8"
        )
        for token in (
            "STATUS_NO_FIX",
            "STATUS_SBAS_FIX",
            "STATUS_GBAS_FIX",
            "COVARIANCE_TYPE_UNKNOWN",
            "10.0 m",
            "GpsGateReanchor",
            "transaction_id",
            "automatic_reset_enabled: false",
        ):
            self.assertIn(token, gps_document)

    def test_mermaid_artifacts_exist(self):
        source = PACKAGE / "docs" / "localization_architecture.mmd"
        svg = PACKAGE / "docs" / "images" / "localization_architecture.svg"
        png = PACKAGE / "docs" / "images" / "localization_architecture.png"
        self.assertGreater(source.stat().st_size, 500)
        source_text = source.read_text(encoding="utf-8")
        self.assertIn("LocalizationStatusManager", source_text)
        self.assertIn("RelocalizationCoordinator", source_text)
        self.assertIn("LocalizationSupervisor", source_text)
        svg_text = svg.read_text(encoding="utf-8")
        self.assertTrue(svg_text.startswith("<svg"))
        svg_root = ET.fromstring(svg_text)
        for element in svg_root.iter():
            with self.subTest(svg_path=element.attrib.get("id", element.tag)):
                self.assertNotRegex(element.attrib.get("d", ""), r"[QC]")
        png_bytes = png.read_bytes()
        self.assertEqual(b"\x89PNG\r\n\x1a\n", png_bytes[:8])
        width, height = struct.unpack(">II", png_bytes[16:24])
        self.assertGreaterEqual(width, 3000)
        self.assertGreaterEqual(height, 2000)
        self.assertGreater(png.stat().st_size, 1000)

        detailed_source = PACKAGE / "docs" / "localization_architecture_detailed.mmd"
        detailed_svg = PACKAGE / "docs" / "images" / "localization_architecture_detailed.svg"
        detailed_png = PACKAGE / "docs" / "images" / "localization_architecture_detailed.png"
        detailed_source_text = detailed_source.read_text(encoding="utf-8")
        for token in (
            "quality candidate",
            "transaction_id + stamp + XY 일치",
            "/internal/ekf/global_set_pose",
            "post-reset 증가 stamp 결과 3회",
            "evaluated status / state / valid",
            "recovery state / active",
        ):
            self.assertIn(token, detailed_source_text)
        detailed_svg_text = detailed_svg.read_text(encoding="utf-8")
        self.assertTrue(detailed_svg_text.startswith("<svg"))
        detailed_png_bytes = detailed_png.read_bytes()
        self.assertEqual(b"\x89PNG\r\n\x1a\n", detailed_png_bytes[:8])
        detailed_width, detailed_height = struct.unpack(
            ">II", detailed_png_bytes[16:24]
        )
        self.assertGreaterEqual(detailed_width, 1800)
        self.assertGreaterEqual(detailed_height, 2600)
        self.assertGreater(detailed_png.stat().st_size, 1000)

        relocalization_source = (
            PACKAGE / "docs" / "relocalization_manager_architecture.mmd"
        )
        relocalization_svg = (
            PACKAGE / "docs" / "images" / "relocalization_manager_architecture.svg"
        )
        relocalization_png = (
            PACKAGE / "docs" / "images" / "relocalization_manager_architecture.png"
        )
        relocalization_source_text = relocalization_source.read_text(
            encoding="utf-8"
        )
        for token in (
            "RelocalizationCoordinator + RelocalizationPolicy",
            "마지막 gate 승인 후 > 2.0 s",
            "GPS-only로 우회하지 않음",
            "GpsGateReanchor transaction_id + target pose",
            "active=true → RELOCALIZING, valid=false",
        ):
            self.assertIn(token, relocalization_source_text)
        relocalization_svg_text = relocalization_svg.read_text(encoding="utf-8")
        self.assertTrue(relocalization_svg_text.startswith("<svg"))
        relocalization_svg_root = ET.fromstring(relocalization_svg_text)
        for element in relocalization_svg_root.iter():
            with self.subTest(
                relocalization_svg_path=element.attrib.get("id", element.tag)
            ):
                self.assertNotRegex(element.attrib.get("d", ""), r"[QC]")
        relocalization_png_bytes = relocalization_png.read_bytes()
        self.assertEqual(b"\x89PNG\r\n\x1a\n", relocalization_png_bytes[:8])
        relocalization_width, relocalization_height = struct.unpack(
            ">II", relocalization_png_bytes[16:24]
        )
        self.assertGreaterEqual(relocalization_width, 3800)
        self.assertGreaterEqual(relocalization_height, 6000)
        self.assertGreater(relocalization_png.stat().st_size, 1000)

        flow_source = PACKAGE / "docs" / "relocalization_manager_flow.mmd"
        flow_svg = PACKAGE / "docs" / "images" / "relocalization_manager_flow.svg"
        flow_png = PACKAGE / "docs" / "images" / "relocalization_manager_flow.png"
        flow_source_text = flow_source.read_text(encoding="utf-8")
        for token in (
            "간단 아키텍처 Flow",
            "LiDAR 보조 복구",
            "GPS-only 복구",
            "transaction ACK",
            "/molit/localization/odometry",
        ):
            self.assertIn(token, flow_source_text)
        flow_svg_text = flow_svg.read_text(encoding="utf-8")
        self.assertTrue(flow_svg_text.startswith("<svg"))
        flow_svg_root = ET.fromstring(flow_svg_text)
        for element in flow_svg_root.iter():
            with self.subTest(flow_svg_path=element.attrib.get("id", element.tag)):
                self.assertNotRegex(element.attrib.get("d", ""), r"[QC]")
        flow_png_bytes = flow_png.read_bytes()
        self.assertEqual(b"\x89PNG\r\n\x1a\n", flow_png_bytes[:8])
        flow_width, flow_height = struct.unpack(">II", flow_png_bytes[16:24])
        self.assertGreaterEqual(flow_width, 3800)
        self.assertGreaterEqual(flow_height, 600)
        self.assertGreater(flow_png.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
