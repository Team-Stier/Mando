import importlib.util
import hashlib
import http.client
import json
import os
from pathlib import Path
import shutil
import socket
import sqlite3
import subprocess
import tempfile
import threading
import unittest
from unittest import mock

from stier_slam_core import editor_server
from stier_slam_core import map_release as map_release_module
from stier_slam_core.map_release import ReleaseManager


def write_valid_rtabmap_database(path):
    connection = sqlite3.connect(str(path))
    try:
        connection.executescript("""
            CREATE TABLE Node(id INTEGER PRIMARY KEY, map_id INTEGER NOT NULL, weight INTEGER,
                stamp FLOAT, pose BLOB, ground_truth_pose BLOB, velocity BLOB, label TEXT,
                gps BLOB, env_sensors BLOB, time_enter DATE);
            CREATE TABLE Data(id INTEGER PRIMARY KEY, image BLOB, depth BLOB, calibration BLOB,
                scan BLOB, scan_info BLOB, ground_cells BLOB, obstacle_cells BLOB,
                empty_cells BLOB, cell_size FLOAT, view_point_x FLOAT, view_point_y FLOAT,
                view_point_z FLOAT, user_data BLOB, time_enter DATE);
            CREATE TABLE Link(from_id INTEGER NOT NULL, to_id INTEGER NOT NULL, type INTEGER NOT NULL,
                information_matrix BLOB NOT NULL, transform BLOB, user_data BLOB);
            CREATE TABLE Word(id INTEGER PRIMARY KEY, descriptor_size INTEGER NOT NULL,
                descriptor BLOB NOT NULL, time_enter DATE);
            CREATE TABLE Feature(node_id INTEGER NOT NULL, word_id INTEGER NOT NULL, pos_x FLOAT NOT NULL,
                pos_y FLOAT NOT NULL, size INTEGER NOT NULL, dir FLOAT NOT NULL,
                response FLOAT NOT NULL, octave INTEGER NOT NULL, depth_x FLOAT, depth_y FLOAT,
                depth_z FLOAT, descriptor_size INTEGER, descriptor BLOB);
            CREATE TABLE GlobalDescriptor(node_id INTEGER NOT NULL, type INTEGER NOT NULL,
                info BLOB, data BLOB NOT NULL);
            CREATE TABLE Info(STM_size INTEGER, last_sign_added INTEGER, process_mem_used INTEGER,
                database_mem_used INTEGER, dictionary_size INTEGER, parameters TEXT, time_enter DATE);
            CREATE TABLE Statistics(id INTEGER NOT NULL, stamp FLOAT, data BLOB, wm_state BLOB);
            CREATE TABLE Admin(version TEXT, preview_image BLOB, opt_cloud BLOB, opt_ids BLOB,
                opt_poses BLOB, opt_last_localization BLOB, opt_polygons_size INTEGER,
                opt_polygons BLOB, opt_tex_coords BLOB, opt_tex_materials BLOB, opt_map BLOB,
                opt_map_x_min FLOAT, opt_map_y_min FLOAT, opt_map_resolution FLOAT, time_enter DATE);
        """)
        connection.execute("INSERT INTO Admin(version) VALUES ('0.21.13')")
        connection.execute(
            "INSERT INTO Node VALUES (1,0,0,1.0,?,?,?,?,?,?,?)",
            (b"P" * 48, b"G" * 48, b"V" * 24, "node", b"gps", b"env", "2026-08-08"),
        )
        connection.execute(
            "INSERT INTO Data VALUES (1,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (b"image", b"depth", b"calibration", b"scan", b"scan-info", b"ground",
             b"obstacle", b"empty", 0.05, 0.0, 0.0, 0.0, b"user", "2026-08-08"),
        )
        connection.execute("INSERT INTO Link VALUES (1,1,0,?,?,?)", (b"I", b"T", b"user"))
        connection.commit()
    finally:
        connection.close()


class EditorModuleContractTest(unittest.TestCase):
    def test_editor_server_module_is_available(self):
        """The localhost editor needs an importable, ROS-independent server module."""
        self.assertIsNotNone(
            importlib.util.find_spec("stier_slam_core.editor_server"),
            "stier_slam_core.editor_server is not implemented",
        )

    def test_browser_core_parses_pgm_and_converts_map_coordinates(self):
        """Wrong PGM boundaries or Y inversion would place edits on the wrong course cells."""
        app_path = Path(__file__).parents[1] / "web" / "app.js"
        javascript = r"""
require(process.argv[1]);
const core = globalThis.StierMapEditorCore;
const encoder = new TextEncoder();
const concat = (...parts) => {
  const length = parts.reduce((total, part) => total + part.length, 0);
  const result = new Uint8Array(length);
  let offset = 0;
  for (const part of parts) { result.set(part, offset); offset += part.length; }
  return result;
};
const p2 = core.parsePgm(encoder.encode('P2\n# course map\n2 1\n255\n0 255\n'));
const p5 = core.parsePgm(concat(encoder.encode('P5\n2 1\n255\n'), new Uint8Array([0, 255])));
const p16 = core.parsePgm(concat(
  encoder.encode('P5\n# sixteen bit\n2 1\n1023\n'),
  new Uint8Array([0, 1, 3, 255])
));
const map = {width: 20, height: 10, resolution: 0.1, origin: [-1.0, -2.0, 0.0]};
console.log(JSON.stringify({
  p2: {width: p2.width, height: p2.height, maxValue: p2.maxValue,
       samples: Array.from(p2.samples)},
  p5: Array.from(p5.samples),
  p16: Array.from(p16.samples),
  mapPoint: core.pixelToMap(map, 5, 2, true),
  pixelPoint: core.mapToPixel(map, -0.5, -1.2, true),
  clamped: core.pixelToMap(map, -5, 99, true),
  editingAllowed: core.canEditCanvas(false, {}),
  editingBlocked: core.canEditCanvas(true, {}),
  editingNotReady: core.canEditCanvas(false, null)
}));
"""
        result = subprocess.run(
            ["node", "-e", javascript, str(app_path)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        document = json.loads(result.stdout)
        self.assertEqual(document["p2"], {
            "width": 2, "height": 1, "maxValue": 255, "samples": [0, 255],
        })
        self.assertEqual(document["p5"], [0, 255])
        self.assertEqual(document["p16"], [1, 1023])
        self.assertEqual(document["mapPoint"], [-0.5, -1.2])
        self.assertEqual(document["pixelPoint"], [5, 2])
        self.assertEqual(document["clamped"], [-1, -2])
        self.assertIs(document["editingAllowed"], True)
        self.assertIs(document["editingBlocked"], False)
        self.assertIs(document["editingNotReady"], False)

    def test_script_default_web_root_supports_source_and_install_layouts(self):
        """Direct rosrun must find web assets both before and after Catkin install."""
        script_path = Path(__file__).parents[1] / "scripts" / "map_editor_server.py"
        specification = importlib.util.spec_from_file_location(
            "stier_map_editor_script", script_path
        )
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
        self.assertTrue(
            hasattr(module, "_default_web_root"),
            "the executable needs a testable source/install web-root resolver",
        )
        self.assertEqual(
            module._default_web_root(script_path), Path(__file__).parents[1] / "web"
        )

        with tempfile.TemporaryDirectory() as temporary_directory:
            prefix = Path(temporary_directory)
            installed_script = prefix / "lib" / "stier_slam_core" / "map_editor_server.py"
            installed_script.parent.mkdir(parents=True)
            installed_script.touch()
            installed_web = prefix / "share" / "stier_slam_core" / "web"
            installed_web.mkdir(parents=True)
            self.assertEqual(module._default_web_root(installed_script), installed_web)

    def test_script_removes_only_ros_remapping_arguments(self):
        script_path = Path(__file__).parents[1] / "scripts" / "map_editor_server.py"
        specification = importlib.util.spec_from_file_location(
            "stier_map_editor_script_remaps", script_path
        )
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)

        self.assertEqual(
            module._without_ros_remapping_args([
                "__name:=map_editor",
                "/input:=/mapped",
                "_private:=value",
                "--course-dir",
                "/tmp/course:=literal",
                "9invalid:=value",
                "name with space:=value",
                "--bogus:=x",
            ]),
            [
                "--course-dir",
                "/tmp/course:=literal",
                "9invalid:=value",
                "name with space:=value",
                "--bogus:=x",
            ],
        )


class EditorApiTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.course = self.root / "course"
        source = self.root / "source"
        source.mkdir()
        database = source / "map.db"
        pgm = source / "map.pgm"
        map_yaml = source / "map.yaml"
        write_valid_rtabmap_database(database)
        self.pgm_bytes = b"P5\n4 3\n255\n" + bytes(
            (254, 254, 254, 254, 254, 205, 205, 254, 0, 0, 254, 254)
        )
        pgm.write_bytes(self.pgm_bytes)
        map_yaml.write_text(
            "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        manager = ReleaseManager()
        manager.init_capture(
            self.course, "capture-1", database, map_yaml, "2026-08-08T00:00:00Z"
        )
        manager.init_workspace(self.course, "capture-1", "workspace-1")
        self.web_root = self.root / "web"
        self.web_root.mkdir()
        (self.web_root / "index.html").write_bytes(b"<!doctype html><title>editor</title>")
        (self.web_root / "app.js").write_bytes(b"'use strict';\n")
        (self.web_root / "style.css").write_bytes(b"body{}\n")
        self.server = editor_server.create_editor_server(
            self.course,
            "workspace-1",
            self.web_root,
            host="127.0.0.1",
            port=0,
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.host, self.port = self.server.server_address[:2]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5.0)
        self.temporary_directory.cleanup()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection(self.host, self.port, timeout=5.0)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        content = response.read()
        result = response.status, {key.lower(): value for key, value in response.getheaders()}, content
        connection.close()
        return result

    def raw_request(self, request_bytes):
        with socket.create_connection((self.host, self.port), timeout=5.0) as connection:
            connection.sendall(request_bytes)
            response = b""
            while True:
                block = connection.recv(65536)
                if not block:
                    break
                response += block
        header, content = response.split(b"\r\n\r\n", 1)
        status = int(header.split(b"\r\n", 1)[0].split()[1])
        return status, content

    def test_non_loopback_bind_requires_explicit_unsafe_opt_in(self):
        """A configuration mistake must not expose the file editor to the network."""
        with self.assertRaises(ValueError):
            editor_server.create_editor_server(
                self.course,
                "workspace-1",
                self.web_root,
                host="0.0.0.0",
                port=0,
            )
        unsafe = editor_server.create_editor_server(
            self.course,
            "workspace-1",
            self.web_root,
            host="0.0.0.0",
            port=0,
            unsafe_allow_nonlocal=True,
        )
        unsafe.server_close()

    def test_get_workspace_and_base_pgm_return_verified_snapshots(self):
        """The browser must receive matching metadata, layers, revision, and immutable image."""
        status, headers, content = self.request("GET", "/api/workspace")
        document = json.loads(content.decode("utf-8"))

        self.assertEqual(status, 200)
        self.assertEqual(headers["content-type"], "application/json; charset=utf-8")
        self.assertEqual(headers["cache-control"], "no-store")
        self.assertEqual(headers["x-content-type-options"], "nosniff")
        self.assertNotIn("access-control-allow-origin", headers)
        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(document["workspace_id"], "workspace-1")
        self.assertEqual(document["map"]["image_sha256"], hashlib.sha256(self.pgm_bytes).hexdigest())

        status, headers, content = self.request("GET", "/api/base.pgm")
        self.assertEqual(status, 200)
        self.assertEqual(headers["content-type"], "image/x-portable-graymap")
        self.assertEqual(headers["cache-control"], "no-store")
        self.assertEqual(content, self.pgm_bytes)

    def test_server_pins_course_selected_during_startup(self):
        """Retargeting a supplied symlink must not redirect a running editor."""
        alternate_course = self.root / "alternate-course"
        shutil.copytree(self.course, alternate_course)
        alternate_manager = ReleaseManager()
        alternate_before = alternate_manager.read_workspace(
            alternate_course, "workspace-1"
        )
        alternate_manager.replace_workspace(
            alternate_course,
            "workspace-1",
            alternate_before["revision"],
            {"schema_version": 1, "operations": []},
            {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        )
        course_link = self.root / "selected-course"
        course_link.symlink_to(self.course, target_is_directory=True)
        pinned_server = editor_server.create_editor_server(
            course_link, "workspace-1", self.web_root, host="127.0.0.1", port=0
        )
        pinned_thread = threading.Thread(
            target=pinned_server.serve_forever, daemon=True
        )
        pinned_thread.start()
        try:
            course_link.unlink()
            course_link.symlink_to(alternate_course, target_is_directory=True)
            host, port = pinned_server.server_address[:2]
            connection = http.client.HTTPConnection(host, port, timeout=5.0)
            connection.request("GET", "/api/workspace")
            response = connection.getresponse()
            document = json.loads(response.read().decode("utf-8"))
            connection.close()

            self.assertEqual(response.status, 200)
            self.assertEqual(pinned_server.course_dir, self.course.resolve())
            self.assertEqual(document["semantic_layer"]["features"], [])
        finally:
            pinned_server.shutdown()
            pinned_server.server_close()
            pinned_thread.join(timeout=5.0)

    def test_server_holds_course_identity_after_path_replacement(self):
        """Replacing the canonical pathname must not redirect a running editor."""
        selected_course = self.root / "selected-real-course"
        replacement_course = self.root / "replacement-course"
        moved_course = self.root / "moved-course"
        shutil.copytree(self.course, selected_course)
        shutil.copytree(self.course, replacement_course)
        replacement_manager = ReleaseManager()
        replacement_before = replacement_manager.read_workspace(
            replacement_course, "workspace-1"
        )
        replacement_after = replacement_manager.replace_workspace(
            replacement_course,
            "workspace-1",
            replacement_before["revision"],
            {"schema_version": 1, "operations": []},
            {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        )
        pinned_server = editor_server.create_editor_server(
            selected_course,
            "workspace-1",
            self.web_root,
            host="127.0.0.1",
            port=0,
        )
        pinned_thread = threading.Thread(
            target=pinned_server.serve_forever, daemon=True
        )
        pinned_thread.start()
        try:
            selected_course.rename(moved_course)
            replacement_course.rename(selected_course)
            host, port = pinned_server.server_address[:2]
            connection = http.client.HTTPConnection(host, port, timeout=5.0)
            connection.request("GET", "/api/workspace")
            response = connection.getresponse()
            document = json.loads(response.read().decode("utf-8"))
            connection.close()

            self.assertEqual(response.status, 500)
            self.assertEqual(document["error"]["code"], "internal_error")
            self.assertEqual(
                replacement_manager.read_workspace(selected_course, "workspace-1"),
                replacement_after,
            )
        finally:
            pinned_server.shutdown()
            pinned_server.server_close()
            pinned_thread.join(timeout=5.0)

    def test_course_fd_remains_bound_during_manager_path_resolution(self):
        """A pathname swap after request preflight must still read the pinned course."""
        selected_course = self.root / "race-selected-course"
        replacement_course = self.root / "race-replacement-course"
        moved_course = self.root / "race-moved-course"
        shutil.copytree(self.course, selected_course)
        shutil.copytree(self.course, replacement_course)
        replacement_manager = ReleaseManager()
        replacement_before = replacement_manager.read_workspace(
            replacement_course, "workspace-1"
        )
        replacement_after = replacement_manager.replace_workspace(
            replacement_course,
            "workspace-1",
            replacement_before["revision"],
            {"schema_version": 1, "operations": []},
            {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        )
        pinned_server = editor_server.create_editor_server(
            selected_course,
            "workspace-1",
            self.web_root,
            host="127.0.0.1",
            port=0,
        )
        pinned_thread = threading.Thread(
            target=pinned_server.serve_forever, daemon=True
        )
        pinned_thread.start()
        real_course_path = map_release_module._course_path
        swapped = threading.Event()

        def swap_after_course_path(course_dir, allow_descriptor=False):
            resolved = real_course_path(
                course_dir, allow_descriptor=allow_descriptor
            )
            if (not swapped.is_set()
                    and str(course_dir).startswith("/proc/self/fd/")):
                selected_course.rename(moved_course)
                replacement_course.rename(selected_course)
                swapped.set()
            return resolved

        try:
            with mock.patch.object(
                    map_release_module,
                    "_course_path",
                    side_effect=swap_after_course_path):
                host, port = pinned_server.server_address[:2]
                connection = http.client.HTTPConnection(host, port, timeout=5.0)
                connection.request("GET", "/api/workspace")
                response = connection.getresponse()
                document = json.loads(response.read().decode("utf-8"))
                connection.close()

            self.assertTrue(swapped.is_set())
            self.assertEqual(response.status, 200)
            self.assertEqual(document["semantic_layer"]["features"], [])
            self.assertEqual(
                replacement_manager.read_workspace(selected_course, "workspace-1"),
                replacement_after,
            )
        finally:
            pinned_server.shutdown()
            pinned_server.server_close()
            pinned_thread.join(timeout=5.0)

    def test_inflight_request_keeps_private_course_fd_during_shutdown(self):
        """Closing the server must not let an active request reuse its course FD."""
        replacement_course = self.root / "shutdown-replacement-course"
        shutil.copytree(self.course, replacement_course)
        replacement_manager = ReleaseManager()
        replacement_before = replacement_manager.read_workspace(
            replacement_course, "workspace-1"
        )
        replacement_manager.replace_workspace(
            replacement_course,
            "workspace-1",
            replacement_before["revision"],
            {"schema_version": 1, "operations": []},
            {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        )
        pinned_server = editor_server.create_editor_server(
            self.course,
            "workspace-1",
            self.web_root,
            host="127.0.0.1",
            port=0,
        )
        pinned_thread = threading.Thread(
            target=pinned_server.serve_forever, daemon=True
        )
        entered_manager = threading.Event()
        continue_request = threading.Event()
        request_result = {}
        real_read_workspace = pinned_server.release_manager.read_workspace

        def paused_read_workspace(course_dir, workspace_id):
            entered_manager.set()
            if not continue_request.wait(timeout=10.0):
                raise RuntimeError("test did not release the paused request")
            return real_read_workspace(course_dir, workspace_id)

        def request_workspace():
            try:
                host, port = pinned_server.server_address[:2]
                connection = http.client.HTTPConnection(host, port, timeout=10.0)
                connection.request("GET", "/api/workspace")
                response = connection.getresponse()
                request_result["status"] = response.status
                request_result["document"] = json.loads(
                    response.read().decode("utf-8")
                )
                connection.close()
            except BaseException as error:
                request_result["error"] = error

        request_thread = threading.Thread(target=request_workspace, daemon=True)
        pinned_thread.start()
        close_finished = threading.Event()

        def close_server():
            pinned_server.server_close()
            close_finished.set()

        close_thread = threading.Thread(target=close_server, daemon=True)
        try:
            with mock.patch.object(
                    pinned_server.release_manager,
                    "read_workspace",
                    side_effect=paused_read_workspace):
                request_thread.start()
                self.assertTrue(entered_manager.wait(timeout=5.0))
                pinned_server.shutdown()
                pinned_thread.join(timeout=5.0)
                close_thread.start()
                self.assertFalse(close_finished.wait(timeout=0.25))
                continue_request.set()
                request_thread.join(timeout=10.0)
                close_thread.join(timeout=10.0)

            self.assertFalse(request_thread.is_alive())
            self.assertTrue(close_finished.is_set())
            self.assertNotIn("error", request_result)
            self.assertEqual(request_result["status"], 200)
            self.assertEqual(
                request_result["document"]["semantic_layer"]["features"], []
            )
        finally:
            continue_request.set()
            request_thread.join(timeout=10.0)
            close_thread.join(timeout=10.0)
            if pinned_thread.is_alive():
                pinned_server.shutdown()
                pinned_thread.join(timeout=5.0)
            pinned_server.server_close()

    def test_put_round_trips_both_layers_and_stale_revision_is_409(self):
        """HTTP save must expose ReleaseManager CAS rather than silently overwrite edits."""
        _, _, content = self.request("GET", "/api/workspace")
        before = json.loads(content.decode("utf-8"))
        request_document = {
            "schema_version": 1,
            "expected_revision": before["revision"],
            "static_overlay": {
                "schema_version": 1,
                "operations": [{
                    "operation": "occupied",
                    "polygon": [[0.0, 0.0], [0.2, 0.0], [0.2, 0.2], [0.0, 0.2]],
                }],
            },
            "semantic_layer": {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        }
        body = json.dumps(request_document, separators=(",", ":")).encode("utf-8")
        status, _, content = self.request(
            "PUT", "/api/workspace", body, {"Content-Type": "application/json"}
        )
        after = json.loads(content.decode("utf-8"))

        self.assertEqual(status, 200)
        self.assertNotEqual(after["revision"], before["revision"])
        self.assertEqual(after["static_overlay"], request_document["static_overlay"])
        self.assertEqual(after["semantic_layer"], request_document["semantic_layer"])

        status, _, content = self.request(
            "PUT", "/api/workspace", body, {"Content-Type": "application/json"}
        )
        error = json.loads(content.decode("utf-8"))
        self.assertEqual(status, 409)
        self.assertEqual(error["error"]["code"], "revision_conflict")

    def test_post_release_returns_hashes_and_activates(self):
        """The release endpoint must return hashes for the immutable artifact it selected."""
        _, _, content = self.request("GET", "/api/workspace")
        workspace = json.loads(content.decode("utf-8"))
        body = json.dumps({
            "schema_version": 1,
            "expected_revision": workspace["revision"],
            "release_id": "editor-release",
            "activate": True,
        }).encode("utf-8")

        status, _, content = self.request(
            "POST", "/api/release", body, {"Content-Type": "application/json"}
        )
        summary = json.loads(content.decode("utf-8"))

        self.assertEqual(status, 201)
        self.assertEqual(summary["release_id"], "editor-release")
        self.assertIs(summary["activated"], True)
        self.assertEqual(len(summary["manifest_sha256"]), 64)
        self.assertEqual(set(summary["artifact_hashes"]), {
            "map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json",
        })
        active = json.loads(
            (self.course / "active_release.json").read_text(encoding="utf-8")
        )
        self.assertEqual(active["release_id"], "editor-release")

    def test_post_release_reports_committed_parent_fsync_ambiguity(self):
        """HTTP must preserve hashes and retry guidance after a post-rename failure."""
        _, _, content = self.request("GET", "/api/workspace")
        workspace = json.loads(content.decode("utf-8"))
        real_publish = map_release_module._rename_directory_noreplace
        real_fsync = map_release_module.os.fsync
        published = False
        failed = False

        def publish_then_mark(*arguments, **keywords):
            nonlocal published
            result = real_publish(*arguments, **keywords)
            published = True
            return result

        def fail_first_post_publish_fsync(descriptor):
            nonlocal failed
            if published and not failed:
                failed = True
                raise OSError("injected HTTP release fsync failure")
            return real_fsync(descriptor)

        body = json.dumps({
            "schema_version": 1,
            "expected_revision": workspace["revision"],
            "release_id": "http-publication-indeterminate",
            "activate": True,
        }).encode("utf-8")
        with mock.patch.object(
                map_release_module,
                "_rename_directory_noreplace",
                side_effect=publish_then_mark), mock.patch.object(
                    map_release_module.os,
                    "fsync",
                    side_effect=fail_first_post_publish_fsync):
            status, _, content = self.request(
                "POST", "/api/release", body,
                {"Content-Type": "application/json"},
            )

        summary = json.loads(content.decode("utf-8"))
        self.assertEqual(status, 201)
        self.assertTrue(failed)
        self.assertIs(summary["activated"], False)
        self.assertEqual(
            summary["error"]["code"], "release_publication_indeterminate"
        )
        self.assertEqual(len(summary["manifest_sha256"]), 64)
        self.assertFalse((self.course / "active_release.json").exists())

    def test_strict_json_and_request_schema_fail_without_mutating_workspace(self):
        """Permissive JSON or unknown request fields must not reach the workspace store."""
        before = ReleaseManager().read_workspace(self.course, "workspace-1")
        invalid_bodies = (
            b'{"schema_version":1,"schema_version":1,"expected_revision":"x",'
            b'"static_overlay":{},"semantic_layer":{}}',
            b'{"schema_version":1,"expected_revision":NaN,'
            b'"static_overlay":{},"semantic_layer":{}}',
            json.dumps({
                "schema_version": 1,
                "expected_revision": before["revision"],
                "static_overlay": {"schema_version": 1, "operations": []},
                "semantic_layer": {"schema_version": 1, "features": []},
                "unknown": True,
            }).encode("utf-8"),
        )
        for body in invalid_bodies:
            with self.subTest(body=body):
                status, _, content = self.request(
                    "PUT", "/api/workspace", body, {"Content-Type": "application/json"}
                )
                self.assertEqual(status, 400)
                self.assertIn(
                    json.loads(content.decode("utf-8"))["error"]["code"],
                    {"invalid_json", "invalid_request"},
                )
                self.assertEqual(
                    ReleaseManager().read_workspace(self.course, "workspace-1"), before
                )

    def test_body_framing_size_and_media_type_are_fail_closed(self):
        """Ambiguous framing and oversized or non-JSON input must be rejected before parsing."""
        status, _, content = self.request(
            "PUT", "/api/workspace", b"{}", {"Content-Type": "text/plain"}
        )
        self.assertEqual(status, 415)
        self.assertEqual(json.loads(content)["error"]["code"], "unsupported_media_type")

        status, _, content = self.request(
            "PUT",
            "/api/workspace",
            b"",
            {
                "Content-Type": "application/json",
                "Content-Length": str(editor_server.MAX_REQUEST_BYTES + 1),
            },
        )
        self.assertEqual(status, 413)
        self.assertEqual(json.loads(content)["error"]["code"], "request_too_large")

        host = "{}:{}".format(self.host, self.port).encode("ascii")
        missing_length = (
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + host
            + b"\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
        )
        status, content = self.raw_request(missing_length)
        self.assertEqual(status, 411)
        self.assertEqual(json.loads(content)["error"]["code"], "length_required")

        duplicate_length = (
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + host
            + b"\r\nContent-Type: application/json\r\nContent-Length: 2\r\n"
            + b"Content-Length: 2\r\nConnection: close\r\n\r\n{}"
        )
        status, content = self.raw_request(duplicate_length)
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(content)["error"]["code"], "invalid_framing")

        transfer_encoding = (
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + host
            + b"\r\nContent-Type: application/json\r\nTransfer-Encoding: chunked\r\n"
            + b"Connection: close\r\n\r\n0\r\n\r\n"
        )
        status, content = self.raw_request(transfer_encoding)
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(content)["error"]["code"], "invalid_framing")

    def test_rejection_before_body_read_closes_the_http_connection(self):
        """Unread attacker bytes must not be interpreted as a second keep-alive request."""
        connection = http.client.HTTPConnection(self.host, self.port, timeout=5.0)
        connection.request(
            "PUT",
            "/api/workspace",
            body=b"not-json",
            headers={"Content-Type": "text/plain"},
        )
        try:
            response = connection.getresponse()
            response.read()
            self.assertEqual(response.status, 415)
            self.assertEqual(response.getheader("Connection"), "close")
        finally:
            connection.close()

    def test_host_origin_and_cors_policy_blocks_browser_cross_site_writes(self):
        """DNS rebinding and cross-origin requests must not mutate localhost files."""
        connection = http.client.HTTPConnection(self.host, self.port, timeout=5.0)
        connection.putrequest("GET", "/api/workspace", skip_host=True)
        connection.putheader("Host", "attacker.invalid")
        connection.endheaders()
        response = connection.getresponse()
        content = response.read()
        self.assertEqual(response.status, 403)
        self.assertEqual(json.loads(content)["error"]["code"], "forbidden_host")
        connection.close()

        body = b"{}"
        status, headers, content = self.request(
            "PUT",
            "/api/workspace",
            body,
            {"Content-Type": "application/json", "Origin": "https://attacker.invalid"},
        )
        self.assertEqual(status, 403)
        self.assertEqual(json.loads(content)["error"]["code"], "forbidden_origin")
        self.assertNotIn("access-control-allow-origin", headers)

        status, _, _ = self.request("OPTIONS", "/api/workspace")
        self.assertEqual(status, 405)

    def test_static_allowlist_rejects_traversal_queries_and_dotfiles(self):
        """Static serving must never derive a filesystem path from the request URL."""
        for path in ("/", "/index.html", "/app.js", "/style.css"):
            with self.subTest(path=path):
                status, headers, _ = self.request("GET", path)
                self.assertEqual(status, 200)
                self.assertIn("content-security-policy", headers)
        for path in (
                "/../index.html", "/%2e%2e/index.html", "/.git/config",
                "/index.html?path=../secret", "/web/"):
            with self.subTest(path=path):
                status, _, content = self.request("GET", path)
                self.assertEqual(status, 404)
                self.assertEqual(json.loads(content)["error"]["code"], "not_found")

    def test_release_id_traversal_and_existing_id_return_deterministic_errors(self):
        """A client-controlled release ID must neither escape nor overwrite a release."""
        workspace = ReleaseManager().read_workspace(self.course, "workspace-1")

        def release(release_id, revision=workspace["revision"]):
            return self.request(
                "POST",
                "/api/release",
                json.dumps({
                    "schema_version": 1,
                    "expected_revision": revision,
                    "release_id": release_id,
                    "activate": False,
                }).encode("utf-8"),
                {"Content-Type": "application/json"},
            )

        status, _, content = release("../escape")
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(content)["error"]["code"], "validation_failed")
        self.assertFalse((self.root / "escape").exists())

        status, _, content = release("stable-release")
        self.assertEqual(status, 201)
        release_path = self.course / "releases" / "stable-release"
        original = {path.name: path.read_bytes() for path in release_path.iterdir()}

        status, _, content = release("stable-release")
        self.assertEqual(status, 201)
        self.assertEqual(json.loads(content)["release_id"], "stable-release")
        self.assertEqual(
            {path.name: path.read_bytes() for path in release_path.iterdir()}, original
        )

        newer = ReleaseManager().replace_workspace(
            self.course,
            "workspace-1",
            workspace["revision"],
            workspace["static_overlay"],
            workspace["semantic_layer"],
        )
        status, _, content = release("stable-release", newer["revision"])
        self.assertEqual(status, 409)
        self.assertEqual(json.loads(content)["error"]["code"], "release_exists")
        self.assertEqual(
            {path.name: path.read_bytes() for path in release_path.iterdir()}, original
        )


if __name__ == "__main__":
    unittest.main()
