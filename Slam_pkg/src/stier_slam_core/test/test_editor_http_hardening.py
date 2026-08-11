import http.client
import json
import multiprocessing
import os
from pathlib import Path
import signal
import socket
import sqlite3
import tempfile
import threading
import unittest
from unittest import mock

from stier_slam_core import editor_server
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
            CREATE TABLE Feature(node_id INTEGER NOT NULL, word_id INTEGER NOT NULL,
                pos_x FLOAT NOT NULL, pos_y FLOAT NOT NULL, size INTEGER NOT NULL,
                dir FLOAT NOT NULL, response FLOAT NOT NULL, octave INTEGER NOT NULL,
                depth_x FLOAT, depth_y FLOAT, depth_z FLOAT, descriptor_size INTEGER,
                descriptor BLOB);
            CREATE TABLE GlobalDescriptor(node_id INTEGER NOT NULL, type INTEGER NOT NULL,
                info BLOB, data BLOB NOT NULL);
            CREATE TABLE Info(STM_size INTEGER, last_sign_added INTEGER, process_mem_used INTEGER,
                database_mem_used INTEGER, dictionary_size INTEGER, parameters TEXT,
                time_enter DATE);
            CREATE TABLE Statistics(id INTEGER NOT NULL, stamp FLOAT, data BLOB, wm_state BLOB);
            CREATE TABLE Admin(version TEXT, preview_image BLOB, opt_cloud BLOB, opt_ids BLOB,
                opt_poses BLOB, opt_last_localization BLOB, opt_polygons_size INTEGER,
                opt_polygons BLOB, opt_tex_coords BLOB, opt_tex_materials BLOB, opt_map BLOB,
                opt_map_x_min FLOAT, opt_map_y_min FLOAT, opt_map_resolution FLOAT,
                time_enter DATE);
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


def run_editor_with_blocked_worker(course, web_root, status_pipe):
    """Subprocess fixture proving worker policy bounds interpreter shutdown."""
    editor_server.SERVER_CLOSE_TIMEOUT_SECONDS = 0.1
    server = editor_server.create_editor_server(
        course, "workspace", web_root, host="127.0.0.1", port=0
    )
    never_finish = threading.Event()

    def blocked_read_workspace(course_dir, workspace_id):
        del course_dir, workspace_id
        status_pipe.send(("entered", None))
        never_finish.wait()

    def stop_process(signum, frame):
        del signum, frame
        raise KeyboardInterrupt

    server.release_manager.read_workspace = blocked_read_workspace
    signal.signal(signal.SIGTERM, stop_process)
    status_pipe.send(("ready", server.server_address[1]))
    try:
        server.serve_forever(poll_interval=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        status_pipe.close()


class EditorHttpHardeningRedTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.course = self.root / "course"
        inputs = self.root / "inputs"
        inputs.mkdir()
        database = inputs / "map.db"
        image = inputs / "map.pgm"
        map_yaml = inputs / "map.yaml"
        write_valid_rtabmap_database(database)
        image.write_bytes(b"P5\n2 2\n255\n" + bytes((254, 205, 0, 254)))
        map_yaml.write_text(
            "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        manager = ReleaseManager()
        manager.init_capture(
            self.course, "capture", database, map_yaml, "2026-08-08T00:00:00Z"
        )
        manager.init_workspace(self.course, "capture", "workspace")
        self.web_root = Path(__file__).parents[1] / "web"
        self.server = editor_server.create_editor_server(
            self.course, "workspace", self.web_root, host="127.0.0.1", port=0
        )
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self.host, self.port = self.server.server_address[:2]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=5.0)
        self.temporary_directory.cleanup()

    @property
    def host_header(self):
        return "{}:{}".format(self.host, self.port).encode("ascii")

    def raw_exchange(self, request):
        with socket.create_connection((self.host, self.port), timeout=3.0) as connection:
            connection.settimeout(3.0)
            connection.sendall(request)
            connection.shutdown(socket.SHUT_WR)
            blocks = []
            while True:
                block = connection.recv(65536)
                if not block:
                    break
                blocks.append(block)
        return b"".join(blocks)

    def restart_server(self):
        """Recreate the fixture server after a test patches construction limits."""
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=5.0)
        self.server = editor_server.create_editor_server(
            self.course, "workspace", self.web_root, host="127.0.0.1", port=0
        )
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self.host, self.port = self.server.server_address[:2]

    @staticmethod
    def first_response(response):
        header, remainder = response.split(b"\r\n\r\n", 1)
        lines = header.split(b"\r\n")
        status = int(lines[0].split()[1])
        headers = {}
        for line in lines[1:]:
            name, value = line.split(b":", 1)
            headers[name.decode("ascii").lower()] = value.strip().decode("ascii")
        length = int(headers.get("content-length", "0"))
        return status, headers, remainder[:length]

    def test_get_rejects_transfer_encoding_before_serving_workspace(self):
        """A body on GET must not survive as a second request on the connection."""
        response = self.raw_exchange(
            b"GET /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nTransfer-Encoding: chunked\r\nConnection: close\r\n\r\n0\r\n\r\n"
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 400)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "invalid_framing")

    def test_unknown_http_method_uses_the_json_405_contract(self):
        """Unknown verbs must not bypass Host checks and JSON error handling via stdlib 501."""
        response = self.raw_exchange(
            b"TRACE /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nConnection: close\r\n\r\n"
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 405)
        self.assertEqual(headers.get("content-type"), "application/json; charset=utf-8")
        self.assertEqual(headers.get("allow"), "GET, PUT, POST")
        self.assertEqual(json.loads(content)["error"]["code"], "method_not_allowed")

    def test_body_on_disallowed_method_forces_connection_close(self):
        """An unread DELETE body must never be parsed as the next keep-alive request."""
        response = self.raw_exchange(
            b"DELETE /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Type: application/json\r\nContent-Length: 2\r\n\r\n{}"
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 405)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "method_not_allowed")

    def test_oversized_expect_continue_is_rejected_without_interim_100(self):
        """The server must validate body limits before inviting an oversized upload."""
        response = self.raw_exchange(
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Type: application/json\r\nContent-Length: "
            + str(editor_server.MAX_REQUEST_BYTES + 1).encode("ascii")
            + b"\r\nExpect: 100-continue\r\nConnection: close\r\n\r\n"
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 413)
        self.assertNotIn(b"100 Continue", response)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "request_too_large")

    def test_unsupported_expectation_is_417_without_reading_body(self):
        """Unsupported expectations must fail deterministically instead of entering a body read."""
        response = self.raw_exchange(
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Type: application/json\r\nContent-Length: 2\r\n"
            + b"Expect: stier-extension\r\nConnection: close\r\n\r\n{}"
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 417)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "expectation_failed")

    def test_content_length_parser_is_ascii_bounded_and_deterministic(self):
        """Unicode digits, signs, duplicates, and unbounded integers are never accepted."""
        cases = (
            (b"Content-Length: \xb2\r\n", 400, "invalid_framing"),
            (b"Content-Length: -1\r\n", 400, "invalid_framing"),
            (
                b"Content-Length: 2\r\nContent-Length: 2\r\n",
                400,
                "invalid_framing",
            ),
            (
                b"Content-Length: 999999999999999999999999999999\r\n",
                413,
                "request_too_large",
            ),
        )
        for content_length, expected_status, expected_code in cases:
            with self.subTest(content_length=content_length):
                response = self.raw_exchange(
                    b"PUT /api/workspace HTTP/1.1\r\nHost: " + self.host_header
                    + b"\r\nContent-Type: application/json\r\n"
                    + content_length + b"Connection: close\r\n\r\n{}"
                )
                status, headers, content = self.first_response(response)
                self.assertEqual(status, expected_status)
                self.assertEqual(headers.get("connection"), "close")
                self.assertEqual(json.loads(content)["error"]["code"], expected_code)

    def test_truncated_upload_is_a_bounded_json_framing_error(self):
        response = self.raw_exchange(
            b"PUT /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Type: application/json\r\nContent-Length: 10\r\n"
            + b"Connection: close\r\n\r\n{}"
        )
        status, headers, content = self.first_response(response)
        self.assertEqual(status, 400)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "invalid_framing")

    def test_disallowed_body_cannot_become_a_pipelined_get(self):
        response = self.raw_exchange(
            b"DELETE /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Length: 2\r\n\r\n{}"
            + b"GET /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nConnection: close\r\n\r\n"
        )
        status, headers, content = self.first_response(response)
        self.assertEqual(status, 405)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "method_not_allowed")
        self.assertEqual(response.count(b"HTTP/1.1 "), 1)

    def test_auth_rejections_close_before_unread_bodies_can_desynchronize(self):
        nested_get = (
            b"GET /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nConnection: close\r\n\r\n"
        )
        cases = (
            (
                b"PUT /api/workspace HTTP/1.1\r\nHost: attacker.invalid\r\n"
                + b"Content-Length: 2\r\n\r\n{}" + nested_get,
                "forbidden_host",
            ),
            (
                b"PUT /api/workspace HTTP/1.1\r\nHost: " + self.host_header
                + b"\r\nOrigin: http://attacker.invalid\r\nContent-Length: 2\r\n\r\n{}"
                + nested_get,
                "forbidden_origin",
            ),
            (
                b"TRACE /api/workspace HTTP/1.1\r\nHost: attacker.invalid\r\n"
                + b"Content-Length: 2\r\n\r\n{}" + nested_get,
                "forbidden_host",
            ),
        )
        for request, expected_code in cases:
            with self.subTest(expected_code=expected_code):
                response = self.raw_exchange(request)
                status, headers, content = self.first_response(response)
                self.assertEqual(status, 403)
                self.assertEqual(headers.get("connection"), "close")
                self.assertEqual(json.loads(content)["error"]["code"], expected_code)
                self.assertEqual(response.count(b"HTTP/1.1 "), 1)

    def test_invalid_put_route_closes_before_body_can_become_next_request(self):
        """Routing rejection must not leave an unread body on a keep-alive stream."""
        nested_get = (
            b"GET /api/workspace HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nConnection: close\r\n\r\n"
        )
        response = self.raw_exchange(
            b"PUT /api/workspace?query HTTP/1.1\r\nHost: " + self.host_header
            + b"\r\nContent-Type: application/json\r\nContent-Length: "
            + str(len(nested_get)).encode("ascii")
            + b"\r\n\r\n" + nested_get
        )
        status, headers, content = self.first_response(response)

        self.assertEqual(status, 404)
        self.assertEqual(headers.get("connection"), "close")
        self.assertEqual(json.loads(content)["error"]["code"], "not_found")
        self.assertEqual(response.count(b"HTTP/1.1 "), 1)
        self.assertNotIn(b'"workspace_id":"workspace"', response)

    def test_shutdown_has_a_total_deadline_despite_active_keepalive_requests(self):
        """A client must not extend process shutdown forever by resetting socket timeout."""
        with mock.patch.object(
                editor_server, "REQUEST_SOCKET_TIMEOUT_SECONDS", 0.15, create=True):
            self.restart_server()
            ready = threading.Event()
            begin_post_shutdown_requests = threading.Event()
            post_shutdown_attempted = threading.Event()
            stop_requests = threading.Event()
            initial_statuses = []

            def keepalive_client():
                connection = http.client.HTTPConnection(
                    self.host, self.port, timeout=1.0
                )
                try:
                    connection.request("GET", "/api/workspace")
                    response = connection.getresponse()
                    response.read()
                    initial_statuses.append(response.status)
                    ready.set()
                    begin_post_shutdown_requests.wait(timeout=3.0)
                    while not stop_requests.is_set():
                        try:
                            connection.request("GET", "/api/workspace")
                            response = connection.getresponse()
                            response.read()
                        except (OSError, http.client.HTTPException):
                            post_shutdown_attempted.set()
                            break
                        post_shutdown_attempted.set()
                        if stop_requests.wait(timeout=0.02):
                            break
                finally:
                    ready.set()
                    connection.close()

            client_thread = threading.Thread(target=keepalive_client, daemon=True)
            client_thread.start()
            self.assertTrue(ready.wait(timeout=3.0))
            self.assertEqual(initial_statuses, [200])

            self.server.shutdown()
            self.server_thread.join(timeout=3.0)
            begin_post_shutdown_requests.set()
            self.assertTrue(post_shutdown_attempted.wait(timeout=1.0))

            close_finished = threading.Event()

            def close_server():
                self.server.server_close()
                close_finished.set()

            close_thread = threading.Thread(target=close_server, daemon=True)
            close_thread.start()
            finished_within_deadline = close_finished.wait(timeout=0.6)

            stop_requests.set()
            client_thread.join(timeout=3.0)
            close_thread.join(timeout=3.0)

        self.assertTrue(
            finished_within_deadline,
            "server_close exceeded its total deadline while keep-alive traffic continued",
        )
        self.assertFalse(client_thread.is_alive())
        self.assertFalse(close_thread.is_alive())

    def test_sigterm_exits_process_with_a_manager_worker_past_close_deadline(self):
        """A blocked request worker must not keep the editor process alive forever."""
        context = multiprocessing.get_context("fork")
        parent_pipe, child_pipe = context.Pipe(duplex=False)
        process = context.Process(
            target=run_editor_with_blocked_worker,
            args=(str(self.course), str(self.web_root), child_pipe),
        )
        connection = None
        process.start()
        child_pipe.close()
        try:
            self.assertTrue(parent_pipe.poll(3.0), "child editor did not start")
            status, port = parent_pipe.recv()
            self.assertEqual(status, "ready")
            connection = socket.create_connection(("127.0.0.1", port), timeout=3.0)
            connection.sendall(
                b"GET /api/workspace HTTP/1.1\r\nHost: 127.0.0.1:"
                + str(port).encode("ascii")
                + b"\r\nConnection: close\r\n\r\n"
            )
            self.assertTrue(parent_pipe.poll(3.0), "request did not enter manager")
            self.assertEqual(parent_pipe.recv()[0], "entered")

            os.kill(process.pid, signal.SIGTERM)
            process.join(timeout=2.0)
            self.assertFalse(
                process.is_alive(),
                "non-daemon request worker kept the editor process alive",
            )
        finally:
            if connection is not None:
                connection.close()
            if process.is_alive():
                process.kill()
                process.join(timeout=3.0)
            parent_pipe.close()

    def test_worker_limit_rejects_excess_request_before_entering_manager(self):
        """A finite worker budget must reject overload instead of spawning unbounded threads."""
        with mock.patch.object(
                editor_server, "MAX_EDITOR_WORKERS", 2, create=True):
            self.restart_server()
            entered_count = 0
            entered_lock = threading.Lock()
            two_workers_entered = threading.Event()
            release_workers = threading.Event()
            worker_statuses = []
            worker_errors = []
            probe_statuses = []
            probe_errors = []
            real_read_workspace = self.server.release_manager.read_workspace

            def blocked_read_workspace(course_dir, workspace_id):
                nonlocal entered_count
                with entered_lock:
                    entered_count += 1
                    if entered_count >= 2:
                        two_workers_entered.set()
                if not release_workers.wait(timeout=5.0):
                    raise RuntimeError("test did not release blocked request workers")
                return real_read_workspace(course_dir, workspace_id)

            def request_workspace(statuses, errors):
                connection = http.client.HTTPConnection(
                    self.host, self.port, timeout=3.0
                )
                try:
                    connection.request("GET", "/api/workspace")
                    response = connection.getresponse()
                    response.read()
                    statuses.append(response.status)
                except BaseException as error:
                    errors.append(error)
                finally:
                    connection.close()

            worker_threads = [
                threading.Thread(
                    target=request_workspace,
                    args=(worker_statuses, worker_errors),
                    daemon=True,
                )
                for _ in range(2)
            ]
            probe_thread = threading.Thread(
                target=request_workspace,
                args=(probe_statuses, probe_errors),
                daemon=True,
            )
            completed_before_release = False
            try:
                with mock.patch.object(
                        self.server.release_manager,
                        "read_workspace",
                        side_effect=blocked_read_workspace):
                    for thread in worker_threads:
                        thread.start()
                    self.assertTrue(two_workers_entered.wait(timeout=3.0))
                    probe_thread.start()
                    probe_thread.join(timeout=0.6)
                    completed_before_release = not probe_thread.is_alive()
                    release_workers.set()
                    for thread in worker_threads:
                        thread.join(timeout=3.0)
                    probe_thread.join(timeout=3.0)
            finally:
                release_workers.set()
                for thread in worker_threads:
                    thread.join(timeout=3.0)
                if probe_thread.ident is not None:
                    probe_thread.join(timeout=3.0)

        self.assertTrue(
            completed_before_release,
            "the excess request entered the blocked manager instead of being rejected",
        )
        self.assertEqual(probe_errors, [])
        self.assertEqual(probe_statuses, [503])
        self.assertEqual(worker_errors, [])
        self.assertEqual(sorted(worker_statuses), [200, 200])

    def test_server_close_waits_for_an_inflight_workspace_operation(self):
        """Graceful shutdown must not abandon a request that already entered the manager."""
        entered_manager = threading.Event()
        release_manager = threading.Event()
        request_status = []
        request_errors = []
        close_finished = threading.Event()
        real_read_workspace = self.server.release_manager.read_workspace

        def paused_read_workspace(course_dir, workspace_id):
            entered_manager.set()
            if not release_manager.wait(timeout=5.0):
                raise RuntimeError("test did not release the manager operation")
            return real_read_workspace(course_dir, workspace_id)

        def request_workspace():
            try:
                connection = http.client.HTTPConnection(self.host, self.port, timeout=5.0)
                connection.request("GET", "/api/workspace")
                response = connection.getresponse()
                response.read()
                request_status.append(response.status)
                connection.close()
            except BaseException as error:
                request_errors.append(error)

        def close_server():
            self.server.server_close()
            close_finished.set()

        request_thread = threading.Thread(target=request_workspace, daemon=True)
        close_thread = threading.Thread(target=close_server, daemon=True)
        closed_before_release = None
        try:
            with mock.patch.object(
                    self.server.release_manager,
                    "read_workspace",
                    side_effect=paused_read_workspace):
                request_thread.start()
                self.assertTrue(entered_manager.wait(timeout=3.0))
                self.server.shutdown()
                self.server_thread.join(timeout=3.0)
                close_thread.start()
                closed_before_release = close_finished.wait(timeout=0.25)
                release_manager.set()
                request_thread.join(timeout=5.0)
                close_thread.join(timeout=5.0)
        finally:
            release_manager.set()
            request_thread.join(timeout=5.0)
            close_thread.join(timeout=5.0)

        self.assertFalse(
            closed_before_release,
            "server_close returned while an accepted request was still running",
        )
        self.assertTrue(close_finished.is_set())
        self.assertFalse(request_thread.is_alive())
        self.assertFalse(close_thread.is_alive())
        self.assertEqual(request_errors, [])
        self.assertEqual(request_status, [200])


if __name__ == "__main__":
    unittest.main()
