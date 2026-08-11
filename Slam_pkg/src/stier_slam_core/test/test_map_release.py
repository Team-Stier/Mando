import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
import threading
import unittest
from unittest import mock

from stier_slam_core import map_release as map_release_module
from stier_slam_core.map_release import ReleaseManager, ReleaseValidationError


EMPTY_OVERLAY = b'{"schema_version":1,"operations":[]}\n'
EMPTY_SEMANTIC = b'{"schema_version":1,"features":[]}\n'


def write_valid_rtabmap_database(path, scan=b"laser-scan"):
    """RTAB-Map 0.21.13 schema-shaped fixture; blobs are opaque test payloads."""
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
        connection.execute("INSERT INTO Admin(version) VALUES (?)", ("0.21.13",))
        connection.execute(
            "INSERT INTO Node(id,map_id,weight,stamp,pose,ground_truth_pose,velocity,label,gps,env_sensors,time_enter) "
            "VALUES (1,0,0,1.0,?,?,?,?,?,?,?)",
            (b"P" * 48, b"G" * 48, b"V" * 24, "node-1", b"GPS" * 16,
             b"ENV" * 8, "2026-08-08 00:00:00"),
        )
        connection.execute(
            "INSERT INTO Data(id,image,depth,calibration,scan,scan_info,ground_cells,obstacle_cells,"
            "empty_cells,cell_size,view_point_x,view_point_y,view_point_z,user_data,time_enter) "
            "VALUES (1,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (b"image", b"depth", b"calibration", sqlite3.Binary(scan), b"scan-info",
             b"ground", b"obstacle", b"empty", 0.05, 0.0, 0.0, 0.0, b"user-data",
             "2026-08-08 00:00:00"),
        )
        connection.execute(
            "INSERT INTO Link VALUES (1,1,0,?,?,?)",
            (b"I" * (36 * 8), b"T" * 48, b"link-user-data"),
        )
        connection.commit()
    finally:
        connection.close()


def replace_workspace_in_process(
        course, workspace_id, expected_revision, overlay, semantic, start, results):
    """Run one CAS attempt in a separate process and return a serializable result."""
    start.wait()
    try:
        snapshot = ReleaseManager().replace_workspace(
            course, workspace_id, expected_revision, overlay, semantic
        )
        results.put(("success", snapshot["revision"]))
    except map_release_module.WorkspaceConflictError:
        results.put(("conflict", None))
    except BaseException as error:  # Preserve unexpected child failures for the parent test.
        results.put(("error", repr(error)))


def read_special_file_in_process(path, results):
    """Probe a path in a process so a blocking special-file open is bounded."""
    try:
        map_release_module._read_regular_bytes(Path(path))
        results.put("accepted")
    except ReleaseValidationError:
        results.put("rejected")
    except BaseException as error:
        results.put("error: {!r}".format(error))


class ReleaseManagerTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.course = self.root / "course"
        self.input_directory = self.root / "input"
        self.input_directory.mkdir()
        self.db_source = self.input_directory / "source.db"
        self.pgm_source = self.input_directory / "source.pgm"
        self.yaml_source = self.input_directory / "source.yaml"
        write_valid_rtabmap_database(self.db_source)
        self.database_bytes = self.db_source.read_bytes()
        self.pgm_source.write_bytes(
            b"P5\n4 3\n255\n" + bytes((254, 254, 254, 254, 254, 205, 205, 254, 0, 0, 254, 254))
        )
        self.yaml_source.write_text(
            "image: source.pgm\n"
            "resolution: 0.1\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
        self.manager = ReleaseManager()
        self.manager.init_capture(
            self.course,
            "capture-1",
            self.db_source,
            self.yaml_source,
            now_iso="2026-08-08T00:00:00Z",
        )
        self.manager.init_workspace(self.course, "capture-1", "workspace-1")

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _release(self, release_id="release-1", now_iso="2026-08-08T01:02:03Z"):
        return self.manager.create_release(
            self.course, "capture-1", "workspace-1", release_id, now_iso
        )

    def test_capture_uses_atomic_layout_and_preserves_source_bytes(self):
        """Capture initialization must copy validated inputs without altering their sources."""
        raw = self.course / "raw" / "capture-1"

        self.assertEqual(self.db_source.read_bytes(), self.database_bytes)
        self.assertEqual(
            self.pgm_source.read_bytes(),
            b"P5\n4 3\n255\n" + bytes((254, 254, 254, 254, 254, 205, 205, 254, 0, 0, 254, 254)),
        )
        self.assertEqual({path.name for path in raw.iterdir()}, {
            "map.db", "base.pgm", "base.yaml", "capture_manifest.json"
        })
        self.assertEqual((raw / "map.db").read_bytes(), self.database_bytes)
        self.assertFalse(any(path.name.startswith(".capture-1.") for path in (self.course / "raw").iterdir()))

    def test_capture_rejects_symlinked_source_image(self):
        """Resolving a final image symlink before checking would bypass O_NOFOLLOW."""
        real_image = self.input_directory / "real.pgm"
        real_image.write_bytes(self.pgm_source.read_bytes())
        self.pgm_source.unlink()
        self.pgm_source.symlink_to(real_image)

        with self.assertRaises(ReleaseValidationError):
            self.manager.init_capture(
                self.course, "symlink-image", self.db_source, self.yaml_source
            )

        self.assertFalse((self.course / "raw" / "symlink-image").exists())

    def test_capture_rejects_symlinked_source_database(self):
        """A database symlink must never become an immutable raw capture snapshot."""
        real_database = self.input_directory / "real.db"
        real_database.write_bytes(self.db_source.read_bytes())
        self.db_source.unlink()
        self.db_source.symlink_to(real_database)

        with self.assertRaises(ReleaseValidationError):
            self.manager.init_capture(
                self.course, "symlink-db", self.db_source, self.yaml_source
            )

        self.assertFalse((self.course / "raw" / "symlink-db").exists())

    def test_regular_file_open_rejects_fifo_without_blocking(self):
        """Opening a FIFO before fstat would hang validation waiting for a writer."""
        fifo = self.input_directory / "source.fifo"
        os.mkfifo(str(fifo))
        context = multiprocessing.get_context("fork")
        results = context.Queue()
        process = context.Process(
            target=read_special_file_in_process,
            args=(str(fifo), results),
        )

        process.start()
        process.join(0.75)
        try:
            self.assertFalse(
                process.is_alive(),
                "regular-file validation blocked while opening a FIFO",
            )
        finally:
            if process.is_alive():
                process.terminate()
                process.join(2.0)

        self.assertEqual(results.get(timeout=1.0), "rejected")

    def test_capture_publishes_opened_source_snapshots_despite_path_replacement(self):
        """Reopening sources inside publication would copy replacement path contents."""
        original_database = self.db_source.read_bytes()
        original_image = self.pgm_source.read_bytes()
        replacement_image = (
            b"P5\n4 3\n255\n"
            + bytes((0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        )
        publish = map_release_module._create_directory_atomically

        def replace_paths_then_publish(parent, artifact_id, builder, **keywords):
            self.db_source.write_bytes(b"replacement-database")
            self.pgm_source.write_bytes(replacement_image)
            return publish(parent, artifact_id, builder, **keywords)

        with mock.patch.object(
                map_release_module,
                "_create_directory_atomically",
                side_effect=replace_paths_then_publish):
            capture = self.manager.init_capture(
                self.course, "snapshot-capture", self.db_source, self.yaml_source
            )

        self.assertEqual((capture / "map.db").read_bytes(), original_database)
        self.assertEqual((capture / "base.pgm").read_bytes(), original_image)

    def test_capture_holds_image_directory_across_intermediate_path_replacement(self):
        """Replacing an image directory must not redirect an already-authorized capture."""
        image_directory = self.input_directory / "nested"
        image_directory.mkdir()
        image_path = image_directory / "map.pgm"
        original_image = self.pgm_source.read_bytes()
        image_path.write_bytes(original_image)
        attacker_directory = self.root / "attacker-images"
        attacker_directory.mkdir()
        attacker_image = (
            b"P5\n4 3\n255\n"
            + bytes((0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        )
        (attacker_directory / "map.pgm").write_bytes(attacker_image)
        yaml_path = self.input_directory / "nested.yaml"
        yaml_path.write_text(
            "image: nested/map.pgm\n"
            "resolution: 0.1\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
        moved_directory = self.root / "original-images"
        real_open = map_release_module.os.open
        swapped = False

        def swap_before_image_open(path, flags, *args, **kwargs):
            nonlocal swapped
            if (not swapped and Path(path).name == "map.pgm"
                    and not flags & getattr(os, "O_DIRECTORY", 0)):
                swapped = True
                image_directory.rename(moved_directory)
                image_directory.symlink_to(attacker_directory, target_is_directory=True)
            return real_open(path, flags, *args, **kwargs)

        with mock.patch.object(
                map_release_module.os, "open", side_effect=swap_before_image_open):
            capture = self.manager.init_capture(
                self.course, "directory-race", self.db_source, yaml_path
            )

        self.assertTrue(swapped)
        self.assertEqual((capture / "base.pgm").read_bytes(), original_image)

    def test_capture_streams_large_database_without_full_byte_reader(self):
        """Large RTAB-Map databases must be copied and validated in bounded chunks."""
        large_database = self.input_directory / "large.db"
        write_valid_rtabmap_database(
            large_database, (b"0123456789abcdef" * (256 * 1024)) + b"tail"
        )
        database_identity = large_database.stat().st_dev, large_database.stat().st_ino
        real_snapshot = map_release_module._read_source_snapshot
        real_read = map_release_module.os.read
        requested_sizes = []

        def reject_database_snapshot(path, description):
            if Path(path) == large_database:
                raise AssertionError("database entered the full-byte snapshot reader")
            return real_snapshot(path, description)

        def record_database_reads(descriptor, count):
            metadata = os.fstat(descriptor)
            if (metadata.st_dev, metadata.st_ino) == database_identity:
                requested_sizes.append(count)
            return real_read(descriptor, count)

        with mock.patch.object(
                map_release_module,
                "_read_source_snapshot",
                side_effect=reject_database_snapshot), mock.patch.object(
                    map_release_module.os, "read", side_effect=record_database_reads):
            capture = self.manager.init_capture(
                self.course, "large-database", large_database, self.yaml_source
            )

        self.assertEqual((capture / "map.db").stat().st_size, large_database.stat().st_size)
        self.assertGreater(len(requested_sizes), 1)
        self.assertLessEqual(max(requested_sizes), map_release_module._STREAM_CHUNK_SIZE)

        real_regular_reader = map_release_module._read_regular_bytes

        def reject_database_validation(path):
            if Path(path).name == "map.db":
                raise AssertionError("database entered the full-byte artifact reader")
            return real_regular_reader(path)

        with mock.patch.object(
                map_release_module,
                "_read_regular_bytes",
                side_effect=reject_database_validation):
            self.manager.init_workspace(self.course, "large-database", "large-workspace")
            self.manager.create_release(
                self.course,
                "large-database",
                "large-workspace",
                "large-release",
                "2026-08-08T01:02:03Z",
            )

    def test_workspace_records_capture_and_literal_empty_layers(self):
        """A workspace referencing the wrong capture would release mismatched map artifacts."""
        workspace = self.course / "workspaces" / "workspace-1"

        self.assertEqual(
            json.loads((workspace / "source_release.json").read_text(encoding="utf-8")),
            {"schema_version": 1, "capture_id": "capture-1"},
        )
        self.assertEqual((workspace / "static_overlay.json").read_bytes(), EMPTY_OVERLAY)
        self.assertEqual((workspace / "semantic_layer.json").read_bytes(), EMPTY_SEMANTIC)

    def test_read_workspace_returns_one_verified_editor_snapshot(self):
        """An editor snapshot must bind its map metadata and both layers to one revision."""
        self.assertTrue(
            hasattr(self.manager, "read_workspace"),
            "ReleaseManager must expose read_workspace before the editor can be safe",
        )
        snapshot = self.manager.read_workspace(self.course, "workspace-1")

        self.assertEqual(set(snapshot), {
            "schema_version", "workspace_id", "capture_id", "revision", "map",
            "static_overlay", "semantic_layer",
        })
        self.assertEqual(snapshot["schema_version"], 1)
        self.assertEqual(snapshot["workspace_id"], "workspace-1")
        self.assertEqual(snapshot["capture_id"], "capture-1")
        self.assertEqual(len(snapshot["revision"]), 64)
        int(snapshot["revision"], 16)
        self.assertEqual(snapshot["map"], {
            "width": 4,
            "height": 3,
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "image_url": "/api/base.pgm",
            "image_sha256": hashlib.sha256(self.pgm_source.read_bytes()).hexdigest(),
        })
        self.assertEqual(snapshot["static_overlay"], {
            "schema_version": 1, "operations": [],
        })
        self.assertEqual(snapshot["semantic_layer"], {
            "schema_version": 1, "features": [],
        })
        self.assertEqual(
            self.manager.read_workspace_base_pgm(self.course, "workspace-1"),
            self.pgm_source.read_bytes(),
        )

    def test_editor_reads_and_puts_do_not_validate_the_full_rtabmap_database(self):
        """Interactive editor operations must not hash and quick-check the large DB."""
        with mock.patch.object(
                map_release_module,
                "_require_stream_hash_and_validate_rtabmap",
                side_effect=AssertionError("editor touched map.db")) as database_gate:
            before = self.manager.read_workspace(self.course, "workspace-1")
            self.assertEqual(
                self.manager.read_workspace_base_pgm(self.course, "workspace-1"),
                self.pgm_source.read_bytes(),
            )
            after = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                {"schema_version": 1, "operations": []},
                {"schema_version": 1, "features": []},
            )

        self.assertNotEqual(after["revision"], before["revision"])
        self.assertEqual(database_gate.call_count, 0)

        with mock.patch.object(
                map_release_module,
                "_require_stream_hash_and_validate_rtabmap",
                wraps=map_release_module._require_stream_hash_and_validate_rtabmap
                ) as release_database_gate:
            self.manager.release_workspace(
                self.course,
                "workspace-1",
                after["revision"],
                "database-gated-release",
                False,
                "2026-08-08T02:03:04Z",
            )
        self.assertGreaterEqual(release_database_gate.call_count, 1)

    def test_only_exact_editor_descriptor_root_is_accepted(self):
        descriptor = os.open(str(self.course), os.O_RDONLY | os.O_DIRECTORY)
        try:
            exact_editor_root = Path("/proc/self/fd/{}".format(descriptor))
            self.assertEqual(
                self.manager.read_workspace(exact_editor_root, "workspace-1")[
                    "workspace_id"
                ],
                "workspace-1",
            )
            for alias in (
                    Path("/proc/{}/fd/{}".format(os.getpid(), descriptor)),
                    Path("/dev/fd/{}".format(descriptor))):
                with self.subTest(alias=alias), self.assertRaises(ValueError):
                    self.manager.validate_workspace(alias, "workspace-1")
        finally:
            os.close(descriptor)

    def test_capture_map_swap_changes_revision_and_rejects_stale_put(self):
        """A same-ID capture replacement must invalidate edits based on the old PGM."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        alternate_pgm = self.input_directory / "alternate.pgm"
        alternate_yaml = self.input_directory / "alternate.yaml"
        alternate_pgm.write_bytes(
            b"P5\n4 3\n255\n" + bytes((0, 0, 0, 0, 0, 205, 205, 0, 254, 254, 0, 0))
        )
        alternate_yaml.write_text(
            "image: alternate.pgm\n"
            "resolution: 0.1\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
        self.manager.init_capture(
            self.course,
            "replacement-capture",
            self.db_source,
            alternate_yaml,
            now_iso="2026-08-08T00:30:00Z",
        )
        raw = self.course / "raw"
        replacement = raw / "replacement-capture"
        manifest_path = replacement / "capture_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["capture_id"] = "capture-1"
        manifest_path.write_bytes(
            (json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n").encode(
                "utf-8"
            )
        )
        (raw / "capture-1").rename(raw / "old-capture")
        replacement.rename(raw / "capture-1")

        after = self.manager.read_workspace(self.course, "workspace-1")

        self.assertNotEqual(after["revision"], before["revision"])
        self.assertNotEqual(after["map"]["image_sha256"], before["map"]["image_sha256"])
        with self.assertRaises(map_release_module.WorkspaceConflictError):
            self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                {"schema_version": 1, "operations": []},
                {"schema_version": 1, "features": []},
            )

    def test_release_rejects_capture_swap_after_full_database_validation(self):
        """Full validation of a detached capture must not authorize a stale release."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        raw = self.course / "raw"
        replacement = raw / "replacement-capture"
        shutil.copytree(raw / "capture-1", replacement)
        replacement_database = replacement / "map.db"
        replacement_database.unlink()
        write_valid_rtabmap_database(
            replacement_database, scan=b"replacement-laser-scan"
        )
        manifest_path = replacement / "capture_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["hashes"]["map.db"] = hashlib.sha256(
            replacement_database.read_bytes()
        ).hexdigest()
        manifest_path.write_bytes(
            (json.dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n").encode(
                "utf-8"
            )
        )
        real_database_gate = (
            map_release_module._require_stream_hash_and_validate_rtabmap
        )
        database_gate_calls = 0

        def swap_after_full_database_gate(path, expected, description):
            nonlocal database_gate_calls
            result = real_database_gate(path, expected, description)
            database_gate_calls += 1
            if database_gate_calls == 1:
                (raw / "capture-1").rename(raw / "old-capture")
                replacement.rename(raw / "capture-1")
            return result

        with mock.patch.object(
                map_release_module,
                "_require_stream_hash_and_validate_rtabmap",
                side_effect=swap_after_full_database_gate):
            with self.assertRaises(map_release_module.WorkspaceConflictError):
                self.manager.release_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    "capture-race-release",
                    False,
                    "2026-08-08T02:03:04Z",
                )

        self.assertFalse(
            (self.course / "releases" / "capture-race-release").exists()
        )

    def test_replace_workspace_commits_both_valid_layers_as_one_revision(self):
        """Saving must update both editable layers without mutating their immutable source."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        raw = self.course / "raw" / "capture-1"
        raw_before = {path.name: path.read_bytes() for path in raw.iterdir()}
        source_path = self.course / "workspaces" / "workspace-1" / "source_release.json"
        source_before = source_path.read_bytes()
        overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "occupied",
                "polygon": [[0.0, 0.0], [0.2, 0.0], [0.2, 0.2], [0.0, 0.2]],
            }],
        }
        semantic = {
            "schema_version": 1,
            "features": [
                {
                    "label": "lane_center",
                    "geometry": {
                        "type": "polyline",
                        "coordinates": [[0.0, 0.1], [0.3, 0.1]],
                    },
                },
                {
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.2, 0.2]},
                },
            ],
        }

        self.assertTrue(
            hasattr(self.manager, "replace_workspace"),
            "ReleaseManager must own the workspace transaction",
        )
        after = self.manager.replace_workspace(
            self.course, "workspace-1", before["revision"], overlay, semantic
        )

        self.assertNotEqual(after["revision"], before["revision"])
        self.assertEqual(after["static_overlay"], overlay)
        self.assertEqual(after["semantic_layer"], semantic)
        self.assertEqual(self.manager.read_workspace(self.course, "workspace-1"), after)
        self.assertEqual(source_path.read_bytes(), source_before)
        self.assertEqual({path.name: path.read_bytes() for path in raw.iterdir()}, raw_before)

    def test_replace_workspace_rejects_invalid_semantics_without_partial_save(self):
        """A bad semantic feature must leave the overlay and semantic generation unchanged."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        valid_overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "free",
                "polygon": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]],
            }],
        }
        invalid_features = (
            {"label": "unknown", "geometry": {"type": "point", "coordinates": [0.1, 0.1]}},
            {"label": [], "geometry": {"type": "point", "coordinates": [0.1, 0.1]}},
            {"label": "intersection", "geometry": {"type": "polyline", "coordinates": [[0.0, 0.0], [0.1, 0.1]]}},
            {"label": "stop_line", "geometry": {"type": "point", "coordinates": [0.1, 0.1]}},
            {"label": "lane_center", "geometry": {"type": "polyline", "coordinates": [[0.1, 0.1]]}},
            {"label": "lane_center", "geometry": {"type": "polyline", "coordinates": [[0.1, 0.1], [0.1, 0.1]]}},
            {"label": "intersection", "geometry": {"type": "point", "coordinates": [True, 0.1]}},
            {"label": "intersection", "geometry": {"type": "point", "coordinates": [float("nan"), 0.1]}},
            {"label": "intersection", "geometry": {"type": "point", "coordinates": [float("inf"), 0.1]}},
            {"label": "intersection", "geometry": {"type": "point", "coordinates": [0.5, 0.1]}},
            {"label": "intersection", "geometry": {"type": "point", "coordinates": [0.1, 0.1]}, "extra": 1},
        )

        for feature in invalid_features:
            with self.subTest(feature=feature):
                with self.assertRaises(ReleaseValidationError):
                    self.manager.replace_workspace(
                        self.course,
                        "workspace-1",
                        before["revision"],
                        valid_overlay,
                        {"schema_version": 1, "features": [feature]},
                    )
                self.assertEqual(
                    self.manager.read_workspace(self.course, "workspace-1"), before
                )

    def test_replace_workspace_rejects_stale_revision_with_dedicated_conflict(self):
        """A stale browser must not overwrite a newer workspace generation."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        empty_overlay = {"schema_version": 1, "operations": []}
        first_semantic = {
            "schema_version": 1,
            "features": [{
                "label": "intersection",
                "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
            }],
        }
        self.manager.replace_workspace(
            self.course, "workspace-1", before["revision"], empty_overlay, first_semantic
        )

        with self.assertRaises(map_release_module.WorkspaceConflictError):
            self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                empty_overlay,
                {"schema_version": 1, "features": []},
            )

    def test_concurrent_replacements_from_one_revision_have_one_winner(self):
        """The CAS check must be serialized across simultaneous editor requests."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        semantic = {"schema_version": 1, "features": []}
        overlays = [
            {
                "schema_version": 1,
                "operations": [{
                    "operation": operation,
                    "polygon": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]],
                }],
            }
            for operation in ("occupied", "free")
        ]

        def replace(overlay):
            try:
                return self.manager.replace_workspace(
                    self.course, "workspace-1", before["revision"], overlay, semantic
                )
            except BaseException as error:  # Return the real competing outcome to the test.
                return error

        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(replace, overlays))

        successes = [result for result in results if isinstance(result, dict)]
        conflicts = [
            result for result in results
            if isinstance(result, map_release_module.WorkspaceConflictError)
        ]
        self.assertEqual(len(successes), 1)
        self.assertEqual(len(conflicts), 1)
        self.assertEqual(
            self.manager.read_workspace(self.course, "workspace-1"), successes[0]
        )

    def test_concurrent_identical_replacements_still_have_one_winner(self):
        """Even a byte-identical save must advance the CAS generation for the losing tab."""
        overlay = {"schema_version": 1, "operations": []}
        semantic = {"schema_version": 1, "features": []}
        initial = self.manager.read_workspace(self.course, "workspace-1")
        before = self.manager.replace_workspace(
            self.course, "workspace-1", initial["revision"], overlay, semantic
        )

        def replace():
            try:
                return self.manager.replace_workspace(
                    self.course, "workspace-1", before["revision"], overlay, semantic
                )
            except BaseException as error:
                return error

        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(lambda _: replace(), range(2)))

        self.assertEqual(sum(isinstance(result, dict) for result in results), 1)
        self.assertEqual(sum(
            isinstance(result, map_release_module.WorkspaceConflictError)
            for result in results
        ), 1)
        self.assertNotEqual(
            self.manager.read_workspace(self.course, "workspace-1")["revision"],
            before["revision"],
        )

    def test_process_replacements_from_one_revision_have_one_winner(self):
        """The workspace lock must serialize separate editor server processes too."""
        context = multiprocessing.get_context("fork")
        start = context.Event()
        results = context.Queue()
        before = self.manager.read_workspace(self.course, "workspace-1")
        semantic = {"schema_version": 1, "features": []}
        overlays = [
            {
                "schema_version": 1,
                "operations": [{
                    "operation": operation,
                    "polygon": [
                        [0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]
                    ],
                }],
            }
            for operation in ("occupied", "free")
        ]
        processes = [
            context.Process(
                target=replace_workspace_in_process,
                args=(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    overlay,
                    semantic,
                    start,
                    results,
                ),
            )
            for overlay in overlays
        ]

        try:
            for process in processes:
                process.start()
            start.set()
            outcomes = [results.get(timeout=10) for _ in processes]
            for process in processes:
                process.join(timeout=10)
        finally:
            for process in processes:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=10)
            results.close()
            results.join_thread()

        self.assertEqual([process.exitcode for process in processes], [0, 0])
        self.assertEqual(sum(status == "success" for status, _ in outcomes), 1)
        self.assertEqual(sum(status == "conflict" for status, _ in outcomes), 1)
        self.assertFalse([detail for status, detail in outcomes if status == "error"])
        self.assertEqual(
            self.manager.read_workspace(self.course, "workspace-1")["revision"],
            next(detail for status, detail in outcomes if status == "success"),
        )

    def test_replacing_legacy_lock_inode_cannot_split_workspace_cas(self):
        """A replaceable sidecar lock must not authorize two same-revision writers."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        lock_path = self.course / "workspaces" / ".workspace-workspace-1.lock"
        if not lock_path.exists():
            lock_path.write_bytes(b"legacy-attacker-lock\n")
        lock_bytes = lock_path.read_bytes()
        first_state_read = threading.Event()
        release_first = threading.Event()
        second_overlapped = threading.Event()
        state_call_lock = threading.Lock()
        state_call_count = 0
        real_workspace_state = map_release_module._workspace_state

        def pause_first_state(*arguments, **keywords):
            nonlocal state_call_count
            state = real_workspace_state(*arguments, **keywords)
            with state_call_lock:
                state_call_count += 1
                call_number = state_call_count
            if call_number == 1:
                first_state_read.set()
                self.assertTrue(release_first.wait(timeout=10.0))
            elif call_number == 2:
                second_overlapped.set()
            return state

        overlays = [
            {
                "schema_version": 1,
                "operations": [{
                    "operation": operation,
                    "polygon": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]],
                }],
            }
            for operation in ("occupied", "free")
        ]

        def replace(overlay):
            try:
                return self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    overlay,
                    {"schema_version": 1, "features": []},
                )
            except BaseException as error:
                return error

        with mock.patch.object(
                map_release_module,
                "_workspace_state",
                side_effect=pause_first_state), ThreadPoolExecutor(
                    max_workers=2) as executor:
            first = executor.submit(replace, overlays[0])
            self.assertTrue(first_state_read.wait(timeout=5.0))
            lock_path.unlink()
            lock_path.write_bytes(lock_bytes)
            second = executor.submit(replace, overlays[1])
            overlapped_before_first_commit = second_overlapped.wait(timeout=1.0)
            release_first.set()
            outcomes = [first.result(timeout=10.0), second.result(timeout=10.0)]

        successes = [outcome for outcome in outcomes if isinstance(outcome, dict)]
        conflicts = [
            outcome for outcome in outcomes
            if isinstance(outcome, map_release_module.WorkspaceConflictError)
        ]
        self.assertFalse(overlapped_before_first_commit)
        self.assertEqual(len(successes), 1)
        self.assertEqual(len(conflicts), 1)
        self.assertEqual(
            self.manager.read_workspace(self.course, "workspace-1"), successes[0]
        )

    def test_workspaces_parent_replacement_fails_closed_before_commit(self):
        """State and exchange must stay bound to one verified workspaces directory."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        workspaces = self.course / "workspaces"
        replacement = self.root / "replacement-workspaces"
        moved = self.root / "moved-workspaces"
        shutil.copytree(workspaces, replacement)
        temporary_created = threading.Event()
        continue_commit = threading.Event()
        real_make_temporary = map_release_module._make_temporary_directory_at

        def pause_after_temporary(parent_descriptor, workspace_id):
            name = real_make_temporary(parent_descriptor, workspace_id)
            temporary_created.set()
            self.assertTrue(continue_commit.wait(timeout=10.0))
            return name

        outcome = {}

        def replace():
            try:
                outcome["result"] = self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    {
                        "schema_version": 1,
                        "operations": [{
                            "operation": "occupied",
                            "polygon": [
                                [0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]
                            ],
                        }],
                    },
                    {"schema_version": 1, "features": []},
                )
            except BaseException as error:
                outcome["error"] = error

        with mock.patch.object(
                map_release_module,
                "_make_temporary_directory_at",
                side_effect=pause_after_temporary):
            thread = threading.Thread(target=replace)
            thread.start()
            self.assertTrue(temporary_created.wait(timeout=5.0))
            workspaces.rename(moved)
            replacement.rename(workspaces)
            continue_commit.set()
            thread.join(timeout=10.0)

        self.assertFalse(thread.is_alive())
        self.assertNotIn("result", outcome)
        self.assertIsInstance(
            outcome.get("error"),
            (map_release_module.WorkspaceConflictError, ReleaseValidationError),
        )
        selected = self.manager.read_workspace(self.course, "workspace-1")
        self.assertEqual(selected["capture_id"], before["capture_id"])
        self.assertEqual(selected["map"], before["map"])
        self.assertEqual(selected["static_overlay"], before["static_overlay"])
        self.assertEqual(selected["semantic_layer"], before["semantic_layer"])

    def test_workspace_replace_does_not_follow_replaced_temporary_entry(self):
        """Layer writes must stay pinned when the temporary name is replaced."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        attacker_parent = self.root / "workspace-temporary-attacker"
        attacker_parent.mkdir()
        real_make_temporary = map_release_module._make_temporary_directory_at

        def redirect_temporary(parent_descriptor, workspace_id):
            name = real_make_temporary(parent_descriptor, workspace_id)
            os.rename(
                name,
                name + ".moved",
                src_dir_fd=parent_descriptor,
                dst_dir_fd=parent_descriptor,
            )
            os.symlink(str(attacker_parent), name, dir_fd=parent_descriptor)
            return name

        with mock.patch.object(
                map_release_module,
                "_make_temporary_directory_at",
                side_effect=redirect_temporary):
            with self.assertRaises((OSError, ReleaseValidationError)):
                self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    {"schema_version": 1, "operations": []},
                    {"schema_version": 1, "features": []},
                )

        self.assertEqual(list(attacker_parent.iterdir()), [])
        self.assertEqual(
            self.manager.read_workspace(self.course, "workspace-1"), before
        )

    def _assert_workspace_read_parent_swap_fails_closed(self, read_operation):
        before = self.manager.read_workspace(self.course, "workspace-1")
        workspaces = self.course / "workspaces"
        replacement = self.root / "replacement-workspaces-after-read"
        moved = self.root / "moved-workspaces-after-read"
        shutil.copytree(workspaces, replacement)
        real_workspace_state = map_release_module._workspace_state

        def swap_after_state_read(*arguments, **keywords):
            state = real_workspace_state(*arguments, **keywords)
            workspaces.rename(moved)
            replacement.rename(workspaces)
            return state

        with mock.patch.object(
                map_release_module,
                "_workspace_state",
                side_effect=swap_after_state_read):
            with self.assertRaises(map_release_module.WorkspaceConflictError):
                read_operation()

        selected = self.manager.read_workspace(self.course, "workspace-1")
        self.assertEqual(selected["capture_id"], before["capture_id"])
        self.assertEqual(selected["map"], before["map"])
        self.assertEqual(selected["static_overlay"], before["static_overlay"])
        self.assertEqual(selected["semantic_layer"], before["semantic_layer"])

    def test_workspace_read_rejects_parent_swap_after_state_snapshot(self):
        """GET workspace must not return a snapshot from a detached parent."""
        self._assert_workspace_read_parent_swap_fails_closed(
            lambda: self.manager.read_workspace(self.course, "workspace-1")
        )

    def test_base_pgm_read_rejects_parent_swap_after_state_snapshot(self):
        """GET base PGM must not return bytes authorized by a detached parent."""
        self._assert_workspace_read_parent_swap_fails_closed(
            lambda: self.manager.read_workspace_base_pgm(
                self.course, "workspace-1"
            )
        )

    def test_workspace_replace_rejects_parent_swap_after_updated_state_read(self):
        """PUT must recheck its pinned parent after reading the committed generation."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        workspaces = self.course / "workspaces"
        replacement = self.root / "replacement-workspaces-after-update"
        moved = self.root / "moved-workspaces-after-update"
        shutil.copytree(workspaces, replacement)
        real_workspace_state = map_release_module._workspace_state
        state_reads = 0

        def swap_after_updated_state_read(*arguments, **keywords):
            nonlocal state_reads
            state = real_workspace_state(*arguments, **keywords)
            state_reads += 1
            if state_reads == 2:
                workspaces.rename(moved)
                replacement.rename(workspaces)
            return state

        with mock.patch.object(
                map_release_module,
                "_workspace_state",
                side_effect=swap_after_updated_state_read):
            with self.assertRaises(map_release_module.WorkspaceConflictError):
                self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    {
                        "schema_version": 1,
                        "operations": [{
                            "operation": "occupied",
                            "polygon": [
                                [0.0, 0.0], [0.1, 0.0],
                                [0.1, 0.1], [0.0, 0.1],
                            ],
                        }],
                    },
                    {"schema_version": 1, "features": []},
                )

        selected = self.manager.read_workspace(self.course, "workspace-1")
        self.assertEqual(selected["capture_id"], before["capture_id"])
        self.assertEqual(selected["map"], before["map"])
        self.assertEqual(selected["static_overlay"], before["static_overlay"])
        self.assertEqual(selected["semantic_layer"], before["semantic_layer"])

    def test_release_workspace_publishes_exact_revision_and_optional_activation(self):
        """The editor release response must describe a fully validated immutable release."""
        snapshot = self.manager.read_workspace(self.course, "workspace-1")
        self.assertTrue(
            hasattr(self.manager, "release_workspace"),
            "ReleaseManager must bind release creation to the editor revision",
        )

        summary = self.manager.release_workspace(
            self.course,
            "workspace-1",
            snapshot["revision"],
            "editor-release",
            True,
            "2026-08-08T02:03:04Z",
        )

        self.assertEqual(set(summary), {
            "schema_version", "release_id", "activated", "manifest_sha256",
            "artifact_hashes",
        })
        self.assertEqual(summary["schema_version"], 1)
        self.assertEqual(summary["release_id"], "editor-release")
        self.assertIs(summary["activated"], True)
        self.assertEqual(len(summary["manifest_sha256"]), 64)
        self.assertEqual(set(summary["artifact_hashes"]), {
            "map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json",
        })
        release = self.course / "releases" / "editor-release"
        for name, digest in summary["artifact_hashes"].items():
            self.assertEqual(digest, hashlib.sha256((release / name).read_bytes()).hexdigest())
        active = json.loads(
            (self.course / "active_release.json").read_text(encoding="utf-8")
        )
        self.assertEqual(active["release_id"], "editor-release")
        self.assertEqual(active["manifest_sha256"], summary["manifest_sha256"])

    def test_activation_failure_returns_committed_summary_and_retry_is_idempotent(self):
        """A failed activation must expose the committed release and permit an exact retry."""
        snapshot = self.manager.read_workspace(self.course, "workspace-1")
        release_id = "activation-retry-release"
        real_atomic_write_at = map_release_module._atomic_write_bytes_at
        activation_attempts = 0

        def fail_first_activation(parent_descriptor, destination_name, content):
            nonlocal activation_attempts
            if destination_name == "active_release.json":
                activation_attempts += 1
                if activation_attempts == 1:
                    raise OSError(
                        "injected activation failure at {}".format(
                            self.course / "active_release.json"
                        )
                    )
            return real_atomic_write_at(
                parent_descriptor, destination_name, content
            )

        with mock.patch.object(
                map_release_module,
                "_atomic_write_bytes_at",
                side_effect=fail_first_activation):
            partial = self.manager.release_workspace(
                self.course,
                "workspace-1",
                snapshot["revision"],
                release_id,
                True,
                "2026-08-08T02:03:04Z",
            )
            newer = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                snapshot["revision"],
                snapshot["static_overlay"],
                snapshot["semantic_layer"],
            )
            retry = self.manager.release_workspace(
                self.course,
                "workspace-1",
                snapshot["revision"],
                release_id,
                True,
                "2026-08-08T02:04:05Z",
            )

        self.assertIs(partial["activated"], False)
        self.assertEqual(partial["error"]["code"], "activation_failed")
        self.assertIsInstance(partial["error"].get("message"), str)
        self.assertTrue(partial["error"]["message"])
        self.assertNotIn(str(self.course), partial["error"]["message"])
        self.assertEqual(partial["release_id"], release_id)
        self.assertEqual(len(partial["manifest_sha256"]), 64)
        self.assertEqual(set(partial["artifact_hashes"]), {
            "map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json",
        })
        release = self.course / "releases" / release_id
        for name, digest in partial["artifact_hashes"].items():
            self.assertEqual(digest, hashlib.sha256((release / name).read_bytes()).hexdigest())

        self.assertIs(retry["activated"], True)
        self.assertEqual(retry["release_id"], release_id)
        self.assertEqual(retry["manifest_sha256"], partial["manifest_sha256"])
        self.assertEqual(retry["artifact_hashes"], partial["artifact_hashes"])
        active = json.loads(
            (self.course / "active_release.json").read_text(encoding="utf-8")
        )
        self.assertEqual(active["release_id"], release_id)
        self.assertEqual(active["manifest_sha256"], partial["manifest_sha256"])

        with self.assertRaises(FileExistsError):
            self.manager.release_workspace(
                self.course,
                "workspace-1",
                newer["revision"],
                release_id,
                False,
                "2026-08-08T02:05:06Z",
            )

    def test_release_parent_fsync_failure_returns_hashes_and_retry_guidance(self):
        """A renamed release with uncertain durability needs a structured result."""
        before = self.manager.read_workspace(self.course, "workspace-1")
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
                raise OSError("injected release parent fsync failure")
            return real_fsync(descriptor)

        with mock.patch.object(
                map_release_module,
                "_rename_directory_noreplace",
                side_effect=publish_then_mark), mock.patch.object(
                    map_release_module.os,
                    "fsync",
                    side_effect=fail_first_post_publish_fsync):
            summary = self.manager.release_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                "publication-indeterminate",
                False,
                "2026-08-08T04:05:06Z",
            )

        self.assertTrue(published)
        self.assertTrue(failed)
        self.assertIs(summary["activated"], False)
        self.assertEqual(
            summary["error"]["code"], "release_publication_indeterminate"
        )
        self.assertEqual(len(summary["manifest_sha256"]), 64)
        self.assertTrue(
            (self.course / "releases" / "publication-indeterminate" / "manifest.json").is_file()
        )

        retry = self.manager.release_workspace(
            self.course,
            "workspace-1",
            before["revision"],
            "publication-indeterminate",
            False,
            "2099-01-01T00:00:00Z",
        )
        self.assertNotIn("error", retry)
        self.assertEqual(retry["manifest_sha256"], summary["manifest_sha256"])

    def test_activation_never_reports_success_for_replaced_temporary_bytes(self):
        """Activation success must stay bound to the file inode that was written."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        real_replace = map_release_module.os.replace

        def replace_substituted_source(source, destination, *arguments, **keywords):
            source_parent = keywords.get("src_dir_fd")
            if Path(destination).name == "active_release.json":
                moved = str(source) + ".opened"
                if source_parent is None:
                    os.rename(str(source), moved)
                    Path(source).write_bytes(b'{"substituted":true}\n')
                else:
                    os.rename(
                        source,
                        moved,
                        src_dir_fd=source_parent,
                        dst_dir_fd=source_parent,
                    )
                    descriptor = os.open(
                        source,
                        os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                        0o600,
                        dir_fd=source_parent,
                    )
                    try:
                        os.write(descriptor, b'{"substituted":true}\n')
                    finally:
                        os.close(descriptor)
            return real_replace(source, destination, *arguments, **keywords)

        with mock.patch.object(
                map_release_module.os,
                "replace",
                side_effect=replace_substituted_source):
            summary = self.manager.release_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                "activation-substitution",
                True,
                "2026-08-08T05:06:07Z",
            )

        self.assertIs(summary["activated"], False)
        self.assertEqual(summary["error"]["code"], "activation_failed")

    def test_activation_never_reports_success_for_same_inode_content_mutation(self):
        """Keeping the inode is insufficient when its bytes change during publish."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        real_replace = map_release_module.os.replace
        mutated = False

        def replace_then_mutate(source, destination, *arguments, **keywords):
            nonlocal mutated
            result = real_replace(source, destination, *arguments, **keywords)
            if Path(destination).name == "active_release.json":
                destination_parent = keywords.get("dst_dir_fd")
                descriptor = os.open(
                    destination,
                    os.O_WRONLY | os.O_TRUNC | os.O_NOFOLLOW,
                    dir_fd=destination_parent,
                )
                try:
                    os.write(descriptor, b'{"substituted":true}\n')
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                mutated = True
            return result

        with mock.patch.object(
                map_release_module.os,
                "replace",
                side_effect=replace_then_mutate):
            summary = self.manager.release_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                "activation-same-inode-mutation",
                True,
                "2026-08-08T05:07:08Z",
            )

        self.assertTrue(mutated)
        self.assertIs(summary["activated"], False)
        self.assertEqual(summary["error"]["code"], "activation_failed")

    def test_activation_reports_indeterminate_after_replace_fsync_failure(self):
        """A replaced active pointer must never be reported as activated false."""
        snapshot = self.manager.read_workspace(self.course, "workspace-1")
        real_replace = map_release_module.os.replace
        real_fsync = map_release_module.os.fsync
        replaced = False
        injected = False

        def replace_then_mark(*arguments, **keywords):
            nonlocal replaced
            result = real_replace(*arguments, **keywords)
            if Path(arguments[1]).name == "active_release.json":
                replaced = True
            return result

        def fail_active_parent_fsync(descriptor):
            nonlocal injected
            if replaced and not injected:
                injected = True
                raise OSError(
                    "injected fsync failure at {}".format(self.course)
                )
            return real_fsync(descriptor)

        with mock.patch.object(
                map_release_module.os,
                "replace",
                side_effect=replace_then_mark), mock.patch.object(
                    map_release_module.os,
                    "fsync",
                    side_effect=fail_active_parent_fsync):
            summary = self.manager.release_workspace(
                self.course,
                "workspace-1",
                snapshot["revision"],
                "fsync-indeterminate-release",
                True,
                "2026-08-08T02:03:04Z",
            )

        self.assertTrue(injected)
        self.assertEqual(summary["activated"], "indeterminate")
        self.assertEqual(summary["error"]["code"], "activation_indeterminate")
        self.assertNotIn(str(self.course), summary["error"]["message"])
        active = json.loads(
            (self.course / "active_release.json").read_text(encoding="utf-8")
        )
        self.assertEqual(active["release_id"], "fsync-indeterminate-release")
        self.assertEqual(active["manifest_sha256"], summary["manifest_sha256"])

    def test_release_workspace_rejects_stale_revision_before_any_publication(self):
        """A release request from a stale tab must not create or activate an artifact."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        after = self.manager.replace_workspace(
            self.course,
            "workspace-1",
            before["revision"],
            {"schema_version": 1, "operations": []},
            {
                "schema_version": 1,
                "features": [{
                    "label": "intersection",
                    "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
                }],
            },
        )
        self.assertNotEqual(after["revision"], before["revision"])

        with self.assertRaises(map_release_module.WorkspaceConflictError):
            self.manager.release_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                "stale-release",
                True,
                "2026-08-08T02:03:04Z",
            )

        self.assertFalse((self.course / "releases" / "stale-release").exists())
        self.assertFalse((self.course / "active_release.json").exists())

    def test_failed_workspace_exchange_preserves_generation_and_removes_temporary(self):
        """A publication failure before the kernel exchange must leave no partial layer pair."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        with mock.patch.object(
                map_release_module,
                "_exchange_directories",
                side_effect=OSError("injected exchange failure")):
            with self.assertRaises(OSError):
                self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    {"schema_version": 1, "operations": []},
                    {
                        "schema_version": 1,
                        "features": [{
                            "label": "intersection",
                            "geometry": {
                                "type": "point", "coordinates": [0.1, 0.1]
                            },
                        }],
                    },
                )

        self.assertEqual(self.manager.read_workspace(self.course, "workspace-1"), before)
        workspaces = self.course / "workspaces"
        self.assertFalse(any(
            path.name.startswith(".workspace-1.") for path in workspaces.iterdir()
        ))

    def test_workspace_fsync_failure_returns_committed_indeterminate_revision(self):
        """A committed save must not be reported as an ordinary failed request."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        real_exchange = map_release_module._exchange_directories
        real_fsync = map_release_module.os.fsync
        exchanged = False
        failed = False

        def exchange_then_mark(*arguments, **keywords):
            nonlocal exchanged
            result = real_exchange(*arguments, **keywords)
            exchanged = True
            return result

        def fail_first_post_exchange_fsync(descriptor):
            nonlocal failed
            if exchanged and not failed:
                failed = True
                raise OSError("injected workspace parent fsync failure")
            return real_fsync(descriptor)

        with mock.patch.object(
                map_release_module,
                "_exchange_directories",
                side_effect=exchange_then_mark), mock.patch.object(
                    map_release_module.os,
                    "fsync",
                    side_effect=fail_first_post_exchange_fsync):
            summary = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                {"schema_version": 1, "operations": []},
                {
                    "schema_version": 1,
                    "features": [{
                        "label": "intersection",
                        "geometry": {
                            "type": "point", "coordinates": [0.1, 0.1]
                        },
                    }],
                },
            )

        current = self.manager.read_workspace(self.course, "workspace-1")
        self.assertTrue(exchanged)
        self.assertTrue(failed)
        self.assertNotEqual(summary["revision"], before["revision"])
        self.assertEqual(summary["revision"], current["revision"])
        self.assertEqual(
            summary["error"]["code"], "workspace_durability_indeterminate"
        )

    def test_post_exchange_identity_failure_returns_committed_indeterminate_revision(self):
        """Every failure after the directory exchange must preserve the commit outcome."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        real_exchange = map_release_module._exchange_directories
        real_require = map_release_module._require_workspace_lease_entries
        exchanged = False
        failed = False

        def exchange_then_mark(*arguments, **keywords):
            nonlocal exchanged
            result = real_exchange(*arguments, **keywords)
            exchanged = True
            return result

        def fail_first_post_exchange_check(*arguments, **keywords):
            nonlocal failed
            if exchanged and not failed:
                failed = True
                raise map_release_module.WorkspaceConflictError(
                    "injected post-exchange identity failure"
                )
            return real_require(*arguments, **keywords)

        with mock.patch.object(
                map_release_module,
                "_exchange_directories",
                side_effect=exchange_then_mark), mock.patch.object(
                    map_release_module,
                    "_require_workspace_lease_entries",
                    side_effect=fail_first_post_exchange_check):
            summary = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                {"schema_version": 1, "operations": []},
                {
                    "schema_version": 1,
                    "features": [{
                        "label": "intersection",
                        "geometry": {
                            "type": "point", "coordinates": [0.1, 0.1]
                        },
                    }],
                },
            )

        current = self.manager.read_workspace(self.course, "workspace-1")
        self.assertTrue(exchanged)
        self.assertTrue(failed)
        self.assertEqual(summary["revision"], current["revision"])
        self.assertNotEqual(summary["revision"], before["revision"])
        self.assertEqual(
            summary["error"]["code"], "workspace_durability_indeterminate"
        )
        self.assertFalse(any(
            path.name.startswith(".workspace-1.")
            for path in (self.course / "workspaces").iterdir()
        ))

    def test_cleanup_failure_returns_commit_without_deleting_unowned_directories(self):
        """Deferred cleanup must never infer deletion authority from a filename."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        first_overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "occupied",
                "polygon": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]],
            }],
        }
        second_overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "free",
                "polygon": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]],
            }],
        }
        semantic = {"schema_version": 1, "features": []}
        real_remove = map_release_module._remove_directory_tree_at
        removal_attempts = 0

        def fail_first_cleanup(parent_descriptor, name, expected_identity=None):
            nonlocal removal_attempts
            removal_attempts += 1
            if removal_attempts == 1:
                raise OSError("injected old-generation cleanup failure")
            return real_remove(
                parent_descriptor, name, expected_identity=expected_identity
            )

        workspaces = self.course / "workspaces"
        with mock.patch.object(
                map_release_module,
                "_remove_directory_tree_at",
                side_effect=fail_first_cleanup):
            committed = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                before["revision"],
                first_overlay,
                semantic,
            )
            self.assertEqual(
                self.manager.read_workspace(self.course, "workspace-1"), committed
            )
            self.assertNotEqual(committed["revision"], before["revision"])
            self.assertEqual(committed["static_overlay"], first_overlay)

            orphan_paths = [
                path for path in workspaces.iterdir()
                if path.name.startswith(".workspace-1.")
            ]
            self.assertEqual(len(orphan_paths), 1)
            unrelated = workspaces / ".workspace-1.deadbeefdeadbeef"
            unrelated.mkdir()
            (unrelated / "keep.txt").write_text("operator data", encoding="utf-8")

            second = self.manager.replace_workspace(
                self.course,
                "workspace-1",
                committed["revision"],
                second_overlay,
                semantic,
            )

        self.assertEqual(self.manager.read_workspace(self.course, "workspace-1"), second)
        self.assertEqual(second["static_overlay"], second_overlay)
        self.assertTrue(orphan_paths[0].is_dir())
        self.assertTrue(unrelated.is_dir())
        self.assertEqual(
            (unrelated / "keep.txt").read_text(encoding="utf-8"),
            "operator data",
        )

    def test_workspace_lock_symlink_is_rejected(self):
        """A lock-path symlink must not split concurrent editors onto attacker-selected files."""
        self.manager.read_workspace(self.course, "workspace-1")
        lock_path = self.course / "workspaces" / ".workspace-workspace-1.lock"
        lock_path.unlink()
        target = self.root / "attacker-lock"
        target.write_bytes(b"")
        lock_path.symlink_to(target)

        with self.assertRaises(ReleaseValidationError):
            self.manager.read_workspace(self.course, "workspace-1")

    def test_replace_and_release_race_uses_only_one_complete_generation(self):
        """A release racing a save may use old state or conflict, but never mixed layers."""
        before = self.manager.read_workspace(self.course, "workspace-1")
        new_overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "occupied",
                "polygon": [[0.0, 0.0], [0.2, 0.0], [0.2, 0.2], [0.0, 0.2]],
            }],
        }
        new_semantic = {
            "schema_version": 1,
            "features": [{
                "label": "intersection",
                "geometry": {"type": "point", "coordinates": [0.1, 0.1]},
            }],
        }
        start = threading.Barrier(2)

        def replace():
            start.wait()
            try:
                return self.manager.replace_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    new_overlay,
                    new_semantic,
                )
            except BaseException as error:
                return error

        def release():
            start.wait()
            try:
                return self.manager.release_workspace(
                    self.course,
                    "workspace-1",
                    before["revision"],
                    "racing-release",
                    False,
                    "2026-08-08T02:03:04Z",
                )
            except BaseException as error:
                return error

        with ThreadPoolExecutor(max_workers=2) as executor:
            replace_future = executor.submit(replace)
            release_future = executor.submit(release)
            replace_result = replace_future.result()
            release_result = release_future.result()

        self.assertIsInstance(replace_result, dict)
        release_path = self.course / "releases" / "racing-release"
        if isinstance(release_result, dict):
            self.assertEqual(
                json.loads((release_path / "static_overlay.json").read_text(encoding="utf-8")),
                before["static_overlay"],
            )
            self.assertEqual(
                json.loads((release_path / "semantic_layer.json").read_text(encoding="utf-8")),
                before["semantic_layer"],
            )
        else:
            self.assertIsInstance(release_result, map_release_module.WorkspaceConflictError)
            self.assertFalse(release_path.exists())
        after = self.manager.read_workspace(self.course, "workspace-1")
        self.assertEqual(after["static_overlay"], new_overlay)
        self.assertEqual(after["semantic_layer"], new_semantic)

    def test_release_contains_hashed_artifacts_and_source_database_reference(self):
        """Missing or wrong artifact hashes would allow silent release corruption."""
        release = self._release()
        manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))

        self.assertEqual(
            set(path.name for path in release.iterdir()),
            {"map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json", "manifest.json"},
        )
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["release_id"], "release-1")
        self.assertEqual(manifest["workspace_id"], "workspace-1")
        self.assertEqual(manifest["source_capture_id"], "capture-1")
        self.assertEqual(manifest["created_at"], "2026-08-08T01:02:03Z")
        self.assertEqual(manifest["map"], {"resolution": 0.1, "origin": [0.0, 0.0, 0.0]})
        self.assertEqual(
            manifest["source_database"],
            {
                "relative_path": "raw/capture-1/map.db",
                "sha256": hashlib.sha256(self.database_bytes).hexdigest(),
            },
        )
        self.assertEqual(set(manifest["hashes"]), {
            "map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json"
        })
        for name, digest in manifest["hashes"].items():
            with self.subTest(name=name):
                self.assertEqual(len(digest), 64)
                self.assertEqual(digest, hashlib.sha256((release / name).read_bytes()).hexdigest())

    def test_overlay_is_rasterized_into_release_while_raw_capture_is_unchanged(self):
        """Releasing an edit must never write through to immutable raw capture bytes."""
        workspace = self.course / "workspaces" / "workspace-1"
        overlay = {
            "schema_version": 1,
            "operations": [{
                "operation": "occupied",
                "polygon": [[0.1, 0.1], [0.3, 0.1], [0.3, 0.3], [0.1, 0.3]],
            }],
        }
        (workspace / "static_overlay.json").write_text(
            json.dumps(overlay, separators=(",", ":")) + "\n", encoding="utf-8"
        )
        raw = self.course / "raw" / "capture-1"
        raw_snapshot = {path.name: path.read_bytes() for path in raw.iterdir()}

        release = self._release()

        self.assertEqual({path.name: path.read_bytes() for path in raw.iterdir()}, raw_snapshot)
        self.assertNotEqual((release / "map.pgm").read_bytes(), (raw / "base.pgm").read_bytes())

    def test_repeated_capture_workspace_and_release_ids_refuse_overwrite(self):
        """Reusing an immutable ID must not replace any existing artifact directory."""
        release = self._release()
        release_snapshot = {path.name: path.read_bytes() for path in release.iterdir()}

        with self.assertRaises(FileExistsError):
            self.manager.init_capture(self.course, "capture-1", self.db_source, self.yaml_source)
        with self.assertRaises(FileExistsError):
            self.manager.init_workspace(self.course, "capture-1", "workspace-1")
        with self.assertRaises(FileExistsError):
            self._release(now_iso="2099-01-01T00:00:00Z")

        self.assertEqual({path.name: path.read_bytes() for path in release.iterdir()}, release_snapshot)

    def test_atomic_publication_refuses_concurrent_empty_target(self):
        """Kernel publication must not replace a target hidden from userspace checks."""
        parent = self.course / "releases"
        parent.mkdir(exist_ok=True)
        target = parent / "concurrent-empty"
        target.mkdir()
        path_exists = Path.exists

        def hide_target(path):
            if path == target:
                return False
            return path_exists(path)

        with mock.patch.object(Path, "exists", autospec=True, side_effect=hide_target):
            with self.assertRaises(FileExistsError):
                map_release_module._create_directory_atomically(
                    parent,
                    "concurrent-empty",
                    lambda temporary: (temporary / "artifact").write_bytes(b"new"),
                )

        self.assertEqual(list(target.iterdir()), [])

    def test_atomic_publication_holds_parent_across_path_replacement(self):
        """A verified storage parent must stay bound to one directory until rename."""
        parent = self.course / "releases"
        parent.mkdir(exist_ok=True)
        moved_parent = self.root / "original-releases"
        attacker_parent = self.root / "attacker-releases"
        attacker_parent.mkdir()
        real_mkdir = map_release_module.os.mkdir
        swapped = False

        def swap_before_temporary_mkdir(path, *args, **kwargs):
            nonlocal swapped
            if not swapped and Path(path).name.startswith(".parent-race."):
                swapped = True
                parent.rename(moved_parent)
                parent.symlink_to(attacker_parent, target_is_directory=True)
            return real_mkdir(path, *args, **kwargs)

        with mock.patch.object(
                map_release_module.os, "mkdir", side_effect=swap_before_temporary_mkdir):
            result = map_release_module._create_directory_atomically(
                parent,
                "parent-race",
                lambda temporary: (temporary / "artifact").write_bytes(b"safe"),
            )

        self.assertTrue(swapped)
        self.assertEqual((moved_parent / "parent-race" / "artifact").read_bytes(), b"safe")
        self.assertEqual(list(attacker_parent.iterdir()), [])
        self.assertEqual(result.resolve(), (moved_parent / "parent-race").resolve())

    def test_atomic_publication_does_not_follow_replaced_temporary_entry(self):
        """Builder writes must stay on the created directory inode, not its name."""
        parent = self.course / "releases"
        parent.mkdir(exist_ok=True)
        attacker_parent = self.root / "temporary-entry-attacker"
        attacker_parent.mkdir()
        captured = {}
        real_make_temporary = map_release_module._make_temporary_directory_at

        def remember_temporary(parent_descriptor, artifact_id):
            name = real_make_temporary(parent_descriptor, artifact_id)
            captured.update(parent_descriptor=parent_descriptor, name=name)
            return name

        def replace_entry_then_build(temporary):
            parent_descriptor = captured["parent_descriptor"]
            temporary_name = captured["name"]
            moved_name = temporary_name + ".moved"
            os.rename(
                temporary_name,
                moved_name,
                src_dir_fd=parent_descriptor,
                dst_dir_fd=parent_descriptor,
            )
            os.symlink(
                str(attacker_parent), temporary_name,
                dir_fd=parent_descriptor,
            )
            (temporary / "artifact").write_bytes(b"safe")

        with mock.patch.object(
                map_release_module,
                "_make_temporary_directory_at",
                side_effect=remember_temporary):
            with self.assertRaises((OSError, ReleaseValidationError)):
                map_release_module._create_directory_atomically(
                    parent, "temporary-entry-race", replace_entry_then_build
                )

        self.assertEqual(list(attacker_parent.iterdir()), [])
        self.assertFalse((parent / "temporary-entry-race").exists())

    def test_capture_rejects_symlinked_raw_storage_parent(self):
        """Capture publication must not follow a course raw-directory symlink."""
        raw = self.course / "raw"
        real_raw = self.root / "real-raw"
        raw.rename(real_raw)
        raw.symlink_to(real_raw, target_is_directory=True)

        with self.assertRaises(ReleaseValidationError):
            self.manager.init_capture(
                self.course, "symlink-raw", self.db_source, self.yaml_source
            )

        self.assertFalse((real_raw / "symlink-raw").exists())

    def test_capture_rejects_storage_parent_replaced_after_verification(self):
        """Closing the verified parent before publication must not authorize a replacement inode."""
        real_storage_parent = map_release_module._storage_parent
        moved_raw = self.root / "verified-raw"
        replacement_raw = self.course / "raw"
        swapped = False

        def replace_after_verification(course_dir, name, create):
            nonlocal swapped
            verified = real_storage_parent(course_dir, name, create)
            parent = verified[0] if isinstance(verified, tuple) else verified
            if name == "raw" and not swapped:
                swapped = True
                Path(parent).rename(moved_raw)
                Path(parent).mkdir()
            return verified

        with mock.patch.object(
                map_release_module,
                "_storage_parent",
                side_effect=replace_after_verification):
            with self.assertRaises(ReleaseValidationError):
                self.manager.init_capture(
                    self.course,
                    "replaced-raw",
                    self.db_source,
                    self.yaml_source,
                )

        self.assertTrue(swapped)
        self.assertFalse((replacement_raw / "replaced-raw").exists())
        self.assertFalse((moved_raw / "replaced-raw").exists())

    def test_workspace_rejects_symlinked_workspaces_storage_parent(self):
        """Workspace publication must not follow a course workspaces symlink."""
        workspaces = self.course / "workspaces"
        real_workspaces = self.root / "real-workspaces"
        workspaces.rename(real_workspaces)
        workspaces.symlink_to(real_workspaces, target_is_directory=True)

        with self.assertRaises(ReleaseValidationError):
            self.manager.init_workspace(self.course, "capture-1", "symlink-workspaces")

        self.assertFalse((real_workspaces / "symlink-workspaces").exists())

    def test_release_rejects_symlinked_releases_storage_parent(self):
        """Release publication must not follow a course releases-directory symlink."""
        real_releases = self.root / "real-releases"
        real_releases.mkdir()
        (self.course / "releases").symlink_to(real_releases, target_is_directory=True)

        with self.assertRaises(ReleaseValidationError):
            self._release("symlink-releases")

        self.assertFalse((real_releases / "symlink-releases").exists())

    def test_all_identifiers_reject_path_traversal(self):
        """A traversal ID must not create or read artifacts outside the course root."""
        with self.assertRaises(ValueError):
            self.manager.init_capture(self.course, "../escape", self.db_source, self.yaml_source)
        with self.assertRaises(ValueError):
            self.manager.init_workspace(self.course, "../escape", "workspace-2")
        with self.assertRaises(ValueError):
            self.manager.init_workspace(self.course, "capture-1", "../escape")
        with self.assertRaises(ValueError):
            self.manager.create_release(
                self.course, "capture-1", "../escape", "release-2", "2026-08-08T00:00:00Z"
            )
        with self.assertRaises(ValueError):
            self.manager.create_release(
                self.course, "capture-1", "workspace-1", "../escape", "2026-08-08T00:00:00Z"
            )
        with self.assertRaises(ValueError):
            self.manager.activate_release(self.course, "../escape")
        self.assertFalse((self.root / "escape").exists())

    def test_corrupted_release_hash_prevents_activation(self):
        """Activation must verify bytes again instead of trusting a stale manifest."""
        release = self._release()
        (release / "map.pgm").write_bytes((release / "map.pgm").read_bytes() + b"corrupt")

        with self.assertRaises(ReleaseValidationError):
            self.manager.activate_release(self.course, "release-1")

        self.assertFalse((self.course / "active_release.json").exists())

    def test_corrupted_raw_database_prevents_release_creation(self):
        """A changed raw database must invalidate its immutable capture reference."""
        (self.course / "raw" / "capture-1" / "map.db").write_bytes(b"changed")

        with self.assertRaises(ReleaseValidationError):
            self._release()

        self.assertFalse((self.course / "releases" / "release-1").exists())

    def test_corrupted_raw_database_prevents_release_activation(self):
        """Activation must revalidate the immutable database reference after release."""
        self._release()
        (self.course / "raw" / "capture-1" / "map.db").write_bytes(b"changed")

        with self.assertRaises(ReleaseValidationError):
            self.manager.activate_release(self.course, "release-1")

        self.assertFalse((self.course / "active_release.json").exists())

    def test_activation_rejects_releases_parent_replaced_after_descriptor_open(self):
        """Release validation must stay bound to its opened storage-parent inode."""
        self._release()
        releases = self.course / "releases"
        detached_releases = self.root / "detached-releases"
        replacement_releases = self.root / "replacement-releases"
        shutil.copytree(releases, replacement_releases)
        replacement_manifest_path = (
            replacement_releases / "release-1" / "manifest.json"
        )
        replacement_manifest = json.loads(
            replacement_manifest_path.read_text(encoding="utf-8")
        )
        replacement_manifest["created_at"] = "2026-08-08T12:34:56Z"
        replacement_manifest_path.write_bytes(
            map_release_module._json_bytes(replacement_manifest)
        )
        real_open_directory_at = map_release_module._open_directory_at
        swapped = False

        def open_then_replace(parent_descriptor, name, description):
            nonlocal swapped
            descriptor = real_open_directory_at(
                parent_descriptor, name, description
            )
            if (not swapped and name == "releases"
                    and description == "releases storage parent"):
                swapped = True
                releases.rename(detached_releases)
                replacement_releases.rename(releases)
            return descriptor

        with mock.patch.object(
                map_release_module,
                "_open_directory_at",
                side_effect=open_then_replace):
            with self.assertRaises(ReleaseValidationError):
                self.manager.activate_release(self.course, "release-1")

        self.assertTrue(swapped)
        self.assertFalse((self.course / "active_release.json").exists())

    def test_activation_rejects_raw_parent_replaced_after_descriptor_open(self):
        """Raw DB validation must retain and recheck its storage-parent inode."""
        self._release()
        raw = self.course / "raw"
        detached_raw = self.root / "detached-raw"
        replacement_raw = self.root / "replacement-raw"
        shutil.copytree(raw, replacement_raw)
        replacement_manifest_path = (
            replacement_raw / "capture-1" / "capture_manifest.json"
        )
        replacement_manifest = json.loads(
            replacement_manifest_path.read_text(encoding="utf-8")
        )
        replacement_manifest["created_at"] = "2026-08-08T12:34:56Z"
        replacement_manifest_path.write_bytes(
            map_release_module._json_bytes(replacement_manifest)
        )
        real_open_directory_at = map_release_module._open_directory_at
        swapped = False

        def open_then_replace(parent_descriptor, name, description):
            nonlocal swapped
            descriptor = real_open_directory_at(
                parent_descriptor, name, description
            )
            if (not swapped and name == "raw"
                    and description == "raw storage parent"):
                swapped = True
                raw.rename(detached_raw)
                replacement_raw.rename(raw)
            return descriptor

        with mock.patch.object(
                map_release_module,
                "_open_directory_at",
                side_effect=open_then_replace):
            with self.assertRaises(ReleaseValidationError):
                self.manager.activate_release(self.course, "release-1")

        self.assertTrue(swapped)
        self.assertFalse((self.course / "active_release.json").exists())

    def test_activation_rejects_course_root_replaced_after_validation(self):
        """Validation and active-pointer publication must share one named course."""
        self._release()
        detached_course = self.root / "detached-course"
        real_validate_at = map_release_module._validated_release_at
        swapped = False

        def validate_then_replace(course_descriptor, release_id):
            nonlocal swapped
            result = real_validate_at(course_descriptor, release_id)
            if not swapped:
                swapped = True
                self.course.rename(detached_course)
                self.course.mkdir()
            return result

        with mock.patch.object(
                map_release_module,
                "_validated_release_at",
                side_effect=validate_then_replace):
            with self.assertRaises(ReleaseValidationError):
                self.manager.activate_release(self.course, "release-1")

        self.assertTrue(swapped)
        self.assertFalse((self.course / "active_release.json").exists())
        self.assertFalse((detached_course / "active_release.json").exists())

    def test_editor_activation_rejects_course_root_replaced_after_validation(self):
        """The editor transaction must not activate a detached course inode."""
        self._release()
        snapshot = self.manager.read_workspace(self.course, "workspace-1")
        detached_course = self.root / "detached-editor-course"
        real_validate_at = map_release_module._validated_release_at
        swapped = False

        def validate_then_replace(course_descriptor, release_id):
            nonlocal swapped
            result = real_validate_at(course_descriptor, release_id)
            if not swapped:
                swapped = True
                self.course.rename(detached_course)
                self.course.mkdir()
            return result

        with mock.patch.object(
                map_release_module,
                "_validated_release_at",
                side_effect=validate_then_replace):
            summary = self.manager.release_workspace(
                self.course,
                "workspace-1",
                snapshot["revision"],
                "release-1",
                True,
                "2026-08-08T02:03:04Z",
            )

        self.assertTrue(swapped)
        self.assertIs(summary["activated"], False)
        self.assertEqual(summary["error"]["code"], "activation_failed")
        self.assertFalse((self.course / "active_release.json").exists())
        self.assertFalse((detached_course / "active_release.json").exists())

    def test_activation_uses_digest_of_exact_validated_manifest_snapshot(self):
        """Reopening manifest after validation could activate unvalidated replacement bytes."""
        release = self._release()
        manifest_path = release / "manifest.json"
        validated_bytes = manifest_path.read_bytes()
        validated_digest = hashlib.sha256(validated_bytes).hexdigest()
        replacement = validated_bytes + b" "
        validate_at = map_release_module._validated_release_at

        def validate_then_replace(course_descriptor, release_id):
            result = validate_at(course_descriptor, release_id)
            manifest_path.write_bytes(replacement)
            return result

        with mock.patch.object(
                map_release_module,
                "_validated_release_at",
                side_effect=validate_then_replace):
            active_path = self.manager.activate_release(self.course, "release-1")

        active = json.loads(active_path.read_text(encoding="utf-8"))
        self.assertEqual(active["manifest_sha256"], validated_digest)

    def test_hash_validation_rejects_real_artifact_symlink(self):
        """An immutable release artifact must be a real regular file, not a symlink."""
        release = self._release()
        map_path = release / "map.pgm"
        backup_path = self.root / "map.pgm.backup"
        map_path.rename(backup_path)
        map_path.symlink_to(backup_path)

        with self.assertRaises(ReleaseValidationError):
            self.manager.activate_release(self.course, "release-1")

        self.assertTrue(map_path.is_symlink())
        self.assertFalse((self.course / "active_release.json").exists())

    def test_activation_atomically_replaces_complete_manifest(self):
        """Replacing in place would make an already-open reader observe mixed activation bytes."""
        self._release("release-1", "2026-08-08T01:00:00Z")
        self._release("release-2", "2026-08-08T02:00:00Z")
        active = self.manager.activate_release(self.course, "release-1")

        with active.open("rb") as previous_reader:
            self.manager.activate_release(self.course, "release-2")
            old_document = json.loads(previous_reader.read().decode("utf-8"))

        new_document = json.loads(active.read_text(encoding="utf-8"))
        self.assertEqual(old_document["release_id"], "release-1")
        self.assertEqual(new_document["release_id"], "release-2")
        self.assertEqual(set(new_document), {"schema_version", "release_id", "manifest_sha256"})
        self.assertFalse(any(path.name.startswith(".active_release.json.") for path in self.course.iterdir()))

    def test_invalid_workspace_json_schema_prevents_release(self):
        """Unknown layer fields must not be published into an immutable release."""
        workspace = self.course / "workspaces" / "workspace-1"
        invalid_documents = (
            ("static_overlay.json", {"schema_version": 1, "operations": [], "unknown": 1}),
            ("semantic_layer.json", {"schema_version": 1, "features": [], "unknown": 1}),
        )
        for name, document in invalid_documents:
            with self.subTest(name=name):
                original = (workspace / name).read_bytes()
                (workspace / name).write_text(json.dumps(document), encoding="utf-8")
                try:
                    with self.assertRaises(ReleaseValidationError):
                        self._release("release-{}".format(name.split(".")[0]))
                finally:
                    (workspace / name).write_bytes(original)

    def test_capture_manifest_rejects_float_schema_version(self):
        """A capture manifest must use integer schema 1, not equal-valued 1.0."""
        raw = self.course / "raw" / "capture-1"
        capture_manifest_path = raw / "capture_manifest.json"
        capture_manifest = json.loads(capture_manifest_path.read_text(encoding="utf-8"))
        capture_manifest["schema_version"] = 1.0
        capture_manifest_path.write_text(json.dumps(capture_manifest), encoding="utf-8")

        with self.assertRaises(ReleaseValidationError):
            self.manager.init_workspace(self.course, "capture-1", "float-capture")

    def test_workspace_source_rejects_float_schema_version(self):
        """A workspace source must use integer schema 1, not equal-valued 1.0."""
        workspace = self.course / "workspaces" / "workspace-1"
        source_path = workspace / "source_release.json"
        source_path.write_bytes(b'{"schema_version":1.0,"capture_id":"capture-1"}\n')

        with self.assertRaises(ReleaseValidationError):
            self.manager.validate_workspace(self.course, "workspace-1")

    def test_semantic_layer_rejects_float_schema_version(self):
        """A semantic layer must use integer schema 1, not equal-valued 1.0."""
        workspace = self.course / "workspaces" / "workspace-1"
        semantic_path = workspace / "semantic_layer.json"
        semantic_path.write_bytes(b'{"schema_version":1.0,"features":[]}\n')

        with self.assertRaises(ReleaseValidationError):
            self._release("float-semantic")

    def test_release_manifest_rejects_float_schema_version(self):
        """A release manifest must use integer schema 1, not equal-valued 1.0."""
        release = self._release("float-release")
        manifest_path = release / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["schema_version"] = 1.0
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        with self.assertRaises(ReleaseValidationError):
            self.manager.activate_release(self.course, "float-release")

    def test_release_manifest_rejects_boolean_map_origin_metadata(self):
        """False must not compare equal to a numeric zero map origin."""
        release = self._release("bool-origin")
        manifest_path = release / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["map"]["origin"] = [False, 0.0, 0.0]
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        with self.assertRaises(ReleaseValidationError):
            self.manager.activate_release(self.course, "bool-origin")

    def test_non_standard_json_constants_prevent_release(self):
        """Python's permissive NaN parser must not publish invalid JSON artifacts."""
        semantic = self.course / "workspaces" / "workspace-1" / "semantic_layer.json"
        semantic.write_bytes(b'{"schema_version":1,"features":[NaN]}\n')

        with self.assertRaises(ReleaseValidationError):
            self._release()

        self.assertFalse((self.course / "releases" / "release-1").exists())


class MapReleaseCliTest(unittest.TestCase):
    def test_cli_runs_capture_workspace_validate_release_and_activate_workflow(self):
        """A wrapper with stale arguments or hidden policy would break the operator workflow."""
        with tempfile.TemporaryDirectory() as temporary_name:
            root = Path(temporary_name)
            course = root / "course"
            source = root / "source"
            source.mkdir()
            database = source / "map.db"
            pgm = source / "map.pgm"
            yaml_path = source / "map.yaml"
            write_valid_rtabmap_database(database)
            pgm.write_bytes(b"P5\n2 1\n255\n\xfe\x00")
            yaml_path.write_text(
                "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
                encoding="utf-8",
            )
            script = Path(__file__).parents[1] / "scripts" / "map_release_cli.py"
            commands = (
                ("init-capture", "--course", str(course), "--capture", "cap", "--db", str(database), "--map-yaml", str(yaml_path)),
                ("init-workspace", "--course", str(course), "--capture", "cap", "--workspace", "work"),
                ("validate", "--course", str(course), "--workspace", "work"),
                ("release", "--course", str(course), "--workspace", "work", "--release", "rel"),
                ("activate", "--course", str(course), "--release", "rel"),
            )
            results = []
            for command in commands:
                result = subprocess.run(
                    ["/usr/bin/python3", str(script)] + list(command),
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
                results.append(result)
                self.assertEqual(result.returncode, 0, result.stderr)

            validation = json.loads(results[2].stdout)
            activation = json.loads((course / "active_release.json").read_text(encoding="utf-8"))
            self.assertEqual(validation["capture_id"], "cap")
            self.assertEqual(validation["operation_count"], 0)
            self.assertEqual(activation["release_id"], "rel")


if __name__ == "__main__":
    unittest.main()
