from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from dataclasses import replace
import errno
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import tempfile
import threading
import unittest
from unittest import mock

from stier_slam_core import map_release as map_release_module
from stier_slam_core.map_release import ReleaseManager, ReleaseValidationError
from stier_slam_core.runtime_database import (
    guard_main,
    prepare_runtime_config,
    prepare_runtime_database,
)


def write_valid_rtabmap_database(path, scan=b"laser-scan", version="0.21.13"):
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
        connection.execute("INSERT INTO Admin(version) VALUES (?)", (version,))
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


def remove_schema_column(path, table, column):
    connection = sqlite3.connect(str(path))
    try:
        existing = connection.execute("PRAGMA table_info({})".format(table)).fetchall()
        kept = [row for row in existing if row[1] != column]
        connection.execute("ALTER TABLE {} RENAME TO old_{}".format(table, table))
        definitions = []
        for row in kept:
            definition = '"{}" {}'.format(row[1], row[2] or "BLOB")
            if row[3]:
                definition += " NOT NULL"
            if row[5]:
                definition += " PRIMARY KEY"
            definitions.append(definition)
        connection.execute("CREATE TABLE {} ({})".format(table, ",".join(definitions)))
        names = ",".join('"{}"'.format(row[1]) for row in kept)
        connection.execute(
            "INSERT INTO {} ({}) SELECT {} FROM old_{}".format(table, names, names, table)
        )
        connection.execute("DROP TABLE old_{}".format(table))
        connection.commit()
    finally:
        connection.close()


def snapshot_tree(root):
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in root.rglob("*") if path.is_file() and not path.is_symlink()
    }


def guarded_result(arguments, execvp):
    try:
        return guard_main(arguments, execvp=execvp)
    except SystemExit as error:
        return error.code


class RuntimeDatabaseGuardTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.course = self.root / "course"
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()
        self.source_database = self.inputs / "source.db"
        write_valid_rtabmap_database(
            self.source_database, b"rtabmap-source-database\x00" * 200000
        )
        image = self.inputs / "source.pgm"
        image.write_bytes(b"P5\n2 2\n255\n" + bytes((254, 254, 0, 254)))
        yaml = self.inputs / "source.yaml"
        yaml.write_text(
            "image: source.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
        self.manager = ReleaseManager()
        self.manager.init_capture(self.course, "capture", self.source_database, yaml)
        self.manager.init_workspace(self.course, "capture", "workspace")
        self.release = self.manager.create_release(
            self.course, "capture", "workspace", "release", "2026-08-08T00:00:00Z")
        self.manifest = self.release / "manifest.json"
        self.release_map = self.release / "map.yaml"
        self.released_database = self.course / "raw" / "capture" / "map.db"
        self.runtime = self.root / "runtime" / "rtabmap.db"
        self.source_config = self.inputs / "rtabmap.ini"
        self.source_config.write_text("[Core]\nReg/Force3DoF=true\n", encoding="utf-8")
        self.runtime_config = self.root / "runtime" / "rtabmap.ini"
        self.runtime.parent.mkdir()

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_preflight_copies_verified_database_without_mutating_source(self):
        source_hash = hashlib.sha256(self.released_database.read_bytes()).hexdigest()
        result = prepare_runtime_database(
            self.manifest, self.release_map, self.released_database, self.runtime)
        self.assertEqual(result, self.runtime)
        self.assertEqual(hashlib.sha256(self.runtime.read_bytes()).hexdigest(), source_hash)
        self.runtime.write_bytes(b"mutable runtime state")
        self.assertEqual(hashlib.sha256(self.released_database.read_bytes()).hexdigest(), source_hash)

    def test_runtime_entrypoints_reject_descriptor_root_aliases_before_writes(self):
        """Pinned descriptor roots are private to editor workspace transactions."""
        immutable_before = snapshot_tree(self.course)
        inputs_before = snapshot_tree(self.inputs)
        input_descriptor = os.open(str(self.inputs), os.O_RDONLY | os.O_DIRECTORY)
        course_descriptor = os.open(str(self.course), os.O_RDONLY | os.O_DIRECTORY)
        runtime_descriptor = os.open(
            str(self.runtime.parent), os.O_RDONLY | os.O_DIRECTORY
        )
        try:
            descriptor_source_config = Path(
                "/proc/self/fd/{}/rtabmap.ini".format(input_descriptor)
            )
            numeric_descriptor_source_config = Path(
                "/proc/{}/fd/{}/rtabmap.ini".format(
                    os.getpid(), input_descriptor
                )
            )
            descriptor_manifest = Path(
                "/proc/self/fd/{}/releases/release/manifest.json".format(
                    course_descriptor
                )
            )
            descriptor_runtime_config = Path(
                "/proc/self/fd/{}/descriptor-runtime.ini".format(
                    runtime_descriptor
                )
            )
            descriptor_bundle = Path(
                "/proc/self/fd/{}/descriptor-bundle".format(runtime_descriptor)
            )
            execute_target = self.runtime.parent / "execute-alias.ini"

            with self.assertRaises(ReleaseValidationError):
                self.manager.plan_runtime_artifacts(
                    source_config_path=descriptor_source_config,
                    runtime_config_path=self.runtime_config,
                )
            with self.assertRaises(ReleaseValidationError):
                self.manager.plan_runtime_artifacts(
                    source_config_path=numeric_descriptor_source_config,
                    runtime_config_path=self.runtime_config,
                )
            with self.assertRaises(ReleaseValidationError):
                self.manager.plan_runtime_artifacts(
                    source_config_path=self.source_config,
                    runtime_config_path=descriptor_runtime_config,
                )
            with self.assertRaises(ReleaseValidationError):
                self.manager.plan_runtime_artifacts(
                    release_manifest_path=descriptor_manifest,
                    map_yaml_path=self.release_map,
                    source_database_path=self.released_database,
                    runtime_database_path=self.runtime,
                )
            with self.assertRaises(ReleaseValidationError):
                self.manager.prepare_localization_runtime_bundle(
                    self.manifest,
                    self.release_map,
                    self.released_database,
                    self.source_config,
                    descriptor_bundle,
                    runtime_root_path=self.runtime.parent,
                )
            with self.assertRaises(ReleaseValidationError):
                self.manager.execute_runtime_plan(
                    map_release_module.RuntimeArtifactPlan(
                        source_config=descriptor_source_config,
                        runtime_config=execute_target,
                    )
                )
        finally:
            os.close(runtime_descriptor)
            os.close(course_descriptor)
            os.close(input_descriptor)

        self.assertEqual(snapshot_tree(self.course), immutable_before)
        self.assertEqual(snapshot_tree(self.inputs), inputs_before)
        self.assertFalse((self.runtime.parent / "descriptor-runtime.ini").exists())
        self.assertFalse((self.runtime.parent / "descriptor-bundle").exists())
        self.assertFalse((self.runtime.parent / "execute-alias.ini").exists())

    def test_execute_rejects_a_forged_plan_before_protected_overwrite(self):
        """Only a plan returned by the complete preflight may authorize writes."""
        immutable_before = snapshot_tree(self.course)
        protected_target = self.course / "raw" / "capture" / "base.pgm"
        forged = map_release_module.RuntimeArtifactPlan(
            source_config=self.source_config,
            runtime_config=protected_target,
        )

        with self.assertRaises(ReleaseValidationError):
            self.manager.execute_runtime_plan(forged)

        authorized = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=self.runtime_config,
        )
        with self.assertRaises(ReleaseValidationError):
            self.manager.execute_runtime_plan(
                replace(authorized, runtime_config=protected_target)
            )

        self.assertEqual(snapshot_tree(self.course), immutable_before)

    def test_execute_rejects_directly_authorized_forged_plan_before_raw_overwrite(self):
        """Caller-constructed authorization must not permit an immutable overwrite."""
        immutable_before = snapshot_tree(self.course)
        protected_target = self.course / "raw" / "capture" / "base.pgm"
        untrusted = map_release_module.RuntimeArtifactPlan(
            source_config=self.source_config,
            runtime_config=protected_target,
        )
        forged = replace(
            untrusted,
            _authorization=map_release_module._RuntimePlanAuthorization(
                map_release_module._runtime_plan_values(untrusted)
            ),
        )

        rejected = False
        try:
            self.manager.execute_runtime_plan(forged)
        except ReleaseValidationError:
            rejected = True

        self.assertEqual(snapshot_tree(self.course), immutable_before)
        self.assertTrue(rejected, "directly constructed authorization must be rejected")

    def test_execute_rejects_source_config_replaced_after_preflight(self):
        """Execution must not consume a source config that changed after preflight."""
        plan = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=self.runtime_config,
        )
        replacement = self.inputs / "replacement-rtabmap.ini"
        replacement.write_text("[Core]\nReg/Force3DoF=false\n", encoding="utf-8")
        os.replace(str(replacement), str(self.source_config))

        rejected = False
        try:
            self.manager.execute_runtime_plan(plan)
        except ReleaseValidationError:
            rejected = True

        self.assertFalse(self.runtime_config.exists())
        self.assertTrue(rejected, "post-preflight source replacement must be rejected")

    def test_factory_rejects_config_only_target_inside_immutable_storage(self):
        """Config-only preflight must not authorize a raw/release overwrite."""
        immutable_before = snapshot_tree(self.course)
        protected_target = self.course / "raw" / "capture" / "base.pgm"

        with self.assertRaises(ReleaseValidationError):
            self.manager.plan_runtime_artifacts(
                source_config_path=self.source_config,
                runtime_config_path=protected_target,
            )

        self.assertEqual(snapshot_tree(self.course), immutable_before)

    def test_execute_rejects_runtime_parent_replaced_after_revalidation(self):
        """A safe lexical destination must stay bound to its preflight parent inode."""
        runtime_target = self.runtime.parent / "base.pgm"
        plan = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=runtime_target,
        )
        protected = self.course / "raw" / "capture" / "base.pgm"
        protected_bytes = protected.read_bytes()
        displaced_runtime = self.root / "displaced-runtime"
        capture = protected.parent
        real_open_bound = map_release_module._open_bound_runtime_source

        @contextmanager
        def swap_destination_parent(path, binding):
            with real_open_bound(path, binding) as descriptor:
                self.runtime.parent.rename(displaced_runtime)
                capture.rename(self.runtime.parent)
                yield descriptor

        with mock.patch.object(
                map_release_module,
                "_open_bound_runtime_source",
                side_effect=swap_destination_parent):
            with self.assertRaises(ReleaseValidationError):
                self.manager.execute_runtime_plan(plan)

        self.assertEqual(
            (self.runtime.parent / "base.pgm").read_bytes(), protected_bytes
        )

    def test_execute_never_succeeds_for_replaced_runtime_temporary_inode(self):
        """The inode verified through the open FD must be the inode renamed into place."""
        runtime_target = self.runtime.parent / "rtabmap.ini"
        plan = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=runtime_target,
        )
        real_replace = map_release_module.os.replace
        displaced_names = []

        def replace_substituted_source(source, destination, *arguments, **keywords):
            source_parent = keywords.get("src_dir_fd")
            destination_parent = keywords.get("dst_dir_fd")
            if (Path(destination).name == runtime_target.name
                    and source_parent is not None
                    and not displaced_names):
                displaced = str(source) + ".opened"
                os.rename(
                    source,
                    displaced,
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
                    os.write(descriptor, b"substituted runtime bytes\n")
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                displaced_names.append(displaced)
            return real_replace(
                source,
                destination,
                *arguments,
                src_dir_fd=source_parent,
                dst_dir_fd=destination_parent,
            )

        with mock.patch.object(
                map_release_module.os,
                "replace",
                side_effect=replace_substituted_source):
            with self.assertRaises(ReleaseValidationError):
                self.manager.execute_runtime_plan(plan)

        self.assertEqual(runtime_target.read_bytes(), b"substituted runtime bytes\n")
        self.assertEqual(len(displaced_names), 1)
        self.assertEqual(
            (self.runtime.parent / displaced_names[0]).read_bytes(),
            self.source_config.read_bytes(),
        )

    def test_execute_never_succeeds_for_same_inode_runtime_content_mutation(self):
        """Runtime success requires the published inode to retain the copied digest."""
        runtime_target = self.runtime.parent / "same-inode-rtabmap.ini"
        plan = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=runtime_target,
        )
        real_replace = map_release_module.os.replace
        mutated = False

        def replace_then_mutate(source, destination, *arguments, **keywords):
            nonlocal mutated
            result = real_replace(source, destination, *arguments, **keywords)
            if Path(destination).name == runtime_target.name:
                descriptor = os.open(
                    destination,
                    os.O_WRONLY | os.O_TRUNC | os.O_NOFOLLOW,
                    dir_fd=keywords.get("dst_dir_fd"),
                )
                try:
                    os.write(descriptor, b"same inode substituted bytes\n")
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                mutated = True
            return result

        with mock.patch.object(
                map_release_module.os,
                "replace",
                side_effect=replace_then_mutate):
            with self.assertRaises(ReleaseValidationError):
                self.manager.execute_runtime_plan(plan)

        self.assertTrue(mutated)
        self.assertEqual(
            runtime_target.read_bytes(), b"same inode substituted bytes\n"
        )

    def test_runtime_destination_check_preserves_an_inner_copy_error(self):
        """A secondary parent check must not mask the operation's primary failure."""
        expected_parent = map_release_module._runtime_destination_parent_identity(
            self.runtime, "runtime database"
        )
        with mock.patch.object(
                map_release_module,
                "_runtime_destination_parent_identity",
                side_effect=ReleaseValidationError("secondary parent failure")):
            with self.assertRaisesRegex(OSError, "primary copy failure"):
                with map_release_module._open_bound_runtime_destination(
                        self.runtime,
                        expected_parent,
                        "runtime database"):
                    raise OSError("primary copy failure")

    def test_fresh_mapping_root_cannot_be_replaced_after_plan_revalidation(self):
        """A renamed immutable directory must never become a fresh mapping root."""
        protected_database = self.inputs / "protected-mapping.db"
        write_valid_rtabmap_database(protected_database)
        self.manager.init_capture(
            self.course,
            "protected-mapping",
            protected_database,
            self.inputs / "source.yaml",
        )
        protected_capture = self.course / "raw" / "protected-mapping"
        mapping_root = self.root / "fresh-bound-mapping-root"
        mapping_session = mapping_root / "session"
        plan = self.manager.plan_runtime_artifacts(
            source_config_path=self.source_config,
            runtime_config_path=mapping_session / "rtabmap.ini",
            mapping_database_path=mapping_session / "mapping.db",
            mapping_runtime_root_path=mapping_root,
            mapping_session_dir_path=mapping_session,
        )
        real_open_source = map_release_module._open_bound_runtime_source
        swapped = False

        @contextmanager
        def swap_root_after_revalidation(path, binding):
            nonlocal swapped
            with real_open_source(path, binding) as descriptor:
                protected_capture.rename(mapping_root)
                swapped = True
                yield descriptor

        with mock.patch.object(
                map_release_module,
                "_open_bound_runtime_source",
                side_effect=swap_root_after_revalidation):
            with self.assertRaises(ReleaseValidationError):
                self.manager.execute_runtime_plan(plan)

        self.assertTrue(swapped)
        self.assertFalse((mapping_root / "session").exists())

    def test_fresh_bundle_root_cannot_be_replaced_after_layout_validation(self):
        """Bundle staging must stay bound to the root validated for this invocation."""
        protected_database = self.inputs / "protected-bundle.db"
        write_valid_rtabmap_database(protected_database)
        self.manager.init_capture(
            self.course,
            "protected-bundle",
            protected_database,
            self.inputs / "source.yaml",
        )
        protected_capture = self.course / "raw" / "protected-bundle"
        runtime_root = self.root / "fresh-bound-bundle-root"
        bundle = runtime_root / "bundle"
        real_validate_layout = map_release_module._validate_localization_runtime_layout
        swapped = False

        def validate_layout_then_swap(*arguments, **keywords):
            nonlocal swapped
            result = real_validate_layout(*arguments, **keywords)
            if not swapped:
                protected_capture.rename(runtime_root)
                swapped = True
            return result

        with mock.patch.object(
                map_release_module,
                "_validate_localization_runtime_layout",
                side_effect=validate_layout_then_swap):
            with self.assertRaises(ReleaseValidationError):
                self.manager.prepare_localization_runtime_bundle(
                    self.manifest,
                    self.release_map,
                    self.released_database,
                    self.source_config,
                    bundle,
                    runtime_root_path=runtime_root,
                )

        self.assertTrue(swapped)
        self.assertFalse(bundle.exists())

    def test_preflight_rejects_mismatched_source_corrupt_release_and_same_path(self):
        other = self.inputs / "other.db"
        other.write_bytes(b"wrong")
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(self.manifest, self.release_map, other, self.runtime)
        self.release_map.write_text("not a map", encoding="utf-8")
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(
                self.manifest, self.release_map, self.released_database, self.runtime)
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(
                self.manifest, self.release_map, self.released_database, self.released_database)

    def test_preflight_rejects_symlink_and_atomically_replaces_existing_runtime_file(self):
        target = self.inputs / "target.db"
        target.write_bytes(self.released_database.read_bytes())
        link = self.inputs / "source-link.db"
        link.symlink_to(target)
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(self.manifest, self.release_map, link, self.runtime)
        self.runtime.write_bytes(b"old runtime")
        prepare_runtime_database(self.manifest, self.release_map, self.released_database, self.runtime)
        self.assertEqual(self.runtime.read_bytes(), self.released_database.read_bytes())
        self.assertFalse(any(path.name.startswith(".rtabmap.db.") for path in self.runtime.parent.iterdir()))

    def test_preflight_streams_large_database_and_rejects_unsafe_runtime_aliases(self):
        source_identity = (self.released_database.stat().st_dev, self.released_database.stat().st_ino)
        real_read = os.read
        requests = []

        def capture_reads(descriptor, size):
            metadata = os.fstat(descriptor)
            if (metadata.st_dev, metadata.st_ino) == source_identity:
                requests.append(size)
            return real_read(descriptor, size)

        with mock.patch("stier_slam_core.map_release.os.read", side_effect=capture_reads):
            prepare_runtime_database(
                self.manifest, self.release_map, self.released_database, self.runtime)
        self.assertGreater(len(requests), 1)
        self.assertLessEqual(max(requests), 1024 * 1024)

        target = self.runtime.parent / "target.db"
        target.write_bytes(b"target")
        unsafe_link = self.runtime.parent / "unsafe-link.db"
        unsafe_link.symlink_to(target)
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(
                self.manifest, self.release_map, self.released_database, unsafe_link)
        hardlink = self.runtime.parent / "source-hardlink.db"
        os.link(self.released_database, hardlink)
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_database(
                self.manifest, self.release_map, self.released_database, hardlink)

    def test_runtime_config_copy_preserves_static_ini_and_allows_runtime_mutation(self):
        source_bytes = self.source_config.read_bytes()
        self.assertEqual(
            prepare_runtime_config(self.source_config, self.runtime_config), self.runtime_config
        )
        self.runtime_config.write_text("[Core]\nchanged=true\n", encoding="utf-8")
        self.assertEqual(self.source_config.read_bytes(), source_bytes)
        self.assertNotEqual(self.runtime_config.read_bytes(), source_bytes)
        with self.assertRaises(ReleaseValidationError):
            prepare_runtime_config(self.source_config, self.source_config)

    def test_guard_executes_command_only_after_successful_preflight(self):
        execvp = mock.Mock()
        self.assertEqual(guard_main([
            "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database), "--runtime-database", str(self.runtime),
            "--source-config", str(self.source_config), "--runtime-config", str(self.runtime_config),
            "--", "rtabmap", "--quiet"], execvp=execvp), 0)
        self.assertEqual(self.runtime.read_bytes(), self.released_database.read_bytes())
        self.assertEqual(self.runtime_config.read_bytes(), self.source_config.read_bytes())
        execvp.assert_called_once_with("rtabmap", ["rtabmap", "--quiet"])

    def test_guard_returns_error_without_exec_when_preflight_fails(self):
        execvp = mock.Mock()
        with mock.patch("stier_slam_core.runtime_database.prepare_runtime_plan",
                        side_effect=ReleaseValidationError("bad release")):
            self.assertEqual(guard_main([
                "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
                "--source-database", str(self.released_database), "--runtime-database", str(self.runtime),
                "--source-config", str(self.source_config), "--runtime-config", str(self.runtime_config),
                "--", "rtabmap"], execvp=execvp), 2)
        execvp.assert_not_called()

    def test_guard_rejects_localization_cross_aliases_before_any_runtime_copy(self):
        protected = {
            path: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (
                self.manifest, self.release_map, self.release / "map.pgm",
                self.release / "static_overlay.json", self.release / "semantic_layer.json",
                self.released_database, self.course / "raw" / "capture" / "capture_manifest.json",
                self.course / "raw" / "capture" / "base.pgm",
                self.course / "raw" / "capture" / "base.yaml", self.source_config,
            )
        }
        cases = (
            ("runtime_config", self.manifest),
            ("runtime_config", self.release_map),
            ("runtime_config", self.released_database),
            ("runtime_database", self.manifest),
            ("runtime_database", self.release_map),
            ("runtime_database", self.source_config),
            ("runtime_database", self.runtime_config),
        )
        for index, (field, target) in enumerate(cases):
            with self.subTest(field=field, target=target.name):
                # Use distinct, pristine artifact sets per case so a vulnerable
                # implementation cannot hide one overwrite behind another.
                if index:
                    self.tearDown()
                    self.setUp()
                    protected = {
                        path: hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in (
                            self.manifest, self.release_map, self.release / "map.pgm",
                            self.release / "static_overlay.json", self.release / "semantic_layer.json",
                            self.released_database, self.course / "raw" / "capture" / "capture_manifest.json",
                            self.course / "raw" / "capture" / "base.pgm",
                            self.course / "raw" / "capture" / "base.yaml", self.source_config,
                        )
                    }
                    target = {
                        "manifest.json": self.manifest,
                        "map.yaml": self.release_map,
                        "map.db": self.released_database,
                        "rtabmap.ini": self.source_config,
                    }[target.name]
                runtime_database = target if field == "runtime_database" else self.runtime
                runtime_config = target if field == "runtime_config" else self.runtime_config
                execvp = mock.Mock()
                self.assertEqual(guard_main([
                    "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
                    "--source-database", str(self.released_database), "--runtime-database", str(runtime_database),
                    "--source-config", str(self.source_config), "--runtime-config", str(runtime_config),
                    "--", "rtabmap"], execvp=execvp), 2)
                self.assertFalse(self.runtime.exists())
                self.assertFalse(self.runtime_config.exists())
                for path, source_hash in protected.items():
                    self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), source_hash)
                execvp.assert_not_called()

        execvp = mock.Mock()
        self.assertEqual(guard_main([
            "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database), "--runtime-database", str(self.runtime_config),
            "--source-config", str(self.source_config), "--runtime-config", str(self.runtime_config),
            "--", "rtabmap"], execvp=execvp), 2)
        self.assertFalse(self.runtime_config.exists())
        execvp.assert_not_called()

    def test_guard_rejects_mapping_aliases_and_immutable_storage_targets_before_copy(self):
        execvp = mock.Mock()
        for mapping_database, runtime_config in (
                (self.source_config, self.runtime_config),
                (self.runtime_config, self.runtime_config)):
            with self.subTest(mapping_database=mapping_database, runtime_config=runtime_config):
                self.assertEqual(guard_main([
                    "--source-config", str(self.source_config), "--runtime-config", str(runtime_config),
                    "--mapping-database", str(mapping_database), "--", "rtabmap"], execvp=execvp), 2)
        execvp.assert_not_called()

    def test_mapping_guard_creates_only_one_private_session_below_declared_runtime_root(self):
        runtime_root = self.root / "mapping-runtime"
        session = runtime_root / "mapping-session"
        database = session / "stier_slam_mapping.db"
        runtime_config = session / "rtabmap.ini"
        execvp = mock.Mock()
        result = guarded_result([
            "--source-config", str(self.source_config),
            "--runtime-config", str(runtime_config),
            "--mapping-database", str(database),
            "--mapping-runtime-root", str(runtime_root),
            "--mapping-session-dir", str(session),
            "--", "rtabmap"], execvp)
        self.assertEqual(result, 0)
        self.assertTrue(runtime_root.is_dir())
        self.assertEqual(runtime_config.read_bytes(), self.source_config.read_bytes())
        self.assertFalse(database.exists())
        execvp.assert_called_once_with("rtabmap", ["rtabmap"])

    def test_mapping_guard_rejects_outside_paths_and_existing_inode_aliases(self):
        runtime_root = self.root / "mapping-runtime"
        session = runtime_root / "mapping-session"
        session.mkdir(parents=True)
        runtime_config = session / "rtabmap.ini"
        protected_before = snapshot_tree(self.course)
        cases = (
            self.source_config,
            self.released_database,
            self.course / "releases" / "release" / "new-mapping.db",
        )
        for database in cases:
            with self.subTest(database=database):
                execvp = mock.Mock()
                self.assertEqual(guarded_result([
                    "--source-config", str(self.source_config),
                    "--runtime-config", str(runtime_config),
                    "--mapping-database", str(database),
                    "--mapping-runtime-root", str(runtime_root),
                    "--mapping-session-dir", str(session),
                    "--", "rtabmap"], execvp), 2)
                execvp.assert_not_called()
                self.assertFalse(runtime_config.exists())

        hardlink = session / "hardlinked.db"
        os.link(self.released_database, hardlink)
        execvp = mock.Mock()
        self.assertEqual(guarded_result([
            "--source-config", str(self.source_config),
            "--runtime-config", str(runtime_config),
            "--mapping-database", str(hardlink),
            "--mapping-runtime-root", str(runtime_root),
            "--mapping-session-dir", str(session),
            "--", "rtabmap"], execvp), 2)
        execvp.assert_not_called()
        self.assertEqual(snapshot_tree(self.course), protected_before)

    def test_guard_prepares_mapping_and_localization_only_after_unified_plan(self):
        mapping_root = self.root / "mapping-runtime"
        mapping_session = mapping_root / "session"
        mapping_database = mapping_session / "mapping.db"
        mapping_config = mapping_session / "rtabmap.ini"
        mapping_exec = mock.Mock()
        self.assertEqual(guard_main([
            "--source-config", str(self.source_config), "--runtime-config", str(mapping_config),
            "--mapping-database", str(mapping_database),
            "--mapping-runtime-root", str(mapping_root),
            "--mapping-session-dir", str(mapping_session),
            "--", "rtabmap"], execvp=mapping_exec), 0)
        mapping_exec.assert_called_once()
        self.assertEqual(mapping_config.read_bytes(), self.source_config.read_bytes())

        localization_exec = mock.Mock()
        self.assertEqual(guard_main([
            "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database), "--runtime-database", str(self.runtime),
            "--source-config", str(self.source_config), "--runtime-config", str(self.runtime_config),
            "--", "rtabmap"], execvp=localization_exec), 0)
        localization_exec.assert_called_once()
        self.assertEqual(self.runtime.read_bytes(), self.released_database.read_bytes())

    def test_capture_rejects_invalid_or_incompatible_rtabmap_databases_without_publication(self):
        cases = {
            "empty.db": b"",
            "bytes.db": b"not sqlite",
        }
        for name, content in cases.items():
            source = self.inputs / name
            source.write_bytes(content)
            with self.subTest(name=name), self.assertRaises(ReleaseValidationError):
                self.manager.init_capture(self.course, "bad-{}".format(name), source, self.inputs / "source.yaml")
            self.assertFalse((self.course / "raw" / "bad-{}".format(name)).exists())
            self.assertFalse(any(path.name.startswith("tmp") for path in self.course.iterdir()))

        plain = self.inputs / "plain.db"
        sqlite3.connect(str(plain)).close()
        with self.assertRaises(ReleaseValidationError):
            self.manager.init_capture(self.course, "bad-plain", plain, self.inputs / "source.yaml")
        self.assertFalse((self.course / "raw" / "bad-plain").exists())
        self.assertFalse(any(path.name.startswith("tmp") for path in self.course.iterdir()))

    def test_noetic_rtabmap_version_pattern_accepts_02113(self):
        self.assertTrue(
            map_release_module.re.fullmatch(r"0\.21\.(?:0|[1-9][0-9]*)", "0.21.13")
        )

    def test_rtabmap_02113_schema_rejects_missing_full_columns_tables_and_future_patch(self):
        cases = (
            ("admin-map-resolution", "Admin", "opt_map_resolution", None),
            ("node-time", "Node", "time_enter", None),
            ("data-user", "Data", "user_data", None),
            ("link-user", "Link", "user_data", None),
            ("statistics", None, None, "Statistics"),
        )
        for capture_id, table, column, drop_table in cases:
            database = self.inputs / "{}.db".format(capture_id)
            write_valid_rtabmap_database(database)
            if drop_table:
                connection = sqlite3.connect(str(database))
                try:
                    connection.execute("DROP TABLE {}".format(drop_table))
                    connection.commit()
                finally:
                    connection.close()
            else:
                remove_schema_column(database, table, column)
            source_hash = hashlib.sha256(database.read_bytes()).hexdigest()
            with self.subTest(capture_id=capture_id), self.assertRaises(ReleaseValidationError):
                self.manager.init_capture(
                    self.course, capture_id, database, self.inputs / "source.yaml"
                )
            self.assertEqual(hashlib.sha256(database.read_bytes()).hexdigest(), source_hash)
            self.assertFalse((self.course / "raw" / capture_id).exists())

        future = self.inputs / "future.db"
        write_valid_rtabmap_database(future, version="0.21.999")
        with self.assertRaises(ReleaseValidationError):
            self.manager.init_capture(
                self.course, "future-version", future, self.inputs / "source.yaml"
            )
        self.assertFalse((self.course / "raw" / "future-version").exists())

    def test_sqlite_validation_reads_the_open_snapshot_fd_when_path_is_replaced(self):
        invalid = self.inputs / "invalid-source.db"
        invalid.write_bytes(b"invalid SQLite source remains invalid")
        source_hash = hashlib.sha256(invalid.read_bytes()).hexdigest()
        replacement = self.inputs / "replacement.db"
        write_valid_rtabmap_database(replacement)
        course_before = snapshot_tree(self.course)
        real_connect = map_release_module.sqlite3.connect
        observed_uris = []

        def replace_snapshot_name_before_connect(database_uri, *args, **kwargs):
            observed_uris.append(database_uri)
            marker = "/proc/self/fd/"
            if marker in database_uri:
                descriptor = int(database_uri.split(marker, 1)[1].split("?", 1)[0])
                snapshot_path = Path(os.readlink("/proc/self/fd/{}".format(descriptor)))
            else:
                snapshot_path = Path(
                    database_uri.split("?", 1)[0].replace("file://", "", 1)
                )
            os.replace(str(replacement), str(snapshot_path))
            return real_connect(database_uri, *args, **kwargs)

        with mock.patch.object(
                map_release_module.sqlite3, "connect",
                side_effect=replace_snapshot_name_before_connect):
            with self.assertRaises(ReleaseValidationError):
                self.manager.init_capture(
                    self.course, "snapshot-swap", invalid, self.inputs / "source.yaml"
                )

        self.assertTrue(observed_uris)
        self.assertTrue(all("/proc/self/fd/" in uri for uri in observed_uris))
        self.assertEqual(hashlib.sha256(invalid.read_bytes()).hexdigest(), source_hash)
        self.assertEqual(snapshot_tree(self.course), course_before)
        self.assertFalse((self.course / "raw" / "snapshot-swap").exists())

    def test_localization_runtime_bundle_is_private_atomic_and_source_independent(self):
        first = self.root / "runtime" / "bundle-first"
        second = self.root / "runtime" / "bundle-second"
        for bundle in (first, second):
            result = ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
            self.assertEqual(result, bundle)
            self.assertTrue((bundle / ".ready.json").is_file())
            self.assertEqual((bundle / "map.yaml").read_bytes(), self.release_map.read_bytes())
            self.assertEqual((bundle / "map.pgm").read_bytes(), (self.release / "map.pgm").read_bytes())
            self.assertEqual((bundle / "map.db").read_bytes(), self.released_database.read_bytes())
            self.assertEqual((bundle / "rtabmap.ini").read_bytes(), self.source_config.read_bytes())

        first_db = (first / "map.db").read_bytes()
        self.released_database.write_bytes(b"source changed after staging")
        self.assertEqual((first / "map.db").read_bytes(), first_db)

    def test_guard_prepares_the_same_bundle_idempotently_for_map_server_and_rtabmap(self):
        runtime_root = self.root / "runtime"
        bundle = runtime_root / "shared-bundle"
        common = [
            "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database),
            "--source-config", str(self.source_config),
            "--runtime-root", str(runtime_root),
            "--runtime-bundle", str(bundle),
        ]
        map_server_exec = mock.Mock()
        rtabmap_exec = mock.Mock()
        self.assertEqual(guarded_result(
            common + ["--bundle-role", "map-server", "--", "map_server", str(bundle / "map.yaml")],
            map_server_exec), 0)
        self.assertEqual(guarded_result(
            common + ["--bundle-role", "rtabmap", "--", "rtabmap"], rtabmap_exec), 0)
        self.assertTrue((bundle / ".ready.json").is_file())
        map_server_exec.assert_called_once_with("map_server", ["map_server", str(bundle / "map.yaml")])
        rtabmap_exec.assert_called_once_with("rtabmap", ["rtabmap"])

    def test_bundle_mode_rejects_individual_runtime_path_overrides_before_staging(self):
        runtime_root = self.root / "runtime"
        bundle = runtime_root / "override-bundle"
        protected_before = snapshot_tree(self.course)
        execvp = mock.Mock()
        result = guarded_result([
            "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database),
            "--runtime-database", str(self.manifest),
            "--source-config", str(self.source_config),
            "--runtime-config", str(self.release_map),
            "--runtime-bundle", str(bundle),
            "--", "rtabmap"], execvp)
        self.assertEqual(result, 2)
        self.assertFalse(bundle.exists())
        self.assertEqual(snapshot_tree(self.course), protected_before)
        execvp.assert_not_called()

    def test_ready_manifest_binds_hashes_of_every_staged_artifact(self):
        bundle = self.root / "runtime" / "hash-bound-bundle"
        ReleaseManager.prepare_localization_runtime_bundle(
            self.manifest, self.release_map, self.released_database, self.source_config, bundle
        )
        ready = json.loads((bundle / ".ready.json").read_text(encoding="utf-8"))
        self.assertEqual(
            ready.get("artifact_sha256"),
            {
                name: hashlib.sha256((bundle / name).read_bytes()).hexdigest()
                for name in ("map.yaml", "map.pgm", "map.db", "rtabmap.ini")
            },
        )

    def test_bundle_roles_reject_tampering_without_blocking_map_server_on_mutable_db_ini(self):
        for mutable_name in ("map.db", "rtabmap.ini"):
            bundle = self.root / "runtime" / "mutable-{}".format(mutable_name.replace(".", "-"))
            ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
            (bundle / mutable_name).write_bytes(b"runtime mutation")
            common = [
                "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
                "--source-database", str(self.released_database),
                "--source-config", str(self.source_config),
                "--runtime-root", str(bundle.parent), "--runtime-bundle", str(bundle),
            ]
            map_exec = mock.Mock()
            rtab_exec = mock.Mock()
            with self.subTest(role="map-server", mutable_name=mutable_name):
                self.assertEqual(guarded_result(
                    common + ["--bundle-role", "map-server", "--", "map_server", str(bundle / "map.yaml")],
                    map_exec), 0)
                map_exec.assert_called_once()
            with self.subTest(role="rtabmap", mutable_name=mutable_name):
                self.assertEqual(guarded_result(
                    common + ["--bundle-role", "rtabmap", "--", "rtabmap"], rtab_exec), 2)
                rtab_exec.assert_not_called()

        for map_name in ("map.yaml", "map.pgm"):
            bundle = self.root / "runtime" / "tampered-{}".format(map_name.replace(".", "-"))
            ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
            (bundle / map_name).write_bytes(b"tampered map")
            with self.subTest(map_name=map_name), self.assertRaises(ReleaseValidationError):
                ReleaseManager.prepare_localization_runtime_bundle(
                    self.manifest, self.release_map, self.released_database,
                    self.source_config, bundle,
                )

    def test_rtabmap_role_rejects_writable_bundle_hardlinks_without_touching_sources(self):
        for runtime_name, source in (
                ("map.db", self.released_database),
                ("rtabmap.ini", self.source_config)):
            bundle = self.root / "runtime" / "hardlink-{}".format(
                runtime_name.replace(".", "-")
            )
            ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
            source_bytes = source.read_bytes()
            source_hash = hashlib.sha256(source_bytes).hexdigest()
            (bundle / runtime_name).unlink()
            os.link(source, bundle / runtime_name)
            execvp = mock.Mock()
            result = guarded_result([
                "--release-manifest", str(self.manifest),
                "--map-yaml", str(self.release_map),
                "--source-database", str(self.released_database),
                "--source-config", str(self.source_config),
                "--runtime-root", str(bundle.parent),
                "--runtime-bundle", str(bundle),
                "--bundle-role", "rtabmap", "--", "rtabmap",
            ], execvp)

            with self.subTest(runtime_name=runtime_name):
                self.assertEqual(result, 2)
                execvp.assert_not_called()
                self.assertEqual(source.read_bytes(), source_bytes)
                self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), source_hash)

    def test_map_server_rejects_non_regular_mutable_bundle_entries(self):
        for runtime_name, replacement_kind in (
                ("map.db", "symlink"),
                ("rtabmap.ini", "directory")):
            bundle = self.root / "runtime" / "unsafe-{}".format(
                replacement_kind
            )
            ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
            target = bundle / runtime_name
            target.unlink()
            if replacement_kind == "symlink":
                target.symlink_to(self.source_config)
            else:
                target.mkdir()
            execvp = mock.Mock()
            result = guarded_result([
                "--release-manifest", str(self.manifest),
                "--map-yaml", str(self.release_map),
                "--source-database", str(self.released_database),
                "--source-config", str(self.source_config),
                "--runtime-root", str(bundle.parent),
                "--runtime-bundle", str(bundle),
                "--bundle-role", "map-server", "--", "map_server",
                str(bundle / "map.yaml"),
            ], execvp)

            with self.subTest(runtime_name=runtime_name, replacement_kind=replacement_kind):
                self.assertEqual(result, 2)
                execvp.assert_not_called()

    def test_bundle_rejects_intermediate_symlink_before_any_release_write(self):
        runtime_root = self.root / "private-runtime"
        runtime_root.mkdir()
        alias = runtime_root / "redirect"
        alias.symlink_to(self.release, target_is_directory=True)
        bundle = alias / "nested" / "bundle"
        release_before = snapshot_tree(self.release)
        names_before = {path.name for path in self.release.iterdir()}
        with self.assertRaises(ReleaseValidationError):
            ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )
        self.assertEqual(snapshot_tree(self.release), release_before)
        self.assertEqual({path.name for path in self.release.iterdir()}, names_before)
        self.assertFalse((self.release / "nested").exists())

    def test_eight_concurrent_publishers_leave_one_complete_bundle_and_no_temps(self):
        bundle = self.root / "runtime" / "concurrent-bundle"
        publication_barrier = threading.Barrier(8)
        real_publish = map_release_module._rename_directory_noreplace

        def publish_together(*arguments):
            publication_barrier.wait(timeout=10)
            return real_publish(*arguments)

        def stage(_):
            return ReleaseManager.prepare_localization_runtime_bundle(
                self.manifest, self.release_map, self.released_database,
                self.source_config, bundle,
            )

        with mock.patch.object(
                map_release_module, "_rename_directory_noreplace",
                side_effect=publish_together):
            with ThreadPoolExecutor(max_workers=8) as executor:
                results = list(executor.map(stage, range(8)))
        self.assertEqual(results, [bundle] * 8)
        self.assertEqual(
            {path.name for path in bundle.iterdir()},
            {"map.yaml", "map.pgm", "map.db", "rtabmap.ini", ".ready.json"},
        )
        self.assertEqual(
            [path for path in bundle.parent.iterdir()
             if path.is_dir() and path.name.startswith(".")], []
        )
        ready = (bundle / ".ready.json").read_bytes()
        self.assertTrue(ready)

    def test_fresh_runtime_root_mkdir_race_allows_eight_bundle_guards(self):
        runtime_root = self.root / "fresh-localization-runtime"
        bundle = runtime_root / "shared-bundle"
        mkdir_barrier = threading.Barrier(8)
        real_mkdir = os.mkdir
        executed = []
        executed_lock = threading.Lock()

        def mkdir_together(path, mode=0o777, *, dir_fd=None):
            if path == runtime_root.name and dir_fd is not None:
                mkdir_barrier.wait(timeout=10)
            return real_mkdir(path, mode, dir_fd=dir_fd)

        def execute(command, arguments):
            with executed_lock:
                executed.append((command, tuple(arguments)))

        common = [
            "--release-manifest", str(self.manifest),
            "--map-yaml", str(self.release_map),
            "--source-database", str(self.released_database),
            "--source-config", str(self.source_config),
            "--runtime-root", str(runtime_root),
            "--runtime-bundle", str(bundle),
            "--bundle-role", "rtabmap", "--", "rtabmap",
        ]
        with mock.patch.object(map_release_module.os, "mkdir", side_effect=mkdir_together):
            with ThreadPoolExecutor(max_workers=8) as executor:
                results = list(executor.map(
                    lambda _: guarded_result(common, execute), range(8)
                ))

        self.assertEqual(results, [0] * 8)
        self.assertEqual(len(executed), 8)
        self.assertEqual(
            {path.name for path in runtime_root.iterdir()}, {bundle.name}
        )
        self.assertEqual(
            {path.name for path in bundle.iterdir()},
            {"map.yaml", "map.pgm", "map.db", "rtabmap.ini", ".ready.json"},
        )
        self.assertFalse(any(path.name.startswith(".") for path in runtime_root.iterdir()))

    def test_fresh_mapping_session_mkdir_race_allows_eight_guards(self):
        runtime_root = self.root / "fresh-mapping-runtime"
        runtime_root.mkdir()
        session = runtime_root / "shared-session"
        runtime_config = session / "rtabmap.ini"
        database = session / "stier_slam_mapping.db"
        mkdir_barrier = threading.Barrier(8)
        real_mkdir = os.mkdir
        executed = []
        executed_lock = threading.Lock()

        def mkdir_together(path, mode=0o777, *, dir_fd=None):
            if path == session.name and dir_fd is not None:
                mkdir_barrier.wait(timeout=10)
            return real_mkdir(path, mode, dir_fd=dir_fd)

        def execute(command, arguments):
            with executed_lock:
                executed.append((command, tuple(arguments)))

        common = [
            "--source-config", str(self.source_config),
            "--runtime-config", str(runtime_config),
            "--mapping-database", str(database),
            "--mapping-runtime-root", str(runtime_root),
            "--mapping-session-dir", str(session),
            "--", "rtabmap",
        ]
        with mock.patch.object(map_release_module.os, "mkdir", side_effect=mkdir_together):
            with ThreadPoolExecutor(max_workers=8) as executor:
                results = list(executor.map(
                    lambda _: guarded_result(common, execute), range(8)
                ))

        self.assertEqual(results, [0] * 8)
        self.assertEqual(len(executed), 8)
        self.assertEqual({path.name for path in runtime_root.iterdir()}, {session.name})
        self.assertEqual({path.name for path in session.iterdir()}, {runtime_config.name})
        self.assertEqual(runtime_config.read_bytes(), self.source_config.read_bytes())
        self.assertFalse(any(path.name.startswith(".") for path in session.iterdir()))

    def test_private_runtime_mkdir_race_rejects_symlink_and_non_directory_winners(self):
        for winner_kind in ("symlink", "file"):
            runtime_root = self.root / "hostile-{}-runtime".format(winner_kind)
            real_mkdir = os.mkdir
            injected = [False]

            def inject_winner(path, mode=0o777, *, dir_fd=None):
                if path == runtime_root.name and dir_fd is not None and not injected[0]:
                    injected[0] = True
                    if winner_kind == "symlink":
                        os.symlink(str(self.release), path, dir_fd=dir_fd)
                    else:
                        descriptor = os.open(
                            path, os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                            0o600, dir_fd=dir_fd,
                        )
                        os.close(descriptor)
                    raise FileExistsError(errno.EEXIST, "winner created unsafe entry", path)
                return real_mkdir(path, mode, dir_fd=dir_fd)

            with self.subTest(winner_kind=winner_kind), mock.patch.object(
                    map_release_module.os, "mkdir", side_effect=inject_winner):
                with self.assertRaises(ReleaseValidationError):
                    map_release_module._secure_ensure_directory(runtime_root)

    def test_bundle_never_publishes_partial_directory_or_reuses_another_manifest(self):
        bundle = self.root / "runtime" / "bundle"
        with mock.patch("stier_slam_core.map_release._copy_verified_runtime_file_at",
                        side_effect=ReleaseValidationError("copy failure")):
            with self.assertRaises(ReleaseValidationError):
                ReleaseManager.prepare_localization_runtime_bundle(
                    self.manifest, self.release_map, self.released_database,
                    self.source_config, bundle,
                )
        self.assertFalse(bundle.exists())
        self.assertFalse(any(path.name == ".ready.json" for path in bundle.parent.glob("**/.ready.json")))

        ReleaseManager.prepare_localization_runtime_bundle(
            self.manifest, self.release_map, self.released_database, self.source_config, bundle
        )
        self.manager.init_workspace(self.course, "capture", "workspace-second")
        other_release = self.manager.create_release(
            self.course, "capture", "workspace-second", "release-second", "2026-08-08T00:01:00Z"
        )
        with self.assertRaises(ReleaseValidationError):
            ReleaseManager.prepare_localization_runtime_bundle(
                other_release / "manifest.json", other_release / "map.yaml", self.released_database,
                self.source_config, bundle,
            )

    def test_localization_rejects_corrupt_schema_or_incompatible_database_before_runtime_copy(self):
        source_hash = hashlib.sha256(self.released_database.read_bytes()).hexdigest()
        for sql in (
                "DELETE FROM Data",
                "UPDATE Admin SET version = '0.22.0'",
                "DROP TABLE Link"):
            with self.subTest(sql=sql):
                connection = sqlite3.connect(str(self.released_database))
                try:
                    connection.execute(sql)
                    connection.commit()
                finally:
                    connection.close()
                self._rewrite_capture_and_release_database_hashes()
                execvp = mock.Mock()
                self.assertEqual(guard_main([
                    "--release-manifest", str(self.manifest), "--map-yaml", str(self.release_map),
                    "--source-database", str(self.released_database), "--runtime-database", str(self.runtime),
                    "--source-config", str(self.source_config), "--runtime-config", str(self.runtime_config),
                    "--", "rtabmap"], execvp=execvp), 2)
                self.assertFalse(self.runtime.exists())
                self.assertFalse(self.runtime_config.exists())
                execvp.assert_not_called()
                self.tearDown()
                self.setUp()

    def _rewrite_capture_and_release_database_hashes(self):
        """Keep immutable-manifest integrity checks focused on DB schema validation."""
        database_hash = hashlib.sha256(self.released_database.read_bytes()).hexdigest()
        capture_manifest = self.course / "raw" / "capture" / "capture_manifest.json"
        document = __import__("json").loads(capture_manifest.read_text(encoding="utf-8"))
        document["hashes"]["map.db"] = database_hash
        capture_manifest.write_text(__import__("json").dumps(document, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")
        manifest = __import__("json").loads(self.manifest.read_text(encoding="utf-8"))
        manifest["source_database"]["sha256"] = database_hash
        self.manifest.write_text(__import__("json").dumps(manifest, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")
