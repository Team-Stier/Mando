"""불변 capture, workspace, release 및 activation artifact를 관리한다."""

import ctypes
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import sqlite3
import stat
import tempfile
import threading
import time
import weakref
from dataclasses import dataclass, field, replace

from stier_slam_core.grid_map import (
    MapFormatError,
    grid_map_from_pgm,
    load_ros_map,
    parse_ros_map_yaml,
    write_ros_map,
)
from stier_slam_core.map_overlay import OverlayValidationError, apply_overlay, validate_overlay


_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_CAPTURE_FILES = {"map.db", "base.pgm", "base.yaml", "capture_manifest.json"}
_RELEASE_ARTIFACTS = {
    "map.pgm", "map.yaml", "static_overlay.json", "semantic_layer.json"
}
_RELEASE_FILES = _RELEASE_ARTIFACTS | {"manifest.json"}
_EMPTY_OVERLAY = b'{"schema_version":1,"operations":[]}\n'
_EMPTY_SEMANTIC = b'{"schema_version":1,"features":[]}\n'
_RENAME_NOREPLACE = 1
_RENAME_EXCHANGE = 2
_STREAM_CHUNK_SIZE = 1024 * 1024
_RUNTIME_DIRECTORY_XATTR = b"user.stier_slam_runtime_path_v1"
_RUNTIME_DIRECTORY_MARKER_RETRIES = 100
_RUNTIME_DIRECTORY_MARKER_DELAY_SECONDS = 0.005
_RUNTIME_PUBLICATION_RETRIES = 100
_RUNTIME_PUBLICATION_RETRY_SECONDS = 0.001
_REVISION_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_DESCRIPTOR_COURSE_PATTERN = re.compile(r"^/proc/self/fd/([0-9]+)$")
_DESCRIPTOR_ROOT_PATTERN = re.compile(
    r"^/(?:proc/(?:self|thread-self|[0-9]+)/fd|dev/fd)/[0-9]+(?:/|$)"
)
_SEMANTIC_GEOMETRY_BY_LABEL = {
    "lane_center": "polyline",
    "stop_line": "polyline",
    "intersection": "point",
    "course_boundary": "polyline",
}
_RTABMAP_02113_REQUIRED_COLUMNS = {
    "Admin": {
        "version", "preview_image", "opt_cloud", "opt_ids", "opt_poses",
        "opt_last_localization", "opt_polygons_size", "opt_polygons", "opt_tex_coords",
        "opt_tex_materials", "opt_map", "opt_map_x_min", "opt_map_y_min",
        "opt_map_resolution", "time_enter",
    },
    "Node": {
        "id", "map_id", "weight", "stamp", "pose", "ground_truth_pose", "velocity",
        "label", "gps", "env_sensors", "time_enter",
    },
    "Data": {
        "id", "image", "depth", "calibration", "scan", "scan_info", "ground_cells",
        "obstacle_cells", "empty_cells", "cell_size", "view_point_x", "view_point_y",
        "view_point_z", "user_data", "time_enter",
    },
    "Link": {
        "from_id", "to_id", "type", "information_matrix", "transform", "user_data",
    },
    "Word": {"id", "descriptor_size", "descriptor", "time_enter"},
    "Feature": {
        "node_id", "word_id", "pos_x", "pos_y", "size", "dir", "response", "octave",
        "depth_x", "depth_y", "depth_z", "descriptor_size", "descriptor",
    },
    "GlobalDescriptor": {"node_id", "type", "info", "data"},
    "Info": {
        "STM_size", "last_sign_added", "process_mem_used", "database_mem_used",
        "dictionary_size", "parameters", "time_enter",
    },
    "Statistics": {"id", "stamp", "data", "wm_state"},
}
_LIBC = ctypes.CDLL(None, use_errno=True)
_RENAMEAT2 = getattr(_LIBC, "renameat2", None)
if _RENAMEAT2 is not None:
    _RENAMEAT2.argtypes = (
        ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
        ctypes.c_uint,
    )
    _RENAMEAT2.restype = ctypes.c_int


class ReleaseValidationError(ValueError):
    """변경 가능 또는 불변 map artifact의 검증이 실패할 때 발생한다."""


class WorkspaceConflictError(ReleaseValidationError):
    """workspace revision이 다른 editor에 의해 교체됐을 때 발생한다."""


class _ActivationDurabilityIndeterminate(OSError):
    """active pointer는 교체됐지만 directory fsync가 확인되지 않은 상태를 뜻한다."""


class _DirectoryPublicationIndeterminate(OSError):
    """directory는 rename됐지만 영구적인 publish가 확인되지 않은 상태를 뜻한다."""


class _WorkspaceDurabilityIndeterminate(OSError):
    """workspace generation은 교환됐지만 parent 영속성이 확인되지 않은 상태를 뜻한다."""


class _WorkspaceLease:
    """역할:
    한 workspace transaction 동안 course와 workspaces directory의 inode를 고정한다.

    동작 방식:
    lock을 획득한 directory descriptor와 device/inode identity를 보관해, 경로가 실행
    중 교체되더라도 검증했던 동일 directory에서만 읽기와 쓰기가 이뤄지게 한다.
    """

    def __init__(
            self, course_descriptor, workspaces_descriptor, workspace_id, exclusive):
        self.course_descriptor = course_descriptor
        self.workspaces_descriptor = workspaces_descriptor
        self.workspace_id = workspace_id
        self.exclusive = exclusive
        course_metadata = os.fstat(course_descriptor)
        workspaces_metadata = os.fstat(workspaces_descriptor)
        self.course_identity = (course_metadata.st_dev, course_metadata.st_ino)
        self.workspaces_identity = (
            workspaces_metadata.st_dev, workspaces_metadata.st_ino
        )

    @property
    def course_path(self):
        return Path("/proc/self/fd/{}".format(self.course_descriptor))


@dataclass(frozen=True)
class RuntimeArtifactPlan:
    """역할:
    RTAB-Map runtime에 복사할 source와 destination 경로의 preflight 결과를 보관한다.

    동작 방식:
    config, database, release 및 mapping 경로를 완전 검증한 뒤 authorization과 함께
    묶으며, factory가 만든 동일 plan이 한 번 claim된 경우에만 실제 쓰기를 허용한다.
    """

    source_config: object = None
    runtime_config: object = None
    source_database: object = None
    runtime_database: object = None
    database_sha256: object = None
    release_manifest: object = None
    map_yaml: object = None
    mapping_database: object = None
    mapping_runtime_root: object = None
    mapping_session_dir: object = None
    _authorization: object = field(default=None, repr=False, compare=False)


@dataclass(frozen=True)
class _RuntimePlanAuthorization:
    """역할:
    preflight가 승인한 ``RuntimeArtifactPlan``의 값 snapshot을 증명한다.

    동작 방식:
    검증 직후의 plan field tuple을 보관하고 claim 시 현재 값과 비교해 사후 변경된
    plan이 runtime write에 사용되는 것을 차단한다.
    """

    values: tuple


@dataclass(frozen=True)
class _RuntimeSourceBinding:
    """역할:
    runtime source file의 inode identity와 내용 hash를 하나로 묶는다.

    동작 방식:
    preflight와 copy 시점의 file identity 및 SHA-256을 비교할 수 있게 해 검증 후 source
    교체나 내용 변경을 감지한다.
    """

    identity: tuple
    sha256: str


@dataclass(frozen=True)
class _RuntimeDirectoryBinding:
    """역할:
    아직 존재하지 않을 수도 있는 runtime directory를 검증된 ancestor에 고정한다.

    동작 방식:
    target, 가장 가까운 기존 anchor와 그 inode, 생성해야 할 하위 경로를 저장해 symlink
    추적 없이 승인된 directory tree만 생성하게 한다.
    """

    target: object
    anchor: object
    anchor_identity: tuple
    missing_parts: tuple


@dataclass(frozen=True)
class _RuntimeExecutionBinding:
    """역할:
    runtime plan 실행에 필요한 source와 destination binding을 한 객체로 전달한다.

    동작 방식:
    config/database source identity와 각 destination parent 또는 mapping session binding을
    보관하고, plan claim 이후 atomic copy 단계에서 다시 검증한다.
    """

    source: object
    runtime_config_parent: object
    runtime_database_parent: object
    mapping_database_parent: object
    mapping_session: object


_RUNTIME_PLAN_REGISTRY = {}
_RUNTIME_PLAN_REGISTRY_LOCK = threading.Lock()


def _runtime_plan_values(plan):
    return (
        plan.source_config,
        plan.runtime_config,
        plan.source_database,
        plan.runtime_database,
        plan.database_sha256,
        plan.release_manifest,
        plan.map_yaml,
        plan.mapping_database,
        plan.mapping_runtime_root,
        plan.mapping_session_dir,
    )


def _authorize_runtime_plan(plan, execution_binding):
    authorized = replace(
        plan,
        _authorization=_RuntimePlanAuthorization(_runtime_plan_values(plan)),
    )
    plan_id = id(authorized)

    def remove_stale(reference, registered_id=plan_id):
        with _RUNTIME_PLAN_REGISTRY_LOCK:
            current = _RUNTIME_PLAN_REGISTRY.get(registered_id)
            if current is not None and current[0] is reference:
                _RUNTIME_PLAN_REGISTRY.pop(registered_id, None)

    reference = weakref.ref(authorized, remove_stale)
    with _RUNTIME_PLAN_REGISTRY_LOCK:
        _RUNTIME_PLAN_REGISTRY[plan_id] = (
            reference,
            _runtime_plan_values(authorized),
            execution_binding,
        )
    return authorized


def _claim_runtime_plan(plan):
    """runtime write 전에 factory가 만든 정확한 plan object를 한 번만 사용한다."""
    with _RUNTIME_PLAN_REGISTRY_LOCK:
        registered = _RUNTIME_PLAN_REGISTRY.pop(id(plan), None)
    if registered is None:
        raise ReleaseValidationError(
            "runtime plan was not produced by complete preflight"
        )
    reference, values, execution_binding = registered
    if (reference() is not plan
            or values != _runtime_plan_values(plan)
            or not isinstance(plan._authorization, _RuntimePlanAuthorization)
            or plan._authorization.values != values):
        raise ReleaseValidationError(
            "runtime plan changed after complete preflight"
        )
    return execution_binding


class ReleaseManager:
    """역할:
    raw capture, 편집 workspace와 불변 map release의 전체 생명주기를 관리한다.

    동작 방식:
    RTAB-Map database와 ROS map 및 overlay를 엄격히 검증하고 descriptor-bound transaction
    으로 workspace를 갱신하며, 새 release를 원자적으로 게시하고 active release를 선택한다.
    """

    @staticmethod
    def init_capture(course_dir, capture_id, db_path, map_yaml_path, now_iso=None):
        capture_id = _validate_id(capture_id, "capture_id")
        course_dir = _course_path(course_dir)
        db_path = Path(db_path)
        map_yaml_path = Path(map_yaml_path)
        try:
            with _open_path_parent(map_yaml_path, "map YAML source") as (
                    yaml_parent_descriptor, yaml_name):
                yaml_bytes = _read_regular_at(
                    yaml_parent_descriptor, yaml_name, "map YAML source"
                )
                yaml_document = parse_ros_map_yaml(yaml_bytes)
                with _open_relative_regular(
                        yaml_parent_descriptor,
                        yaml_document["image"],
                        "map image source") as image_descriptor:
                    image_bytes = _read_descriptor_snapshot(
                        image_descriptor, "map image source"
                    )
                grid = grid_map_from_pgm(yaml_document, image_bytes)
        except (MapFormatError, ReleaseValidationError) as error:
            raise ReleaseValidationError("source map is invalid") from error
        created_at = _validate_timestamp(now_iso or _utc_now_iso())
        raw_parent, raw_parent_identity = _storage_parent(
            course_dir, "raw", create=True
        )

        with _open_regular_path(db_path, "database source") as database_descriptor:
            with tempfile.NamedTemporaryFile(dir=str(course_dir), delete=False) as database_snapshot:
                snapshot_path = Path(database_snapshot.name)
                try:
                    database_hash = _snapshot_regular_descriptor(
                        database_descriptor, database_snapshot, "database source"
                    )
                    _validate_rtabmap_database_snapshot(database_snapshot, "database source")

                    def build(temporary):
                        _copy_snapshot_fsynced(database_snapshot, temporary / "map.db")
                        _write_file_fsynced(temporary / "base.pgm", image_bytes)
                        _write_file_fsynced(
                            temporary / "base.yaml", _map_yaml_bytes(grid, "base.pgm")
                        )
                        load_ros_map(temporary / "base.yaml")
                        hashes = {
                            "map.db": database_hash,
                            "base.pgm": _sha256_internal_file(temporary / "base.pgm"),
                            "base.yaml": _sha256_internal_file(temporary / "base.yaml"),
                        }
                        manifest = {
                            "schema_version": 1,
                            "capture_id": capture_id,
                            "created_at": created_at,
                            "hashes": hashes,
                        }
                        _write_file_fsynced(
                            temporary / "capture_manifest.json", _json_bytes(manifest)
                        )
                    return _create_directory_atomically(
                        raw_parent,
                        capture_id,
                        build,
                        expected_parent_identity=raw_parent_identity,
                    )
                finally:
                    try:
                        snapshot_path.unlink()
                    except FileNotFoundError:
                        pass

    initialize_capture = init_capture

    @staticmethod
    def init_workspace(course_dir, capture_id, workspace_id):
        capture_id = _validate_id(capture_id, "capture_id")
        workspace_id = _validate_id(workspace_id, "workspace_id")
        course_dir = _course_path(course_dir)
        _validated_capture(course_dir, capture_id)
        workspace_parent, workspace_parent_identity = _storage_parent(
            course_dir, "workspaces", create=True
        )

        def build(temporary):
            _write_file_fsynced(
                temporary / "source_release.json",
                _json_bytes({"schema_version": 1, "capture_id": capture_id}),
            )
            _write_file_fsynced(temporary / "static_overlay.json", _EMPTY_OVERLAY)
            _write_file_fsynced(temporary / "semantic_layer.json", _EMPTY_SEMANTIC)

        return _create_directory_atomically(
            workspace_parent,
            workspace_id,
            build,
            expected_parent_identity=workspace_parent_identity,
        )

    initialize_workspace = init_workspace

    @staticmethod
    def read_workspace(course_dir, workspace_id):
        """revision token이 포함된 검증된 editor snapshot 하나를 반환한다."""
        workspace_id = _validate_id(workspace_id, "workspace_id")
        course_dir = _course_path(course_dir, allow_descriptor=True)
        with _workspace_lock(course_dir, workspace_id, exclusive=False) as lease:
            state = _workspace_state(course_dir, workspace_id, lease)
            _require_workspace_snapshot_current(lease, state)
        return _workspace_document(workspace_id, state)

    @staticmethod
    def read_workspace_base_pgm(course_dir, workspace_id):
        """workspace가 참조하는 검증된 불변 PGM을 반환한다."""
        workspace_id = _validate_id(workspace_id, "workspace_id")
        course_dir = _course_path(course_dir, allow_descriptor=True)
        with _workspace_lock(course_dir, workspace_id, exclusive=False) as lease:
            state = _workspace_state(course_dir, workspace_id, lease)
            _require_workspace_snapshot_current(lease, state)
        return state["base_pgm_bytes"]

    @staticmethod
    def replace_workspace(
            course_dir, workspace_id, expected_revision, static_overlay, semantic_layer):
        """revision이 여전히 일치하면 두 editable layer를 원자적으로 교체한다."""
        workspace_id = _validate_id(workspace_id, "workspace_id")
        expected_revision = _validate_workspace_revision(expected_revision)
        course_dir = _course_path(course_dir, allow_descriptor=True)
        with _workspace_lock(course_dir, workspace_id, exclusive=True) as lease:
            state = _workspace_state(course_dir, workspace_id, lease)
            _require_workspace_snapshot_current(lease, state)
            if _workspace_revision(state) != expected_revision:
                raise WorkspaceConflictError("workspace revision is stale")
            overlay_bytes, semantic_bytes = _validated_editor_layer_bytes(
                state["grid"], static_overlay, semantic_layer
            )
            durability_indeterminate = False
            try:
                _replace_workspace_generation(
                    state["source_bytes"],
                    overlay_bytes,
                    semantic_bytes,
                    lease,
                    state,
                )
            except _WorkspaceDurabilityIndeterminate:
                durability_indeterminate = True
            updated_state = _workspace_state(course_dir, workspace_id, lease)
            _require_workspace_snapshot_current(lease, updated_state)
            if durability_indeterminate and any(
                    updated_state[key] != expected
                    for key, expected in (
                        ("source_bytes", state["source_bytes"]),
                        ("overlay_bytes", overlay_bytes),
                        ("semantic_bytes", semantic_bytes),
                        ("capture_identity", state["capture_identity"]),
                        (
                            "capture_artifact_identities",
                            state["capture_artifact_identities"],
                        ),
                        ("capture_manifest_bytes", state["capture_manifest_bytes"]),
                        ("base_yaml_bytes", state["base_yaml_bytes"]),
                        ("base_pgm_bytes", state["base_pgm_bytes"]),
                    )):
                raise WorkspaceConflictError(
                    "workspace commit outcome changed after replacement"
                )
        document = _workspace_document(workspace_id, updated_state)
        if durability_indeterminate:
            document["error"] = {
                "code": "workspace_durability_indeterminate",
                "message": (
                    "workspace generation was replaced, but durable storage could "
                    "not be confirmed; verify or save this revision again"
                ),
            }
        return document

    @staticmethod
    def validate_workspace(course_dir, workspace_id):
        workspace_id = _validate_id(workspace_id, "workspace_id")
        course_dir = _course_path(course_dir)
        with _workspace_lock(course_dir, workspace_id, exclusive=False) as lease:
            state = _workspace_state(course_dir, workspace_id, lease)
            _require_full_capture_matches_state(lease, state)
            _require_workspace_snapshot_current(lease, state)
        return {
            "schema_version": 1,
            "workspace_id": workspace_id,
            "capture_id": state["capture_id"],
            "width": state["grid"].width,
            "height": state["grid"].height,
            "resolution": state["grid"].resolution,
            "origin": [state["grid"].origin_x, state["grid"].origin_y, 0.0],
            "operation_count": len(state["overlay"]["operations"]),
            "feature_count": len(state["semantic"]["features"]),
        }

    validate = validate_workspace

    @staticmethod
    def release_workspace(
            course_dir, workspace_id, expected_revision, release_id, activate, now_iso):
        """정확히 하나의 workspace revision을 publish하고 선택적으로 활성화한다."""
        workspace_id = _validate_id(workspace_id, "workspace_id")
        expected_revision = _validate_workspace_revision(expected_revision)
        release_id = _validate_id(release_id, "release_id")
        if type(activate) is not bool:
            raise ReleaseValidationError("activate must be a boolean")
        created_at = _validate_timestamp(now_iso)
        course_dir = _course_path(course_dir, allow_descriptor=True)
        with _workspace_lock(course_dir, workspace_id, exclusive=True) as lease:
            publication_indeterminate = False
            if _release_exists_at_course(lease.course_descriptor, release_id):
                validated = _validated_release_for_retry_at(
                    lease.course_descriptor,
                    release_id,
                    workspace_id,
                    expected_revision,
                )
                try:
                    _fsync_release_parent(lease.course_descriptor)
                except OSError:
                    publication_indeterminate = True
            else:
                state = _workspace_state(course_dir, workspace_id, lease)
                if _workspace_revision(state) != expected_revision:
                    raise WorkspaceConflictError("workspace revision is stale")
                _require_full_capture_matches_state(lease, state)
                _require_workspace_snapshot_current(lease, state)
                try:
                    _create_release_from_state(
                        lease.course_path, workspace_id, release_id, created_at, state
                    )
                    validated = _validated_release_at(
                        lease.course_descriptor, release_id
                    )
                    if not _release_matches_workspace_state(
                            validated, workspace_id, state):
                        raise ReleaseValidationError(
                            "published release does not match workspace state"
                        )
                except _DirectoryPublicationIndeterminate:
                    validated = _validated_release_at(
                        lease.course_descriptor, release_id
                    )
                    if not _release_matches_workspace_state(
                            validated, workspace_id, state):
                        raise ReleaseValidationError(
                            "published release does not match workspace state"
                        )
                    publication_indeterminate = True
                except FileExistsError:
                    validated = _validated_release_for_retry_at(
                        lease.course_descriptor,
                        release_id,
                        workspace_id,
                        expected_revision,
                    )
                    try:
                        _fsync_release_parent(lease.course_descriptor)
                    except OSError:
                        publication_indeterminate = True
            if publication_indeterminate:
                return {
                    "schema_version": 1,
                    "release_id": release_id,
                    "activated": False,
                    "manifest_sha256": validated["manifest_sha256"],
                    "artifact_hashes": validated["manifest"]["hashes"],
                    "error": {
                        "code": "release_publication_indeterminate",
                        "message": (
                            "immutable release exists, but durable publication could "
                            "not be confirmed; verify or retry the same release ID"
                        ),
                    },
                }
            if activate:
                try:
                    _require_course_path_identity(
                        course_dir, lease.course_identity
                    )
                    _activate_validated_release_at(
                        lease.course_descriptor, release_id, validated
                    )
                    _require_course_path_identity(
                        course_dir, lease.course_identity
                    )
                except _ActivationDurabilityIndeterminate:
                    return {
                        "schema_version": 1,
                        "release_id": release_id,
                        "activated": "indeterminate",
                        "manifest_sha256": validated["manifest_sha256"],
                        "artifact_hashes": validated["manifest"]["hashes"],
                        "error": {
                            "code": "activation_indeterminate",
                            "message": (
                                "active pointer matches this release, but durable "
                                "activation could not be confirmed; verify or retry"
                            ),
                        },
                    }
                except (OSError, ReleaseValidationError):
                    return {
                        "schema_version": 1,
                        "release_id": release_id,
                        "activated": False,
                        "manifest_sha256": validated["manifest_sha256"],
                        "artifact_hashes": validated["manifest"]["hashes"],
                        "error": {
                            "code": "activation_failed",
                            "message": (
                                "immutable release was published, but activation failed; "
                                "retry the same release ID"
                            ),
                        },
                    }
            else:
                _require_course_path_identity(
                    course_dir, lease.course_identity
                )
        return {
            "schema_version": 1,
            "release_id": release_id,
            "activated": activate,
            "manifest_sha256": validated["manifest_sha256"],
            "artifact_hashes": validated["manifest"]["hashes"],
        }

    @staticmethod
    def create_release(course_dir, capture_id, workspace_id, release_id, now_iso):
        capture_id = _validate_id(capture_id, "capture_id")
        workspace_id = _validate_id(workspace_id, "workspace_id")
        release_id = _validate_id(release_id, "release_id")
        created_at = _validate_timestamp(now_iso)
        course_dir = _course_path(course_dir)
        with _workspace_lock(course_dir, workspace_id, exclusive=True) as lease:
            state = _workspace_state(course_dir, workspace_id, lease)
            if state["capture_id"] != capture_id:
                raise ReleaseValidationError("workspace capture_id does not match request")
            _require_full_capture_matches_state(lease, state)
            _require_workspace_snapshot_current(lease, state)
            return _create_release_from_state(
                lease.course_path, workspace_id, release_id, created_at, state
            )

    @staticmethod
    def activate_release(course_dir, release_id):
        release_id = _validate_id(release_id, "release_id")
        course_dir = _course_path(course_dir)
        with _open_course_directory(
                course_dir, allow_descriptor=True) as course_descriptor:
            course_identity = _directory_identity(os.fstat(course_descriptor))
            validated_release = _validated_release_at(
                course_descriptor, release_id
            )
            _require_course_path_identity(course_dir, course_identity)
            _activate_validated_release_at(
                course_descriptor, release_id, validated_release
            )
            _require_course_path_identity(course_dir, course_identity)
        return course_dir / "active_release.json"

    activate = activate_release

    @staticmethod
    def plan_runtime_artifacts(
            source_config_path=None, runtime_config_path=None, mapping_database_path=None,
            mapping_runtime_root_path=None, mapping_session_dir_path=None,
            release_manifest_path=None, map_yaml_path=None, source_database_path=None,
            runtime_database_path=None):
        """write 전에 모든 immutable/runtime 관계를 검증한다."""
        source_config = _optional_absolute_path(source_config_path, "RTAB-Map config")
        runtime_config = _optional_absolute_path(runtime_config_path, "runtime RTAB-Map config")
        mapping_database = _optional_absolute_path(mapping_database_path, "mapping database")
        mapping_runtime_root = _optional_absolute_path(
            mapping_runtime_root_path, "mapping runtime root"
        )
        mapping_session_dir = _optional_absolute_path(
            mapping_session_dir_path, "mapping session directory"
        )
        release_values = (
            release_manifest_path, map_yaml_path, source_database_path, runtime_database_path
        )
        if source_config is None and runtime_config is not None:
            raise ReleaseValidationError("runtime RTAB-Map config requires source config")
        if source_config is not None and runtime_config is None:
            raise ReleaseValidationError("source RTAB-Map config requires runtime config")
        if mapping_database is not None and any(release_values):
            raise ReleaseValidationError("mapping database cannot be combined with localization release")
        mapping_layout = (mapping_database, mapping_runtime_root, mapping_session_dir)
        if any(mapping_layout) and not all(mapping_layout):
            raise ReleaseValidationError(
                "mapping database, runtime root, and session directory are required together"
            )
        if any(release_values) and not all(release_values):
            raise ReleaseValidationError(
                "release manifest, map YAML, source DB, and runtime DB are required together"
            )
        initial_mapping_session_binding = (
            _runtime_directory_binding(
                mapping_session_dir, "mapping session directory"
            )
            if mapping_session_dir is not None else None
        )

        protected = []
        if source_config is not None:
            protected.append((source_config, "checked-in RTAB-Map config"))
        if any(release_values):
            release_plan = _validated_localization_release(
                release_manifest_path, map_yaml_path, source_database_path, runtime_database_path
            )
            protected.extend(release_plan["protected"])
            release_manifest = release_plan["release_manifest"]
            map_yaml = release_plan["map_yaml"]
            runtime_database = release_plan["runtime_database"]
            source_database = release_plan["source_database"]
            database_sha256 = release_plan["database_sha256"]
            course_dir = release_plan["course_dir"]
        else:
            release_manifest = None
            map_yaml = None
            runtime_database = None
            source_database = None
            database_sha256 = None
            course_dir = None

        _validate_runtime_targets(
            protected,
            source_config,
            runtime_config,
            mapping_database,
            runtime_database,
            course_dir,
        )
        if mapping_database is not None:
            _validate_mapping_runtime_layout(
                source_config, runtime_config, mapping_database,
                mapping_runtime_root, mapping_session_dir,
            )
            if (_runtime_directory_binding(
                    mapping_session_dir, "mapping session directory")
                    != initial_mapping_session_binding):
                raise ReleaseValidationError(
                    "mapping session changed during preflight"
                )
        source_binding = (
            _runtime_source_binding(source_config)
            if source_config is not None else None
        )
        execution_binding = _RuntimeExecutionBinding(
            source=source_binding,
            runtime_config_parent=_runtime_destination_parent_identity(
                runtime_config, "runtime config"
            ) if runtime_config is not None else None,
            runtime_database_parent=_runtime_destination_parent_identity(
                runtime_database, "runtime database"
            ) if runtime_database is not None else None,
            mapping_database_parent=_runtime_destination_parent_identity(
                mapping_database, "mapping database"
            ) if mapping_database is not None else None,
            mapping_session=initial_mapping_session_binding,
        )
        if mapping_database is None:
            if (runtime_config is not None
                    and execution_binding.runtime_config_parent is None):
                raise ReleaseValidationError(
                    "runtime config parent must exist during preflight"
                )
            if (runtime_database is not None
                    and execution_binding.runtime_database_parent is None):
                raise ReleaseValidationError(
                    "runtime database parent must exist during preflight"
                )
        return _authorize_runtime_plan(RuntimeArtifactPlan(
            source_config=source_config,
            runtime_config=runtime_config,
            source_database=source_database,
            runtime_database=runtime_database,
            database_sha256=database_sha256,
            release_manifest=release_manifest,
            map_yaml=map_yaml,
            mapping_database=mapping_database,
            mapping_runtime_root=mapping_runtime_root,
            mapping_session_dir=mapping_session_dir,
        ), execution_binding)

    @staticmethod
    def execute_runtime_plan(plan):
        """검증된 plan이 허가한 범위 내의 atomic copy만 수행한다."""
        if not isinstance(plan, RuntimeArtifactPlan):
            raise ValueError("plan must be a RuntimeArtifactPlan")
        execution_binding = _claim_runtime_plan(plan)
        for value, description in (
                (plan.source_config, "RTAB-Map config"),
                (plan.runtime_config, "runtime RTAB-Map config"),
                (plan.release_manifest, "release manifest"),
                (plan.map_yaml, "map YAML"),
                (plan.source_database, "source database"),
                (plan.runtime_database, "runtime database"),
                (plan.mapping_database, "mapping database"),
                (plan.mapping_runtime_root, "mapping runtime root"),
                (plan.mapping_session_dir, "mapping session directory")):
            if value is not None:
                _absolute_path(value, description)
        revalidated = ReleaseManager.plan_runtime_artifacts(
            source_config_path=plan.source_config,
            runtime_config_path=plan.runtime_config,
            mapping_database_path=plan.mapping_database,
            mapping_runtime_root_path=plan.mapping_runtime_root,
            mapping_session_dir_path=plan.mapping_session_dir,
            release_manifest_path=plan.release_manifest,
            map_yaml_path=plan.map_yaml,
            source_database_path=plan.source_database,
            runtime_database_path=plan.runtime_database,
        )
        revalidated_binding = _claim_runtime_plan(revalidated)
        if (_runtime_plan_values(revalidated) != _runtime_plan_values(plan)
                or revalidated_binding != execution_binding):
            raise ReleaseValidationError(
                "runtime inputs changed after complete preflight"
            )
        source_binding = execution_binding.source
        with _open_bound_runtime_source(
                plan.source_config, source_binding) as source_config_descriptor:
            if plan.mapping_runtime_root is not None:
                with _open_bound_runtime_directory(
                        execution_binding.mapping_session,
                        "mapping session directory") as descriptor:
                    session_identity = _directory_identity(os.fstat(descriptor))
                    for expected_parent in (
                            execution_binding.runtime_config_parent,
                            execution_binding.mapping_database_parent):
                        if (expected_parent is not None
                                and expected_parent != session_identity):
                            raise ReleaseValidationError(
                                "mapping session changed after preflight"
                            )
                    _copy_verified_runtime_descriptor_at(
                        source_config_descriptor,
                        descriptor,
                        plan.runtime_config.name,
                        expected_hash=source_binding.sha256,
                        expected_identity=source_binding.identity,
                        runtime_description="runtime config",
                    )
                    if (_runtime_destination_parent_identity(
                            plan.runtime_config, "runtime config")
                            != session_identity):
                        raise ReleaseValidationError(
                            "mapping session changed during execution"
                        )
            else:
                with ExitStack() as destinations:
                    config_destination = (
                        destinations.enter_context(_open_bound_runtime_destination(
                            plan.runtime_config,
                            execution_binding.runtime_config_parent,
                            "runtime config",
                        ))
                        if plan.source_config is not None else None
                    )
                    database_destination = (
                        destinations.enter_context(_open_bound_runtime_destination(
                            plan.runtime_database,
                            execution_binding.runtime_database_parent,
                            "runtime database",
                        ))
                        if plan.source_database is not None else None
                    )
                    if config_destination is not None:
                        destination_descriptor, destination_name = config_destination
                        _copy_verified_runtime_descriptor_at(
                            source_config_descriptor,
                            destination_descriptor,
                            destination_name,
                            expected_hash=source_binding.sha256,
                            expected_identity=source_binding.identity,
                            runtime_description="runtime config",
                        )
                    if database_destination is not None:
                        destination_descriptor, destination_name = database_destination
                        _copy_verified_runtime_file_at(
                            plan.source_database,
                            destination_descriptor,
                            destination_name,
                            plan.database_sha256,
                            runtime_description="runtime database",
                        )
        return plan

    @staticmethod
    def prepare_runtime_database(
            release_manifest_path, map_yaml_path, source_database_path, runtime_database_path):
        """DB만 localization runtime으로 복사하기 위한 호환 helper다."""
        plan = ReleaseManager.plan_runtime_artifacts(
            release_manifest_path=release_manifest_path,
            map_yaml_path=map_yaml_path,
            source_database_path=source_database_path,
            runtime_database_path=runtime_database_path,
        )
        ReleaseManager.execute_runtime_plan(plan)
        return plan.runtime_database

    @staticmethod
    def prepare_runtime_config(source_config_path, runtime_config_path):
        """config만 runtime으로 복사하기 위한 호환 helper다."""
        plan = ReleaseManager.plan_runtime_artifacts(
            source_config_path=source_config_path,
            runtime_config_path=runtime_config_path,
        )
        ReleaseManager.execute_runtime_plan(plan)
        return plan.runtime_config

    @staticmethod
    def prepare_localization_runtime_bundle(
            release_manifest_path, map_yaml_path, source_database_path,
            source_config_path, runtime_bundle_path, runtime_root_path=None,
            bundle_role="rtabmap"):
        """localization launch 하나를 위해 불변 release 하나를 원자적으로 staging한다."""
        if bundle_role not in ("map-server", "rtabmap"):
            raise ReleaseValidationError("runtime bundle role is invalid")
        bundle = _absolute_path(runtime_bundle_path, "runtime bundle")
        runtime_root = _absolute_path(
            runtime_root_path if runtime_root_path is not None else bundle.parent,
            "runtime root",
        )
        source_config = _absolute_path(source_config_path, "RTAB-Map config")
        runtime_root_binding = _runtime_directory_binding(
            runtime_root, "runtime root"
        )
        release_plan = _validated_localization_release(
            release_manifest_path, map_yaml_path, source_database_path,
            runtime_database_path=bundle / "map.db",
        )
        source_metadata = {
            "map.db": _regular_path_metadata(
                release_plan["source_database"], "source database"
            ),
            "rtabmap.ini": _regular_path_metadata(source_config, "RTAB-Map config"),
        }
        _validate_localization_runtime_layout(
            runtime_root, bundle, release_plan["course_dir"], source_config
        )
        if (_runtime_directory_binding(runtime_root, "runtime root")
                != runtime_root_binding):
            raise ReleaseValidationError("runtime root changed during preflight")
        config_sha256 = _sha256_file(source_config)
        identity = {
            "schema_version": 1,
            "manifest_sha256": release_plan["manifest_sha256"],
            "artifact_sha256": {
                "map.yaml": release_plan["manifest"]["hashes"]["map.yaml"],
                "map.pgm": release_plan["manifest"]["hashes"]["map.pgm"],
                "map.db": release_plan["database_sha256"],
                "rtabmap.ini": config_sha256,
            },
        }
        with _open_bound_runtime_directory(
                runtime_root_binding, "runtime root") as root_descriptor:
            if _entry_exists_at(root_descriptor, bundle.name):
                _validate_existing_runtime_bundle_at(
                    root_descriptor, bundle.name, identity, bundle_role, source_metadata
                )
                return bundle
            temporary_name = _make_temporary_directory_at(root_descriptor, bundle.name)
            release_dir = release_plan["release_dir"]
            temporary_identity = None
            try:
                temporary_descriptor = _open_directory_at(
                    root_descriptor, temporary_name, "temporary runtime bundle"
                )
                try:
                    temporary_identity = _directory_identity(
                        os.fstat(temporary_descriptor)
                    )
                    _copy_verified_runtime_file_at(
                        release_dir / "map.yaml", temporary_descriptor, "map.yaml",
                        identity["artifact_sha256"]["map.yaml"], "runtime map YAML",
                    )
                    _copy_verified_runtime_file_at(
                        release_dir / "map.pgm", temporary_descriptor, "map.pgm",
                        identity["artifact_sha256"]["map.pgm"], "runtime map image",
                    )
                    _copy_verified_runtime_file_at(
                        release_plan["source_database"], temporary_descriptor, "map.db",
                        identity["artifact_sha256"]["map.db"], "runtime database",
                    )
                    _copy_verified_runtime_file_at(
                        source_config, temporary_descriptor, "rtabmap.ini",
                        identity["artifact_sha256"]["rtabmap.ini"], "runtime config",
                    )
                    _write_bytes_at_fsynced(
                        temporary_descriptor, ".ready.json", _json_bytes(identity)
                    )
                    os.fsync(temporary_descriptor)
                    _require_directory_entry_identity(
                        root_descriptor,
                        temporary_name,
                        temporary_identity,
                        "temporary runtime bundle",
                    )
                finally:
                    os.close(temporary_descriptor)
                try:
                    _rename_directory_noreplace(
                        root_descriptor, temporary_name, root_descriptor, bundle.name
                    )
                    os.fsync(root_descriptor)
                    _require_directory_entry_identity(
                        root_descriptor,
                        bundle.name,
                        temporary_identity,
                        "runtime bundle",
                    )
                except FileExistsError:
                    _validate_existing_runtime_bundle_at(
                        root_descriptor, bundle.name, identity, bundle_role, source_metadata
                    )
                else:
                    _validate_existing_runtime_bundle_at(
                        root_descriptor, bundle.name, identity, bundle_role, source_metadata
                    )
            finally:
                if (temporary_identity is not None
                        and _entry_exists_at(root_descriptor, temporary_name)):
                    _remove_directory_tree_at(
                        root_descriptor,
                        temporary_name,
                        expected_identity=temporary_identity,
                    )
        return bundle


def _validated_localization_release(
        release_manifest_path, map_yaml_path, source_database_path, runtime_database_path):
    manifest_path = _absolute_path(release_manifest_path, "release manifest")
    if manifest_path.name != "manifest.json":
        raise ReleaseValidationError("release manifest must be named manifest.json")
    release = manifest_path.parent
    releases_parent = release.parent
    if releases_parent.name != "releases":
        raise ReleaseValidationError("release manifest is not inside releases")
    release_id = _validate_id(release.name, "release_id")
    course_dir = releases_parent.parent
    expected_manifest = course_dir / "releases" / release_id / "manifest.json"
    _require_exact_path(manifest_path, expected_manifest, "release manifest")

    # strict manifest schema, release artifact 및 raw DB hash를 검증한다.
    validated_release = _validated_release(course_dir, release_id)
    manifest = validated_release["manifest"]
    if _sha256_file(manifest_path) != validated_release["manifest_sha256"]:
        raise ReleaseValidationError("release manifest changed while validating")

    expected_map_yaml = release / "map.yaml"
    expected_database = course_dir / "raw" / manifest["source_capture_id"] / "map.db"
    provided_map_yaml = _absolute_path(map_yaml_path, "map YAML")
    provided_database = _absolute_path(source_database_path, "source database")
    runtime_database = _absolute_path(runtime_database_path, "runtime database")
    _require_exact_path(provided_map_yaml, expected_map_yaml, "map YAML")
    _require_exact_path(provided_database, expected_database, "source database")
    _require_hash(provided_map_yaml, manifest["hashes"]["map.yaml"], "release map YAML")
    protected = [
        (release / name, "released artifact") for name in _RELEASE_FILES
    ] + [
        (course_dir / "raw" / manifest["source_capture_id"] / name, "capture artifact")
        for name in _CAPTURE_FILES
    ]
    return {
        "course_dir": course_dir,
        "release_dir": release,
        "release_manifest": manifest_path,
        "map_yaml": provided_map_yaml,
        "manifest": manifest,
        "manifest_sha256": validated_release["manifest_sha256"],
        "protected": protected,
        "source_database": provided_database,
        "runtime_database": runtime_database,
        "database_sha256": manifest["source_database"]["sha256"],
    }


def _create_release_from_state(course_dir, workspace_id, release_id, created_at, state):
    releases_parent, releases_parent_identity = _storage_parent(
        course_dir, "releases", create=True
    )
    target = releases_parent / release_id
    if target.exists():
        raise FileExistsError("release_id already exists: {}".format(release_id))
    try:
        released_grid = apply_overlay(state["grid"], state["overlay"])
    except OverlayValidationError as error:
        raise ReleaseValidationError("static overlay is invalid") from error

    def build(temporary):
        write_ros_map(released_grid, temporary / "map.pgm", temporary / "map.yaml")
        _write_file_fsynced(temporary / "static_overlay.json", state["overlay_bytes"])
        _write_file_fsynced(temporary / "semantic_layer.json", state["semantic_bytes"])
        hashes = {
            name: _sha256_internal_file(temporary / name)
            for name in sorted(_RELEASE_ARTIFACTS)
        }
        manifest = {
            "schema_version": 1,
            "release_id": release_id,
            "workspace_id": workspace_id,
            "workspace_revision": _workspace_revision(state),
            "source_capture_id": state["capture_id"],
            "source_database": {
                "relative_path": "raw/{}/map.db".format(state["capture_id"]),
                "sha256": state["capture_manifest"]["hashes"]["map.db"],
            },
            "map": {
                "resolution": released_grid.resolution,
                "origin": [released_grid.origin_x, released_grid.origin_y, 0.0],
            },
            "created_at": created_at,
            "hashes": hashes,
        }
        _write_file_fsynced(temporary / "manifest.json", _json_bytes(manifest))

    return _create_directory_atomically(
        releases_parent,
        release_id,
        build,
        expected_parent_identity=releases_parent_identity,
    )


def _workspace_state(course_dir, workspace_id, lease):
    if not isinstance(lease, _WorkspaceLease) or lease.workspace_id != workspace_id:
        raise ReleaseValidationError("workspace lease is invalid")
    workspace_descriptor = _open_directory_at(
        lease.workspaces_descriptor, workspace_id, "workspace"
    )
    try:
        workspace_metadata = os.fstat(workspace_descriptor)
        expected_files = {
            "source_release.json", "static_overlay.json", "semantic_layer.json"
        }
        if set(os.listdir(workspace_descriptor)) != expected_files:
            raise ReleaseValidationError("workspace files do not match the supported schema")
        source_bytes = _read_regular_at(
            workspace_descriptor, "source_release.json", "workspace source"
        )
        overlay_bytes = _read_regular_at(
            workspace_descriptor, "static_overlay.json", "workspace overlay"
        )
        semantic_bytes = _read_regular_at(
            workspace_descriptor, "semantic_layer.json", "workspace semantic layer"
        )
    finally:
        os.close(workspace_descriptor)
    source = _decode_json(source_bytes, "source_release.json")
    if (not isinstance(source, dict)
            or set(source) != {"schema_version", "capture_id"}
            or not _is_schema_version_one(source.get("schema_version"))):
        raise ReleaseValidationError("source_release.json has an invalid schema")
    try:
        capture_id = _validate_id(source["capture_id"], "capture_id")
    except ValueError as error:
        raise ReleaseValidationError("source_release.json has an invalid capture_id") from error
    capture_state = _validated_capture_at(
        lease.course_descriptor, capture_id, validate_database=False
    )
    capture_manifest = capture_state["manifest"]
    grid = capture_state["grid"]
    overlay = _decode_json(overlay_bytes, "static_overlay.json")
    semantic = _decode_json(semantic_bytes, "semantic_layer.json")
    try:
        validate_overlay(grid, overlay)
    except OverlayValidationError as error:
        raise ReleaseValidationError("static_overlay.json is invalid") from error
    _validate_semantic_layer(grid, semantic)
    return {
        "capture_id": capture_id,
        "capture_manifest": capture_manifest,
        "grid": grid,
        "workspace_identity": (workspace_metadata.st_dev, workspace_metadata.st_ino),
        "source_bytes": source_bytes,
        "overlay": overlay,
        "overlay_bytes": overlay_bytes,
        "semantic": semantic,
        "semantic_bytes": semantic_bytes,
        "base_pgm_bytes": capture_state["base_pgm_bytes"],
        "capture_manifest_bytes": capture_state["manifest_bytes"],
        "base_yaml_bytes": capture_state["base_yaml_bytes"],
        "capture_identity": capture_state["capture_identity"],
        "capture_artifact_identities": capture_state["artifact_identities"],
    }


def _workspace_document(workspace_id, state):
    grid = state["grid"]
    return {
        "schema_version": 1,
        "workspace_id": workspace_id,
        "capture_id": state["capture_id"],
        "revision": _workspace_revision(state),
        "map": {
            "width": grid.width,
            "height": grid.height,
            "resolution": grid.resolution,
            "origin": [grid.origin_x, grid.origin_y, 0.0],
            "image_url": "/api/base.pgm",
            "image_sha256": state["capture_manifest"]["hashes"]["base.pgm"],
        },
        "static_overlay": state["overlay"],
        "semantic_layer": state["semantic"],
    }


def _workspace_revision(state):
    digest = hashlib.sha256()
    digest.update(b"workspace-directory\0")
    for value in state["workspace_identity"]:
        digest.update(str(value).encode("ascii"))
        digest.update(b"\0")
    digest.update(b"capture-directory\0")
    for value in state["capture_identity"]:
        digest.update(str(value).encode("ascii"))
        digest.update(b"\0")
    for name in sorted(state["capture_artifact_identities"]):
        digest.update(name.encode("ascii"))
        digest.update(b"\0")
        for value in state["capture_artifact_identities"][name]:
            digest.update(str(value).encode("ascii"))
            digest.update(b"\0")
    for content in (
            state["capture_manifest_bytes"], state["base_yaml_bytes"],
            state["base_pgm_bytes"], state["source_bytes"],
            state["overlay_bytes"], state["semantic_bytes"]):
        digest.update(len(content).to_bytes(8, byteorder="big"))
        digest.update(content)
    return digest.hexdigest()


def _validate_workspace_revision(value):
    if not isinstance(value, str) or not _REVISION_PATTERN.fullmatch(value):
        raise ReleaseValidationError("expected_revision must be a lowercase SHA-256 digest")
    return value


@contextmanager
def _workspace_lock(course_dir, workspace_id, exclusive):
    """교체할 수 없게 고정된 course directory FD에서 transaction을 직렬화한다."""
    with _open_course_directory(course_dir, allow_descriptor=True) as course_descriptor:
        lock_mode = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
        try:
            fcntl.flock(course_descriptor, lock_mode)
            workspaces_descriptor = _open_directory_at(
                course_descriptor, "workspaces", "workspaces storage parent"
            )
            try:
                _validate_legacy_workspace_lock_at(
                    workspaces_descriptor, workspace_id
                )
                yield _WorkspaceLease(
                    course_descriptor, workspaces_descriptor, workspace_id, exclusive
                )
            finally:
                os.close(workspaces_descriptor)
        finally:
            fcntl.flock(course_descriptor, fcntl.LOCK_UN)


def _validate_legacy_workspace_lock_at(parent_descriptor, workspace_id):
    """이전 lock path의 inode를 신뢰하지 않으면서 fail-closed 처리를 유지한다."""
    lock_name = ".workspace-{}.lock".format(workspace_id)
    flags = os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        descriptor = os.open(lock_name, flags, 0o600, dir_fd=parent_descriptor)
    except OSError as error:
        raise ReleaseValidationError("workspace lock is unsafe") from error
    try:
        metadata = _regular_file_metadata(descriptor, "workspace lock")
        if metadata.st_nlink != 1:
            raise ReleaseValidationError("workspace lock must not be hard linked")
    finally:
        os.close(descriptor)


def _validated_editor_layer_bytes(grid, overlay, semantic):
    try:
        overlay_bytes = _json_bytes(overlay)
        semantic_bytes = _json_bytes(semantic)
    except (TypeError, ValueError) as error:
        raise ReleaseValidationError("workspace layers must be strict JSON") from error
    overlay_snapshot = _decode_json(overlay_bytes, "static_overlay.json")
    semantic_snapshot = _decode_json(semantic_bytes, "semantic_layer.json")
    try:
        validate_overlay(grid, overlay_snapshot)
    except OverlayValidationError as error:
        raise ReleaseValidationError("static_overlay.json is invalid") from error
    _validate_semantic_layer(grid, semantic_snapshot)
    return overlay_bytes, semantic_bytes


def _replace_workspace_generation(
        source_bytes, overlay_bytes, semantic_bytes, lease, state):
    if not lease.exclusive:
        raise ReleaseValidationError("workspace replacement requires an exclusive lease")
    parent_descriptor = lease.workspaces_descriptor
    workspace_id = lease.workspace_id
    if not _entry_exists_at(parent_descriptor, workspace_id):
        raise ReleaseValidationError("workspace does not exist")
    temporary_name = _make_temporary_directory_at(parent_descriptor, workspace_id)
    temporary_descriptor = None
    workspace_descriptor = None
    temporary_identity = None
    old_workspace_identity = None
    exchanged = False
    post_commit_error = None
    try:
        temporary_descriptor = _open_directory_at(
            parent_descriptor, temporary_name, "temporary workspace generation"
        )
        temporary_identity = _directory_identity(os.fstat(temporary_descriptor))
        workspace_descriptor = _open_directory_at(
            parent_descriptor, workspace_id, "workspace generation"
        )
        old_workspace_identity = _directory_identity(os.fstat(workspace_descriptor))
        if old_workspace_identity != state["workspace_identity"]:
            raise WorkspaceConflictError("workspace storage identity changed")
        _write_bytes_at_fsynced(
            temporary_descriptor, "source_release.json", source_bytes
        )
        _write_bytes_at_fsynced(
            temporary_descriptor, "static_overlay.json", overlay_bytes
        )
        _write_bytes_at_fsynced(
            temporary_descriptor, "semantic_layer.json", semantic_bytes
        )
        os.fsync(temporary_descriptor)
        _require_directory_entry_identity(
            parent_descriptor,
            temporary_name,
            temporary_identity,
            "temporary workspace generation",
        )
        _require_workspace_lease_entries(lease, state)
        _exchange_directories(
            parent_descriptor, temporary_name, parent_descriptor, workspace_id
        )
        exchanged = True
        try:
            os.fsync(parent_descriptor)
        except OSError as error:
            post_commit_error = error
        try:
            _require_workspace_lease_entries(
                lease,
                state,
                expected_workspace_identity=temporary_identity,
            )
        except (OSError, ReleaseValidationError) as error:
            if post_commit_error is None:
                post_commit_error = error
        try:
            _require_directory_entry_identity(
                parent_descriptor,
                temporary_name,
                old_workspace_identity,
                "old workspace generation",
            )
        except (OSError, ReleaseValidationError) as error:
            if post_commit_error is None:
                post_commit_error = error
    except BaseException:
        if (not exchanged and temporary_identity is not None
                and _entry_exists_at(parent_descriptor, temporary_name)):
            try:
                _remove_directory_tree_at(
                    parent_descriptor,
                    temporary_name,
                    expected_identity=temporary_identity,
                )
            except (OSError, ReleaseValidationError):
                pass
        raise
    finally:
        if workspace_descriptor is not None:
            os.close(workspace_descriptor)
        if temporary_descriptor is not None:
            os.close(temporary_descriptor)
    # exchange가 commit 지점이다. cleanup 실패를 transaction 실패로 잘못 보고하면 안 되며,
    # 이전 generation을 제거하기 전에 identity를 검사한다.
    try:
        _remove_directory_tree_at(
            parent_descriptor,
            temporary_name,
            expected_identity=old_workspace_identity,
        )
    except (OSError, ReleaseValidationError):
        pass
    if post_commit_error is not None:
        raise _WorkspaceDurabilityIndeterminate(
            "workspace durability is indeterminate"
        ) from post_commit_error


def _require_workspace_lease_entries(
        lease, state, expected_workspace_identity=None):
    try:
        parent_metadata = os.stat(
            "workspaces", dir_fd=lease.course_descriptor, follow_symlinks=False
        )
        workspace_metadata = os.stat(
            lease.workspace_id,
            dir_fd=lease.workspaces_descriptor,
            follow_symlinks=False,
        )
    except OSError as error:
        raise WorkspaceConflictError("workspace storage identity changed") from error
    parent_identity = (parent_metadata.st_dev, parent_metadata.st_ino)
    workspace_identity = (workspace_metadata.st_dev, workspace_metadata.st_ino)
    expected = (
        state["workspace_identity"]
        if expected_workspace_identity is None else expected_workspace_identity
    )
    if parent_identity != lease.workspaces_identity or workspace_identity != expected:
        raise WorkspaceConflictError("workspace storage identity changed")


def _require_workspace_snapshot_current(lease, state):
    _require_workspace_lease_entries(lease, state)
    raw_descriptor = _open_directory_at(
        lease.course_descriptor, "raw", "raw storage parent"
    )
    capture_descriptor = None
    try:
        capture_descriptor = _open_directory_at(
            raw_descriptor, state["capture_id"], "capture"
        )
        _require_capture_entries(
            raw_descriptor,
            capture_descriptor,
            state["capture_id"],
            state["capture_identity"],
            state["capture_artifact_identities"],
        )
    finally:
        if capture_descriptor is not None:
            os.close(capture_descriptor)
        os.close(raw_descriptor)


def _exchange_directories(
        source_parent_descriptor, source_name, target_parent_descriptor, target_name):
    if _RENAMEAT2 is None:
        raise ReleaseValidationError("renameat2(RENAME_EXCHANGE) is unavailable")
    result = _RENAMEAT2(
        source_parent_descriptor,
        os.fsencode(source_name),
        target_parent_descriptor,
        os.fsencode(target_name),
        _RENAME_EXCHANGE,
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in (errno.ENOSYS, errno.EINVAL):
        raise ReleaseValidationError(
            "renameat2(RENAME_EXCHANGE) is unsupported by this filesystem"
        )
    raise OSError(error_number, os.strerror(error_number), target_name)


def _validated_capture(course_dir, capture_id):
    with _open_course_directory(course_dir, allow_descriptor=True) as course_descriptor:
        return _validated_capture_at(
            course_descriptor, capture_id, validate_database=True
        )


def _validated_capture_at(course_descriptor, capture_id, validate_database):
    raw_descriptor = _open_directory_at(
        course_descriptor, "raw", "raw storage parent"
    )
    capture_descriptor = None
    try:
        raw_identity = _directory_identity(os.fstat(raw_descriptor))
        capture_descriptor = _open_directory_at(
            raw_descriptor, capture_id, "capture"
        )
        capture_metadata = os.fstat(capture_descriptor)
        if set(os.listdir(capture_descriptor)) != _CAPTURE_FILES:
            raise ReleaseValidationError(
                "capture files do not match the immutable schema"
            )
        artifact_metadata = {
            name: os.stat(
                name, dir_fd=capture_descriptor, follow_symlinks=False
            )
            for name in _CAPTURE_FILES
        }
        if any(not stat.S_ISREG(metadata.st_mode)
               for metadata in artifact_metadata.values()):
            raise ReleaseValidationError(
                "capture files do not match the immutable schema"
            )
        manifest_bytes = _read_regular_at(
            capture_descriptor, "capture_manifest.json", "capture manifest"
        )
        manifest = _decode_json(manifest_bytes, "capture_manifest.json")
        if (not isinstance(manifest, dict)
                or set(manifest) != {
                    "schema_version", "capture_id", "created_at", "hashes"
                }
                or not _is_schema_version_one(manifest.get("schema_version"))
                or manifest.get("capture_id") != capture_id):
            raise ReleaseValidationError("capture manifest has an invalid schema")
        _validate_timestamp_as_release_error(
            manifest.get("created_at"), "capture created_at"
        )
        hashes = manifest.get("hashes")
        expected = {"map.db", "base.pgm", "base.yaml"}
        if not isinstance(hashes, dict) or set(hashes) != expected:
            raise ReleaseValidationError("capture manifest hashes are incomplete")
        for name in expected:
            _validate_sha256(hashes[name], "capture artifact")
        base_pgm_bytes = _read_regular_at(
            capture_descriptor, "base.pgm", "capture artifact"
        )
        base_yaml_bytes = _read_regular_at(
            capture_descriptor, "base.yaml", "capture artifact"
        )
        if hashlib.sha256(base_pgm_bytes).hexdigest() != hashes["base.pgm"]:
            raise ReleaseValidationError("capture artifact hash mismatch")
        if hashlib.sha256(base_yaml_bytes).hexdigest() != hashes["base.yaml"]:
            raise ReleaseValidationError("capture artifact hash mismatch")
        if validate_database:
            _require_stream_hash_and_validate_rtabmap(
                Path("/proc/self/fd/{}".format(capture_descriptor)) / "map.db",
                hashes["map.db"],
                "capture artifact",
            )
        grid = _grid_from_artifact_bytes(
            base_yaml_bytes, base_pgm_bytes, "base.pgm"
        )
        _require_capture_entries(
            raw_descriptor,
            capture_descriptor,
            capture_id,
            (capture_metadata.st_dev, capture_metadata.st_ino),
            {
                name: _file_identity(metadata)
                for name, metadata in artifact_metadata.items()
            },
        )
        _require_directory_entry_identity(
            course_descriptor,
            "raw",
            raw_identity,
            "raw storage parent",
        )
        return {
            "manifest": manifest,
            "manifest_bytes": manifest_bytes,
            "grid": grid,
            "base_pgm_bytes": base_pgm_bytes,
            "base_yaml_bytes": base_yaml_bytes,
            "capture_identity": (
                capture_metadata.st_dev, capture_metadata.st_ino
            ),
            "artifact_identities": {
                name: _file_identity(metadata)
                for name, metadata in artifact_metadata.items()
            },
        }
    finally:
        if capture_descriptor is not None:
            os.close(capture_descriptor)
        os.close(raw_descriptor)


def _require_capture_entries(
        raw_descriptor, capture_descriptor, capture_id,
        capture_identity, artifact_identities):
    try:
        named_capture = os.stat(
            capture_id, dir_fd=raw_descriptor, follow_symlinks=False
        )
        current_artifacts = {
            name: os.stat(
                name, dir_fd=capture_descriptor, follow_symlinks=False
            )
            for name in _CAPTURE_FILES
        }
    except OSError as error:
        raise WorkspaceConflictError("capture identity changed while validating") from error
    if ((named_capture.st_dev, named_capture.st_ino) != capture_identity
            or any(
                _file_identity(current_artifacts[name]) != identity
                for name, identity in artifact_identities.items()
            )):
        raise WorkspaceConflictError("capture identity changed while validating")


def _require_full_capture_matches_state(lease, state):
    capture = _validated_capture_at(
        lease.course_descriptor, state["capture_id"], validate_database=True
    )
    if (capture["manifest_bytes"] != state["capture_manifest_bytes"]
            or capture["base_yaml_bytes"] != state["base_yaml_bytes"]
            or capture["base_pgm_bytes"] != state["base_pgm_bytes"]
            or capture["capture_identity"] != state["capture_identity"]
            or capture["artifact_identities"]
            != state["capture_artifact_identities"]):
        raise WorkspaceConflictError("workspace capture changed while validating")
    return capture


def _validated_release(course_dir, release_id):
    with _open_course_directory(course_dir, allow_descriptor=True) as course_descriptor:
        return _validated_release_at(course_descriptor, release_id)


def _validated_release_at(course_descriptor, release_id):
    """고정된 course-relative FD를 통해 release와 raw capture를 검증한다."""
    releases_descriptor = _open_directory_at(
        course_descriptor, "releases", "releases storage parent"
    )
    release_descriptor = None
    try:
        releases_identity = _directory_identity(os.fstat(releases_descriptor))
        release_descriptor = _open_directory_at(
            releases_descriptor, release_id, "release"
        )
        release_identity = _directory_identity(os.fstat(release_descriptor))
        if set(os.listdir(release_descriptor)) != _RELEASE_FILES:
            raise ReleaseValidationError(
                "release files do not match the immutable schema"
            )
        artifact_metadata = {
            name: os.stat(
                name, dir_fd=release_descriptor, follow_symlinks=False
            )
            for name in _RELEASE_FILES
        }
        if any(not stat.S_ISREG(metadata.st_mode)
               for metadata in artifact_metadata.values()):
            raise ReleaseValidationError(
                "release files do not match the immutable schema"
            )
        artifact_identities = {
            name: _file_identity(metadata)
            for name, metadata in artifact_metadata.items()
        }

        manifest_bytes = _read_regular_at(
            release_descriptor, "manifest.json", "release manifest"
        )
        manifest = _decode_json(manifest_bytes, "manifest.json")
        expected_manifest_keys = {
            "schema_version", "release_id", "workspace_id", "source_capture_id",
            "workspace_revision", "source_database", "map", "created_at", "hashes"
        }
        if (not isinstance(manifest, dict) or set(manifest) != expected_manifest_keys
                or not _is_schema_version_one(manifest.get("schema_version"))
                or manifest.get("release_id") != release_id):
            raise ReleaseValidationError("release manifest has an invalid schema")
        for key in ("workspace_id", "source_capture_id"):
            try:
                _validate_id(manifest.get(key), key)
            except ValueError as error:
                raise ReleaseValidationError(
                    "release manifest has an invalid {}".format(key)
                ) from error
        try:
            _validate_workspace_revision(manifest.get("workspace_revision"))
        except ReleaseValidationError as error:
            raise ReleaseValidationError(
                "release manifest has an invalid workspace_revision"
            ) from error
        _validate_timestamp_as_release_error(
            manifest.get("created_at"), "release created_at"
        )
        hashes = manifest.get("hashes")
        if not isinstance(hashes, dict) or set(hashes) != _RELEASE_ARTIFACTS:
            raise ReleaseValidationError("release manifest hashes are incomplete")
        artifact_bytes = {}
        for name in _RELEASE_ARTIFACTS:
            expected_hash = hashes[name]
            _validate_sha256(expected_hash, "release artifact")
            content = _read_regular_at(
                release_descriptor, name, "release artifact"
            )
            if hashlib.sha256(content).hexdigest() != expected_hash:
                raise ReleaseValidationError(
                    "release artifact SHA-256 mismatch: {}".format(name)
                )
            artifact_bytes[name] = content

        database = manifest.get("source_database")
        capture_id = manifest["source_capture_id"]
        expected_database_path = "raw/{}/map.db".format(capture_id)
        if (not isinstance(database, dict)
                or set(database) != {"relative_path", "sha256"}
                or database.get("relative_path") != expected_database_path):
            raise ReleaseValidationError("source database reference is invalid")
        _validate_sha256(database.get("sha256"), "source database")
        capture = _validated_capture_at(
            course_descriptor, capture_id, validate_database=True
        )
        if capture["manifest"]["hashes"]["map.db"] != database["sha256"]:
            raise ReleaseValidationError("source database SHA-256 mismatch: map.db")

        grid = _grid_from_artifact_bytes(
            artifact_bytes["map.yaml"], artifact_bytes["map.pgm"], "map.pgm"
        )
        map_metadata = manifest.get("map")
        if not _map_metadata_matches_grid(map_metadata, grid):
            raise ReleaseValidationError(
                "release map metadata does not match map.yaml"
            )
        overlay = _decode_json(
            artifact_bytes["static_overlay.json"], "static_overlay.json"
        )
        semantic = _decode_json(
            artifact_bytes["semantic_layer.json"], "semantic_layer.json"
        )
        try:
            validate_overlay(grid, overlay)
        except OverlayValidationError as error:
            raise ReleaseValidationError(
                "released static overlay is invalid"
            ) from error
        _validate_semantic_layer(grid, semantic)
        _require_release_entries(
            course_descriptor,
            releases_descriptor,
            release_descriptor,
            release_id,
            releases_identity,
            release_identity,
            artifact_identities,
        )
        return {
            "manifest": manifest,
            "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
            "artifact_bytes": artifact_bytes,
            "grid": grid,
        }
    finally:
        if release_descriptor is not None:
            os.close(release_descriptor)
        os.close(releases_descriptor)


def _require_release_entries(
        course_descriptor, releases_descriptor, release_descriptor, release_id,
        releases_identity, release_identity, artifact_identities):
    _require_directory_entry_identity(
        course_descriptor,
        "releases",
        releases_identity,
        "releases storage parent",
    )
    _require_directory_entry_identity(
        releases_descriptor,
        release_id,
        release_identity,
        "release",
    )
    if set(os.listdir(release_descriptor)) != _RELEASE_FILES:
        raise ReleaseValidationError("release files changed while validating")
    try:
        current_artifacts = {
            name: os.stat(
                name, dir_fd=release_descriptor, follow_symlinks=False
            )
            for name in _RELEASE_FILES
        }
    except OSError as error:
        raise ReleaseValidationError(
            "release files changed while validating"
        ) from error
    if any(
            not stat.S_ISREG(current_artifacts[name].st_mode)
            or _file_identity(current_artifacts[name]) != identity
            for name, identity in artifact_identities.items()):
        raise ReleaseValidationError("release files changed while validating")


def _release_matches_workspace_state(validated, workspace_id, state):
    manifest = validated["manifest"]
    try:
        expected_grid = apply_overlay(state["grid"], state["overlay"])
    except OverlayValidationError:
        return False
    return (
        manifest.get("workspace_id") == workspace_id
        and manifest.get("workspace_revision") == _workspace_revision(state)
        and manifest.get("source_capture_id") == state["capture_id"]
        and manifest.get("source_database", {}).get("sha256")
        == state["capture_manifest"]["hashes"]["map.db"]
        and validated["artifact_bytes"]["static_overlay.json"]
        == state["overlay_bytes"]
        and validated["artifact_bytes"]["semantic_layer.json"]
        == state["semantic_bytes"]
        and validated["grid"] == expected_grid
    )


def _release_exists_at_course(course_descriptor, release_id):
    try:
        releases_descriptor = os.open(
            "releases", _directory_open_flags(), dir_fd=course_descriptor
        )
    except FileNotFoundError:
        return False
    except OSError as error:
        raise ReleaseValidationError("releases storage parent is unsafe") from error
    try:
        return _entry_exists_at(releases_descriptor, release_id)
    finally:
        os.close(releases_descriptor)


def _fsync_release_parent(course_descriptor):
    releases_descriptor = _open_directory_at(
        course_descriptor, "releases", "releases storage parent"
    )
    try:
        os.fsync(releases_descriptor)
    finally:
        os.close(releases_descriptor)


def _validated_release_for_retry_at(
        course_descriptor, release_id, workspace_id, expected_revision):
    try:
        validated = _validated_release_at(course_descriptor, release_id)
    except ReleaseValidationError as error:
        raise FileExistsError(
            "release_id already exists with invalid artifacts: {}".format(release_id)
        ) from error
    manifest = validated["manifest"]
    if (manifest.get("workspace_id") != workspace_id
            or manifest.get("workspace_revision") != expected_revision):
        raise FileExistsError(
            "release_id already exists for another workspace request: {}".format(
                release_id
            )
        )
    return validated


def _activate_validated_release_at(
        course_descriptor, release_id, validated_release):
    active = {
        "schema_version": 1,
        "release_id": release_id,
        "manifest_sha256": validated_release["manifest_sha256"],
    }
    active_bytes = _json_bytes(active)
    try:
        _atomic_write_bytes_at(
            course_descriptor, "active_release.json", active_bytes
        )
    except (OSError, ReleaseValidationError) as error:
        try:
            selected = _read_regular_at(
                course_descriptor,
                "active_release.json",
                "active release pointer",
            ) == active_bytes
        except (OSError, ReleaseValidationError):
            selected = False
        if selected:
            raise _ActivationDurabilityIndeterminate(
                "active release durability is indeterminate"
            ) from error
        raise


def _validate_semantic_layer(grid, document):
    if (not isinstance(document, dict) or set(document) != {"schema_version", "features"}
            or not _is_schema_version_one(document.get("schema_version"))
            or not isinstance(document.get("features"), list)):
        raise ReleaseValidationError("semantic_layer.json has an invalid schema")
    for feature in document["features"]:
        if not isinstance(feature, dict) or set(feature) != {"label", "geometry"}:
            raise ReleaseValidationError("semantic feature fields are invalid")
        label = feature["label"]
        if not isinstance(label, str):
            raise ReleaseValidationError("semantic feature label is unsupported")
        expected_geometry = _SEMANTIC_GEOMETRY_BY_LABEL.get(label)
        if expected_geometry is None:
            raise ReleaseValidationError("semantic feature label is unsupported")
        geometry = feature["geometry"]
        if (not isinstance(geometry, dict)
                or set(geometry) != {"type", "coordinates"}
                or geometry.get("type") != expected_geometry):
            raise ReleaseValidationError("semantic feature geometry is invalid")
        if expected_geometry == "point":
            _validate_semantic_point(grid, geometry["coordinates"])
        else:
            coordinates = geometry["coordinates"]
            if not isinstance(coordinates, list) or len(coordinates) < 2:
                raise ReleaseValidationError("semantic polyline needs at least two points")
            points = [_validate_semantic_point(grid, point) for point in coordinates]
            if len(set(points)) != len(points):
                raise ReleaseValidationError("semantic polyline points must be distinct")


def _validate_semantic_point(grid, coordinates):
    if not isinstance(coordinates, list) or len(coordinates) != 2:
        raise ReleaseValidationError("semantic point must be [x, y]")
    values = []
    for value in coordinates:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ReleaseValidationError("semantic coordinates must be finite numbers")
        try:
            normalized = float(value)
        except OverflowError as error:
            raise ReleaseValidationError(
                "semantic coordinates must be finite numbers"
            ) from error
        if not math.isfinite(normalized):
            raise ReleaseValidationError("semantic coordinates must be finite numbers")
        values.append(normalized)
    x, y = values
    if x < grid.origin_x or x > grid.max_x or y < grid.origin_y or y > grid.max_y:
        raise ReleaseValidationError("semantic coordinate is outside the map extent")
    return x, y


def _is_schema_version_one(value):
    return type(value) is int and value == 1


def _map_metadata_matches_grid(document, grid):
    if not isinstance(document, dict) or set(document) != {"resolution", "origin"}:
        return False
    resolution = _finite_metadata_number(document["resolution"])
    origin = document["origin"]
    if resolution is None or not isinstance(origin, list) or len(origin) != 3:
        return False
    normalized_origin = [_finite_metadata_number(value) for value in origin]
    if any(value is None for value in normalized_origin) or normalized_origin[2] != 0.0:
        return False
    return (resolution == grid.resolution
            and normalized_origin == [grid.origin_x, grid.origin_y, 0.0])


def _finite_metadata_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        normalized = float(value)
    except OverflowError:
        return None
    return normalized if math.isfinite(normalized) else None


def _absolute_path(path, description):
    try:
        result = Path(os.path.abspath(os.path.expanduser(os.fspath(path))))
    except (TypeError, ValueError) as error:
        raise ReleaseValidationError("{} path is invalid".format(description)) from error
    if "\x00" in str(result):
        raise ReleaseValidationError("{} path is invalid".format(description))
    if _DESCRIPTOR_ROOT_PATTERN.match(str(result)):
        raise ReleaseValidationError(
            "{} must not use a descriptor-root path".format(description)
        )
    return result


def _optional_absolute_path(path, description):
    return None if path is None else _absolute_path(path, description)


def _validate_runtime_targets(
        protected, source_config, runtime_config, mapping_database, runtime_database, course_dir):
    targets = [
        (runtime_config, "runtime config"),
        (mapping_database, "mapping database"),
        (runtime_database, "runtime database"),
    ]
    target_metadata = {
        path: _runtime_target_metadata(path, description)
        for path, description in targets if path is not None
    }
    protected_metadata = [
        (path, description, _regular_path_metadata(path, description))
        for path, description in protected
    ]
    for path, description in targets:
        if path is None:
            continue
        if any(parent.name in ("raw", "releases") for parent in path.parents):
            raise ReleaseValidationError(
                "{} must be outside reserved immutable storage".format(description)
            )
        if course_dir is not None and (
                _is_within(path, course_dir / "raw")
                or _is_within(path, course_dir / "releases")):
            raise ReleaseValidationError("{} must be outside immutable course storage".format(description))
        if (description == "runtime config" and source_config is not None
                and _is_within(path, source_config.parent)):
            raise ReleaseValidationError("runtime config must be outside source config directory")
        for protected_path, protected_description, protected_stat in protected_metadata:
            if path == protected_path or _same_inode(target_metadata[path], protected_stat):
                raise ReleaseValidationError(
                    "{} aliases protected {}".format(description, protected_description)
                )
    present = [(path, description) for path, description in targets if path is not None]
    for index, (first_path, first_description) in enumerate(present):
        for second_path, second_description in present[index + 1:]:
            if (first_path == second_path
                    or _same_inode(target_metadata[first_path], target_metadata[second_path])):
                raise ReleaseValidationError(
                    "{} must differ from {}".format(first_description, second_description)
                )


def _validate_mapping_runtime_layout(
        source_config, runtime_config, mapping_database, runtime_root, session_dir):
    if source_config is None or runtime_config is None:
        raise ReleaseValidationError("mapping runtime requires source and runtime config")
    if session_dir.parent != runtime_root:
        raise ReleaseValidationError("mapping session must be a direct child of runtime root")
    if runtime_config.parent != session_dir or mapping_database.parent != session_dir:
        raise ReleaseValidationError("mapping DB and config must stay inside one session directory")
    package_root = (
        source_config.parent.parent
        if source_config.parent.name == "config" else source_config.parent
    )
    if (_is_within(runtime_root, package_root)
            or _is_within(source_config, runtime_root)
            or any(part in ("raw", "releases") for part in runtime_root.parts)):
        raise ReleaseValidationError("mapping runtime root overlaps protected source storage")
    _preflight_directory_components(runtime_root, "mapping runtime root")
    _preflight_directory_components(session_dir, "mapping session directory")
    metadata = _runtime_target_metadata(mapping_database, "mapping database")
    if metadata is not None and metadata.st_nlink != 1:
        raise ReleaseValidationError("mapping database must not have inode aliases")


def _preflight_directory_components(path, description):
    """아무것도 생성하지 않고 기존 symlink와 directory가 아닌 항목을 모두 거절한다."""
    absolute = _absolute_path(path, description)
    descriptor = _open_root_directory(description)
    try:
        for component in absolute.parts[1:]:
            try:
                next_descriptor = os.open(component, _directory_open_flags(), dir_fd=descriptor)
            except FileNotFoundError:
                return
            except OSError as error:
                raise ReleaseValidationError(
                    "{} has an unsafe existing path component".format(description)
                ) from error
            os.close(descriptor)
            descriptor = next_descriptor
    finally:
        os.close(descriptor)


def _secure_ensure_directory(path, direct_parent=None):
    """검증된 directory descriptor를 기준으로 private directory tree를 생성한다."""
    absolute = _absolute_path(path, "private runtime directory")
    if direct_parent is not None:
        parent = _absolute_path(direct_parent, "private runtime parent")
        if absolute.parent != parent:
            raise ReleaseValidationError("private runtime directory has wrong parent")
    descriptor = _open_root_directory("private runtime directory")
    try:
        for component in absolute.parts[1:]:
            try:
                next_descriptor = os.open(component, _directory_open_flags(), dir_fd=descriptor)
            except FileNotFoundError:
                try:
                    os.mkdir(component, 0o700, dir_fd=descriptor)
                except FileExistsError:
                    pass
                except OSError as error:
                    raise ReleaseValidationError(
                        "unable to create private runtime directory"
                    ) from error
                else:
                    try:
                        os.fsync(descriptor)
                    except OSError as error:
                        raise ReleaseValidationError(
                            "unable to sync private runtime directory"
                        ) from error
                next_descriptor = _open_directory_at(
                    descriptor, component, "private runtime directory"
                )
            except OSError as error:
                raise ReleaseValidationError(
                    "private runtime path contains a symlink or non-directory"
                ) from error
            os.close(descriptor)
            descriptor = next_descriptor
    finally:
        os.close(descriptor)


def _validate_localization_runtime_layout(runtime_root, bundle, course_dir, source_config):
    if bundle.parent != runtime_root:
        raise ReleaseValidationError("runtime bundle must be a direct child of runtime root")
    source_package = (
        source_config.parent.parent
        if source_config.parent.name == "config" else source_config.parent
    )
    if (_is_within(runtime_root, course_dir / "raw")
            or _is_within(runtime_root, course_dir / "releases")
            or _is_within(runtime_root, source_package)
            or _is_within(course_dir, runtime_root)
            or _is_within(source_config, runtime_root)
            or any(part in ("raw", "releases") for part in runtime_root.parts)):
        raise ReleaseValidationError("runtime root overlaps immutable source storage")
    _preflight_directory_components(runtime_root, "runtime root")
    _preflight_directory_components(bundle, "runtime bundle")


def _validate_existing_runtime_bundle_at(
        root_descriptor, bundle_name, expected_identity, bundle_role,
        expected_source_metadata=None):
    bundle_descriptor = _open_directory_at(
        root_descriptor, bundle_name, "runtime bundle"
    )
    try:
        expected_files = {"map.yaml", "map.pgm", "map.db", "rtabmap.ini", ".ready.json"}
        if set(os.listdir(bundle_descriptor)) != expected_files:
            raise ReleaseValidationError("runtime bundle is incomplete or unsafe")
        identity = _decode_json(
            _read_regular_at(bundle_descriptor, ".ready.json", "runtime bundle manifest"),
            ".ready.json",
        )
        if identity != expected_identity:
            raise ReleaseValidationError("runtime bundle belongs to a different immutable release")
        names = {"map.yaml", "map.pgm"}
        if bundle_role == "rtabmap":
            names.update(("map.db", "rtabmap.ini"))
        for name in ("map.yaml", "map.pgm", "map.db", "rtabmap.ini"):
            with _open_regular_at(
                    bundle_descriptor, name, "runtime bundle artifact") as descriptor:
                metadata = _regular_file_metadata(descriptor, "runtime bundle artifact")
                if bundle_role == "rtabmap" and name in ("map.db", "rtabmap.ini"):
                    source = (expected_source_metadata or {}).get(name)
                    if metadata.st_nlink != 1 or _same_inode(metadata, source):
                        raise ReleaseValidationError(
                            "writable runtime bundle artifact must have one private inode: {}".format(
                                name
                            )
                        )
                actual = (
                    _sha256_descriptor(descriptor, "runtime bundle artifact")
                    if name in names else None
                )
            if name in names and actual != expected_identity["artifact_sha256"][name]:
                raise ReleaseValidationError(
                    "runtime bundle artifact hash mismatch: {}".format(name)
                )
    finally:
        os.close(bundle_descriptor)


def _validate_existing_runtime_bundle(bundle, expected_identity, bundle_role="rtabmap"):
    """descriptor-bound bundle validator를 사용하는 호환 wrapper다."""
    with _open_path_parent(bundle, "runtime bundle") as (root_descriptor, bundle_name):
        _validate_existing_runtime_bundle_at(
            root_descriptor, bundle_name, expected_identity, bundle_role
        )


def _directory_identity(metadata):
    return metadata.st_dev, metadata.st_ino


def _require_course_path_identity(course_dir, expected_identity):
    """이름이 지정된 course path가 여전히 고정된 transaction inode를 가리키는지 확인한다."""
    with _open_course_directory(
            course_dir, allow_descriptor=True) as current_descriptor:
        if _directory_identity(os.fstat(current_descriptor)) != expected_identity:
            raise ReleaseValidationError(
                "course directory changed during transaction"
            )


def _require_regular_entry_identity(
        parent_descriptor, name, expected_identity, description):
    try:
        metadata = os.stat(
            name, dir_fd=parent_descriptor, follow_symlinks=False
        )
    except OSError as error:
        raise ReleaseValidationError(
            "{} entry changed".format(description)
        ) from error
    if (not stat.S_ISREG(metadata.st_mode)
            or _directory_identity(metadata) != expected_identity):
        raise ReleaseValidationError("{} entry changed".format(description))


def _require_runtime_publication_content_at(
        parent_descriptor, name, temporary_descriptor, expected_sha256, description):
    if (_sha256_descriptor(temporary_descriptor, "{} temporary".format(description))
            != expected_sha256):
        raise ReleaseValidationError(
            "{} temporary content changed".format(description)
        )
    for attempt in range(_RUNTIME_PUBLICATION_RETRIES):
        with _open_regular_at(parent_descriptor, name, description) as descriptor:
            if _sha256_descriptor(descriptor, description) != expected_sha256:
                raise ReleaseValidationError(
                    "{} changed during publication".format(description)
                )
            metadata = _regular_file_metadata(descriptor, description)
        if metadata.st_nlink > 1:
            raise ReleaseValidationError(
                "{} must have one private inode".format(description)
            )
        try:
            named_metadata = os.stat(
                name, dir_fd=parent_descriptor, follow_symlinks=False
            )
        except OSError as error:
            raise ReleaseValidationError(
                "{} entry changed".format(description)
            ) from error
        if (stat.S_ISREG(named_metadata.st_mode)
                and named_metadata.st_nlink == 1
                and _directory_identity(named_metadata)
                == _directory_identity(metadata)):
            return
        if (not stat.S_ISREG(named_metadata.st_mode)
                or named_metadata.st_nlink > 1):
            raise ReleaseValidationError(
                "{} must have one private inode".format(description)
            )
        if attempt + 1 < _RUNTIME_PUBLICATION_RETRIES:
            time.sleep(_RUNTIME_PUBLICATION_RETRY_SECONDS)
            continue
        raise ReleaseValidationError(
            "{} did not stabilize after publication".format(description)
        )


def _require_directory_entry_identity(
        parent_descriptor, name, expected_identity, description):
    try:
        metadata = os.stat(
            name, dir_fd=parent_descriptor, follow_symlinks=False
        )
    except OSError as error:
        raise ReleaseValidationError(
            "{} entry changed".format(description)
        ) from error
    if (not stat.S_ISDIR(metadata.st_mode)
            or _directory_identity(metadata) != expected_identity):
        raise ReleaseValidationError("{} entry changed".format(description))


def _remove_open_directory_contents(descriptor):
    """이미 고정된 private directory inode를 통해서만 항목을 제거한다."""
    for entry in os.listdir(descriptor):
        metadata = os.stat(entry, dir_fd=descriptor, follow_symlinks=False)
        if stat.S_ISDIR(metadata.st_mode):
            child_descriptor = _open_directory_at(
                descriptor, entry, "temporary directory child"
            )
            try:
                child_identity = _directory_identity(os.fstat(child_descriptor))
                _remove_open_directory_contents(child_descriptor)
                _require_directory_entry_identity(
                    descriptor,
                    entry,
                    child_identity,
                    "temporary directory child",
                )
                os.rmdir(entry, dir_fd=descriptor)
            finally:
                os.close(child_descriptor)
        else:
            os.unlink(entry, dir_fd=descriptor)
    os.fsync(descriptor)


def _remove_directory_tree_at(parent_descriptor, name, expected_identity=None):
    if not name.startswith(".") or name in (".", "..") or "/" in name:
        raise ReleaseValidationError("refusing to remove a non-temporary runtime directory")
    descriptor = _open_directory_at(
        parent_descriptor, name, "temporary runtime directory"
    )
    try:
        actual_identity = _directory_identity(os.fstat(descriptor))
        if (expected_identity is not None
                and actual_identity != expected_identity):
            raise ReleaseValidationError("temporary directory identity changed")
        _remove_open_directory_contents(descriptor)
        _require_directory_entry_identity(
            parent_descriptor,
            name,
            actual_identity,
            "temporary runtime directory",
        )
        os.rmdir(name, dir_fd=parent_descriptor)
    finally:
        os.close(descriptor)
    os.fsync(parent_descriptor)


def _runtime_target_metadata(path, description):
    absolute = _absolute_path(path, description)
    descriptor = _open_root_directory(description)
    try:
        for component in absolute.parts[1:-1]:
            try:
                next_descriptor = os.open(
                    component, _directory_open_flags(), dir_fd=descriptor
                )
            except FileNotFoundError:
                return None
            except OSError as error:
                raise ReleaseValidationError(
                    "{} has an unsafe path component".format(description)
                ) from error
            os.close(descriptor)
            descriptor = next_descriptor
        try:
            metadata = os.stat(
                absolute.parts[-1], dir_fd=descriptor, follow_symlinks=False
            )
        except FileNotFoundError:
            return None
    finally:
        os.close(descriptor)
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise ReleaseValidationError("{} must be a regular non-symlink file".format(description))
    return metadata


def _regular_path_metadata(path, description):
    with _open_regular_path(path, description) as descriptor:
        return _regular_file_metadata(descriptor, description)


def _runtime_source_binding(path):
    description = "checked-in RTAB-Map config"
    with _open_regular_path(path, description) as descriptor:
        metadata = _regular_file_metadata(descriptor, description)
        digest = _sha256_descriptor(descriptor, description)
        stable_metadata = _regular_file_metadata(descriptor, description)
        if _file_identity(metadata) != _file_identity(stable_metadata):
            raise ReleaseValidationError(
                "{} changed while being bound".format(description)
            )
        return _RuntimeSourceBinding(_file_identity(stable_metadata), digest)


def _runtime_destination_parent_identity(path, description):
    """link를 따라가지 않고 기존 destination parent identity 하나를 반환한다."""
    absolute = _absolute_path(path, description)
    descriptor = _open_root_directory(description)
    try:
        for component in absolute.parts[1:-1]:
            try:
                next_descriptor = os.open(
                    component, _directory_open_flags(), dir_fd=descriptor
                )
            except FileNotFoundError:
                return None
            except OSError as error:
                raise ReleaseValidationError(
                    "{} parent is unsafe".format(description)
                ) from error
            os.close(descriptor)
            descriptor = next_descriptor
        return _directory_identity(os.fstat(descriptor))
    finally:
        os.close(descriptor)


def _runtime_directory_binding(path, description):
    """directory target을 가장 가까운 기존 nofollow ancestor에 고정한다."""
    absolute = _absolute_path(path, description)
    components = absolute.parts[1:]
    descriptor = _open_root_directory(description)
    anchor = Path("/")
    try:
        for index, component in enumerate(components):
            try:
                next_descriptor = os.open(
                    component, _directory_open_flags(), dir_fd=descriptor
                )
            except FileNotFoundError:
                return _RuntimeDirectoryBinding(
                    target=absolute,
                    anchor=anchor,
                    anchor_identity=_directory_identity(os.fstat(descriptor)),
                    missing_parts=tuple(components[index:]),
                )
            except OSError as error:
                raise ReleaseValidationError(
                    "{} has an unsafe existing path component".format(description)
                ) from error
            os.close(descriptor)
            descriptor = next_descriptor
            anchor /= component
        return _RuntimeDirectoryBinding(
            target=absolute,
            anchor=anchor,
            anchor_identity=_directory_identity(os.fstat(descriptor)),
            missing_parts=(),
        )
    finally:
        os.close(descriptor)


def _runtime_directory_marker(path):
    digest = hashlib.sha256()
    digest.update(b"stier-slam-private-runtime-directory-v1\0")
    digest.update(os.fsencode(str(path)))
    return digest.hexdigest().encode("ascii")


def _mark_created_runtime_directory(descriptor, path, description):
    try:
        os.setxattr(
            descriptor,
            _RUNTIME_DIRECTORY_XATTR,
            _runtime_directory_marker(path),
            os.XATTR_CREATE,
        )
        os.fsync(descriptor)
    except (AttributeError, OSError) as error:
        raise ReleaseValidationError(
            "{} provenance could not be recorded".format(description)
        ) from error


def _require_created_runtime_directory_marker(descriptor, path, description):
    expected = _runtime_directory_marker(path)
    for attempt in range(_RUNTIME_DIRECTORY_MARKER_RETRIES):
        try:
            actual = os.getxattr(descriptor, _RUNTIME_DIRECTORY_XATTR)
        except OSError as error:
            if (error.errno == getattr(errno, "ENODATA", None)
                    and attempt + 1 < _RUNTIME_DIRECTORY_MARKER_RETRIES):
                time.sleep(_RUNTIME_DIRECTORY_MARKER_DELAY_SECONDS)
                continue
            raise ReleaseValidationError(
                "{} appeared without trusted provenance".format(description)
            ) from error
        if actual != expected:
            raise ReleaseValidationError(
                "{} provenance belongs to another path".format(description)
            )
        return
    raise ReleaseValidationError(
        "{} appeared without trusted provenance".format(description)
    )


def _runtime_directory_path_identity(path, description):
    with _open_directory_path(path, description) as descriptor:
        return _directory_identity(os.fstat(descriptor))


@contextmanager
def _open_bound_runtime_directory(binding, description):
    """preflight에서 승인된 ancestor 아래에만 private target을 생성한다."""
    if not isinstance(binding, _RuntimeDirectoryBinding):
        raise ReleaseValidationError(
            "{} binding is missing".format(description)
        )
    with _open_directory_path(binding.anchor, description) as anchor_descriptor:
        if (_directory_identity(os.fstat(anchor_descriptor))
                != binding.anchor_identity):
            raise ReleaseValidationError(
                "{} ancestor changed after preflight".format(description)
            )
        descriptor = os.dup(anchor_descriptor)
        current_path = binding.anchor
        try:
            for component in binding.missing_parts:
                component_path = current_path / component
                created = False
                try:
                    os.mkdir(component, 0o700, dir_fd=descriptor)
                    created = True
                except FileExistsError:
                    pass
                except OSError as error:
                    raise ReleaseValidationError(
                        "unable to create {}".format(description)
                    ) from error
                next_descriptor = _open_directory_at(
                    descriptor, component, description
                )
                try:
                    if created:
                        _mark_created_runtime_directory(
                            next_descriptor, component_path, description
                        )
                        os.fsync(descriptor)
                    else:
                        _require_created_runtime_directory_marker(
                            next_descriptor, component_path, description
                        )
                except BaseException:
                    os.close(next_descriptor)
                    raise
                os.close(descriptor)
                descriptor = next_descriptor
                current_path = component_path
            if current_path != binding.target:
                raise ReleaseValidationError(
                    "{} binding does not reach its target".format(description)
                )
            target_identity = _directory_identity(os.fstat(descriptor))
            if (_runtime_directory_path_identity(binding.target, description)
                    != target_identity):
                raise ReleaseValidationError(
                    "{} changed during creation".format(description)
                )
            try:
                yield descriptor
            except BaseException:
                raise
            else:
                if (_runtime_directory_path_identity(binding.target, description)
                        != target_identity):
                    raise ReleaseValidationError(
                        "{} changed during execution".format(description)
                    )
        finally:
            os.close(descriptor)


@contextmanager
def _open_bound_runtime_destination(path, expected_parent_identity, description):
    if expected_parent_identity is None:
        raise ReleaseValidationError(
            "{} parent was absent during preflight".format(description)
        )
    with _open_path_parent(path, description) as (parent_descriptor, name):
        if _directory_identity(os.fstat(parent_descriptor)) != expected_parent_identity:
            raise ReleaseValidationError(
                "{} parent changed after preflight".format(description)
            )
        try:
            yield parent_descriptor, name
        except BaseException:
            raise
        else:
            current_identity = _runtime_destination_parent_identity(path, description)
            if current_identity != expected_parent_identity:
                raise ReleaseValidationError(
                    "{} parent changed during execution".format(description)
                )


@contextmanager
def _open_bound_runtime_source(path, expected_binding):
    if path is None:
        if expected_binding is not None:
            raise ReleaseValidationError("runtime source binding is inconsistent")
        yield None
        return
    if not isinstance(expected_binding, _RuntimeSourceBinding):
        raise ReleaseValidationError("runtime source binding is missing")
    description = "checked-in RTAB-Map config"
    with _open_regular_path(path, description) as descriptor:
        metadata = _regular_file_metadata(descriptor, description)
        if _file_identity(metadata) != expected_binding.identity:
            raise ReleaseValidationError(
                "checked-in RTAB-Map config changed after preflight"
            )
        digest = _sha256_descriptor(descriptor, description)
        if digest != expected_binding.sha256:
            raise ReleaseValidationError(
                "checked-in RTAB-Map config changed after preflight"
            )
        yield descriptor


def _same_inode(first, second):
    return first is not None and second is not None and (
        first.st_dev, first.st_ino) == (second.st_dev, second.st_ino)


def _is_within(path, parent):
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _require_exact_path(provided, expected, description):
    if provided != expected:
        raise ReleaseValidationError("{} does not match the immutable release".format(description))
    # descriptor-relative nofollow 방식으로 열어 다른 process가 사용하기 전에
    # 마지막 또는 중간 symlink를 거절한다.
    with _open_regular_path(provided, description):
        pass


def _copy_verified_runtime_file(
        source_path, destination_path, expected_hash=None, runtime_description="runtime file"):
    """검증된 불변 source를 runtime path로 원자적으로 stream copy한다."""
    with _open_path_parent(destination_path, runtime_description) as (
            destination_parent_descriptor, destination_name):
        _copy_verified_runtime_file_at(
            source_path, destination_parent_descriptor, destination_name,
            expected_hash, runtime_description,
        )


def _copy_verified_runtime_file_at(
        source_path, destination_parent_descriptor, destination_name,
        expected_hash=None, runtime_description="runtime file"):
    description = "immutable runtime source"
    if expected_hash is not None:
        _validate_sha256(expected_hash, description)
    with _open_regular_path(source_path, description) as source_descriptor:
        _copy_verified_runtime_descriptor_at(
            source_descriptor,
            destination_parent_descriptor,
            destination_name,
            expected_hash=expected_hash,
            runtime_description=runtime_description,
        )


def _copy_verified_runtime_descriptor_at(
        source_descriptor, destination_parent_descriptor, destination_name,
        expected_hash=None, expected_identity=None,
        runtime_description="runtime file"):
    """이미 열려 있고 identity가 고정된 source를 atomic runtime file로 복사한다."""
    description = "immutable runtime source"
    if expected_hash is not None:
        _validate_sha256(expected_hash, description)
    before = _regular_file_metadata(source_descriptor, description)
    if (expected_identity is not None
            and _file_identity(before) != expected_identity):
        raise ReleaseValidationError("immutable runtime source identity mismatch")
    _reject_unsafe_runtime_target(
        destination_parent_descriptor, destination_name, before, runtime_description
    )
    temporary_name, temporary_descriptor = _open_runtime_temporary(
        destination_parent_descriptor, destination_name
    )
    temporary_identity = _directory_identity(os.fstat(temporary_descriptor))
    try:
        os.lseek(source_descriptor, 0, os.SEEK_SET)
        digest = hashlib.sha256()
        byte_count = 0
        while True:
            block = os.read(source_descriptor, _STREAM_CHUNK_SIZE)
            if not block:
                break
            _write_all(temporary_descriptor, block)
            digest.update(block)
            byte_count += len(block)
        os.fsync(temporary_descriptor)
        after = _regular_file_metadata(source_descriptor, description)
        _require_stable_file(before, after, byte_count, description)
        if (expected_identity is not None
                and _file_identity(after) != expected_identity):
            raise ReleaseValidationError("immutable runtime source identity mismatch")
        copied_sha256 = digest.hexdigest()
        if expected_hash is not None and copied_sha256 != expected_hash:
            raise ReleaseValidationError("immutable runtime source SHA-256 mismatch")
        _require_regular_entry_identity(
            destination_parent_descriptor,
            temporary_name,
            temporary_identity,
            "runtime temporary file",
        )
        os.replace(
            temporary_name,
            destination_name,
            src_dir_fd=destination_parent_descriptor,
            dst_dir_fd=destination_parent_descriptor,
        )
        _require_runtime_publication_content_at(
            destination_parent_descriptor,
            destination_name,
            temporary_descriptor,
            copied_sha256,
            runtime_description,
        )
        os.fsync(destination_parent_descriptor)
        _require_runtime_publication_content_at(
            destination_parent_descriptor,
            destination_name,
            temporary_descriptor,
            copied_sha256,
            runtime_description,
        )
    except BaseException:
        try:
            _require_regular_entry_identity(
                destination_parent_descriptor,
                temporary_name,
                temporary_identity,
                "runtime temporary file",
            )
            os.unlink(temporary_name, dir_fd=destination_parent_descriptor)
        except (OSError, ReleaseValidationError):
            pass
        raise
    finally:
        os.close(temporary_descriptor)


def _write_bytes_at_fsynced(parent_descriptor, name, content):
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    descriptor = os.open(name, flags, 0o600, dir_fd=parent_descriptor)
    try:
        _write_all(descriptor, content)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _reject_unsafe_runtime_target(parent_descriptor, name, source_metadata, description):
    try:
        metadata = os.stat(name, dir_fd=parent_descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise ReleaseValidationError("{} must be a regular non-symlink file".format(description))
    if (metadata.st_dev, metadata.st_ino) == (source_metadata.st_dev, source_metadata.st_ino):
        raise ReleaseValidationError("{} must not alias source file".format(description))


def _open_runtime_temporary(parent_descriptor, destination_name):
    for _ in range(100):
        name = ".{}.{}.tmp".format(destination_name, secrets.token_hex(8))
        try:
            descriptor = os.open(name, _runtime_create_flags(), 0o600, dir_fd=parent_descriptor)
        except FileExistsError:
            continue
        return name, descriptor
    raise FileExistsError("unable to allocate runtime database temporary file")


def _runtime_create_flags():
    flags = os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    return flags


def _write_all(descriptor, block):
    offset = 0
    while offset < len(block):
        written = os.write(descriptor, block[offset:])
        if written <= 0:
            raise OSError("unable to write runtime database")
        offset += written


def _read_source_snapshot(path, description):
    with _open_regular_path(path, description) as descriptor:
        return _read_descriptor_snapshot(descriptor, description)


def _read_regular_at(parent_descriptor, name, description):
    with _open_regular_at(parent_descriptor, name, description) as descriptor:
        return _read_descriptor_snapshot(descriptor, description)


def _read_descriptor_snapshot(descriptor, description):
    before = _regular_file_metadata(descriptor, description)
    os.lseek(descriptor, 0, os.SEEK_SET)
    content = bytearray()
    while True:
        block = os.read(descriptor, _STREAM_CHUNK_SIZE)
        if not block:
            break
        content.extend(block)
    after = _regular_file_metadata(descriptor, description)
    _require_stable_file(before, after, len(content), description)
    return bytes(content)


@contextmanager
def _open_regular_path(path, description):
    with _open_path_parent(path, description) as (parent_descriptor, name):
        with _open_regular_at(parent_descriptor, name, description) as descriptor:
            yield descriptor


@contextmanager
def _open_path_parent(path, description):
    _require_descriptor_path_support()
    try:
        absolute = Path(os.path.abspath(os.path.expanduser(os.fspath(path))))
    except (TypeError, ValueError) as error:
        raise ReleaseValidationError("{} path is invalid".format(description)) from error
    parts = absolute.parts
    if len(parts) < 2 or not absolute.is_absolute() or "\x00" in str(absolute):
        raise ReleaseValidationError("{} path is invalid".format(description))
    if (len(parts) >= 6 and parts[1:4] == ("proc", "self", "fd")
            and parts[4].isdigit()):
        if any(component in ("", ".", "..") for component in parts[5:]):
            raise ReleaseValidationError("{} path is invalid".format(description))
        try:
            descriptor = os.dup(int(parts[4]))
        except (OSError, OverflowError, ValueError) as error:
            raise ReleaseValidationError(
                "{} descriptor root is unavailable".format(description)
            ) from error
        try:
            if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
                raise ReleaseValidationError(
                    "{} descriptor root is not a directory".format(description)
                )
            for component in parts[5:-1]:
                next_descriptor = _open_directory_at(
                    descriptor, component, description
                )
                os.close(descriptor)
                descriptor = next_descriptor
            yield descriptor, parts[-1]
        finally:
            os.close(descriptor)
        return
    descriptor = _open_root_directory(description)
    try:
        for component in parts[1:-1]:
            next_descriptor = _open_directory_at(descriptor, component, description)
            os.close(descriptor)
            descriptor = next_descriptor
        yield descriptor, parts[-1]
    finally:
        os.close(descriptor)


@contextmanager
def _open_relative_regular(parent_descriptor, relative_path, description):
    relative = Path(relative_path)
    parts = relative.parts
    if (relative.is_absolute() or not parts
            or any(part in ("", ".", "..") for part in parts)):
        raise ReleaseValidationError("map image path escapes the YAML directory")
    descriptor = os.dup(parent_descriptor)
    try:
        for component in parts[:-1]:
            next_descriptor = _open_directory_at(descriptor, component, description)
            os.close(descriptor)
            descriptor = next_descriptor
        with _open_regular_at(descriptor, parts[-1], description) as file_descriptor:
            yield file_descriptor
    finally:
        os.close(descriptor)


@contextmanager
def _open_regular_at(parent_descriptor, name, description):
    # 공격자가 예상된 regular file을 FIFO나 device로 바꿔도 O_NONBLOCK을 사용하면
    # open 자체를 안전하게 수행하고 fstat으로 종류를 판별할 수 있다.
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        descriptor = os.open(name, flags, dir_fd=parent_descriptor)
    except OSError as error:
        raise ReleaseValidationError(
            "{} must be a regular non-symlink file".format(description)
        ) from error
    try:
        _regular_file_metadata(descriptor, description)
        yield descriptor
    finally:
        os.close(descriptor)


def _open_root_directory(description):
    try:
        return os.open("/", _directory_open_flags())
    except OSError as error:
        raise ReleaseValidationError("unable to open root for {}".format(description)) from error


def _open_directory_at(parent_descriptor, name, description):
    try:
        descriptor = os.open(
            name, _directory_open_flags(), dir_fd=parent_descriptor
        )
    except OSError as error:
        raise ReleaseValidationError(
            "{} has a non-directory or symlink path component".format(description)
        ) from error
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode):
        os.close(descriptor)
        raise ReleaseValidationError("{} path component is not a directory".format(description))
    return descriptor


def _directory_open_flags():
    _require_descriptor_path_support()
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_DIRECTORY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    return flags


def _require_descriptor_path_support():
    if not hasattr(os, "O_NOFOLLOW") or not hasattr(os, "O_DIRECTORY"):
        raise ReleaseValidationError("O_NOFOLLOW and O_DIRECTORY are required")


def _regular_file_metadata(descriptor, description):
    metadata = os.fstat(descriptor)
    if not stat.S_ISREG(metadata.st_mode):
        raise ReleaseValidationError("{} must be a regular file".format(description))
    return metadata


def _file_identity(metadata):
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _require_stable_file(before, after, byte_count, description):
    if _file_identity(before) != _file_identity(after):
        raise ReleaseValidationError("{} changed while being read".format(description))
    if byte_count != after.st_size:
        raise ReleaseValidationError("{} size changed while being read".format(description))


def _snapshot_regular_descriptor(source_descriptor, target_stream, description):
    before = _regular_file_metadata(source_descriptor, description)
    os.lseek(source_descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    byte_count = 0
    while True:
        block = os.read(source_descriptor, _STREAM_CHUNK_SIZE)
        if not block:
            break
        target_stream.write(block)
        digest.update(block)
        byte_count += len(block)
    target_stream.flush()
    os.fsync(target_stream.fileno())
    after = _regular_file_metadata(source_descriptor, description)
    _require_stable_file(before, after, byte_count, description)
    target_stream.seek(0)
    return digest.hexdigest()


def _copy_snapshot_fsynced(source_stream, target):
    source_stream.seek(0)
    with target.open("xb") as target_stream:
        while True:
            block = source_stream.read(_STREAM_CHUNK_SIZE)
            if not block:
                break
            target_stream.write(block)
        target_stream.flush()
        os.fsync(target_stream.fileno())


def _storage_parent(course_dir, name, create):
    if name not in ("raw", "workspaces", "releases"):
        raise ValueError("unsupported storage parent")
    if create:
        course_dir.mkdir(parents=True, exist_ok=True)
    parent = course_dir / name
    if create:
        try:
            os.mkdir(str(parent))
        except FileExistsError:
            pass
        except OSError as error:
            raise ReleaseValidationError("unable to create {} storage".format(name)) from error
    identity = _open_verified_directory(
        parent, "{} storage parent".format(name)
    )
    return parent, identity


def _open_verified_directory(path, description):
    with _open_directory_path(path, description) as descriptor:
        return _directory_identity(os.fstat(descriptor))


@contextmanager
def _open_directory_path(path, description):
    with _open_path_parent(path, description) as (parent_descriptor, name):
        descriptor = _open_directory_at(parent_descriptor, name, description)
        try:
            yield descriptor
        finally:
            os.close(descriptor)


@contextmanager
def _open_course_directory(course_dir, allow_descriptor=False):
    path = _course_path(course_dir, allow_descriptor=allow_descriptor)
    descriptor_match = _DESCRIPTOR_COURSE_PATTERN.fullmatch(str(path))
    if descriptor_match is None:
        with _open_directory_path(path, "course directory") as descriptor:
            yield descriptor
        return
    source_descriptor = int(descriptor_match.group(1))
    try:
        descriptor = os.open(".", _directory_open_flags(), dir_fd=source_descriptor)
    except OSError as error:
        raise ReleaseValidationError("course directory descriptor is unavailable") from error
    try:
        if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
            raise ReleaseValidationError("course directory must be a directory")
        yield descriptor
    finally:
        os.close(descriptor)


def _rename_directory_noreplace(
        source_parent_descriptor, source_name, target_parent_descriptor, target_name):
    if _RENAMEAT2 is None:
        raise ReleaseValidationError("renameat2(RENAME_NOREPLACE) is unavailable")
    result = _RENAMEAT2(
        source_parent_descriptor,
        os.fsencode(source_name),
        target_parent_descriptor,
        os.fsencode(target_name),
        _RENAME_NOREPLACE,
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in (errno.EEXIST, errno.ENOTEMPTY):
        raise FileExistsError(error_number, os.strerror(error_number), target_name)
    if error_number in (errno.ENOSYS, errno.EINVAL):
        raise ReleaseValidationError(
            "renameat2(RENAME_NOREPLACE) is unsupported by this filesystem"
        )
    raise OSError(error_number, os.strerror(error_number), target_name)


def _create_directory_atomically(
        parent, artifact_id, builder, expected_parent_identity=None):
    with _open_directory_path(parent, "artifact storage parent") as parent_descriptor:
        if (expected_parent_identity is not None
                and _directory_identity(os.fstat(parent_descriptor))
                != expected_parent_identity):
            raise ReleaseValidationError(
                "artifact storage parent changed after verification"
            )
        if _entry_exists_at(parent_descriptor, artifact_id):
            raise FileExistsError("artifact ID already exists: {}".format(artifact_id))
        temporary_name = _make_temporary_directory_at(parent_descriptor, artifact_id)
        descriptor_path = Path("/proc/self/fd/{}".format(parent_descriptor))
        temporary_descriptor = None
        temporary_identity = None
        try:
            temporary_descriptor = _open_directory_at(
                parent_descriptor, temporary_name, "temporary artifact"
            )
            temporary_identity = _directory_identity(
                os.fstat(temporary_descriptor)
            )
            temporary = Path(
                "/proc/self/fd/{}".format(temporary_descriptor)
            )
            builder(temporary)
            os.fsync(temporary_descriptor)
            _require_directory_entry_identity(
                parent_descriptor,
                temporary_name,
                temporary_identity,
                "temporary artifact",
            )
            _rename_directory_noreplace(
                parent_descriptor,
                temporary_name,
                parent_descriptor,
                artifact_id,
            )
            try:
                os.fsync(parent_descriptor)
                _require_directory_entry_identity(
                    parent_descriptor,
                    artifact_id,
                    temporary_identity,
                    "published artifact",
                )
                stable_parent = Path(os.readlink(str(descriptor_path)))
            except (OSError, ReleaseValidationError) as error:
                raise _DirectoryPublicationIndeterminate(
                    "artifact publication durability is indeterminate"
                ) from error
            return stable_parent / artifact_id
        except BaseException:
            if (temporary_identity is not None
                    and _entry_exists_at(parent_descriptor, temporary_name)):
                try:
                    _remove_directory_tree_at(
                        parent_descriptor,
                        temporary_name,
                        expected_identity=temporary_identity,
                    )
                except (OSError, ReleaseValidationError):
                    pass
            raise
        finally:
            if temporary_descriptor is not None:
                os.close(temporary_descriptor)


def _entry_exists_at(parent_descriptor, name):
    try:
        os.stat(name, dir_fd=parent_descriptor, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return True


def _make_temporary_directory_at(parent_descriptor, artifact_id):
    for _ in range(100):
        name = ".{}.{}".format(artifact_id, secrets.token_hex(8))
        try:
            os.mkdir(name, 0o700, dir_fd=parent_descriptor)
            return name
        except FileExistsError:
            continue
    raise FileExistsError("unable to allocate temporary artifact directory")


def _fsync_directory_at(parent_descriptor, name):
    descriptor = _open_directory_at(parent_descriptor, name, "temporary artifact")
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_file_fsynced(path, content):
    with path.open("xb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


def _atomic_write_bytes(path, content):
    with _open_path_parent(path, "atomic destination") as (
            parent_descriptor, destination_name):
        _atomic_write_bytes_at(parent_descriptor, destination_name, content)


def _atomic_write_bytes_at(parent_descriptor, destination_name, content):
    temporary_name, temporary_descriptor = _open_runtime_temporary(
        parent_descriptor, destination_name
    )
    temporary_identity = _directory_identity(os.fstat(temporary_descriptor))
    try:
        _write_all(temporary_descriptor, content)
        os.fsync(temporary_descriptor)
        _require_regular_entry_identity(
            parent_descriptor,
            temporary_name,
            temporary_identity,
            "atomic temporary file",
        )
        os.replace(
            temporary_name,
            destination_name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
        )
        _require_regular_entry_identity(
            parent_descriptor,
            destination_name,
            temporary_identity,
            "atomic destination",
        )
        os.fsync(parent_descriptor)
        _require_regular_entry_identity(
            parent_descriptor,
            destination_name,
            temporary_identity,
            "atomic destination",
        )
        if (_read_descriptor_snapshot(
                temporary_descriptor, "atomic destination") != content):
            raise ReleaseValidationError(
                "atomic destination changed during publication"
            )
    except BaseException:
        try:
            _require_regular_entry_identity(
                parent_descriptor,
                temporary_name,
                temporary_identity,
                "atomic temporary file",
            )
            os.unlink(temporary_name, dir_fd=parent_descriptor)
        except (OSError, ReleaseValidationError):
            pass
        raise
    finally:
        os.close(temporary_descriptor)


def _map_yaml_bytes(grid, image_name):
    return (
        "image: {}\n"
        "resolution: {}\n"
        "origin: [{}, {}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    ).format(
        json.dumps(image_name), repr(grid.resolution), repr(grid.origin_x), repr(grid.origin_y)
    ).encode("utf-8")


def _json_bytes(document):
    return (json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
            + "\n").encode("utf-8")


def _load_json(path):
    return _decode_json(_read_regular_bytes(path), path.name)


def _decode_json(content, name):
    try:
        return json.loads(
            content.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeError, json.JSONDecodeError, ReleaseValidationError) as error:
        raise ReleaseValidationError("{} is not strict JSON".format(name)) from error


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ReleaseValidationError("duplicate JSON key: {}".format(key))
        result[key] = value
    return result


def _reject_json_constant(value):
    raise ReleaseValidationError("non-standard JSON constant: {}".format(value))


def _read_regular_bytes(path):
    return _read_source_snapshot(path, "artifact {}".format(path.name))


def _grid_from_artifact_bytes(yaml_bytes, image_bytes, expected_image_name):
    try:
        document = parse_ros_map_yaml(yaml_bytes)
        if document["image"] != expected_image_name:
            raise MapFormatError("map YAML image does not match artifact name")
        return grid_map_from_pgm(document, image_bytes)
    except MapFormatError as error:
        raise ReleaseValidationError("map artifact is invalid") from error


def _require_hash(path, expected, description):
    _validate_sha256(expected, description)
    content = _read_regular_bytes(path)
    if hashlib.sha256(content).hexdigest() != expected:
        raise ReleaseValidationError("{} SHA-256 mismatch: {}".format(description, path.name))
    return content


def _require_stream_hash(path, expected, description):
    _validate_sha256(expected, description)
    actual = _sha256_file(path)
    if actual != expected:
        raise ReleaseValidationError("{} SHA-256 mismatch: {}".format(description, path.name))


def _require_stream_hash_and_validate_rtabmap(path, expected, description):
    """안정된 source file snapshot 하나에서 hash와 SQLite 내용을 검증한다."""
    _validate_sha256(expected, description)
    with _open_regular_path(path, description) as descriptor:
        with tempfile.NamedTemporaryFile(delete=False) as snapshot:
            snapshot_path = Path(snapshot.name)
            try:
                actual = _snapshot_regular_descriptor(descriptor, snapshot, description)
                if actual != expected:
                    raise ReleaseValidationError(
                        "{} SHA-256 mismatch: {}".format(description, path.name)
                    )
                _validate_rtabmap_database_snapshot(snapshot, description)
            finally:
                try:
                    snapshot_path.unlink()
                except FileNotFoundError:
                    pass


def _validate_rtabmap_database_snapshot(snapshot, description):
    """RTAB-Map 0.21 LiDAR database 계약을 검사하는 read-only SQLite gate다."""
    try:
        snapshot.flush()
        os.fsync(snapshot.fileno())
        snapshot.seek(0, os.SEEK_END)
        if snapshot.tell() == 0:
            raise ReleaseValidationError("{} is an empty RTAB-Map database".format(description))
        uri = "file:/proc/self/fd/{}?mode=ro&immutable=1".format(snapshot.fileno())
        connection = sqlite3.connect(uri, uri=True)
        try:
            quick_check = connection.execute("PRAGMA quick_check").fetchall()
            if quick_check != [("ok",)]:
                raise ReleaseValidationError("{} SQLite quick_check failed".format(description))
            _require_rtabmap_schema(connection, description)
        finally:
            connection.close()
    except ReleaseValidationError:
        raise
    except (OSError, sqlite3.DatabaseError, sqlite3.Error) as error:
        raise ReleaseValidationError("{} is not a valid RTAB-Map SQLite database".format(description)) from error
    finally:
        snapshot.seek(0)


def _require_rtabmap_schema(connection, description):
    for table, columns in _RTABMAP_02113_REQUIRED_COLUMNS.items():
        found = {row[1] for row in connection.execute("PRAGMA table_info({})".format(table))}
        if not columns.issubset(found):
            raise ReleaseValidationError("{} is missing required RTAB-Map {} schema".format(description, table))
    versions = connection.execute("SELECT version FROM Admin").fetchall()
    if len(versions) != 1 or not isinstance(versions[0][0], str):
        raise ReleaseValidationError("{} has an invalid RTAB-Map Admin version".format(description))
    version_match = re.fullmatch(r"0\.21\.(0|[1-9][0-9]*)", versions[0][0])
    if version_match is None or int(version_match.group(1)) > 13:
        raise ReleaseValidationError(
            "{} RTAB-Map version is newer than supported Noetic 0.21.13".format(description)
        )
    if connection.execute("SELECT 1 FROM Node LIMIT 1").fetchone() is None:
        raise ReleaseValidationError("{} has no RTAB-Map nodes".format(description))
    scan = connection.execute(
        "SELECT 1 FROM Node JOIN Data ON Data.id = Node.id "
        "WHERE Data.scan IS NOT NULL AND length(Data.scan) > 0 LIMIT 1"
    ).fetchone()
    if scan is None:
        raise ReleaseValidationError("{} has no LiDAR scan data for a RTAB-Map node".format(description))


def _validate_sha256(expected, description):
    if (not isinstance(expected, str) or len(expected) != 64
            or any(character not in "0123456789abcdef" for character in expected)):
        raise ReleaseValidationError("{} SHA-256 is invalid".format(description))


def _sha256_file(path):
    with _open_regular_path(path, "artifact {}".format(path.name)) as descriptor:
        return _sha256_descriptor(descriptor, "artifact {}".format(path.name))


def _sha256_internal_file(path):
    try:
        with path.open("rb") as stream:
            return _sha256_descriptor(stream.fileno(), "artifact {}".format(path.name))
    except OSError as error:
        raise ReleaseValidationError(
            "artifact {} must be a regular file".format(path.name)
        ) from error


def _sha256_descriptor(descriptor, description):
    before = _regular_file_metadata(descriptor, description)
    os.lseek(descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    byte_count = 0
    while True:
        block = os.read(descriptor, _STREAM_CHUNK_SIZE)
        if not block:
            break
        digest.update(block)
        byte_count += len(block)
    after = _regular_file_metadata(descriptor, description)
    _require_stable_file(before, after, byte_count, description)
    return digest.hexdigest()


def _validate_id(value, name):
    if (not isinstance(value, str) or not _ID_PATTERN.fullmatch(value)
            or value in (".", "..")):
        raise ValueError("{} must be a safe 1-64 character identifier".format(name))
    return value


def _course_path(course_dir, allow_descriptor=False):
    try:
        path = Path(course_dir)
    except TypeError as error:
        raise ValueError("course_dir must be path-like") from error
    if "\x00" in str(path):
        raise ValueError("course_dir contains a null byte")
    descriptor_match = _DESCRIPTOR_COURSE_PATTERN.fullmatch(str(path))
    if descriptor_match is not None:
        if not allow_descriptor:
            raise ValueError("course_dir descriptor roots are reserved for editor workspaces")
        try:
            metadata = os.fstat(int(descriptor_match.group(1)))
        except (OSError, OverflowError, ValueError) as error:
            raise ValueError("course_dir descriptor is unavailable") from error
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError("course_dir descriptor must reference a directory")
        return path
    if _DESCRIPTOR_ROOT_PATTERN.match(str(path)):
        raise ValueError("course_dir descriptor root is not allowed")
    return path.resolve()


def _validate_timestamp(value):
    if not isinstance(value, str) or not value:
        raise ValueError("now_iso must be a non-empty ISO-8601 timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError("now_iso must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("now_iso must include a timezone")
    return value


def _validate_timestamp_as_release_error(value, description):
    try:
        _validate_timestamp(value)
    except ValueError as error:
        raise ReleaseValidationError("{} is invalid".format(description)) from error


def _utc_now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _fsync_directory(directory):
    descriptor = os.open(str(directory), os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
