"""map workspace 편집을 위한 localhost 전용 표준 라이브러리 HTTP adapter다."""

from contextlib import contextmanager
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import re
import socket
import stat
import threading
import time
from urllib.parse import urlsplit

from stier_slam_core.map_release import (
    ReleaseManager,
    ReleaseValidationError,
    WorkspaceConflictError,
)


MAX_REQUEST_BYTES = 2 * 1024 * 1024
MAX_CONTENT_LENGTH_DIGITS = 20
REQUEST_SOCKET_TIMEOUT_SECONDS = 5.0
SERVER_CLOSE_TIMEOUT_SECONDS = 6.0
MAX_EDITOR_WORKERS = 16
_STATIC_TYPES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}
_SECURITY_HEADERS = {
    "X-Content-Type-Options": "nosniff",
    "X-Frame-Options": "DENY",
    "Referrer-Policy": "no-referrer",
    "Content-Security-Policy": (
        "default-src 'self'; script-src 'self'; style-src 'self'; "
        "img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; "
        "base-uri 'none'; frame-ancestors 'none'"
    ),
}


class _EditorHttpServer(ThreadingHTTPServer):
    """역할:
    로컬 map editor HTTP 요청을 제한된 worker 수로 안전하게 처리한다.

    동작 방식:
    요청 socket timeout과 worker semaphore를 적용하고, 각 manager operation 동안 고정된
    course directory descriptor를 복제해 사용하며 shutdown 시 worker와 descriptor를 정리한다.
    """

    # 요청에는 제한된 graceful drain 시간이 주어진다. 이후에는 daemon worker가
    # SIGTERM의 process 종료를 막을 수 없다. 모든 manager publish는 transaction으로
    # 처리되며 process가 descriptor를 닫으면 fail-closed로 동작한다.
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True

    def __init__(self, *arguments, **keywords):
        super().__init__(*arguments, **keywords)
        self._shutting_down = False
        self._worker_slots = threading.BoundedSemaphore(MAX_EDITOR_WORKERS)
        self._worker_condition = threading.Condition()
        self._active_workers = 0

    def get_request(self):
        request, client_address = super().get_request()
        request.settimeout(REQUEST_SOCKET_TIMEOUT_SECONDS)
        return request, client_address

    def process_request(self, request, client_address):
        if not self._worker_slots.acquire(blocking=False):
            self._reject_worker_overflow(request)
            self.shutdown_request(request)
            return
        with self._worker_condition:
            self._active_workers += 1
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._worker_finished()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._worker_finished()

    def _worker_finished(self):
        with self._worker_condition:
            self._active_workers -= 1
            self._worker_condition.notify_all()
        self._worker_slots.release()

    @staticmethod
    def _reject_worker_overflow(request):
        content = b'{"schema_version":1,"error":{"code":"server_busy","message":"editor worker limit reached"}}\n'
        headers = [
            b"HTTP/1.1 503 Service Unavailable",
            b"Content-Type: application/json; charset=utf-8",
            "Content-Length: {}".format(len(content)).encode("ascii"),
            b"Cache-Control: no-store",
            b"Connection: close",
        ]
        headers.extend(
            "{}: {}".format(name, value).encode("ascii")
            for name, value in _SECURITY_HEADERS.items()
        )
        try:
            request.sendall(b"\r\n".join(headers) + b"\r\n\r\n" + content)
        except OSError:
            pass

    def shutdown(self):
        self._shutting_down = True
        return super().shutdown()

    @contextmanager
    def pinned_course_access(self):
        """manager 작업 하나가 끝날 때까지 private course FD를 유지한다."""
        request_descriptor = None
        with self.course_descriptor_lock:
            try:
                if self.course_descriptor is None:
                    raise OSError("pinned course directory is unavailable")
                request_descriptor = os.dup(self.course_descriptor)
                pinned = os.fstat(request_descriptor)
                current = os.stat(self.course_dir, follow_symlinks=False)
                if (not stat.S_ISDIR(current.st_mode)
                        or (current.st_dev, current.st_ino)
                        != (pinned.st_dev, pinned.st_ino)):
                    raise OSError("pinned course directory identity changed")
            except (OSError, TypeError):
                if request_descriptor is not None:
                    os.close(request_descriptor)
                raise
        try:
            yield Path("/proc/self/fd/{}".format(request_descriptor))
        finally:
            os.close(request_descriptor)

    def server_close(self):
        self._shutting_down = True
        try:
            super().server_close()
            deadline = time.monotonic() + SERVER_CLOSE_TIMEOUT_SECONDS
            with self._worker_condition:
                while self._active_workers:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._worker_condition.wait(timeout=remaining)
        finally:
            lock = getattr(self, "course_descriptor_lock", None)
            if lock is not None:
                with lock:
                    descriptor = getattr(self, "course_descriptor", None)
                    if descriptor is not None:
                        os.close(descriptor)
                        self.course_descriptor = None


class _StrictJsonError(ValueError):
    pass


def create_editor_server(
        course_dir, workspace_id, web_root, host="127.0.0.1", port=8765,
        unsafe_allow_nonlocal=False):
    """고정 입력을 모두 검증한 뒤 테스트 가능한 threaded server를 만든다."""
    _validate_bind_host(host, unsafe_allow_nonlocal)
    if isinstance(port, bool) or not isinstance(port, int) or port < 0 or port > 65535:
        raise ValueError("port must be an integer between 0 and 65535")
    pinned_course_dir, course_descriptor = _open_pinned_course(course_dir)
    course_access_path = Path("/proc/self/fd/{}".format(course_descriptor))
    manager = ReleaseManager()
    try:
        manager.read_workspace(course_access_path, workspace_id)
        static_assets = _load_static_assets(web_root)
        server = _EditorHttpServer((host, port), _EditorRequestHandler)
    except BaseException:
        os.close(course_descriptor)
        raise
    server.course_dir = pinned_course_dir
    server.course_descriptor_lock = threading.Lock()
    server.course_descriptor = course_descriptor
    server.workspace_id = workspace_id
    server.release_manager = manager
    server.static_assets = static_assets
    bound_host, bound_port = server.server_address[:2]
    display_host = "[{}]".format(bound_host) if ":" in bound_host else bound_host
    server.allowed_host = "{}:{}".format(display_host, bound_port)
    server.expected_origin = "http://{}".format(server.allowed_host)
    return server


def _open_pinned_course(course_dir):
    try:
        pinned = Path(course_dir).resolve(strict=True)
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        raise ValueError("course_dir must resolve to an existing directory") from error
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        descriptor = os.open(str(pinned), flags)
    except OSError as error:
        raise ValueError("course_dir must resolve to an existing directory") from error
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise ValueError("course_dir must resolve to an existing directory")
    return pinned, descriptor


def _validate_bind_host(host, unsafe_allow_nonlocal):
    if not isinstance(host, str) or not host or "\x00" in host:
        raise ValueError("host must be a non-empty IP address")
    if type(unsafe_allow_nonlocal) is not bool:
        raise ValueError("unsafe_allow_nonlocal must be a boolean")
    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise ValueError("host must be an IP address") from error
    if not address.is_loopback and not unsafe_allow_nonlocal:
        raise ValueError("non-loopback bind requires explicit unsafe opt-in")


def _load_static_assets(web_root):
    root = Path(os.path.abspath(os.path.expanduser(os.fspath(web_root))))
    if root.is_symlink() or not root.is_dir():
        raise ValueError("web_root must be a real directory")
    assets = {}
    for route, (name, content_type) in _STATIC_TYPES.items():
        if name in assets:
            continue
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        try:
            descriptor = os.open(str(root / name), flags)
        except OSError as error:
            raise ValueError("web_root is missing a required static asset") from error
        try:
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode):
                raise ValueError("static assets must be regular files")
            with os.fdopen(descriptor, "rb", closefd=False) as stream:
                content = stream.read()
        finally:
            os.close(descriptor)
        assets[name] = (content_type, content)
    return assets


class _EditorRequestHandler(BaseHTTPRequestHandler):
    """역할:
    map workspace 조회·편집 API와 정적 web asset 응답을 처리한다.

    동작 방식:
    Host, Origin, method, body 크기와 strict JSON을 검증한 뒤 ``ReleaseManager`` 작업을
    실행하고, 성공 결과나 안정된 error schema를 HTTP 응답으로 반환한다.
    """

    protocol_version = "HTTP/1.1"
    server_version = "StierMapEditor/1"
    sys_version = ""

    def log_message(self, format_string, *arguments):
        del format_string, arguments

    def parse_request(self):
        parsed = super().parse_request()
        if parsed and self.server._shutting_down:
            self.close_connection = True
            return False
        return parsed

    def __getattr__(self, name):
        if name.startswith("do_"):
            return self._method_not_allowed
        raise AttributeError(name)

    def handle_expect_100(self):
        """표준 라이브러리가 임시 응답을 보내기 전에 안전하지 않은 upload를 거절한다."""
        expectations = self.headers.get_all("Expect", failobj=[])
        if (len(expectations) != 1
                or expectations[0].lower() != "100-continue"
                or self.command not in ("PUT", "POST")):
            self._reject_prebody(
                417, "expectation_failed", "request expectation is unsupported"
            )
            return False
        if not self._authorize_state_change():
            return False
        if self._validated_json_body_length() is None:
            return False
        self.send_response_only(100)
        self.end_headers()
        return True

    def do_GET(self):
        if not self._authorize_host():
            return
        if not self._require_empty_request_body():
            return
        route = self._route()
        if route is None:
            return
        if route == "/api/workspace":
            try:
                with self.server.pinned_course_access() as course_dir:
                    document = self.server.release_manager.read_workspace(
                        course_dir, self.server.workspace_id
                    )
            except (OSError, TypeError, ValueError) as error:
                self._manager_error(error)
                return
            self._send_json(200, document)
            return
        if route == "/api/base.pgm":
            try:
                with self.server.pinned_course_access() as course_dir:
                    content = self.server.release_manager.read_workspace_base_pgm(
                        course_dir, self.server.workspace_id
                    )
            except (OSError, TypeError, ValueError) as error:
                self._manager_error(error)
                return
            self._send_bytes(200, "image/x-portable-graymap", content, no_store=True)
            return
        static = _STATIC_TYPES.get(route)
        if static is None:
            self._error(404, "not_found", "resource not found")
            return
        name, _ = static
        content_type, content = self.server.static_assets[name]
        self._send_bytes(200, content_type, content, no_store=False)

    def do_PUT(self):
        if not self._authorize_state_change():
            return
        route = self._route()
        if route is None:
            return
        if route != "/api/workspace":
            self.close_connection = True
            self._error(404, "not_found", "resource not found")
            return
        document = self._read_json_document()
        if document is None:
            return
        if (not isinstance(document, dict)
                or set(document) != {
                    "schema_version", "expected_revision", "static_overlay", "semantic_layer"
                }
                or type(document.get("schema_version")) is not int
                or document.get("schema_version") != 1):
            self._error(400, "invalid_request", "workspace request schema is invalid")
            return
        try:
            with self.server.pinned_course_access() as course_dir:
                result = self.server.release_manager.replace_workspace(
                    course_dir,
                    self.server.workspace_id,
                    document["expected_revision"],
                    document["static_overlay"],
                    document["semantic_layer"],
                )
        except (OSError, TypeError, ValueError) as error:
            self._manager_error(error)
            return
        self._send_json(200, result)

    def do_POST(self):
        if not self._authorize_state_change():
            return
        route = self._route()
        if route is None:
            return
        if route != "/api/release":
            self.close_connection = True
            self._error(404, "not_found", "resource not found")
            return
        document = self._read_json_document()
        if document is None:
            return
        if (not isinstance(document, dict)
                or set(document) != {
                    "schema_version", "expected_revision", "release_id", "activate"
                }
                or type(document.get("schema_version")) is not int
                or document.get("schema_version") != 1
                or type(document.get("activate")) is not bool):
            self._error(400, "invalid_request", "release request schema is invalid")
            return
        try:
            with self.server.pinned_course_access() as course_dir:
                result = self.server.release_manager.release_workspace(
                    course_dir,
                    self.server.workspace_id,
                    document["expected_revision"],
                    document["release_id"],
                    document["activate"],
                    _utc_now_iso(),
                )
        except (OSError, TypeError, ValueError) as error:
            self._manager_error(error)
            return
        self._send_json(201, result)

    def do_DELETE(self):
        self._method_not_allowed()

    def do_PATCH(self):
        self._method_not_allowed()

    def do_OPTIONS(self):
        self._method_not_allowed()

    def do_HEAD(self):
        self._method_not_allowed()

    def _method_not_allowed(self):
        if not self._authorize_host():
            return
        self._close_if_unread_or_ambiguous_body()
        self._error(
            405, "method_not_allowed", "method is not allowed",
            extra_headers={"Allow": "GET, PUT, POST"},
        )

    def _route(self):
        try:
            parsed = urlsplit(self.path)
        except ValueError:
            self.close_connection = True
            self._error(404, "not_found", "resource not found")
            return None
        if parsed.query or parsed.fragment or not parsed.path.startswith("/"):
            self.close_connection = True
            self._error(404, "not_found", "resource not found")
            return None
        return parsed.path

    def _authorize_host(self):
        hosts = self.headers.get_all("Host", failobj=[])
        if len(hosts) != 1 or hosts[0] != self.server.allowed_host:
            self.close_connection = True
            self._error(403, "forbidden_host", "request Host is not allowed")
            return False
        return True

    def _authorize_state_change(self):
        if not self._authorize_host():
            return False
        origin_values = self.headers.get_all("Origin", failobj=[])
        if len(origin_values) > 1:
            self.close_connection = True
            self._error(403, "forbidden_origin", "request Origin is not allowed")
            return False
        if origin_values and origin_values[0] != self.server.expected_origin:
            self.close_connection = True
            self._error(403, "forbidden_origin", "request Origin is not allowed")
            return False
        return True

    def _read_json_document(self):
        length = self._validated_json_body_length()
        if length is None:
            return None
        try:
            content = self.rfile.read(length)
        except (OSError, socket.timeout):
            self._reject_prebody(
                400, "invalid_framing", "request body is truncated"
            )
            return None
        if len(content) != length:
            self._reject_prebody(
                400, "invalid_framing", "request body is truncated"
            )
            return None
        try:
            return json.loads(
                content.decode("utf-8"),
                object_pairs_hook=_unique_json_object,
                parse_constant=_reject_json_constant,
            )
        except (UnicodeError, json.JSONDecodeError, _StrictJsonError, RecursionError):
            self._error(400, "invalid_json", "request body is not strict JSON")
            return None

    def _validated_json_body_length(self):
        expectations = self.headers.get_all("Expect", failobj=[])
        if expectations and (len(expectations) != 1
                or expectations[0].lower() != "100-continue"):
            self._reject_prebody(
                417, "expectation_failed", "request expectation is unsupported"
            )
            return None
        if self.headers.get_all("Transfer-Encoding", failobj=[]):
            self._reject_prebody(
                400, "invalid_framing", "Transfer-Encoding is unsupported"
            )
            return None
        length = self._parse_content_length(required=True)
        if length is None:
            return None
        if length > MAX_REQUEST_BYTES:
            self._reject_prebody(
                413, "request_too_large", "request body exceeds 2 MiB"
            )
            return None
        if self.headers.get_content_type() != "application/json":
            self._reject_prebody(
                415, "unsupported_media_type", "application/json is required"
            )
            return None
        charset = self.headers.get_content_charset("utf-8")
        if not isinstance(charset, str) or charset.lower() != "utf-8":
            self._reject_prebody(
                415, "unsupported_media_type", "JSON charset must be UTF-8"
            )
            return None
        return length

    def _parse_content_length(self, required):
        lengths = self.headers.get_all("Content-Length", failobj=[])
        if not lengths:
            if required:
                self._reject_prebody(
                    411, "length_required", "Content-Length is required"
                )
                return None
            return 0
        if len(lengths) != 1:
            self._reject_prebody(
                400, "invalid_framing", "Content-Length is invalid"
            )
            return None
        value = lengths[0]
        if (len(value) > MAX_CONTENT_LENGTH_DIGITS
                and re.fullmatch(r"[0-9]+", value, flags=re.ASCII)):
            self._reject_prebody(
                413, "request_too_large", "request body exceeds 2 MiB"
            )
            return None
        if not re.fullmatch(r"[0-9]+", value, flags=re.ASCII):
            self._reject_prebody(
                400, "invalid_framing", "Content-Length is invalid"
            )
            return None
        return int(value)

    def _require_empty_request_body(self):
        if self.headers.get_all("Expect", failobj=[]):
            self._reject_prebody(
                417, "expectation_failed", "request expectation is unsupported"
            )
            return False
        if self.headers.get_all("Transfer-Encoding", failobj=[]):
            self._reject_prebody(
                400, "invalid_framing", "Transfer-Encoding is unsupported"
            )
            return False
        length = self._parse_content_length(required=False)
        if length is None:
            return False
        if length != 0:
            self._reject_prebody(
                400, "invalid_framing", "request method does not accept a body"
            )
            return False
        return True

    def _close_if_unread_or_ambiguous_body(self):
        if (self.headers.get_all("Transfer-Encoding", failobj=[])
                or self.headers.get_all("Expect", failobj=[])):
            self.close_connection = True
            return
        lengths = self.headers.get_all("Content-Length", failobj=[])
        if not lengths:
            return
        if (len(lengths) != 1
                or not re.fullmatch(r"[0-9]+", lengths[0], flags=re.ASCII)
                or len(lengths[0]) > MAX_CONTENT_LENGTH_DIGITS
                or int(lengths[0]) != 0):
            self.close_connection = True

    def _reject_prebody(self, status, code, message):
        self.close_connection = True
        self._error(status, code, message)

    def _manager_error(self, error):
        if isinstance(error, WorkspaceConflictError):
            self._error(409, "revision_conflict", "workspace revision is stale")
        elif isinstance(error, FileExistsError):
            self._error(409, "release_exists", "release ID already exists")
        elif isinstance(error, (ReleaseValidationError, TypeError, ValueError)):
            self._error(400, "validation_failed", str(error))
        else:
            self._error(500, "internal_error", "internal editor error")

    def _send_json(self, status, document, extra_headers=None):
        content = (json.dumps(
            document, sort_keys=True, separators=(",", ":"), allow_nan=False
        ) + "\n").encode("utf-8")
        self._send_bytes(
            status,
            "application/json; charset=utf-8",
            content,
            no_store=True,
            extra_headers=extra_headers,
        )

    def _error(self, status, code, message, extra_headers=None):
        self._send_json(
            status,
            {"schema_version": 1, "error": {"code": code, "message": message}},
            extra_headers=extra_headers,
        )

    def _send_bytes(
            self, status, content_type, content, no_store, extra_headers=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store" if no_store else "no-cache")
        for name, value in _SECURITY_HEADERS.items():
            self.send_header(name, value)
        if self.close_connection:
            self.send_header("Connection", "close")
        for name, value in (extra_headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(content)


def _unique_json_object(pairs):
    document = {}
    for key, value in pairs:
        if key in document:
            raise _StrictJsonError("duplicate key")
        document[key] = value
    return document


def _reject_json_constant(value):
    raise _StrictJsonError("non-standard JSON constant: {}".format(value))


def _utc_now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
