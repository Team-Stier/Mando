import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


WORKSPACE = Path(__file__).resolve().parents[3]
SCRIPT = WORKSPACE / "scripts" / "bootstrap_dependencies.sh"
PACKAGES = (
    "build-essential",
    "cmake",
    "nodejs",
    "python3-nose",
    "python3-setuptools",
    "python3-yaml",
    "ros-noetic-catkin",
    "ros-noetic-map-server",
    "ros-noetic-robot-localization",
    "ros-noetic-rosbag",
    "ros-noetic-rosnode",
    "ros-noetic-rostest",
    "ros-noetic-rtabmap-ros",
    "ros-noetic-rviz",
    "ros-noetic-tf2-sensor-msgs",
)


class BootstrapDependenciesTest(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.calls = self.root / "calls"
        self._write_executable(
            "apt-get",
            "#!/bin/sh\nprintf 'apt-get\\t%s\\n' \"$*\" >> \"$FAKE_CALLS\"\n",
        )
        self._write_executable(
            "sudo",
            textwrap.dedent(
                """\
                #!/bin/sh
                printf 'sudo\\t%s\\n' "$*" >> "$FAKE_CALLS"
                exec "$@"
                """
            ),
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _write_executable(self, name, content):
        path = self.bin / name
        path.write_text(content, encoding="utf-8")
        path.chmod(0o755)

    def _set_installed(self, installed):
        cases = "\n".join(
            "  {}) printf '%s\\n' 'install ok installed' ;;".format(package)
            for package in installed
        )
        self._write_executable(
            "dpkg-query",
            textwrap.dedent(
                """\
                #!/bin/sh
                package=""
                for argument in "$@"; do package="$argument"; done
                case "$package" in
                {cases}
                  *) exit 1 ;;
                esac
                """
            ).format(cases=cases),
        )

    def _run(self, *arguments):
        environment = os.environ.copy()
        environment.update({
            "PATH": "{}:/usr/bin:/bin".format(self.bin),
            "FAKE_CALLS": str(self.calls),
        })
        return subprocess.run(
            [str(SCRIPT), *arguments],
            cwd=str(WORKSPACE),
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )

    def test_requires_exactly_one_supported_mode(self):
        """Accepting legacy or trailing arguments could silently select a mutating mode."""
        self._set_installed(PACKAGES)

        for arguments in ((), ("check",), ("--unknown",), ("--check", "extra")):
            with self.subTest(arguments=arguments):
                result = self._run(*arguments)
                self.assertEqual(result.returncode, 2)
                self.assertIn("Usage:", result.stderr)

    def test_check_reports_every_missing_package_without_invoking_privileged_tools(self):
        """A read-only preflight must expose all gaps without invoking sudo or apt."""
        missing = ("python3-yaml", "ros-noetic-rtabmap-ros", "ros-noetic-tf2-sensor-msgs")
        self._set_installed(package for package in PACKAGES if package not in missing)

        result = self._run("--check")

        self.assertEqual(result.returncode, 1)
        self.assertEqual(
            result.stdout.splitlines(),
            ["Missing package: {}".format(package) for package in missing],
        )
        self.assertFalse(self.calls.exists())

    def test_check_succeeds_when_every_explicit_package_is_installed(self):
        """A successful check must mean every declared package has installed status."""
        self._set_installed(PACKAGES)

        result = self._run("--check")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "All required packages are installed.\n")
        self.assertFalse(self.calls.exists())

    def test_install_prints_and_passes_only_the_sorted_explicit_package_list(self):
        """Install mode must not let unchecked arguments reach apt-get."""
        self._set_installed(())

        result = self._run("--install")

        self.assertEqual(result.returncode, 0, result.stderr)
        expected_lines = ["Installing packages:"] + [
            "  {}".format(package) for package in PACKAGES
        ]
        self.assertEqual(result.stdout.splitlines(), expected_lines)
        calls = self.calls.read_text(encoding="utf-8").splitlines()
        expected_arguments = "apt-get install {}".format(" ".join(PACKAGES))
        self.assertEqual(calls, [
            "sudo\t{}".format(expected_arguments),
            "apt-get\tinstall {}".format(" ".join(PACKAGES)),
        ])


if __name__ == "__main__":
    unittest.main()
