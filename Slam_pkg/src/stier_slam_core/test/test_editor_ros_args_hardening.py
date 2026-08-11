import contextlib
import importlib.util
import io
from pathlib import Path
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "map_editor_server.py"


def load_editor_script():
    specification = importlib.util.spec_from_file_location(
        "stier_map_editor_script_hardening", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class EditorRosArgumentsHardeningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_editor_script()

    def test_known_option_values_containing_remap_delimiter_are_preserved(self):
        """Parser-owned values are data even when they contain the ROS ':=' delimiter."""
        filtered = self.module._without_ros_remapping_args([
            "__name:=map_editor",
            "--course-dir",
            "/tmp/course:=literal",
            "--workspace-id",
            "workspace:=literal",
            "--web-root",
            "/tmp/web:=literal",
            "/input:=/mapped",
        ])

        self.assertEqual(filtered, [
            "--course-dir",
            "/tmp/course:=literal",
            "--workspace-id",
            "workspace:=literal",
            "--web-root",
            "/tmp/web:=literal",
        ])
        parsed = self.module._parser().parse_args(filtered)
        self.assertEqual(parsed.course_dir, "/tmp/course:=literal")
        self.assertEqual(parsed.workspace_id, "workspace:=literal")
        self.assertEqual(parsed.web_root, "/tmp/web:=literal")

    def test_illegal_remap_like_tokens_are_left_for_argparse_to_reject(self):
        """Only canonical ROS remaps may disappear from the executable argv contract."""
        arguments = [
            "--course-dir",
            "/tmp/course",
            "--workspace-id",
            "workspace",
            "9invalid:=value",
            "name with space:=value",
        ]
        filtered = self.module._without_ros_remapping_args(arguments)

        self.assertEqual(filtered, arguments)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                self.module._parser().parse_args(filtered)
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
