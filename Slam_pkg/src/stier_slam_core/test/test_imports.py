import unittest


class ImportTest(unittest.TestCase):
    def test_core_version_is_exposed(self):
        import stier_slam_core

        self.assertEqual(stier_slam_core.__version__, "0.1.0")
