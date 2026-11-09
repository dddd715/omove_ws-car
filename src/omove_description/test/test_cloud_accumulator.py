"""Check bounded temporal assembly and saved point geometry without DDS."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    'accumulator', Path(__file__).parents[1] / 'omove_description/cloud_accumulator.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CloudAccumulatorTest(unittest.TestCase):
    def test_fuses_distinct_frames_and_voxel_deduplicates(self):
        window = module.CloudWindow(voxel_size=.1)
        window.add(1_000_000_000, [[0., 0., .15], [1., 0., .15]])
        window.add(2_000_000_000, [[.01, 0., .15], [2., 0., .15]])
        result = window.assembled()
        self.assertEqual(len(result), 3)
        self.assertTrue(any(np.allclose(p, [.01, 0., .15]) for p in result))
        np.testing.assert_allclose(result[:, 2], [.15]*3)

    def test_temporal_window_discards_old_geometry(self):
        window = module.CloudWindow(window_seconds=1.)
        window.add(1_000_000_000, [[1., 0., 0.]])
        window.add(2_000_000_000, [[2., 0., 0.]])
        window.add(2_100_000_000, [[3., 0., 0.]])
        self.assertEqual(len(window.frames), 2)
        self.assertFalse(any(p[0] == 1. for p in window.assembled()))

    def test_bounds_memory_by_points_and_frames(self):
        window = module.CloudWindow(max_points=3, max_frames=2)
        window.add(1, [[1., 0., 0.], [2., 0., 0.]])
        window.add(2, [[3., 0., 0.], [4., 0., 0.]])
        self.assertLessEqual(window.point_count, 3)
        window.add(3, [[5., 0., 0.]])
        window.add(4, [[6., 0., 0.]])
        self.assertLessEqual(len(window.frames), 2)
        window.add(5, [[float(i), 0., 0.] for i in range(20)])
        self.assertLessEqual(len(window.assembled()), 3)

    def test_time_reset_and_duplicates(self):
        window = module.CloudWindow()
        window.add(100, [[1., 0., 0.]])
        window.add(100, [[2., 0., 0.]])
        self.assertEqual(len(window.frames), 1)
        self.assertTrue(window.add(50, [[3., 0., 0.]]))
        np.testing.assert_allclose(window.assembled(), [[3., 0., 0.]])

    def test_empty_nonfinite_and_clear(self):
        window = module.CloudWindow()
        window.add(1, [[float('nan'), 0., 0.], [1., 2., 3.]])
        window.add(2, [])
        self.assertEqual(len(window.assembled()), 1)
        window.clear()
        self.assertEqual(window.assembled().shape, (0, 3))
        self.assertIsNone(window.latest_ns)

    def test_pcd_keeps_xyz_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'cloud.pcd'
            module.save_pcd(path, [[1., 2., .15], [3., 4., .15]], 'odom', 123)
            original = path.read_bytes()
            text = original.decode()
            self.assertIn('FIELDS x y z', text)
            self.assertIn('POINTS 2', text)
            self.assertIn('frame_id=odom', text)
            np.testing.assert_allclose(
                np.loadtxt(text.split('DATA ascii\n')[1].splitlines()),
                [[1, 2, .15], [3, 4, .15]])
            with self.assertRaises(FileExistsError):
                module.save_pcd(path, [[8., 9., 10.]], 'odom', 124)
            self.assertEqual(path.read_bytes(), original)

    def test_invalid_limits_and_empty_save(self):
        for kwargs in [{'voxel_size': 0.}, {'window_seconds': -1.},
                       {'max_points': 0}, {'max_frames': .5}]:
            with self.assertRaises(ValueError):
                module.CloudWindow(**kwargs)
        with self.assertRaises(ValueError):
            module.save_pcd('/tmp/never_created_empty_cloud.pcd', [], 'odom', None)


if __name__ == '__main__':
    unittest.main()
