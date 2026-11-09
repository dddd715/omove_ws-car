"""Check scan geometry, invalid returns, and real coordinate transforms."""
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2

spec = importlib.util.spec_from_file_location(
    'scan_cloud', Path(__file__).parents[1] / 'omove_description/scan_to_cloud.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp.sec = 101
    msg.header.stamp.nanosec = 1234
    msg.angle_min = 0.0
    msg.angle_increment = math.pi / 2
    msg.angle_max = (len(ranges) - 1) * msg.angle_increment
    msg.range_min, msg.range_max = 0.15, 8.0
    msg.ranges = ranges
    return msg


def xyz(cloud):
    return point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'])


def transform():
    tf = TransformStamped()
    tf.header.frame_id = 'odom'
    tf.child_frame_id = 'laser'
    tf.transform.rotation.w = 1.0
    return tf


class ScanCloudTest(unittest.TestCase):
    def test_filter_invalid_returns_and_do_not_require_intensity(self):
        msg = scan([1., 2., float('nan'), float('inf'), .1, 8.])
        msg.intensities = [9.]  # XYZ-only projection ignores short intensity arrays.
        result = module.project_scan(msg, 'laser')
        np.testing.assert_allclose(xyz(result), [[1, 0, 0], [0, 2, 0]], atol=1e-6)
        self.assertEqual([f.name for f in result.fields], ['x', 'y', 'z'])

    def test_rotation_translation_header_and_input_preservation(self):
        msg, tf = scan([2.]), transform()
        tf.transform.rotation.z = math.sin(math.pi/4)
        tf.transform.rotation.w = math.cos(math.pi/4)
        tf.transform.translation.x, tf.transform.translation.y = 1., 2.
        tf.transform.translation.z = .3
        result = module.project_scan(msg, 'odom', tf)
        np.testing.assert_allclose(xyz(result), [[1, 4, .3]], atol=1e-6)
        self.assertEqual(result.header.frame_id, 'odom')
        self.assertEqual(result.header.stamp, msg.header.stamp)
        self.assertEqual(msg.header.frame_id, 'laser')

    def test_transform_uses_full_3d_rotation(self):
        msg, tf = scan([float('inf'), 1.]), transform()
        tf.transform.rotation.x = math.sin(math.pi/4)
        tf.transform.rotation.w = math.cos(math.pi/4)
        np.testing.assert_allclose(
            xyz(module.project_scan(msg, 'odom', tf)), [[0, 0, 1]], atol=1e-6)

    def test_empty_scan(self):
        result = module.project_scan(scan([]), 'odom', transform())
        self.assertEqual(result.width, 0)
        self.assertEqual(result.header.frame_id, 'odom')

    def test_cannot_relabel_without_transform(self):
        with self.assertRaises(ValueError):
            module.project_scan(scan([1.]), 'odom')
        tf = transform()
        tf.child_frame_id = 'unrelated_laser'
        with self.assertRaises(ValueError):
            module.project_scan(scan([1.]), 'odom', tf)

    def test_bad_scan_metadata(self):
        for attr, value in [('angle_increment', float('nan')),
                            ('range_min', -1.), ('range_max', 0.)]:
            msg = scan([1.])
            setattr(msg, attr, value)
            with self.assertRaises(ValueError):
                module.project_scan(msg, 'laser')
        msg = scan([1.])
        msg.header.frame_id = ''
        with self.assertRaises(ValueError):
            module.project_scan(msg, 'laser')

    def test_invalid_tf(self):
        tf = transform()
        tf.transform.rotation.w = 0.
        with self.assertRaises(ValueError):
            module.project_scan(scan([1.]), 'odom', tf)
        tf = transform()
        tf.transform.translation.x = float('nan')
        with self.assertRaises(ValueError):
            module.project_scan(scan([1.]), 'odom', tf)


if __name__ == '__main__':
    unittest.main()
