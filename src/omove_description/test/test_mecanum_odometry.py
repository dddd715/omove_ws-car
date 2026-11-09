"""Run with: python3 -m unittest discover -s src/omove_description/test"""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from sensor_msgs.msg import JointState

spec = importlib.util.spec_from_file_location(
    'odometry', Path(__file__).parents[1] / 'omove_description/mecanum_odometry.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    sendTransform = publish


class OdometryTest(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            JOINTS=module.MecanumOdometry.JOINTS, radius=.076, lever=.38,
            odom_frame='odom', base_frame='base_footprint',
            x=0., y=0., yaw=0., previous_stamp=None,
            publisher=Sink(), broadcaster=Sink())

    def feed(self, stamp, velocities, names=None):
        msg = JointState()
        msg.header.stamp.sec = stamp
        msg.name = list(names or self.node.JOINTS)
        msg.velocity = [float(v) for v in velocities]
        module.MecanumOdometry.joint_callback(self.node, msg)

    def test_forward_and_tf(self):
        self.feed(1, [1, 1, 1, 1])
        self.feed(2, [1, 1, 1, 1])
        self.assertAlmostEqual(self.node.x, .076)
        odom = self.node.publisher.messages[-1]
        tf = self.node.broadcaster.messages[-1]
        self.assertEqual(tf.header, odom.header)
        self.assertEqual(tf.child_frame_id, 'base_footprint')
        self.assertEqual(tf.transform.translation.x, odom.pose.pose.position.x)

    def test_lateral_and_reordered_joints(self):
        self.feed(1, [-1, 1, 1, -1])
        self.feed(2, [-1, 1, 1, -1], list(reversed(self.node.JOINTS)))
        self.assertAlmostEqual(self.node.y, .076)
        self.assertAlmostEqual(self.node.x, 0.)

    def test_rotation(self):
        self.feed(1, [-1, 1, -1, 1])
        self.feed(2, [-1, 1, -1, 1])
        self.assertAlmostEqual(self.node.yaw, .2)

    def test_curved_motion(self):
        x, y, yaw = module.integrate_pose(0., 0., 0., 1., 0., 1., math.pi / 2)
        self.assertAlmostEqual(x, 1.)
        self.assertAlmostEqual(y, 1.)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_duplicate_and_time_reset(self):
        self.feed(1, [1]*4)
        self.feed(2, [1]*4)
        self.feed(2, [1]*4)
        self.assertEqual(len(self.node.publisher.messages), 2)
        self.feed(0, [1]*4)
        self.assertEqual(self.node.x, 0.)
        self.feed(1, [1]*4)
        self.assertAlmostEqual(self.node.x, .076)

    def test_incomplete_and_invalid_feedback(self):
        self.feed(1, [1])
        self.feed(2, [float('nan')]*4)
        self.assertEqual(self.node.publisher.messages, [])


if __name__ == '__main__':
    unittest.main()
