"""Exercise command limiting and watchdog without starting DDS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

from geometry_msgs.msg import Twist

spec = importlib.util.spec_from_file_location(
    'kinematics', Path(__file__).parents[1] / 'omove_description/mecanum_kinematics.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.commands = []
        self.node = SimpleNamespace(
            r=.076, Lx=.18, Ly=.20, timeout=.5, max_wheel_speed=12.,
            last_command=None, publish_wheels=self.commands.append,
            watchdog_clock=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now)))

    def command(self, x=0., y=0., z=0.):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, z
        module.MecanumKinematics.cmd_cb(self.node, msg)

    def test_silence_and_recovery(self):
        module.MecanumKinematics.watchdog_cb(self.node)
        self.assertEqual(self.commands[-1], [0.]*4)
        self.command(x=.1)
        count = len(self.commands)
        self.now += 499_000_000
        module.MecanumKinematics.watchdog_cb(self.node)
        self.assertEqual(len(self.commands), count)
        self.now += 1_000_000
        module.MecanumKinematics.watchdog_cb(self.node)
        self.assertEqual(self.commands[-1], [0.]*4)
        self.command(y=.1)
        self.assertLess(self.commands[-1][0], 0.)
        self.assertGreater(self.commands[-1][1], 0.)

    def test_uniform_limit_preserves_direction(self):
        self.command(x=2., y=1., z=1.)
        values = self.commands[-1]
        self.assertAlmostEqual(max(abs(v) for v in values), 12.)
        raw = [(2-1-.38)/.076, (2+1+.38)/.076,
               (2+1-.38)/.076, (2-1+.38)/.076]
        for value, expected in zip(values, raw):
            self.assertAlmostEqual(value / values[1], expected / raw[1])

    def test_invalid_commands_stop_immediately(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            self.command(x=.1)
            self.command(y=value)
            self.assertEqual(self.commands[-1], [0.]*4)
            self.assertIsNone(self.node.last_command)

    def test_overflow_stops(self):
        self.command(x=1e308, y=1e308, z=1e308)
        self.assertEqual(self.commands[-1], [0.]*4)


if __name__ == '__main__':
    unittest.main()
