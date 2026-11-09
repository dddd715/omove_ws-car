"""Check the physical roller axes against the X-layout velocity convention."""
import math
from pathlib import Path
import unittest

import xacro


class RollerModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = xacro.process_file(str(
            Path(__file__).parents[1] / 'urdf/omove.urdf.xacro'))
        cls.joints = {j.getAttribute('name'): j for j in
                      cls.document.getElementsByTagName('joint')
                      if j.hasAttribute('type')}

    def test_contact_axes_match_inverse_kinematics(self):
        # At the bottom of the wheel, the roller axis constrains vx - hand*vy.
        for corner, hand in [('front_left', 1), ('front_right', -1),
                             ('rear_left', -1), ('rear_right', 1)]:
            joint = self.joints[f'{corner}_roller_3_joint']
            _, pitch, yaw = map(float, joint.getElementsByTagName('origin')[0]
                                .getAttribute('rpy').split())
            wheel_axis = (math.cos(yaw)*math.sin(pitch),
                          math.sin(yaw)*math.sin(pitch), math.cos(pitch))
            # Wheel frame is rotated -pi/2 about X relative to the chassis.
            chassis_axis = (wheel_axis[0], wheel_axis[2], -wheel_axis[1])
            self.assertAlmostEqual(chassis_axis[1]/chassis_axis[0], -hand)
            self.assertAlmostEqual(chassis_axis[2], 0.)
            dynamics = joint.getElementsByTagName('dynamics')[0]
            self.assertEqual(float(dynamics.getAttribute('friction')), 0.)

    def test_rollers_have_feedback_but_no_motor_commands(self):
        control = self.document.getElementsByTagName('ros2_control')[0]
        passive = [j for j in control.getElementsByTagName('joint')
                   if '_roller_' in j.getAttribute('name')]
        self.assertEqual(len(passive), 48)
        for joint in passive:
            self.assertEqual(len(joint.getElementsByTagName('command_interface')), 0)
            states = {s.getAttribute('name') for s in
                      joint.getElementsByTagName('state_interface')}
            self.assertEqual(states, {'position', 'velocity'})

    def test_contact_and_mass_geometry(self):
        for link in self.document.getElementsByTagName('link'):
            name = link.getAttribute('name')
            if '_roller_' in name:
                mass = float(link.getElementsByTagName('mass')[0].getAttribute('value'))
                self.assertGreater(mass, 0.)
                inertia = link.getElementsByTagName('inertia')[0]
                moments = [float(inertia.getAttribute(k)) for k in ('ixx','iyy','izz')]
                self.assertTrue(all(v > 0 for v in moments))
                self.assertLessEqual(max(moments), sum(moments)-max(moments))
            elif name.endswith('_wheel'):
                radius = float(link.getElementsByTagName('collision')[0]
                               .getElementsByTagName('cylinder')[0].getAttribute('radius'))
                self.assertLess(radius, .061)  # Hub must not obstruct the rollers.


if __name__ == '__main__':
    unittest.main()
