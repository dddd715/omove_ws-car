import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('validation',
    Path(__file__).parents[1] / 'scripts/validate_navigation.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ClearanceTest(unittest.TestCase):
    def test_obstacle_and_wall_overlap(self):
        self.assertLess(module.footprint_clearance((0, 0, 0)), 0.)
        self.assertLess(module.footprint_clearance((2.9, 0, 0)), 0.)
        self.assertGreater(module.footprint_clearance((1.5, 0, 0)), 0.)

    def test_rotation_changes_clearance(self):
        self.assertAlmostEqual(module.footprint_clearance((.6, 0, 0)), .08)
        self.assertAlmostEqual(module.footprint_clearance((.6, 0, math.pi/2)), .12)


if __name__ == '__main__':
    unittest.main()
