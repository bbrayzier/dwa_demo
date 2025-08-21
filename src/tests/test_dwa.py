import unittest
from planner.dwa import DWA

class TestDWA(unittest.TestCase):

    def setUp(self):
        self.dwa = DWA()

    def test_compute_trajectory(self):
        # Test the compute_trajectory method with a sample state and velocity
        state = (0, 0, 0)  # x, y, theta
        velocity = (1, 0)  # linear velocity, angular velocity
        trajectories = self.dwa.compute_trajectory(state, velocity)
        self.assertIsInstance(trajectories, list)
        self.assertGreater(len(trajectories), 0)

    def test_select_best_trajectory(self):
        # Test the select_best_trajectory method with sample trajectories
        trajectories = [
            {'cost': 1.0, 'trajectory': [(0, 0), (1, 1)]},
            {'cost': 0.5, 'trajectory': [(0, 0), (2, 2)]},
            {'cost': 0.8, 'trajectory': [(0, 0), (1, 2)]}
        ]
        best_trajectory = self.dwa.select_best_trajectory(trajectories)
        self.assertEqual(best_trajectory['cost'], 0.5)

if __name__ == '__main__':
    unittest.main()