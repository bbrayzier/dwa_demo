# Generic imports
import unittest

# Local imports
from .. import DwaPlanner


class TestDwaPlanner(unittest.TestCase):
  """Unit tests for the DwaPlanner class."""

  def setUp(self):
    """
    Set up the test environment.

    This method is called before each test. It initialises a DwaPlanner instance
    for use in the tests.
    """
    self.dwa_planner = DwaPlanner()

  def test_compute_trajectory(self):
    """
    Test the compute_trajectory method.

    Verifies that the method returns a list of trajectories and that the list is not empty
    when given a sample state and velocity.
    """
    state = (0, 0, 0)  # x, y, theta
    velocity = (1, 0)  # linear velocity, angular velocity
    trajectories = self.dwa_planner.compute_trajectories(state, velocity)
    self.assertIsInstance(trajectories, list)
    self.assertGreater(len(trajectories), 0)

  def test_select_best_trajectory(self):
    """
    Test the select_best_trajectory method.

    Checks that the method correctly selects the trajectory with the lowest cost
    from a list of sample trajectories.
    """
    trajectories = [
      {'cost': 1.0, 'trajectory': [(0, 0), (1, 1)]},
      {'cost': 0.5, 'trajectory': [(0, 0), (2, 2)]},
      {'cost': 0.8, 'trajectory': [(0, 0), (1, 2)]},
    ]
    best_trajectory = self.dwa_planner.select_best_trajectory(trajectories)
    self.assertEqual(best_trajectory['cost'], 0.5)


if __name__ == '__main__':
  # Run the unit tests
  unittest.main()
