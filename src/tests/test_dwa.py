# Generic imports
import pytest

# Local imports
from ..dwa import DwaPlanner


@pytest.fixture
def dwa_planner():
  """Fixture to provide a DwaPlanner instance for tests."""
  return DwaPlanner()


def test_compute_trajectories_returns_list_and_not_empty(dwa_planner):
  """
  Test that compute_trajectories returns a non-empty list for a sample state and velocity.
  """
  state = (0, 0, 0)  # x, y, theta
  velocity = (1, 0)  # linear velocity, angular velocity
  trajectories = dwa_planner.compute_trajectories(state, velocity)
  assert isinstance(trajectories, list)
  assert len(trajectories) > 0


def test_select_best_trajectory_returns_lowest_cost(dwa_planner):
  """
  Test that select_best_trajectory returns the trajectory with the lowest cost.
  """
  trajectories = [
    {'cost': 1.0, 'trajectory': [(0, 0), (1, 1)]},
    {'cost': 0.5, 'trajectory': [(0, 0), (2, 2)]},
    {'cost': 0.8, 'trajectory': [(0, 0), (1, 2)]},
  ]
  best_trajectory = dwa_planner.select_best_trajectory(trajectories)
  assert best_trajectory['cost']
