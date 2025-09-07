"""This file implements unit tests for the DWA planner.

Copyright (c) 2025 Ben Brayzier
"""

import pytest
from ..dwa.planner import DwaPlanner, Trajectory
from ..dwa.rover_data import RoverState, RoverConfig


def test_dwa_planner_initialization():
  config = RoverConfig()
  planner = DWAPlanner(config)
  assert planner.config == config


def test_dwa_planner_plan_returns_trajectory():
  config = RoverConfig()
  planner = DwaPlanner(config)
  state = RoverState(x=0, y=0, theta=0, v=0, omega=0)
  goal = (5, 5)
  obstacles = [(2, 2), (3, 3)]
  traj = planner.plan(state, goal, obstacles)
  assert isinstance(traj, Trajectory)
  assert hasattr(traj, 'path')
  assert isinstance(traj.path, list)
