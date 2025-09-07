"""This file implements unit tests for the DWA planner.

Copyright (c) 2025 Ben Brayzier
"""

# Local imports
from ..dwa import (
  RoverLimits,
  RoverPose,
  RoverState,
  DwaPlanner,
  DwaObstacle,
  DwaConfig,
  DwaCostWeights,
)
from .test_rover_data import (
  TEST_LIMITS_MAX_VELOCITY_MS,
  TEST_LIMITS_MIN_VELOCITY_MS,
  TEST_LIMITS_MAX_ACCEL_MSS,
  TEST_LIMITS_MAX_YAW_RATE_RADS,
  TEST_LIMITS_MAX_YAW_ACCEL_RADSS,
  TEST_POSE_POS_X_M,
  TEST_POSE_POS_Y_M,
  TEST_POSE_HEADING_RAD,
  TEST_STATE_VELOCITY_MS,
  TEST_STATE_YAW_RATE_RADS,
)


# ---- CONSTANTS ----
# Constants for testing obstacles
TEST_OBSTACLE_X_M = 1.0
TEST_OBSTACLE_Y_M = 2.0
TEST_OBSTACLE_RADIUS_M = 0.5

# Constants for testing DWA weights
TEST_HEADING_COST_WEIGHT = 1.0
TEST_VELOCITY_COST_WEIGHT = 10.0
TEST_OBSTACLE_COST_WEIGHT = 100.0

# Constants for testing DWA config
TEST_CONFIG_VELOCITY_RES_MS = 0.1
TEST_CONFIG_YAW_RATE_RES_RADS = 0.1
TEST_CONFIG_TIME_HORIZON_S = 3.0
TEST_CONFIG_TIME_STEP_S = 0.1
TEST_CONFIG_OBSTACLE_MARGIN_M = 0.2

# Constants for testing DWA planner
TEST_PLANNER_TARGET_POS_M = [5.0, -10.0]


# ---- TEST FUNCTIONS ----
def test_dwa_obstacle_init():
  """Test initialisation of DwaObstacle class"""
  test_obstacle = DwaObstacle(
    position_m=[TEST_OBSTACLE_X_M, TEST_OBSTACLE_Y_M],
    radius_m=TEST_OBSTACLE_RADIUS_M,
  )
  assert test_obstacle.position_m[0] == TEST_OBSTACLE_X_M
  assert test_obstacle.position_m[1] == TEST_OBSTACLE_Y_M
  assert test_obstacle.radius_m == TEST_OBSTACLE_RADIUS_M


def test_dwa_cost_weights_init():
  """Test initialisation of DwaCostWeights class"""
  test_weights = DwaCostWeights(
    heading_cost_weight=TEST_HEADING_COST_WEIGHT,
    velocity_cost_weight=TEST_VELOCITY_COST_WEIGHT,
    obstacle_cost_weight=TEST_OBSTACLE_COST_WEIGHT,
  )
  assert test_weights.heading_cost_weight == TEST_HEADING_COST_WEIGHT
  assert test_weights.velocity_cost_weight == TEST_VELOCITY_COST_WEIGHT
  assert test_weights.obstacle_cost_weight == TEST_OBSTACLE_COST_WEIGHT


def test_dwa_config_init():
  """Test initialisation of DwaConfig class"""
  test_weights = DwaCostWeights(
    heading_cost_weight=TEST_HEADING_COST_WEIGHT,
    velocity_cost_weight=TEST_VELOCITY_COST_WEIGHT,
    obstacle_cost_weight=TEST_OBSTACLE_COST_WEIGHT,
  )
  test_config = DwaConfig(
    velocity_resolution_ms=TEST_CONFIG_VELOCITY_RES_MS,
    yaw_rate_resolution_rads=TEST_CONFIG_YAW_RATE_RES_RADS,
    time_horizon_s=TEST_CONFIG_TIME_HORIZON_S,
    time_step_s=TEST_CONFIG_TIME_STEP_S,
    obstacle_margin_m=TEST_CONFIG_OBSTACLE_MARGIN_M,
    cost_weights=test_weights,
  )
  assert test_config.velocity_resolution_ms == TEST_CONFIG_VELOCITY_RES_MS
  assert test_config.yaw_rate_resolution_rads == TEST_CONFIG_YAW_RATE_RES_RADS
  assert test_config.time_horizon_s == TEST_CONFIG_TIME_HORIZON_S
  assert test_config.time_step_s == TEST_CONFIG_TIME_STEP_S
  assert test_config.obstacle_margin_m == TEST_CONFIG_OBSTACLE_MARGIN_M
  assert test_config.cost_weights == test_weights


def test_dwa_planner_init():
  """Test initialisation of DwaPlanner class"""
  # Create a config and rover limits for the planner
  test_weights = DwaCostWeights(
    heading_cost_weight=TEST_HEADING_COST_WEIGHT,
    velocity_cost_weight=TEST_VELOCITY_COST_WEIGHT,
    obstacle_cost_weight=TEST_OBSTACLE_COST_WEIGHT,
  )
  test_config = DwaConfig(
    velocity_resolution_ms=TEST_CONFIG_VELOCITY_RES_MS,
    yaw_rate_resolution_rads=TEST_CONFIG_YAW_RATE_RES_RADS,
    time_horizon_s=TEST_CONFIG_TIME_HORIZON_S,
    time_step_s=TEST_CONFIG_TIME_STEP_S,
    obstacle_margin_m=TEST_CONFIG_OBSTACLE_MARGIN_M,
    cost_weights=test_weights,
  )
  test_rover_limits = RoverLimits(
    min_velocity_ms=TEST_LIMITS_MIN_VELOCITY_MS,
    max_velocity_ms=TEST_LIMITS_MAX_VELOCITY_MS,
    max_accel_mss=TEST_LIMITS_MAX_ACCEL_MSS,
    max_yaw_rate_rads=TEST_LIMITS_MAX_YAW_RATE_RADS,
    max_yaw_accel_radss=TEST_LIMITS_MAX_YAW_ACCEL_RADSS,
  )

  # Create the planner
  test_planner = DwaPlanner(
    dwa_config_in=test_config,
    rover_limits_in=test_rover_limits,
  )
  assert test_planner.dwa_config == test_config
  assert test_planner.rover_limits == test_rover_limits


def test_dwa_planner_compute_trajectories():
  """Test the compute_trajectories() method of the DwaPlanner class"""
  # Create a config and rover limits for the planner
  test_weights = DwaCostWeights(
    heading_cost_weight=TEST_HEADING_COST_WEIGHT,
    velocity_cost_weight=TEST_VELOCITY_COST_WEIGHT,
    obstacle_cost_weight=TEST_OBSTACLE_COST_WEIGHT,
  )
  test_config = DwaConfig(
    velocity_resolution_ms=TEST_CONFIG_VELOCITY_RES_MS,
    yaw_rate_resolution_rads=TEST_CONFIG_YAW_RATE_RES_RADS,
    time_horizon_s=TEST_CONFIG_TIME_HORIZON_S,
    time_step_s=TEST_CONFIG_TIME_STEP_S,
    obstacle_margin_m=TEST_CONFIG_OBSTACLE_MARGIN_M,
    cost_weights=test_weights,
  )
  test_rover_limits = RoverLimits(
    min_velocity_ms=TEST_LIMITS_MIN_VELOCITY_MS,
    max_velocity_ms=TEST_LIMITS_MAX_VELOCITY_MS,
    max_accel_mss=TEST_LIMITS_MAX_ACCEL_MSS,
    max_yaw_rate_rads=TEST_LIMITS_MAX_YAW_RATE_RADS,
    max_yaw_accel_radss=TEST_LIMITS_MAX_YAW_ACCEL_RADSS,
  )

  # Create the planner
  test_planner = DwaPlanner(
    dwa_config_in=test_config,
    rover_limits_in=test_rover_limits,
  )

  # Create an initial rover state
  test_rover_pose = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  test_rover_state = RoverState(
    pose=test_rover_pose,
    velocity_ms=TEST_STATE_VELOCITY_MS,
    yaw_rate_rads=TEST_STATE_YAW_RATE_RADS,
  )

  # Compute trajectories
  test_trajectories = test_planner.compute_trajectories(
    rover_state_in=test_rover_state
  )

  # Check that some trajectories were generated
  assert len(test_trajectories) > 0

  # Check that the trajectories are within the rover limits
  assert all(
    traj.velocity_ms >= test_planner.rover_limits.min_velocity_ms
    and traj.velocity_ms <= test_planner.rover_limits.max_velocity_ms
    for traj in test_trajectories
  )
  assert all(
    traj.yaw_rate_rads >= -test_planner.rover_limits.max_yaw_rate_rads
    and traj.yaw_rate_rads <= test_planner.rover_limits.max_yaw_rate_rads
    for traj in test_trajectories
  )


def test_dwa_planner_select_best_trajectory():
  """Test the select_best_trajectory() method of the DwaPlanner class"""
  # Create a config and rover limits for the planner
  test_weights = DwaCostWeights(
    heading_cost_weight=TEST_HEADING_COST_WEIGHT,
    velocity_cost_weight=TEST_VELOCITY_COST_WEIGHT,
    obstacle_cost_weight=TEST_OBSTACLE_COST_WEIGHT,
  )
  test_config = DwaConfig(
    velocity_resolution_ms=TEST_CONFIG_VELOCITY_RES_MS,
    yaw_rate_resolution_rads=TEST_CONFIG_YAW_RATE_RES_RADS,
    time_horizon_s=TEST_CONFIG_TIME_HORIZON_S,
    time_step_s=TEST_CONFIG_TIME_STEP_S,
    obstacle_margin_m=TEST_CONFIG_OBSTACLE_MARGIN_M,
    cost_weights=test_weights,
  )
  test_rover_limits = RoverLimits(
    min_velocity_ms=TEST_LIMITS_MIN_VELOCITY_MS,
    max_velocity_ms=TEST_LIMITS_MAX_VELOCITY_MS,
    max_accel_mss=TEST_LIMITS_MAX_ACCEL_MSS,
    max_yaw_rate_rads=TEST_LIMITS_MAX_YAW_RATE_RADS,
    max_yaw_accel_radss=TEST_LIMITS_MAX_YAW_ACCEL_RADSS,
  )

  # Create the planner
  test_planner = DwaPlanner(
    dwa_config_in=test_config,
    rover_limits_in=test_rover_limits,
  )

  # Create an initial rover state
  test_rover_pose = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  test_rover_state = RoverState(
    pose=test_rover_pose,
    velocity_ms=TEST_STATE_VELOCITY_MS,
    yaw_rate_rads=TEST_STATE_YAW_RATE_RADS,
  )

  # Compute trajectories
  test_trajectories = test_planner.compute_trajectories(
    rover_state_in=test_rover_state
  )

  # Select the best trajectory without any obstacles
  best_trajectory = test_planner.select_best_trajectory(
    trajectories_in=test_trajectories,
    target_pos_m_in=TEST_PLANNER_TARGET_POS_M,
  )

  # Check that a trajectory was selected
  assert best_trajectory is not None

  # Create a list of obstacles
  test_obstacles = [
    DwaObstacle(
      position_m=[TEST_OBSTACLE_X_M, TEST_OBSTACLE_Y_M],
      radius_m=TEST_OBSTACLE_RADIUS_M,
    )
  ]

  # Select the best trajectory with obstacles
  best_trajectory = test_planner.select_best_trajectory(
    trajectories_in=test_trajectories,
    target_pos_m_in=TEST_PLANNER_TARGET_POS_M,
    obstacles_in=test_obstacles,
  )

  # Check that a trajectory was selected
  assert best_trajectory is not None
