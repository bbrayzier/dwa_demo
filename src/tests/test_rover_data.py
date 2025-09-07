"""This file implements unit tests for the rover data classes.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import numpy as np

# Local imports
from dwa.rover_data import RoverPose, RoverState, RoverTrajectory, RoverLimits

# ---- CONSTANTS ----
# Constants for testing RoverPose
TEST_POSE_POS_X_M = 1.0
TEST_POSE_POS_Y_M = 2.0
TEST_POSE_HEADING_RAD = 0.5

TEST_POSE_2_POS_X_M = 3.0
TEST_POSE_2_POS_Y_M = 4.0
TEST_POSE_2_HEADING_RAD = 1.0

# Constants for testing RoverState
TEST_STATE_VELOCITY_MS = 1.0
TEST_STATE_YAW_RATE_RADS = 0.1

# Constants for testing RoverTrajectory
TEST_TIME_HOZIRON_S = 10.0
TEST_TIME_STEP_S = 1.0

# Constants for testing RoverLimits
TEST_LIMITS_MAX_VELOCITY_MS = 1.5
TEST_LIMITS_MIN_VELOCITY_MS = 0.5
TEST_LIMITS_MAX_ACCEL_MSS = 0.2
TEST_LIMITS_MAX_YAW_RATE_RADS = 0.3
TEST_LIMITS_MAX_YAW_ACCEL_RADSS = 0.05


# ---- TEST FUNCTIONS ----
def test_rover_pose_init():
  """Test initialisation of RoverPose class"""
  test_pose = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  assert test_pose.position_m[0] == TEST_POSE_POS_X_M
  assert test_pose.position_m[1] == TEST_POSE_POS_Y_M
  assert test_pose.heading_rad == TEST_POSE_HEADING_RAD


def test_rover_state_init():
  """Test initialisation of RoverState class"""
  test_pose = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  test_state = RoverState(
    pose=test_pose,
    velocity_ms=TEST_STATE_VELOCITY_MS,
    yaw_rate_rads=TEST_STATE_YAW_RATE_RADS,
  )
  assert test_state.pose == test_pose
  assert test_state.velocity_ms == TEST_STATE_VELOCITY_MS
  assert test_state.yaw_rate_rads == TEST_STATE_YAW_RATE_RADS


def test_rover_trajectory_init():
  """Test initialisation of RoverTrajectory class"""
  # Create a trajectory with two poses
  test_pose_1 = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  test_pose_2 = RoverPose(
    position_m=[TEST_POSE_2_POS_X_M, TEST_POSE_2_POS_Y_M],
    heading_rad=TEST_POSE_2_HEADING_RAD,
  )
  test_trajectory = RoverTrajectory(
    poses=[test_pose_1, test_pose_2],
    velocity_ms=TEST_STATE_VELOCITY_MS,
    yaw_rate_rads=TEST_STATE_YAW_RATE_RADS,
  )

  # Check the trajectory values
  assert test_trajectory.poses[0] == test_pose_1
  assert test_trajectory.poses[1] == test_pose_2
  assert test_trajectory.velocity_ms == TEST_STATE_VELOCITY_MS
  assert test_trajectory.yaw_rate_rads == TEST_STATE_YAW_RATE_RADS


def test_rover_trajectory_generate():
  """Test the generate() method of the RoverTrajectory class"""
  # Define test inputs
  test_initial_pose = RoverPose(
    position_m=[TEST_POSE_POS_X_M, TEST_POSE_POS_Y_M],
    heading_rad=TEST_POSE_HEADING_RAD,
  )
  test_velocity_ms = TEST_STATE_VELOCITY_MS
  test_yaw_rate_rads = TEST_STATE_YAW_RATE_RADS
  test_time_horizon_s = TEST_TIME_HOZIRON_S
  test_time_step_s = TEST_TIME_STEP_S

  # First test a zero velocity and yaw rate produces a static trajectory
  static_trajectory = RoverTrajectory.generate(
    initial_rover_pose_in=test_initial_pose,
    velocity_ms_in=0.0,
    yaw_rate_rads_in=0.0,
    time_step_s_in=test_time_step_s,
    time_horizon_s_in=test_time_horizon_s,
  )
  assert len(static_trajectory.poses) == int(
    TEST_TIME_HOZIRON_S / TEST_TIME_STEP_S
  )
  for pose in static_trajectory.poses:
    assert pose == test_initial_pose
  assert static_trajectory.velocity_ms == 0.0
  assert static_trajectory.yaw_rate_rads == 0.0

  # Next test a non-zero velocity and zero yaw rate produces a straight trajectory
  straight_trajectory = RoverTrajectory.generate(
    initial_rover_pose_in=test_initial_pose,
    velocity_ms_in=test_velocity_ms,
    yaw_rate_rads_in=0.0,
    time_step_s_in=test_time_step_s,
    time_horizon_s_in=test_time_horizon_s,
  )
  assert len(straight_trajectory.poses) == int(
    TEST_TIME_HOZIRON_S / TEST_TIME_STEP_S
  )
  for idx, pose in enumerate(straight_trajectory.poses):
    # Determine the distance travelled since the start of the trajectory from
    # the velocity, the time step and the current index
    dist_travelled_m = test_velocity_ms * (test_time_step_s * (idx + 1))

    # Determine the expected X and Y positions from the initial position, the
    expected_x_m = test_initial_pose.position_m[0] + (
      dist_travelled_m * np.cos(test_initial_pose.heading_rad)
    )
    expected_y_m = test_initial_pose.position_m[1] + (
      dist_travelled_m * np.sin(test_initial_pose.heading_rad)
    )

    # The expected heading is simply the initial heading
    expected_heading_rad = test_initial_pose.heading_rad

    # Check the position and heading of each generated pose matches the expected
    # values
    assert np.isclose(pose.position_m[0], expected_x_m)
    assert np.isclose(pose.position_m[1], expected_y_m)
    assert np.isclose(pose.heading_rad, expected_heading_rad)

  # Check velocity and yaw rate of trajectory are set correctly
  assert straight_trajectory.velocity_ms == test_velocity_ms
  assert np.isclose(straight_trajectory.yaw_rate_rads, 0.0)

  # Finally test a non-zero velocity and yaw rate produces a moving trajectory
  test_trajectory = RoverTrajectory.generate(
    initial_rover_pose_in=test_initial_pose,
    velocity_ms_in=test_velocity_ms,
    yaw_rate_rads_in=test_yaw_rate_rads,
    time_step_s_in=test_time_step_s,
    time_horizon_s_in=test_time_horizon_s,
  )
  assert len(test_trajectory.poses) == int(
    TEST_TIME_HOZIRON_S / TEST_TIME_STEP_S
  )

  # Initialise a previous pose for comparison
  prev_pose = RoverPose()
  for idx, pose in enumerate(test_trajectory.poses):
    # Just check that the poses are changing over time, not their exact values
    if idx > 0:
      assert not np.isclose(pose.position_m[0], prev_pose.position_m[0])
      assert not np.isclose(pose.position_m[1], prev_pose.position_m[1])
      assert not np.isclose(pose.heading_rad, prev_pose.heading_rad)

    # Store the previous pose for the next iteration
    prev_pose = pose.copy()

  assert test_trajectory.velocity_ms == test_velocity_ms
  assert test_trajectory.yaw_rate_rads == test_yaw_rate_rads


def test_rover_limits_init():
  """Test initialisation of RoverLimits class"""
  test_limits = RoverLimits(
    min_velocity_ms=TEST_LIMITS_MIN_VELOCITY_MS,
    max_velocity_ms=TEST_LIMITS_MAX_VELOCITY_MS,
    max_accel_mss=TEST_LIMITS_MAX_ACCEL_MSS,
    max_yaw_rate_rads=TEST_LIMITS_MAX_YAW_RATE_RADS,
    max_yaw_accel_radss=TEST_LIMITS_MAX_YAW_ACCEL_RADSS,
  )
  assert test_limits.min_velocity_ms == TEST_LIMITS_MIN_VELOCITY_MS
  assert test_limits.max_velocity_ms == TEST_LIMITS_MAX_VELOCITY_MS
  assert test_limits.max_accel_mss == TEST_LIMITS_MAX_ACCEL_MSS
  assert test_limits.max_yaw_rate_rads == TEST_LIMITS_MAX_YAW_RATE_RADS
  assert test_limits.max_yaw_accel_radss == TEST_LIMITS_MAX_YAW_ACCEL_RADSS
