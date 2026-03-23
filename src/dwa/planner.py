"""This file implements a Dynamic Window Approach (DWA) planner for rover
trajectory planning and related data classes.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import numpy as np
from dataclasses import dataclass
from math import ceil

# Local imports
from .rover_data import RoverState, RoverTrajectory, RoverLimits
from ..util import wrap_to_pi, euclidean_distance


# ---- CONSTANTS ----
# Define the length and resolution of the arc used for obstacle scanning, this
# is used to project forwards to check for obstacles along the rover trajectory
OBSTACLE_SCAN_ARC_LENGTH_M = 10.0
OBSTACLE_SCAN_ARC_RESOLUTION_M = 0.05


@dataclass
class DwaObstacle:
  """Class to represent an obstacle in the environment for DWA planning"""

  # Position of the obstacle in metres (x, y) coordinates
  position_m: list[float]

  # Obstacle radius in metres
  radius_m: float


@dataclass
class DwaWeights:
  """Class to provide weighting factors to the DWA objective function"""

  # Objective function weighting factor for the heading, the objective function
  # is unitless so this technically has units of 1/radians
  heading_weight: float

  # Objective function weighting factor for the velocity, the objective function
  # is unitless so this technically has units of 1/(metres/second)
  velocity_weight: float

  # Objective function weighting factor for the obstacle clearance, the
  # objective function is unitless so this technically has units of 1/metres
  obstacle_weight: float


@dataclass
class DwaConfig:
  """Class to provide configuration parameters for the DWA planner"""

  # The resolution of the velocity search space in metres per second
  velocity_resolution_ms: float

  # The resolution of the yaw rate search space in radians per second
  yaw_rate_resolution_rads: float

  # The time horizon for simulating trajectories in seconds
  time_horizon_s: float

  # The time step for simulating trajectories in seconds
  time_step_s: float

  # The minimum distance to obstacles in metres, trajectories that come closer
  # than this will be considered invalid
  obstacle_margin_m: float

  # Weighting factors for the elements of the DWA objective function, used to
  # score trajectories
  weight_factors: DwaWeights


class DwaPlanner:
  """Dynamic Window Approach (DWA) Planner for rover trajectory planning

  This class provides methods to compute possible trajectories for a rover
  using the DWA algorithm and select the best trajectory based on a scoring
  function.
  """

  def __init__(
    self,
    dwa_config_in: DwaConfig,
    rover_limits_in: RoverLimits,
  ):
    """Initialise the DWA planner with rover constraints

    Args:
        dwa_config_in (DwaConfig): Configuration parameters of the DWA planner.
        rover_limits_in (RoverLimits): Rover limits for velocity, acceleration
            and yaw rate.
    """
    self.dwa_config = dwa_config_in
    self.rover_limits = rover_limits_in

    self.reset_best_scores()

  def reset_best_scores(self):
    """Reset the best scores for trajectory evaluation"""
    self.best_score = -float('inf')
    self.best_heading_score = -float('inf')
    self.best_velocity_score = -float('inf')
    self.best_obstacle_score = -float('inf')

  def compute_trajectories(
    self, rover_state_in: RoverState
  ) -> list[RoverTrajectory]:
    """Compute all possible trajectories given the current state of the rover

    Args:
        rover_state_in (RoverState): Current powe and velocity of the rover.

    Returns:
        list[RoverTrajectory]: A list of possible rover trajectories.
    """
    # ---- CREATE DYNAMIC WINDOW ----
    # Create a dynamic window based on the rover's current state and limits
    # (i.e. a set of possible velocities and yaw rates)

    # Calculate the min and max velocities and yaw rates based on current
    # velocity and acceleration limits
    min_velocity_ms = max(
      self.rover_limits.min_velocity_ms,
      rover_state_in.velocity_ms
      - self.rover_limits.max_accel_mss * self.dwa_config.time_step_s,
    )
    max_velocity_ms = min(
      self.rover_limits.max_velocity_ms,
      rover_state_in.velocity_ms
      + self.rover_limits.max_accel_mss * self.dwa_config.time_step_s,
    )
    min_yaw_rate_rads = max(
      -self.rover_limits.max_yaw_rate_rads,
      rover_state_in.yaw_rate_rads
      - self.rover_limits.max_yaw_accel_radss * self.dwa_config.time_step_s,
    )
    max_yaw_rate_rads = min(
      self.rover_limits.max_yaw_rate_rads,
      rover_state_in.yaw_rate_rads
      + self.rover_limits.max_yaw_accel_radss * self.dwa_config.time_step_s,
    )

    # Generate the set of possible velocities and yaw rates within the dynamic
    # window based on the resolutions. Resolution does not need to be used
    # exactly, it's more a rough guide so first detemine the number of discrete
    # values to generate, then use linspace to create the arrays of possible
    # values.
    num_velocities = ceil(
      (max_velocity_ms - min_velocity_ms)
      / self.dwa_config.velocity_resolution_ms
    )
    possible_velocities_ms = np.linspace(
      min_velocity_ms,
      max_velocity_ms,
      num_velocities,
    )
    num_yaw_rates = ceil(
      (max_yaw_rate_rads - min_yaw_rate_rads)
      / self.dwa_config.yaw_rate_resolution_rads
    )
    possible_yaw_rates_rads = np.linspace(
      min_yaw_rate_rads,
      max_yaw_rate_rads,
      num_yaw_rates,
    )

    # ---- SIMULATE TRAJECTORIES ----
    # Simulate trajectories for each combination of velocity and yaw rate in the
    # dynamic window by applying a simple motion model over a fixed time
    # horizon, returning the generated list of trajectories
    return [
      RoverTrajectory.generate(
        initial_rover_pose_in=rover_state_in.pose,
        velocity_ms_in=velocity_ms,
        yaw_rate_rads_in=yaw_rate_rads,
        time_step_s_in=self.dwa_config.time_step_s,
        time_horizon_s_in=self.dwa_config.time_horizon_s,
      )
      for velocity_ms in possible_velocities_ms
      for yaw_rate_rads in possible_yaw_rates_rads
    ]

  def select_best_trajectory(
    self,
    trajectories_in: list[RoverTrajectory],
    target_pos_m_in: list[float],
    obstacles_in: list[DwaObstacle] | None = None,
  ) -> RoverTrajectory:
    """Select the best trajectory from a list of possible trajectories

    Evaluates each trajectory using an objective function that considers
    heading, velocity and obstacle clearance, and returns the trajectory with
    the highest score.

    Args:
        trajectories_in (list[RoverTrajectory]): List of possible rover
            trajectories.
        target_pos_m_in (list[float]): The target position [x, y].
        obstacles_in (list[DwaObstacle] | None, optional): List of obstacles in
            the environment. Defaults to None.

    Raises:
        ValueError: If no trajectories are provided or target position is
            invalid.
        RuntimeError: If no valid trajectories are found.

    Returns:
        RoverTrajectory: The best trajectory based on the scoring function.
    """
    # Sanity check inputs
    if len(trajectories_in) == 0:
      raise ValueError('No trajectories provided to select from')
    elif len(target_pos_m_in) < 2:
      raise ValueError('Target position must be a list of [x, y] coordinates')

    # Reset the best scores before evaluating trajectories
    self.reset_best_scores()

    # Evaluate the score of each trajectory
    for trajectory in trajectories_in:
      trajectory.score = self._evaluate_trajectory(
        trajectory,
        target_pos_m_in,
        obstacles_in,
      )

    # Print the best score and its components for debugging/analysis purposes
    print(
      f'Best trajectory score: {self.best_score:.2f} '
      f'(Heading: {self.best_heading_score:.2f}, '
      f'Velocity: {self.best_velocity_score:.2f}, '
      f'Obstacle: {self.best_obstacle_score:.2f})'
    )

    # Get the scores of all trajectories in a separate list
    trajectory_scores = [trajectory.score for trajectory in trajectories_in]

    # Sanity check a valid trajectory was found (i.e. at least one trajectory
    # has a positive score)
    if not any(score > 0.0 for score in trajectory_scores):
      raise RuntimeError(
        'No valid trajectories found, all trajectories have a negative score'
      )

    # Return the trajectory with the highest score
    return trajectories_in[np.argmax(trajectory_scores)]

  def _evaluate_trajectory(
    self,
    trajectory: RoverTrajectory,
    target_pos_m_in: list[float],
    obstacles_in: list[DwaObstacle] | None = None,
  ) -> float:
    """Evaluates a trajectory based on heading, velocity and obstacle clearance

    This calculates the objective function of the DWA algorithm.

    Args:
        trajectory (RoverTrajectory): The trajectory to evaluate.
        target_pos_m_in (list[float]): The target position [x, y].
        obstacles_in (list[DwaObstacle] | None, optional): List of obstacles in
            the environment. Obstacle clearance is skipped if no obstacles are
            provided. Defaults to None.

    Returns:
        float: The total score of the trajectory (higher is better).
    """
    # Calculate scores for each component of the objective function:
    # - Heading score: How well the trajectory aligns with the target
    # - Velocity score: How fast the trajectory is (prefer faster)
    # - Obstacle score: How close the trajectory comes to obstacles (prefer
    #   further away, negative if collision, skip if no obstacles)
    heading_score = self._calc_heading_score(trajectory, target_pos_m_in)
    velocity_score = self._calc_velocity_score(trajectory)
    if obstacles_in is None or len(obstacles_in) == 0:
      obstacle_score = 0.0
    else:
      obstacle_score = self._calc_obstacle_score(trajectory, obstacles_in)

    # If the obstacle score is negative, return negative infinity (invalid
    # trajectory)
    if obstacle_score < 0:
      return -float('inf')

    # Combine the elements of the objective function into a single score (higher
    # is better)
    total_score = heading_score + velocity_score + obstacle_score

    # If this is the best score we've seen, store it and the components of the
    # score for debugging/analysis purposes
    if total_score > self.best_score:
      self.best_score = total_score
      self.best_heading_score = heading_score
      self.best_velocity_score = velocity_score
      self.best_obstacle_score = obstacle_score

    return total_score

  def _calc_heading_score(
    self, trajectory_in: RoverTrajectory, target_pos_m_in: list[float]
  ) -> float:
    """Calculate the heading score of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.
        target_pos_m_in (list): The target position [x, y].

    Returns:
        float: The heading score (higher is better).
    """
    # Get the final pose of the trajectory
    final_pose = trajectory_in.poses[-1]

    # Calculate the angle to the target from the final position
    angle_to_target_rad = np.arctan2(
      target_pos_m_in[1] - final_pose.position_m[1],
      target_pos_m_in[0] - final_pose.position_m[0],
    )

    # Calculate the difference between the rover's heading and the angle to
    # the target - wrap to [-pi, pi] and take the absolute value to get the
    # smallest angle difference
    heading_diff_rad = abs(
      wrap_to_pi(final_pose.heading_rad - angle_to_target_rad)
    )

    # Return the heading score, this is calculated as 180 degrees minus the
    # heading difference, scaled by the heading weight factor
    return (
      np.pi - heading_diff_rad
    ) * self.dwa_config.weight_factors.heading_weight

  def _calc_velocity_score(self, trajectory_in: RoverTrajectory) -> float:
    """Calculate the velocity score of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.

    Returns:
        float: The velocity score (higher is better).
    """
    # The velocity score is simply the trajectory's velocity, multiplied by the
    # velocity weight factor
    return (
      trajectory_in.velocity_ms * self.dwa_config.weight_factors.velocity_weight
    )

  def _calc_obstacle_score(
    self, trajectory_in: RoverTrajectory, obstacles_in: list[DwaObstacle]
  ) -> float:
    """Calculate the obstacle clearance score of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.
        obstacles_in (list[DwaObstacle]): List of obstacles in the environment.

    Returns:
        float: The obstacle clearance score (higher is better, negative if
            collision).
    """
    # Initialise the minimum distance to an obstacle as infinity
    min_distance_to_obstacle_m = float('inf')

    # Create a generic arc along this trajectory (independent of the speed and
    # yaw rate) to check for obstacles along
    arc = RoverTrajectory.create_arc(
      initial_rover_pose_in=trajectory_in.poses[0],
      curvature_radm_in=trajectory_in.get_curvature(),
      arc_resolution_m_in=OBSTACLE_SCAN_ARC_RESOLUTION_M,
      num_steps_in=int(
        OBSTACLE_SCAN_ARC_LENGTH_M / OBSTACLE_SCAN_ARC_RESOLUTION_M
      ),
    )

    # Check each pose in the arc against each obstacle
    for pose in arc:
      for obstacle in obstacles_in:
        # Calculate the Euclidean distance from the pose to the obstacle, note
        # this can be negative if the pose is within the obstacle radius
        distance_m = (
          euclidean_distance(pose.position_m, obstacle.position_m)
          - obstacle.radius_m
        )

        # Update the minimum distance if this one is smaller
        if distance_m < min_distance_to_obstacle_m:
          min_distance_to_obstacle_m = distance_m

    # Subtract the obstacle margin from the minimum distance to get the
    # effective distance to the obstacle, this means that trajectories that come
    # within the obstacle margin will be considered as a collision
    min_distance_to_obstacle_m -= self.dwa_config.obstacle_margin_m

    # Return the obstacle clearance score, scaled by the obstacle weight factor
    return (
      min_distance_to_obstacle_m
      * self.dwa_config.weight_factors.obstacle_weight
    )
