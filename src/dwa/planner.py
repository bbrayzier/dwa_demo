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


@dataclass
class DwaObstacle:
  """Class to represent an obstacle in the environment for DWA planning"""

  # Position of the obstacle in metres (x, y) coordinates
  position_m: list[float]

  # Obstacle radius in metres
  radius_m: float


@dataclass
class DwaCostWeights:
  """Class to provide weighting factors to the DWA costing functions"""

  # Weighting factor for the heading cost, cost has arbitrary units so this
  # technically has units of 1/radians
  heading_cost_weight: float

  # Weighting factor for the velocity cost, cost has arbitrary units so this
  # technically has units of 1/(m/s)
  velocity_cost_weight: float

  # Weighting factor for the obstacle cost, cost has arbitrary units so this
  # technically has units of 1/metres
  obstacle_cost_weight: float


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

  # Cost weights for the different cost functions
  cost_weights: DwaCostWeights


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

    Evaluates each trajectory using a scoring function that considers
    heading, velocity and obstacle costs, and returns the trajectory with the
    lowest cost.

    Args:
        trajectories_in (list[RoverTrajectory]): List of possible rover
            trajectories.
        target_pos_m_in (list[float]): The target position [x, y].
        obstacles_in (list[DwaObstacle] | None, optional): List of obstacles in
            the environment. Defaults to None.

    Raises:
        ValueError: If no trajectories are provided or target position is
            invalid.

    Returns:
        RoverTrajectory: The best trajectory based on the scoring function.
    """
    # Sanity check inputs
    if len(trajectories_in) == 0:
      raise ValueError('No trajectories provided to select from')
    elif len(target_pos_m_in) < 2:
      raise ValueError('Target position must be a list of [x, y] coordinates')

    # Evaluate the cost of each trajectory
    trajectory_costs = [
      self._evaluate_trajectory(
        trajectory,
        target_pos_m_in,
        obstacles_in,
      )
      for trajectory in trajectories_in
    ]

    # Return the trajectory with the lowest cost
    return trajectories_in[np.argmin(trajectory_costs)]

  def _evaluate_trajectory(
    self,
    trajectory: RoverTrajectory,
    target_pos_m_in: list[float],
    obstacles_in: list[DwaObstacle] | None = None,
  ) -> float:
    """Evaluate a trajectory based on heading, velocity and obstacle costs

    Args:
        trajectory (RoverTrajectory): The trajectory to evaluate.
        target_pos_m_in (list[float]): The target position [x, y].
        obstacles_in (list[DwaObstacle] | None, optional): List of obstacles in
            the environment. Obstacle cost is skipped if no obstacles are
            provided. Defaults to None.

    Returns:
        float: The total cost of the trajectory (lower is better).
    """
    # Calculate individual costs:
    # - Heading cost: How well the trajectory aligns with the target
    # - Velocity cost: How fast the trajectory is (prefer faster)
    # - Obstacle cost: How close the trajectory comes to obstacles (prefer
    #   further away, infinite if collision, skip if no obstacles)
    heading_cost = self._calc_heading_cost(trajectory, target_pos_m_in)
    velocity_cost = self._calc_velocity_cost(trajectory)
    if obstacles_in is None or len(obstacles_in) == 0:
      obstacle_cost = 0.0
    else:
      obstacle_cost = self._calc_obstacle_cost(trajectory, obstacles_in)

    # If the obstacle cost is infinite, return infinite cost (invalid
    # trajectory)
    if obstacle_cost == float('inf'):
      return float('inf')

    # Combine costs into a single score (lower is better) and return it
    return heading_cost + velocity_cost + obstacle_cost

  def _calc_heading_cost(
    self, trajectory_in: RoverTrajectory, target_pos_m_in: list[float]
  ) -> float:
    """Calculate the heading cost of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.
        target_pos_m_in (list): The target position [x, y].

    Returns:
        float: The heading cost (lower is better).
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

    # Return the heading cost, scaled by the heading cost factor
    return heading_diff_rad * self.dwa_config.cost_weights.heading_cost_weight

  def _calc_velocity_cost(self, trajectory_in: RoverTrajectory) -> float:
    """Calculate the velocity cost of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.

    Returns:
        float: The velocity cost (lower is better).
    """
    # The velocity cost is simply the difference between the maximum velocity
    # and the trajectory's velocity, scaled by the velocity cost factor
    return (
      self.rover_limits.max_velocity_ms - trajectory_in.velocity_ms
    ) * self.dwa_config.cost_weights.velocity_cost_weight

  def _calc_obstacle_cost(
    self, trajectory_in: RoverTrajectory, obstacles_in: list[DwaObstacle]
  ) -> float:
    """Calculate the obstacle cost of a trajectory

    Args:
        trajectory_in (RoverTrajectory): The trajectory to evaluate.
        obstacles_in (list[DwaObstacle]): List of obstacles in the environment.

    Returns:
        float: The obstacle cost (lower is better, infinite if collision).
    """
    # Initialise the minimum distance to an obstacle as infinity
    min_distance_to_obstacle_m = float('inf')

    # Check each pose in the trajectory against each obstacle
    for pose in trajectory_in.poses:
      for obstacle in obstacles_in:
        # Calculate the Euclidean distance from the pose to the obstacle
        distance_m = (
          euclidean_distance(pose.position_m, obstacle.position_m)
          - obstacle.radius_m
        )

        # Update the minimum distance if this one is smaller
        if distance_m < min_distance_to_obstacle_m:
          min_distance_to_obstacle_m = distance_m

    # If the minimum distance is less than the obstacle margin, return infinite
    # cost (collision)
    if min_distance_to_obstacle_m < self.dwa_config.obstacle_margin_m:
      return float('inf')

    # Return the obstacle cost, scaled by the obstacle cost factor
    return (
      min_distance_to_obstacle_m
      * self.dwa_config.cost_weights.obstacle_cost_weight
    )
