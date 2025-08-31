# Generic imports
import numpy as np
from dataclasses import dataclass
from math import ceil


@dataclass
class RoverPose:
  """Class to represent the pose (position/attitude) of the rover"""

  # Position in metres (x, y) coordinates
  position_m: list[float] = [0.0, 0.0]

  # Orientation in radians
  heading_rad: float = 0.0

  def copy(self):
    """Create a copy of the RoverPose instance"""
    return RoverPose(
      position_m=[self.position_m[0], self.position_m[1]],
      heading_rad=self.heading_rad,
    )


@dataclass
class RoverVelocity:
  """Class to represent the velocity of the rover"""

  # Linear velocity in metres per second
  linear_velocity_ms: float = 0.0

  # Angular velocity in radians per second
  angular_velocity_rads: float = 0.0


@dataclass
class RoverState:
  """Class to represent the state of the rover"""

  # Position/attitude of the rover
  pose: RoverPose

  # Velocity of the rover
  velocity: RoverVelocity


@dataclass
class RoverLimits:
  """Class to represent the limits of the rover"""

  # Maximum velocity of the rover in metres per second
  max_velocity_ms: float

  # Maximum acceleration of the rover in metres per second per second
  max_accel_mss: float

  # Maximum change of rate of heading in radians per second
  max_yaw_rate_rads: float

  # Maximum change of yaw rate in radians per second per second
  max_yaw_accel_radss: float


@dataclass
class DwaConfig:
  """Class to provide configuration parameters for the DWA planner"""

  # The resolution of the velocity search space in metres per second
  velocity_resolution_ms: float

  # The resolution of the yaw rate search space in metres per second
  yaw_rate_resolution_ms: float

  # The time horizon for simulating trajectories in seconds
  time_horizon_s: float

  # The time step for simulating trajectories in seconds
  time_step_s: float


class DwaPlanner:
  """Dynamic Window Approach (DWA) Planner for roveric trajectory planning

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

  def compute_trajectories(self, rover_state_in: RoverState) -> list:
    """Compute all possible trajectories given the current state of the rover

    Args:
        rover_state_in (RoverState): Current powe and velocity of the rover.

    Returns:
        list: A list of possible trajectories, each trajectory is a list of
            states.
    """
    # ---- CREATE DYNAMIC WINDOW ----
    # Create a dynamic window based on the rover's current state and limits
    # (i.e. a set of possible velocities and yaw rates)

    # Calculate the min and max velocities and yaw rates based on current
    # velocity and acceleration limits
    min_velocity_ms = max(
      0,
      rover_state_in.velocity.linear_velocity_ms
      - self.rover_limits.max_accel_mss * self.dwa_config.time_step_s,
    )
    max_velocity_ms = min(
      self.rover_limits.max_velocity_ms,
      rover_state_in.velocity.linear_velocity_ms
      + self.rover_limits.max_accel_mss * self.dwa_config.time_step_s,
    )
    min_yaw_rate_rads = max(
      -self.rover_limits.max_yaw_rate_rads,
      rover_state_in.velocity.angular_velocity_rads
      - self.rover_limits.max_yaw_accel_radss * self.dwa_config.time_step_s,
    )
    max_yaw_rate_rads = min(
      self.rover_limits.max_yaw_rate_rads,
      rover_state_in.velocity.angular_velocity_rads
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
      / self.dwa_config.yaw_rate_resolution_ms
    )
    possible_yaw_rates_rads = np.linspace(
      min_yaw_rate_rads,
      max_yaw_rate_rads,
      num_yaw_rates,
    )

    # ---- SIMULATE TRAJECTORIES ----
    # Simulate trajectories for each combination of velocity and yaw rate in the
    # dynamic window by applying a simple motion model over a fixed time horizon
    trajectories = [
      self._simulate_trajectory(rover_state_in, velocity_ms, yaw_rate_rads)
      for velocity_ms in possible_velocities_ms
      for yaw_rate_rads in possible_yaw_rates_rads
    ]

    # Return the list of simulated trajectories
    return trajectories

  def select_best_trajectory(self, trajectories, goal):
    """
    Select the best trajectory from a list of trajectories based on proximity to
    the goal.

    Args:
        trajectories (list): List of trajectories to evaluate.
        goal (list): Target position [x, y].

    Returns:
        list: The best trajectory (closest to the goal).
    """
    best_trajectory = None
    best_score = float('-inf')
    # Evaluate each trajectory and select the one with the highest score
    for trajectory in trajectories:
      score = self._evaluate_trajectory(trajectory, goal)
      if score > best_score:
        best_score = score
        best_trajectory = trajectory
    return best_trajectory

  def _simulate_trajectory(
    self,
    rover_state_in: RoverState,
    velocity_ms_in: float,
    yaw_rate_rads_in: float,
  ) -> list[RoverPose]:
    """Simulate a trajectory given an initial state, velocity and yaw rate.

    Args:
        rover_state_in (RoverState): Rover state to start the simulation from.
        velocity_ms (float): Rover velocity in metres per second.
        yaw_rate_rads (float): Rovers yaw rate in radians per second.

    Returns:
        list: Simulated trajectory as a list of rover poses.
    """
    # Simple motion model to simulate the rover's trajectory over the time

    # Copy the initial rover pose to avoid modifying the input
    rover_pose = rover_state_in.pose.copy()

    # Initialize simulation time to zero and set up trajectory as an empty list
    time_s = 0.0
    trajectory = []

    # Simulate for a fixed duration, recording the rover's position at each
    # time step
    while time_s < self.dwa_config.time_horizon_s:
      # Calculate the new pose based on the current pose, velocity and yaw rate
      # TODO: Check below auto-generated logic
      if abs(yaw_rate_rads_in) > 1e-6:
        # If there is a yaw rate, calculate the new position using a circular
        # arc model
        radius_m = velocity_ms_in / yaw_rate_rads_in
        cx = rover_pose.position_m[0] - radius_m * np.sin(
          rover_pose.heading_rad
        )
        cy = rover_pose.position_m[1] + radius_m * np.cos(
          rover_pose.heading_rad
        )

        rover_pose.heading_rad = (
          rover_pose.heading_rad
          + yaw_rate_rads_in * self.dwa_config.time_step_s
        )
        rover_pose.position_m[0] = cx + radius_m * np.sin(
          rover_pose.heading_rad
        )
        rover_pose.position_m[1] = cy - radius_m * np.cos(
          rover_pose.heading_rad
        )

      else:
        # If there is no yaw rate, the rover is moving straight and the heading
        # does not change
        rover_pose.position_m[0] = (
          rover_pose.position_m[0]
          + velocity_ms_in
          * np.cos(rover_pose.heading_rad)
          * self.dwa_config.time_step_s
        )
        rover_pose.position_m[1] = (
          rover_pose.position_m[1]
          + velocity_ms_in
          * np.sin(rover_pose.heading_rad)
          * self.dwa_config.time_step_s
        )

      # Append the updated position to the trajectory
      trajectory.append(rover_pose.copy())

      # Increment the simulatino time by the time step
      time_s += self.dwa_config.time_step_s

    # At end of simulation, return the generated trajectory
    return trajectory

  def _evaluate_trajectory(self, trajectory, goal):
    """
    Evaluate a trajectory based on its final position's distance to the goal.

    Args:
        trajectory (list): Trajectory to evaluate.
        goal (list): Target position [x, y].

    Returns:
        float: Negative distance to the goal (higher score is better).
    """
    final_position = trajectory[-1]
    # Calculate Euclidean distance to the goal
    distance = (
      (final_position[0] - goal[0]) ** 2 + (final_position[1] - goal[1]) ** 2
    ) ** 0.5
    return -distance  # Negative distance as score (closer is better)
