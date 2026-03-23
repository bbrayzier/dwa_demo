"""This file implements data classes to represent the rover state, pose,
trajectory and limits.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import numpy as np
from dataclasses import dataclass, field

# Local imports
from ..util import wrap_to_pi

# ---- CONSTANTS ----
# Define the minimum curvature that is considered a curved trajectory
MIN_CURVATURE_RADM = 1e-6


# ---- CLASSES ----
@dataclass
class RoverPose:
  """Class to represent the pose (position/attitude) of the rover"""

  # Position in metres (x, y) coordinates
  position_m: list[float] = field(default_factory=[0.0, 0.0].copy)

  # Orientation in radians
  heading_rad: float = 0.0

  def copy(self):
    """A method to generate a copy of the RoverPose instance"""
    return RoverPose(
      position_m=self.position_m.copy(),
      heading_rad=self.heading_rad,
    )

  def __eq__(self, other: object) -> bool:
    """Override equality operator for easy comparison of RoverPose objects"""
    return bool(
      isinstance(other, RoverPose)
      and np.allclose(self.position_m, other.position_m)
      and np.isclose(self.heading_rad, other.heading_rad)
    )


@dataclass
class RoverState:
  """Class to represent the state of the rover"""

  # Position/attitude of the rover
  pose: RoverPose

  # Rover velocity in metres per second
  velocity_ms: float = 0.0

  # Rate of change of rover heading in radians per second
  yaw_rate_rads: float = 0.0


@dataclass
class RoverTrajectory:
  """Class to represent a rover trajectory"""

  # A list of rover poses representing the trajectory
  poses: list[RoverPose]

  # Rover velocity in metres per second
  velocity_ms: float = 0.0

  # Rate of change of rover heading in radians per second
  yaw_rate_rads: float = 0.0

  # Score of the trajectory, higher is better, default to 0.0, this can be
  # calculated by the planner and used for debugging/visualisation purposes
  score: float = 0.0

  @staticmethod
  def create_arc(
    initial_rover_pose_in: RoverPose,
    curvature_radm_in: float,
    arc_resolution_m_in: float,
    num_steps_in: int,
  ) -> list[RoverPose]:
    """A utility function to generate an arced trajectory

    Uses a simple motion model to simulate the rover's trajectory along an arc:
    https://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm

    Args:
        initial_rover_pose_in (RoverPose): Rover pose to start the arc from.
        curvature_radm_in (float): The curvature of the arc in radians per metre.
        arc_resolution_m_in (float): The resolution of the arc in metres.
        num_steps_in (int): The number of steps to generate along the arc.

    Returns:
        list[RoverPose]: A list of RoverPose objects representing the arc.
    """
    # Copy the initial rover pose to avoid modifying the input
    rover_pose = initial_rover_pose_in.copy()

    # Set up the arc as an empty list of RoverPose objects
    arc = list[RoverPose]()

    # Calculate the new pose based on the current pose, velocity and yaw rate
    if abs(curvature_radm_in) > MIN_CURVATURE_RADM:
      # If there is a yaw rate, calculate the new position using a circular
      # arc model, first determine the radius of the arc and the centre of
      # rotation
      radius_m = 1 / curvature_radm_in
      cor_x_pos_m = rover_pose.position_m[0] - radius_m * np.sin(
        rover_pose.heading_rad
      )
      cor_y_pos_m = rover_pose.position_m[1] + radius_m * np.cos(
        rover_pose.heading_rad
      )

      # Loop through the arc, recording the rover's pose at each step
      for _ in range(num_steps_in):
        # Update the rover's heading, then use it to determine the new rover
        # position based on the radius and centre of rotation
        rover_pose.heading_rad = wrap_to_pi(
          rover_pose.heading_rad + curvature_radm_in * arc_resolution_m_in
        )
        rover_pose.position_m[0] = cor_x_pos_m + radius_m * np.sin(
          rover_pose.heading_rad
        )
        rover_pose.position_m[1] = cor_y_pos_m - radius_m * np.cos(
          rover_pose.heading_rad
        )

        # Append the updated pose to the list of poses defining the arc
        arc.append(rover_pose.copy())

    else:
      # Loop through the straight path, recording the rover's pose at each step
      for _ in range(num_steps_in):
        # If there is no yaw rate, the rover is moving straight and the heading
        # does not change
        rover_pose.position_m[0] = rover_pose.position_m[
          0
        ] + arc_resolution_m_in * np.cos(rover_pose.heading_rad)
        rover_pose.position_m[1] = rover_pose.position_m[
          1
        ] + arc_resolution_m_in * np.sin(rover_pose.heading_rad)

        # Append the updated pose to the list of poses defining the arc
        arc.append(rover_pose.copy())

    return arc

  def get_curvature(self) -> float:
    """Calculate and return the curvature for this trajectory

    Returns:
        float: The curvature of the trajectory, defined as the rate of change of
            heading with respect to distance travelled, in radians per metre.
    """
    try:
      curvature_radm = self.yaw_rate_rads / self.velocity_ms
      return curvature_radm
    except ZeroDivisionError:
      raise ZeroDivisionError(
        'Velocity is zero, curvature is undefined, ensure RoverTrajectory has '
        'been initialised correctly'
      )

  @classmethod
  def generate(
    cls,
    initial_rover_pose_in: RoverPose,
    velocity_ms_in: float,
    yaw_rate_rads_in: float,
    time_step_s_in: float,
    time_horizon_s_in: float,
  ) -> 'RoverTrajectory':
    """Generate a trajectory given an initial state, velocity and yaw rate.

    Args:
        initial_rover_pose_in (RoverPose): Rover pose to start the simulation
            from.
        velocity_ms_in (float): Rover velocity in metres per second.
        yaw_rate_rads_in (float): Rovers yaw rate in radians per second.
        time_step_s_in (float): Time step for the simulation in seconds.
        time_horizon_s_in: (float): Time horizon for the simulation in seconds.

    Returns:
        RoverTrajectory: The generated rover trajectory.
    """
    # Determine the number of time steps to simulate within the time horizon
    num_time_steps = int(time_horizon_s_in / time_step_s_in)

    # Generate the trajectory as a list of RoverPose objects along an arc
    poses = RoverTrajectory.create_arc(
      initial_rover_pose_in=initial_rover_pose_in,
      curvature_radm_in=yaw_rate_rads_in / velocity_ms_in,
      arc_resolution_m_in=velocity_ms_in * time_step_s_in,
      num_steps_in=num_time_steps,
    )

    # Return the generated trajectory
    return cls(
      poses=poses,
      velocity_ms=velocity_ms_in,
      yaw_rate_rads=yaw_rate_rads_in,
    )


@dataclass
class RoverLimits:
  """Class to represent the limits of the rover"""

  # Minimum velocity of the rover in metres per second (for reasonable traverse
  # speed)
  min_velocity_ms: float

  # Maximum velocity of the rover in metres per second
  max_velocity_ms: float

  # Maximum acceleration of the rover in metres per second per second
  max_accel_mss: float

  # Maximum change of rate of heading in radians per second
  max_yaw_rate_rads: float

  # Maximum change of yaw rate in radians per second per second
  max_yaw_accel_radss: float
