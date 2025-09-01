"""This file implements data classes to represent the rover state, pose,
trajectory and limits.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import numpy as np
from dataclasses import dataclass, field

# Local imports
from ..util import wrap_to_pi


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

    Uses a simple motion model to simulate the rover's trajectory along an arc
    over the time, see:
    https://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm

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
    # Copy the initial rover pose to avoid modifying the input
    rover_pose = initial_rover_pose_in.copy()

    # Determine the number of time steps to simulate within the time horizon
    num_time_steps = int(time_horizon_s_in / time_step_s_in)

    # Set up the trajectory as an empty list of RoverPose objects
    poses = list[RoverPose]()

    # Loop through simulation, recording the rover's position at each time step
    for _ in range(num_time_steps):
      # Calculate the new pose based on the current pose, velocity and yaw rate
      if abs(yaw_rate_rads_in) > 1e-6:
        # If there is a yaw rate, calculate the new position using a circular
        # arc model, first determine the radius of the arc and the centre of
        # rotation
        radius_m = velocity_ms_in / yaw_rate_rads_in
        cor_x_pos_m = rover_pose.position_m[0] - radius_m * np.sin(
          rover_pose.heading_rad
        )
        cor_y_pos_m = rover_pose.position_m[1] + radius_m * np.cos(
          rover_pose.heading_rad
        )

        # Update the rover's heading, then use it to determine the new rover
        # position based on the radius and centre of rotation
        rover_pose.heading_rad = wrap_to_pi(
          rover_pose.heading_rad + yaw_rate_rads_in * time_step_s_in
        )
        rover_pose.position_m[0] = cor_x_pos_m + radius_m * np.sin(
          rover_pose.heading_rad
        )
        rover_pose.position_m[1] = cor_y_pos_m - radius_m * np.cos(
          rover_pose.heading_rad
        )

      else:
        # If there is no yaw rate, the rover is moving straight and the heading
        # does not change
        rover_pose.position_m[0] = (
          rover_pose.position_m[0]
          + velocity_ms_in * np.cos(rover_pose.heading_rad) * time_step_s_in
        )
        rover_pose.position_m[1] = (
          rover_pose.position_m[1]
          + velocity_ms_in * np.sin(rover_pose.heading_rad) * time_step_s_in
        )

      # Append the updated pose to the list of poses defining the trajectory
      poses.append(rover_pose.copy())

    # At the end of simulation, return the generated trajectory
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
