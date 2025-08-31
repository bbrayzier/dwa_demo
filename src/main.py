"""This file implements a simple example of using the DWA planner to navigate a
rover to a target position.

Copyright (c) 2025 Ben Brayzier
"""

# Local imports
from .dwa import (
  DwaPlanner,
  DwaConfig,
  DwaCostWeights,
  RoverState,
  RoverLimits,
  RoverPose,
)
from .util import euclidean_distance


def main():
  # Define rover limits
  rover_limits = RoverLimits(
    min_velocity_ms=0.15,
    max_velocity_ms=0.3,
    max_accel_mss=0.15,
    max_yaw_rate_rads=0.2,
    max_yaw_accel_radss=0.04,
  )

  # Define DWA configuration
  dwa_config = DwaConfig(
    velocity_resolution_ms=0.01,
    yaw_rate_resolution_rads=0.01,
    time_horizon_s=10.0,
    time_step_s=1.0,
    obstacle_margin_m=0.3,
    cost_weights=DwaCostWeights(
      heading_cost_factor=1.0,
      velocity_cost_factor=1.0,
      obstacle_cost_factor=1.0,
    ),
  )

  # Initialise the DWA planner
  dwa_planner = DwaPlanner(
    dwa_config_in=dwa_config,
    rover_limits_in=rover_limits,
  )

  # Set initial rover state
  rover_state = RoverState(
    pose=RoverPose(position_m=[0.0, 0.0], heading_rad=0.0),
    velocity_ms=rover_limits.min_velocity_ms,
    yaw_rate_rads=0.0,
  )

  # Set target position and tolerance
  target_position_m = [5.0, 5.0]
  target_tolerance_m = 0.1

  # Main loop for planning trajectories
  target_reached = False
  time_s = 0.0
  while not target_reached:
    # Compute possible trajectories based on current rover state
    trajectories = dwa_planner.compute_trajectories(rover_state)

    # Select the best trajectory
    best_trajectory = dwa_planner.select_best_trajectory(
      trajectories_in=trajectories,
      target_pos_m_in=target_position_m,
      obstacles_in=[],
    )

    # Update the rover state to the first pose in the best trajectory (i.e the
    # rover position at the end of this time step)
    rover_state = RoverState(
      pose=best_trajectory.poses[0],
      velocity_ms=best_trajectory.velocity_ms,
      yaw_rate_rads=best_trajectory.yaw_rate_rads,
    )

    # Increment the simulation time
    time_s += dwa_config.time_step_s

    # Print the rover state for debugging
    print(
      f'Time: {time_s:.1f}s, Position: {rover_state.pose.position_m}, '
      f'Heading: {rover_state.pose.heading_rad:.2f} rad, '
      f'Velocity: {rover_state.velocity_ms:.2f} m/s, '
      f'Yaw Rate: {rover_state.yaw_rate_rads:.2f} rad/s',
    )

    # Break the loop if the rover is within the target tolerance
    if (
      euclidean_distance(rover_state.pose.position_m, target_position_m)
      < target_tolerance_m
    ):
      print('Target reached!')
      target_reached = True


if __name__ == '__main__':
  main()
