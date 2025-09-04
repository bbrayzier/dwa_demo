"""This file implements a simple example of using the DWA planner to navigate a
rover to a target position.

Copyright (c) 2025 Ben Brayzier
"""

# Local imports
from .dwa.rover_data import (
  RoverState,
  RoverLimits,
  RoverPose,
)
from .dwa.dwa_planner import (
  DwaPlanner,
  DwaConfig,
  DwaCostWeights,
  DwaObstacle,
)
from .dwa.dwa_animation import DwaAnimation
from .util import euclidean_distance

# ---- CONSTANTS ----
SIM_TIME_LIMIT_S = 600.0


def run_dwa_demo() -> None:
  """Run a simple DWA demo, navigating a rover to a target position while
  avoiding obstacles.

  The demo generates a GIF showing the rover's planned trajectories at each time
  step.
  """
  # ---- VISUALISATION SETUP ----
  # Set up list of Plotly frames for animation
  animation = DwaAnimation(
    x_lim_m_in=(-7.5, 12.5),
    y_lim_m_in=(-17.5, 2.5),
  )

  # ---- ROVER SETUP ----
  # Define rover's velocity and acceleration limits
  rover_limits = RoverLimits(
    min_velocity_ms=0.1,
    max_velocity_ms=0.3,
    max_accel_mss=0.15,
    max_yaw_rate_rads=0.2,
    max_yaw_accel_radss=0.04,
  )

  # ---- DWA SETUP ----
  # Define DWA configuration parameters for the planner
  dwa_config = DwaConfig(
    velocity_resolution_ms=0.01,
    yaw_rate_resolution_rads=0.01,
    time_horizon_s=10.0,
    time_step_s=1.0,
    obstacle_margin_m=0.3,
    cost_weights=DwaCostWeights(
      heading_cost_factor=1.0,
      velocity_cost_factor=10.0,
      obstacle_cost_factor=15.0,
    ),
  )

  # Initialise the DWA planner with the configuration and rover limits
  dwa_planner = DwaPlanner(
    dwa_config_in=dwa_config,
    rover_limits_in=rover_limits,
  )

  # Set initial rover state, position at origin, heading along positive X-axis
  rover_state = RoverState(
    pose=RoverPose(position_m=[0.0, 0.0], heading_rad=0.0),
    velocity_ms=rover_limits.min_velocity_ms,
    yaw_rate_rads=0.0,
  )

  # ---- TARGET AND OBSTACLES SETUP ----
  # Set target position and tolerance
  target_position_m = [5.0, -15.0]
  target_tolerance_m = 0.5

  # Set up a list of obstacles
  obstacles = [
    DwaObstacle(position_m=[5.0, -5.0], radius_m=0.5),
    DwaObstacle(position_m=[2.5, -7.0], radius_m=1.0),
    DwaObstacle(position_m=[5.0, -10.0], radius_m=0.8),
    DwaObstacle(position_m=[2.5, -11.0], radius_m=0.5),
  ]

  # ---- SIMULATION LOOP ----
  # Main loop for planning trajectories
  target_reached = False
  time_s = 0.0
  while not target_reached:
    # Increment the simulation time
    time_s += dwa_config.time_step_s

    # Check for time limit exceeded
    if time_s > SIM_TIME_LIMIT_S:
      print('Time limit exceeded, stopping simulation.')
      break

    # Compute possible trajectories based on current rover state
    trajectories = dwa_planner.compute_trajectories(rover_state)

    # Select the best trajectory
    best_trajectory = dwa_planner.select_best_trajectory(
      trajectories_in=trajectories,
      target_pos_m_in=target_position_m,
      obstacles_in=obstacles,
    )

    # Update the rover state to the first pose in the best trajectory (i.e the
    # rover position at the end of this time step)
    rover_state = RoverState(
      pose=best_trajectory.poses[0],
      velocity_ms=best_trajectory.velocity_ms,
      yaw_rate_rads=best_trajectory.yaw_rate_rads,
    )

    # Add a frame for the current time step to the animation
    animation.add_frame(
      trajectories_in=trajectories,
      best_trajectory_in=best_trajectory,
      obstacles_in=obstacles,
      target_pos_m_in=target_position_m,
      time_s_in=time_s,
    )

    # Print the rover state for debugging
    print(
      f'Time / s: {time_s:6.1f}, '
      f'Position / m: [{rover_state.pose.position_m[0]:6.2f}, '
      f'{rover_state.pose.position_m[1]:6.2f}], '
      f'Heading / rad: {rover_state.pose.heading_rad:5.2f}, '
      f'Velocity / m/s: {rover_state.velocity_ms:5.2f}, '
      f'Yaw Rate / rad/s: {rover_state.yaw_rate_rads:5.2f}',
    )

    # Break the loop if the rover is within the target tolerance
    if (
      euclidean_distance(rover_state.pose.position_m, target_position_m)
      < target_tolerance_m
    ):
      print('Target reached!')
      target_reached = True

  # ---- GIF CREATION ----
  # Save the animation as a GIF
  animation.save_gif('dwa_demo.gif')


# Handle direct execution of this script
if __name__ == '__main__':
  run_dwa_demo()
