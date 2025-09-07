"""This file implements a simple example of using the DWA planner to navigate a
rover to a target position.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
from argparse import ArgumentParser

# Local imports
from .dwa import (
  RoverState,
  RoverLimits,
  RoverPose,
  DwaPlanner,
  DwaConfig,
  DwaCostWeights,
  DwaObstacle,
  DwaAnimation,
)
from .util import euclidean_distance

# ---- ROVER START SETUP ----
# Set rover's starting position and heading
ROVER_START_POSITION_M = [0.0, 0.0]
ROVER_START_HEADING_RAD = 0.0

# ---- ROVER LIMITS SETUP ----
# Set rover's velocity and acceleration limits
ROVER_MIN_VELOCITY_MS = 0.1
ROVER_MAX_VELOCITY_MS = 0.3
ROVER_MAX_ACCEL_MSS = 0.15
ROVER_MAX_YAW_RATE_RADS = 0.2
ROVER_MAX_YAW_ACCEL_RADSS = 0.04

# ---- DWA CONFIGURATION SETUP ----
# Set DWA configuration parameters
VELOCITY_RESOLUTION_MS = 0.01
YAW_RATE_RESOLUTION_RADS = 0.01
TIME_HORIZON_S = 10.0
TIME_STEP_S = 1.0
OBSTACLE_MARGIN_M = 0.3

# Set DWA cost function weights
HEADING_COST_FACTOR = 1.0
VELOCITY_COST_FACTOR = 50.0
OBSTACLE_COST_FACTOR = 10.0

# ---- TARGET AND OBSTACLES SETUP ----
# Set target position and tolerance
TARGET_POSITION_M = [5.0, -15.0]
TARGET_TOLERANCE_M = 0.3

# Set up a list of obstacles
OBSTACLE_LIST = [
  DwaObstacle(position_m=[5.0, -5.0], radius_m=0.5),
  DwaObstacle(position_m=[2.5, -7.0], radius_m=1.0),
  DwaObstacle(position_m=[5.0, -10.0], radius_m=0.8),
  DwaObstacle(position_m=[2.5, -11.0], radius_m=0.5),
]

# ---- MISC CONSTANTS ----
# Set simulation time limit
SIM_TIME_LIMIT_S = 300.0

# Animation axis limits
ANIMATION_X_LIMS_M = (-7.5, 12.5)
ANIMATION_Y_LIMS_M = (-17.5, 2.5)


def run_dwa_demo(enable_animation_flag_in: bool = False) -> None:
  """Run a simple DWA demo, navigating a rover to a target position while
  avoiding obstacles.

  Additionally the demo can generate an animation showing the rover's
  planned trajectories at each time step.

  Args:
    enable_animation_flag_in (bool, optional): A flag to enable animation
      generation which is saved as a GIF. Defaults to False.
  """
  # ---- VISUALISATION SETUP ----
  if enable_animation_flag_in:
    # Set up list of Plotly frames for animation
    animation = DwaAnimation(
      x_lim_m_in=ANIMATION_X_LIMS_M,
      y_lim_m_in=ANIMATION_Y_LIMS_M,
    )

  # ---- ROVER SETUP ----
  # Define rover's velocity and acceleration limits
  rover_limits = RoverLimits(
    min_velocity_ms=ROVER_MIN_VELOCITY_MS,
    max_velocity_ms=ROVER_MAX_VELOCITY_MS,
    max_accel_mss=ROVER_MAX_ACCEL_MSS,
    max_yaw_rate_rads=ROVER_MAX_YAW_RATE_RADS,
    max_yaw_accel_radss=ROVER_MAX_YAW_ACCEL_RADSS,
  )

  # ---- DWA SETUP ----
  # Define DWA configuration parameters for the planner
  dwa_config = DwaConfig(
    velocity_resolution_ms=VELOCITY_RESOLUTION_MS,
    yaw_rate_resolution_rads=YAW_RATE_RESOLUTION_RADS,
    time_horizon_s=TIME_HORIZON_S,
    time_step_s=TIME_STEP_S,
    obstacle_margin_m=OBSTACLE_MARGIN_M,
    cost_weights=DwaCostWeights(
      heading_cost_weight=HEADING_COST_FACTOR,
      velocity_cost_weight=VELOCITY_COST_FACTOR,
      obstacle_cost_weight=OBSTACLE_COST_FACTOR,
    ),
  )

  # Initialise the DWA planner with the configuration and rover limits
  dwa_planner = DwaPlanner(
    dwa_config_in=dwa_config,
    rover_limits_in=rover_limits,
  )

  # Set initial rover state, position at origin, heading along positive X-axis
  rover_state = RoverState(
    pose=RoverPose(
      position_m=ROVER_START_POSITION_M,
      heading_rad=ROVER_START_HEADING_RAD,
    ),
    velocity_ms=rover_limits.min_velocity_ms,
    yaw_rate_rads=0.0,
  )

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
      target_pos_m_in=TARGET_POSITION_M,
      obstacles_in=OBSTACLE_LIST,
    )

    # Update the rover state to the first pose in the best trajectory (i.e the
    # rover position at the end of this time step)
    rover_state = RoverState(
      pose=best_trajectory.poses[0],
      velocity_ms=best_trajectory.velocity_ms,
      yaw_rate_rads=best_trajectory.yaw_rate_rads,
    )

    if enable_animation_flag_in:
      # Add a frame for the current time step to the animation
      animation.add_frame(
        trajectories_in=trajectories,
        best_trajectory_in=best_trajectory,
        obstacles_in=OBSTACLE_LIST,
        target_pos_m_in=TARGET_POSITION_M,
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

    # Break the loop if the final position of the selected trajectory is within
    # the target tolerance
    if (
      euclidean_distance(
        best_trajectory.poses[-1].position_m, TARGET_POSITION_M
      )
      < TARGET_TOLERANCE_M
    ):
      print('Target reached!')
      target_reached = True

  # ---- GIF CREATION ----

  if enable_animation_flag_in:
    # Save the animation as a GIF
    animation.save_gif('assets/dwa_demo.gif')


# Handle direct execution of this script
if __name__ == '__main__':
  # Set up command line argument parsing
  parser = ArgumentParser(
    description='Run a simple DWA demo, navigating a rover to a target '
    'position while avoiding obstacles.',
  )

  # Add argument for enabling animation
  parser.add_argument(
    '-a',
    '--animate',
    action='store_true',
    help='Enable animation and save as GIF',
  )

  # Parse the command line arguments
  args = parser.parse_args()

  # Run the DWA demo with appropriate settings
  run_dwa_demo(enable_animation_flag_in=args.animate)
