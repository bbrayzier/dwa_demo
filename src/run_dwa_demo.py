"""This file implements a simple example of using the DWA planner to navigate a
rover to a target position.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import plotly.graph_objects as go

# Local imports
from .dwa import (
  DwaPlanner,
  DwaConfig,
  DwaCostWeights,
  DwaObstacle,
  RoverState,
  RoverLimits,
  RoverPose,
  RoverTrajectory,
)
from .util import euclidean_distance

# ---- CONSTANTS ----
SIM_TIME_LIMIT_S = 600.0


def run_dwa_demo():
  # Define rover limits
  rover_limits = RoverLimits(
    min_velocity_ms=0.1,
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

  # Set up list of Plotly frames for animation
  dwa_frames = list[go.Frame]()

  # Set target position and tolerance
  target_position_m = [5.0, -15.0]
  target_tolerance_m = 0.5

  # Set up a list of obstacles
  obstacles = [
    DwaObstacle(position_m=[5.0, -10.0], radius_m=0.5),
  ]

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

    # Create a Plotly frame for the current time step
    dwa_frames.append(
      create_dwa_frame(
        trajectories_in=trajectories,
        best_trajectory_in=best_trajectory,
        obstacles_in=obstacles,
        time_s=time_s,
      )
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

  # Create the Plotly figure and save to HTML
  fig = go.Figure(
    data=max([frame.data for frame in dwa_frames], key=len),
    layout=go.Layout(
      xaxis=dict(range=[-5, 10], autorange=False),
      yaxis=dict(
        range=[-20, 5], autorange=False, scaleanchor='x', scaleratio=1
      ),
      title=dict(text='DWA Demo', x=0.5),
      updatemenus=[
        dict(
          type='buttons',
          buttons=[dict(label='Play', method='animate', args=[None])],
        )
      ],
    ),
    frames=dwa_frames,
  )
  fig.write_html('dwa_demo.html', include_plotlyjs='cdn')


def create_dwa_frame(
  trajectories_in: list[RoverTrajectory],
  best_trajectory_in: RoverTrajectory,
  obstacles_in: list[DwaObstacle],
  time_s: float,
) -> go.Frame:
  """Create a Plotly frame showing the DWA trajectories and the best trajectory.

  Args:
      trajectories_in (list[RoverTrajectory]): List of all computed
          trajectories.
      best_trajectory_in (RoverTrajectory): The selected best trajectory.
      obstacles_in (list[DwaObstacle]): List of obstacles to plot.
      time_s (float): The current simulation time in seconds.

  Returns:
      go.Frame: A Plotly frame containing the trajectory visualisations.
  """
  # Create traces for all trajectories and obstacles
  traces = list[go.Scatter]()
  for trajectory in trajectories_in:
    traces.append(
      go.Scatter(
        x=[pose.position_m[0] for pose in trajectory.poses],
        y=[pose.position_m[1] for pose in trajectory.poses],
        mode='lines',
        line=dict(color='blue', width=1),
        name='Trajectory',
        showlegend=False,
      )
    )

  # Create trace for the best trajectory
  best_trajectory_trace = go.Scatter(
    x=[pose.position_m[0] for pose in best_trajectory_in.poses],
    y=[pose.position_m[1] for pose in best_trajectory_in.poses],
    mode='lines+markers',
    line=dict(color='red', width=2),
    marker=dict(size=6),
    name='Best Trajectory',
  )
  traces.append(best_trajectory_trace)

  # Create traces for obstacles
  for obstacle in obstacles_in:
    obstacle_trace = go.Scatter(
      x=[obstacle.position_m[0]],
      y=[obstacle.position_m[1]],
      mode='markers',
      marker=dict(size=20 * obstacle.radius_m, color='black', symbol='x'),
      name='Obstacle',
    )
    traces.append(obstacle_trace)

  # Combine all traces into a single frame
  return go.Frame(data=traces, name=f'Time {time_s:.1f}s')


# Handle direct execution of this script
if __name__ == '__main__':
  run_dwa_demo()
