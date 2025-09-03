"""This file implements a simple example of using the DWA planner to navigate a
rover to a target position.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import plotly.graph_objects as go
import gif


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


def run_dwa_demo() -> None:
  """Run a simple DWA demo, navigating a rover to a target position while
  avoiding obstacles.

  The demo generates a GIF showing the rover's planned trajectories at each time
  step.
  """
  # ---- VISUALISATION SETUP ----
  # Set up list of Plotly frames for animation
  dwa_frames: list[go.Figure] = []

  # Configure gif options for Plotly
  gif.options.plotly['width'] = 1000
  gif.options.plotly['height'] = 1000

  # Set up axis limits for plotly figure
  x_lim_m = (-7.5, 12.5)
  y_lim_m = (-17.5, 2.5)

  # Define rover limits
  rover_limits = RoverLimits(
    min_velocity_ms=0.1,
    max_velocity_ms=0.3,
    max_accel_mss=0.15,
    max_yaw_rate_rads=0.2,
    max_yaw_accel_radss=0.04,
  )

  # ---- DWA SETUP ----
  # Define DWA configuration
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

    # Create a Plotly frame for the current time step
    dwa_frames.append(
      create_dwa_frame(
        trajectories_in=trajectories,
        best_trajectory_in=best_trajectory,
        obstacles_in=obstacles,
        target_pos_m_in=target_position_m,
        x_lim_m_in=x_lim_m,
        y_lim_m_in=y_lim_m,
        time_s_in=time_s,
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

  # ---- GIF CREATION ----
  # Convert the list of figures into a GIF, duration between frames is specified
  # in milliseconds so set to 100x time step to give an effective duration of
  # 100th the DWA time step (i.e. speed up the animation by 100x)
  gif.save(
    frames=dwa_frames,
    path='dwa_demo.gif',
    duration=int(dwa_config.time_step_s * 10),
  )


@gif.frame
def create_dwa_frame(
  trajectories_in: list[RoverTrajectory],
  best_trajectory_in: RoverTrajectory,
  obstacles_in: list[DwaObstacle],
  target_pos_m_in: list[float] | None = None,
  x_lim_m_in: tuple[float, float] = (-5.0, 5.0),
  y_lim_m_in: tuple[float, float] = (-5.0, 5.0),
  time_s_in: float | None = None,
) -> go.Figure:
  """Create a Plotly frame showing the DWA trajectories and the best trajectory.

  Args:
      trajectories_in (list[RoverTrajectory]): List of all computed
          trajectories.
      best_trajectory_in (RoverTrajectory): The selected best trajectory.
      obstacles_in (list[DwaObstacle]): List of obstacles to plot.
      target_pos_m_in (list[float], optional): Target position to plot. If
          not provided, the target will not be shown. Defaults to None.
      x_lim_m_in (tuple[float, float], optional): X-axis limits in metres.
          Defaults to (-5.0, 5.0).
      y_lim_m_in (tuple[float, float], optional): Y-axis limits in metres.
          Defaults to (-5.0, 5.0).
      time_s_in (float): The current simulation time in seconds. Optional, if
          not provided the time will not be shown in the title. Defaults to
          None.

  Returns:
      go.Figure: A Plotly figure containing the trajectorys and obstacles.
  """
  # Assemble title for the frame
  title_text = 'DWA Demo'
  if time_s_in is not None:
    title_text += f'   |   Time: {time_s_in:6.1f} s'

  # Create traces for all trajectories and obstacles
  traces = [
    go.Scatter(
      x=[pose.position_m[0] for pose in trajectory.poses],
      y=[pose.position_m[1] for pose in trajectory.poses],
      mode='lines',
      line=dict(color='royalblue', width=1),
      name='Assessed Trajectories',
      showlegend=False,
    )
    for trajectory in trajectories_in
  ]
  # Show legend for the first trajectory only
  traces[0].showlegend = True

  # Create trace for the best trajectory
  traces.append(
    go.Scatter(
      x=[pose.position_m[0] for pose in best_trajectory_in.poses],
      y=[pose.position_m[1] for pose in best_trajectory_in.poses],
      mode='lines',
      line=dict(color='orangered', width=2),
      marker=dict(size=6),
      name='Best Trajectory',
    )
  )

  # Create trace for the target position if provided
  if target_pos_m_in is not None:
    traces.append(
      go.Scatter(
        x=[target_pos_m_in[0]],
        y=[target_pos_m_in[1]],
        mode='markers',
        marker=dict(size=12, color='lightgreen', symbol='x'),
        name='Target Position',
      )
    )

  # Create the Plotly figure from the trajectory traces and set the layout
  fig = go.Figure(
    data=traces,
    layout=go.Layout(
      xaxis=dict(range=x_lim_m_in, title='X / m'),
      yaxis=dict(
        range=y_lim_m_in, scaleanchor='x', scaleratio=1, title='Y / m'
      ),
      title=dict(text=title_text, x=0.5),
      autosize=False,
      width=1600,
      height=1000,
      template='plotly_dark',
      plot_bgcolor='#474747',
      legend=dict(
        orientation='h',
        yanchor='bottom',
        y=1.01,
        xanchor='center',
        x=0.5,
      ),
    ),
  )

  # Add shapes for obstacles, only show legend for the first obstacle
  for idx, obstacle in enumerate(obstacles_in):
    fig.add_shape(
      type='circle',
      xref='x',
      yref='y',
      x0=obstacle.position_m[0] - obstacle.radius_m,
      y0=obstacle.position_m[1] - obstacle.radius_m,
      x1=obstacle.position_m[0] + obstacle.radius_m,
      y1=obstacle.position_m[1] + obstacle.radius_m,
      line=dict(color='slateblue'),
      fillcolor='darkslateblue',
      name='Obstacles',
      showlegend=(idx == 0),
    )

  # Return the created frame as a figure
  return fig


# Handle direct execution of this script
if __name__ == '__main__':
  run_dwa_demo()
