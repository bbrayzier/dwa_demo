"""This file implements a class to generate plotly figures of DWA planning and
save them as a GIF animation.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import gif
import plotly.graph_objects as go

# Local imports
from .planner import DwaObstacle
from .rover_data import RoverTrajectory


# ---- FUNCTIONS ----
# Wrap the frame creation function with the gif.frame decorator, this converts
# the generated figure into a pillow image
@gif.frame
def create_dwa_frame(
  trajectories_in: list[RoverTrajectory],
  best_trajectory_in: RoverTrajectory,
  obstacles_in: list[DwaObstacle] | None = None,
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
      obstacles_in (list[DwaObstacle] | None, optional): List of obstacles to
          plot. Defaults to None.
      target_pos_m_in (list[float], optional): Target position to plot. If
          not provided, the target will not be shown. Defaults to None.
      x_lim_m_in (tuple[float, float], optional): X-axis limits in metres.
          Defaults to (-5.0, 5.0).
      y_lim_m_in (tuple[float, float], optional): Y-axis limits in metres.
          Defaults to (-5.0, 5.0).
      time_s_in (float): The current simulation time in seconds. Optional, if
          not provided the time will not be shown in the title. Defaults to
          None.

  Raises:
      RuntimeError: trajectories_in contains insufficient elements

  Returns:
      go.Figure: A Plotly figure containing the trajectorys and obstacles.
  """
  # Sanity check input args
  if len(trajectories_in) < 1:
    raise RuntimeError(
      'trajectories_in must contain at least 1 RoverTrajectory object'
    )

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
  if obstacles_in is not None:
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


# Wrap the frame creation function with the gif.frame decorator, this converts
# the generated figure into a pillow image
@gif.frame
def create_score_frame(
  trajectories_in: list[RoverTrajectory],
  x_lim_ms_in: tuple[float, float],
  y_lim_rads_in: tuple[float, float],
  c_lim_in: tuple[float, float],
  time_s_in: float | None = None,
) -> go.Figure:
  """Create a Plotly frame showing the score of each DWA trajectory in velocity
  space.

  Args:
      trajectories_in (list[RoverTrajectory]): List of all computed
          trajectories with their scores populated.
      x_lim_ms_in (tuple[float, float]): X-axis limits in m/s for the velocity
          space plot.
      y_lim_rads_in (tuple[float, float]): Y-axis limits in rad/s for the
          yaw rate space plot.
      c_lim_in (tuple[float, float]): Z-axis limits for the score color scale.
      time_s_in (float): The current simulation time in seconds. Optional, if
          not provided the time will not be shown in the title. Defaults to
          None.

  Raises:
      RuntimeError: trajectories_in contains insufficient elements

  Returns:
      go.Figure: A Plotly figure containing the score heatmap in velocity space.
  """
  # Sanity check input args
  if len(trajectories_in) < 1:
    raise RuntimeError(
      'trajectories_in must contain at least 1 RoverTrajectory object'
    )

  # Extract velocities, yaw rates and scores from the trajectories
  velocities_ms = [trajectory.velocity_ms for trajectory in trajectories_in]
  yaw_rates_rads = [trajectory.yaw_rate_rads for trajectory in trajectories_in]
  scores = [trajectory.score for trajectory in trajectories_in]

  # Assemble title for the frame
  title_text = 'DWA Score'
  if time_s_in is not None:
    title_text += f'   |   Time: {time_s_in:6.1f} s'

  # Create trace for the scores in velocity space
  traces = [
    go.Heatmap(
      x=velocities_ms,
      y=yaw_rates_rads,
      z=scores,
      zmin=c_lim_in[0],
      zmax=c_lim_in[1],
      colorscale='Viridis',
      colorbar=dict(title='Score'),
      hoverongaps=False,
    )
  ]

  # Create the Plotly figure from the trajectory traces and set the layout
  fig = go.Figure(
    data=traces,
    layout=go.Layout(
      xaxis=dict(title='Velocity / m/s', range=x_lim_ms_in),
      yaxis=dict(title='Yaw rate / rad/s', range=y_lim_rads_in),
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
  # Return the created frame as a figure
  return fig


# ---- CLASSES ----
class DwaAnimation:
  """Class to manage the creation of animation frames and the final GIF for DWA
  planner visualisation."""

  def __init__(
    self,
    demo_x_lim_m_in: tuple[float, float] = (-5.0, 5.0),
    demo_y_lim_m_in: tuple[float, float] = (-5.0, 5.0),
    score_x_lim_ms_in: tuple[float, float] = (0.0, 0.5),
    score_y_lim_rads_in: tuple[float, float] = (-0.2, 0.2),
    score_c_lim_in: tuple[float, float] = (-1.0, 5.0),
    gif_width_px: int = 1000,
    gif_height_px: int = 1000,
  ) -> None:
    """Initialise the DWA animation manager.

    Args:
        x_lim_m_in (tuple[float, float], optional): X-axis limits in metres.
            Defaults to (-5.0, 5.0).
        y_lim_m_in (tuple[float, float], optional): Y-axis limits in metres.
            Defaults to (-5.0, 5.0).
        gif_width_px (int, optional): Width of the output GIF in pixels.
            Defaults to 1000.
        gif_height_px (int, optional): Height of the output GIF in pixels.
            Defaults to 1000.
    """
    # Set class members from inputs
    self.dwa_frames: list[go.Figure] = []
    self.score_frames: list[go.Figure] = []
    self.demo_x_lim_m = demo_x_lim_m_in
    self.demo_y_lim_m = demo_y_lim_m_in
    self.score_x_lim_ms = score_x_lim_ms_in
    self.score_y_lim_rads = score_y_lim_rads_in
    self.score_c_lim = score_c_lim_in

    # Configure gif options for Plotly
    gif.options.plotly['width'] = gif_width_px
    gif.options.plotly['height'] = gif_height_px

  def add_frame(
    self,
    trajectories_in: list[RoverTrajectory],
    best_trajectory_in: RoverTrajectory,
    obstacles_in: list[DwaObstacle] | None = None,
    target_pos_m_in: list[float] | None = None,
    time_s_in: float | None = None,
  ) -> None:
    """Add a frame to the animation showing the DWA trajectories and the best
    trajectory.

    Args:
        trajectories_in (list[RoverTrajectory]): List of all computed
            trajectories.
        best_trajectory_in (RoverTrajectory): The selected best trajectory.
      obstacles_in (list[DwaObstacle] | None, optional): List of obstacles to
          plot. Defaults to None.
        target_pos_m_in (list[float], optional): Target position to plot. If
            not provided, the target will not be shown. Defaults to None.
        time_s_in (float): The current simulation time in seconds. Optional, if
            not provided the time will not be shown in the title. Defaults to
            None.
    """
    self.dwa_frames.append(
      create_dwa_frame(
        trajectories_in=trajectories_in,
        best_trajectory_in=best_trajectory_in,
        obstacles_in=obstacles_in,
        target_pos_m_in=target_pos_m_in,
        x_lim_m_in=self.demo_x_lim_m,
        y_lim_m_in=self.demo_y_lim_m,
        time_s_in=time_s_in,
      )
    )
    self.score_frames.append(
      create_score_frame(
        trajectories_in=trajectories_in,
        x_lim_ms_in=self.score_x_lim_ms,
        y_lim_rads_in=self.score_y_lim_rads,
        c_lim_in=self.score_c_lim,
        time_s_in=time_s_in,
      )
    )

  def save_demo_gif(
    self, filename_in: str, frame_duration_ms: int = 10
  ) -> None:
    """Save the collected DWA frames as a GIF.

    Args:
        filename_in (str): The output filename for the GIF.
        frame_duration_ms (int, optional): Duration of each frame in
            milliseconds. Defaults to 10.
    """
    gif.save(self.dwa_frames, filename_in, duration=frame_duration_ms)

  def save_score_gif(
    self, filename_in: str, frame_duration_ms: int = 10
  ) -> None:
    """Save the collected score frames as a GIF.

    Args:
        filename_in (str): The output filename for the GIF.
        frame_duration_ms (int, optional): Duration of each frame in
            milliseconds. Defaults to 10.
    """
    gif.save(self.score_frames, filename_in, duration=frame_duration_ms)
