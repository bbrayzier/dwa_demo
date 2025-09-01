"""This file implements some common mathematical utility functions.

Copyright (c) 2025 Ben Brayzier
"""

# Generic imports
import numpy as np


def wrap_to_pi(angle_rad_in: float) -> float:
  """Wrap an angle in radians to the range [-pi, pi].

  Args:
      angle_rad_in (float): Angle in radians to normalise.

  Returns:
      float: Normalised angle in radians.
  """
  while angle_rad_in > np.pi:
    angle_rad_in -= 2 * np.pi
  while angle_rad_in < -np.pi:
    angle_rad_in += 2 * np.pi
  return angle_rad_in


def euclidean_distance(point1_in: list[float], point2_in: list[float]) -> float:
  """Compute the Euclidean distance between two points in 2D space.

  Args:
      point1_in (list[float]): The first point as a list of [x, y] coordinates.
      point2_in (list[float]): The second point as a list of [x, y] coordinates.

  Returns:
      float: The Euclidean distance between the two points.
  """
  return np.hypot(
    point2_in[0] - point1_in[0],
    point2_in[1] - point1_in[1],
  )
