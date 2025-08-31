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
