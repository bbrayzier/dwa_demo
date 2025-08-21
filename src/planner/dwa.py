class DWA:
    def __init__(self, max_speed, max_yaw_rate, dt):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.dt = dt

    def compute_trajectory(self, state, velocity):
        trajectories = []
        for speed in range(-self.max_speed, self.max_speed + 1):
            for yaw_rate in range(-self.max_yaw_rate, self.max_yaw_rate + 1):
                trajectory = self._simulate_trajectory(state, speed, yaw_rate)
                trajectories.append(trajectory)
        return trajectories

    def select_best_trajectory(self, trajectories, goal):
        best_trajectory = None
        best_score = float('-inf')
        for trajectory in trajectories:
            score = self._evaluate_trajectory(trajectory, goal)
            if score > best_score:
                best_score = score
                best_trajectory = trajectory
        return best_trajectory

    def _simulate_trajectory(self, state, speed, yaw_rate):
        # Simulate the trajectory based on the current state, speed, and yaw rate
        # This is a placeholder for the actual simulation logic
        trajectory = []
        for _ in range(10):  # Simulate for 10 time steps
            state[0] += speed * self.dt
            state[1] += speed * self.dt
            trajectory.append(state.copy())
        return trajectory

    def _evaluate_trajectory(self, trajectory, goal):
        # Evaluate the trajectory based on distance to the goal
        final_position = trajectory[-1]
        distance = ((final_position[0] - goal[0]) ** 2 + (final_position[1] - goal[1]) ** 2) ** 0.5
        return -distance  # Return negative distance as score (closer is better)