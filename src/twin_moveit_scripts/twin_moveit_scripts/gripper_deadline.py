"""Clock-domain policy for simulated gripper action results (no ROS dependencies)."""
import math


class SimulationDeadline:
    def __init__(self, wall_start, simulation_budget, freeze_budget=3., wall_budget=120.):
        self.wall_start = wall_start
        self.last_progress_wall = wall_start
        self.simulation_budget = simulation_budget
        self.freeze_budget = freeze_budget
        self.wall_budget = wall_budget
        self.first_sim = self.last_sim = None

    def check(self, wall, fresh_sim):
        """Return an error, or None. None samples mean no fresh clock received yet."""
        if wall-self.wall_start >= self.wall_budget:
            return 'gripper result hard wall deadline exceeded'
        if fresh_sim is not None:
            if not math.isfinite(fresh_sim) or fresh_sim < 0:
                return 'invalid simulation clock during gripper result wait'
            if self.last_sim is not None and fresh_sim < self.last_sim:
                return 'simulation clock reset during gripper result wait'
            if self.first_sim is None:
                self.first_sim = fresh_sim
            if self.last_sim is None or fresh_sim > self.last_sim:
                self.last_progress_wall = wall
            self.last_sim = fresh_sim
            if fresh_sim-self.first_sim >= self.simulation_budget:
                return 'gripper result simulation deadline exceeded'
        if wall-self.last_progress_wall >= self.freeze_budget:
            return 'simulation clock frozen during gripper result wait'
        return None
