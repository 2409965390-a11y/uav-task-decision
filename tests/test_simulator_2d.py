"""运行：python -m unittest discover -s tests -v"""

import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mission_io import load_mission
from observation_simulator import load_noise_config, simulate_observations
from simulator_2d import COLUMNS, check_collision, simulate_motion
from state_estimator import calculate_reliability, calculate_rmse, estimate_state


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        self.mission = load_mission()

    def assert_valid_trajectory(self, mission, dt=1.0):
        trajectory = simulate_motion(mission, dt)
        self.assertEqual(list(trajectory.columns), COLUMNS)
        self.assertTrue(np.isfinite(trajectory.to_numpy()).all())
        np.testing.assert_allclose(trajectory.iloc[0][["x", "y"]], mission["start"])
        np.testing.assert_allclose(trajectory.iloc[-1][["x", "y"]], mission["goal"])
        self.assertEqual(trajectory.iloc[0]["time"], 0)
        self.assertEqual(trajectory.iloc[-1]["task_progress"], 1)
        np.testing.assert_allclose(trajectory.iloc[-1][["vx", "vy"]], [0, 0])
        progress = trajectory["task_progress"].to_numpy()
        self.assertTrue(((progress >= 0) & (progress <= 1)).all())
        self.assertTrue((np.diff(progress) >= -1e-12).all())
        if len(trajectory) > 1:
            self.assertEqual(progress[0], 0)
            self.assertTrue((progress[:-1] < 1).all())
            np.testing.assert_allclose(np.diff(trajectory["time"]), dt)
            positions = trajectory[["x", "y"]].to_numpy()
            displacements = np.diff(positions, axis=0)
            # 按真实CSV位移验证水平/竖直运动，避免只把绘图改成阶梯线。
            self.assertTrue(((displacements[:, 0] == 0) | (displacements[:, 1] == 0)).all())
            self.assertTrue((np.abs(displacements).sum(axis=1) > 0).all())
            velocities = trajectory[["vx", "vy"]].to_numpy()[:-1]
            np.testing.assert_allclose(displacements, velocities * dt, atol=1e-12)
            self.assertTrue((np.linalg.norm(displacements, axis=1) <= mission["speed"] * dt + 1e-10).all())
            for first, last in zip(positions, positions[1:]):
                self.assertFalse(check_collision(first.tolist(), last.tolist(), mission))
                # 独立核对采样段上到原障碍的欧氏距离，不能小于安全距离。
                for fraction in np.linspace(0, 1, 21):
                    x, y = first + fraction * (last - first)
                    for box in mission["obstacles"]:
                        dx = max(box["xmin"] - x, 0, x - box["xmax"])
                        dy = max(box["ymin"] - y, 0, y - box["ymax"])
                        self.assertGreaterEqual(math.hypot(dx, dy) + 1e-10, mission["safety_distance"])
        return trajectory

    def test_existing_mission_avoids_obstacle(self):
        self.assertTrue(check_collision(self.mission["start"], self.mission["goal"], self.mission))
        trajectory = self.assert_valid_trajectory(self.mission)
        self.assertGreater(len(trajectory.attrs["planned_route"]), 2)
        self.assertEqual(self.mission, load_mission())

    def test_teacher_changed_goal_and_map(self):
        self.mission["map_size"] = [20, 20]
        self.mission["start"] = [1, 1]
        self.mission["goal"] = [18, 18]
        self.assert_valid_trajectory(self.mission, 0.5)
        self.mission["goal"] = [15, 4]
        self.assert_valid_trajectory(self.mission, 0.5)

    def test_report_changed_goal(self):
        self.mission["goal"] = [50, 20]
        self.assert_valid_trajectory(self.mission)

    def test_no_obstacles_uses_right_angle_route(self):
        self.mission["obstacles"] = []
        trajectory = self.assert_valid_trajectory(self.mission)
        self.assertEqual(trajectory.attrs["planned_route"], [(6, 6), (6, 61), (61, 61)])
        self.assertEqual(len(trajectory), 111)

    def test_axis_aligned_goal_uses_straight_route(self):
        self.mission["obstacles"] = []
        self.mission["goal"] = [6, 61]
        trajectory = self.assert_valid_trajectory(self.mission)
        self.assertEqual(len(trajectory.attrs["planned_route"]), 2)

    def test_both_l_routes_blocked_need_orthogonal_detour(self):
        self.mission.update({"map_size": [20, 20], "start": [2, 2], "goal": [18, 18],
                             "safety_distance": 0.5, "obstacles": [
                                 {"xmin": 1, "xmax": 3, "ymin": 8, "ymax": 12},
                                 {"xmin": 8, "xmax": 12, "ymin": 1, "ymax": 3},
                             ]})
        trajectory = self.assert_valid_trajectory(self.mission)
        self.assertGreater(len(trajectory.attrs["planned_route"]), 3)

    def test_fractional_coordinates_keep_right_angles(self):
        self.mission.update({"start": [2.25, 2.75], "goal": [60.25, 60.75], "speed": 0.7})
        self.assert_valid_trajectory(self.mission, 0.4)

    def test_high_speed_does_not_tunnel_or_cut_corner(self):
        self.mission["speed"] = 100
        self.assert_valid_trajectory(self.mission, 2)

    def test_segment_crossing_and_touching_are_blocked(self):
        mission = {"map_size": [20, 20], "safety_distance": 0,
                   "obstacles": [{"xmin": 8, "xmax": 12, "ymin": 7, "ymax": 13}]}
        self.assertTrue(check_collision([1, 10], [19, 10], mission))
        self.assertTrue(check_collision([1, 7], [19, 7], mission))
        self.assertTrue(check_collision([8, 7], [8, 7], mission))
        self.assertFalse(check_collision([1, 6], [19, 6], mission))
        self.assertTrue(check_collision([1, 6], [21, 6], mission))
        mission["safety_distance"] = 1
        self.assertTrue(check_collision([1, 6], [19, 6], mission))

    def test_same_start_and_goal(self):
        self.mission["goal"] = self.mission["start"].copy()
        self.assertEqual(len(self.assert_valid_trajectory(self.mission)), 1)

    def test_multiple_and_overlapping_obstacles(self):
        self.mission["obstacles"].extend([
            {"xmin": 11, "xmax": 16, "ymin": 10, "ymax": 18},
            {"xmin": 28, "xmax": 40, "ymin": 22, "ymax": 42},
        ])
        self.assert_valid_trajectory(self.mission)

    def test_unreachable_map_raises(self):
        self.mission["obstacles"] = [{"xmin": 20, "xmax": 24, "ymin": 0, "ymax": 67}]
        with self.assertRaisesRegex(ValueError, "未找到安全"):
            simulate_motion(self.mission)

    def test_invalid_inputs_raise(self):
        for dt in (0, -1, float("nan"), float("inf")):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                simulate_motion(self.mission, dt)
        for key, value in (("speed", 0), ("speed", float("nan")),
                           ("safety_distance", -1), ("map_size", [0, 67]),
                           ("start", [-1, 6]), ("goal", [100, 61]),
                           ("start", [8, 8]), ("goal", [13, 10])):
            mission = copy.deepcopy(self.mission)
            mission[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                simulate_motion(mission)

    def test_deterministic_and_third_week_integration(self):
        ground_truth = simulate_motion(self.mission)
        pd.testing.assert_frame_equal(ground_truth, simulate_motion(self.mission))
        observations = simulate_observations(ground_truth, load_noise_config())
        estimates = estimate_state(observations)
        reliability = calculate_reliability(estimates)
        self.assertEqual(len(estimates), len(ground_truth))
        self.assertEqual(len(reliability), len(ground_truth))
        np.testing.assert_allclose(estimates["time"], ground_truth["time"])
        self.assertTrue(math.isfinite(calculate_rmse(estimates["estimate_error"])))
        self.assertTrue(reliability["reliability_score"].between(0, 1).all())


if __name__ == "__main__":
    unittest.main()
