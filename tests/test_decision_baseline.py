import copy
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from decision_baseline import SensorTape, choose_action, run_strategy
from mission_io import PROJECT_ROOT, load_mission
from observation_simulator import load_noise_config
from online_state import OnlineEstimator


class DecisionTests(unittest.TestCase):
    def setUp(self):
        self.mission = load_mission()
        self.noise = load_noise_config()
        self.experiment = yaml.safe_load((PROJECT_ROOT / "configs/experiment_week04.yaml").read_text(encoding="utf-8"))
        self.thresholds = yaml.safe_load((PROJECT_ROOT / "configs/decision_thresholds.yaml").read_text(encoding="utf-8"))

    def run_pair(self, settings):
        tape = SensorTape.create(settings, self.experiment["max_steps"], 42)
        return {strategy: run_strategy(self.mission, settings, self.thresholds,
                                      self.experiment, tape, strategy) for strategy in ("A", "B")}

    def test_action_thresholds(self):
        for score, action in ((1, "CONTINUE"), (0.75, "CONTINUE"), (0.5, "SLOW_DOWN"),
                              (0.3, "OBSERVE"), (0.1, "HOLD")):
            self.assertEqual(choose_action(score, 0, self.thresholds), action)
        self.assertEqual(choose_action(0, 11, self.thresholds), "RETURN")

    def test_estimator_rejects_jump_without_truth(self):
        estimator = OnlineEstimator([2, 3], noise_std=0)
        result = estimator.update([2, 100], robust=True)
        self.assertFalse(result["observation_accepted"])
        np.testing.assert_allclose(estimator.position, [2, 3])
        estimator.predict([0, 1])
        result = estimator.update([2, 4])
        self.assertEqual(result["reliability_score"], 1)
        self.assertNotIn("true_x", result)

    def test_missing_prediction_and_history(self):
        estimator = OnlineEstimator([2, 3])
        estimator.predict([1, 0])
        first = estimator.update([np.nan, np.nan])
        second = estimator.update([np.nan, np.nan])
        self.assertEqual(first["missing_count"], 1)
        self.assertEqual(second["missing_count"], 2)
        self.assertLess(second["reliability_score"], first["reliability_score"])
        np.testing.assert_allclose(estimator.position, [3, 3])

    def test_sensor_tape_is_reproducible(self):
        first = SensorTape.create(self.noise, 20, 42)
        second = SensorTape.create(self.noise, 20, 42)
        np.testing.assert_array_equal(first.noise, second.noise)
        np.testing.assert_array_equal(first.missing, second.missing)
        np.testing.assert_array_equal(first.bias, second.bias)

    def test_perfect_observations_complete_both(self):
        settings = dict(self.noise, position_noise_std=0, missing_rate=0, outlier_length=0)
        pair = self.run_pair(settings)
        for log, metric in pair.values():
            self.assertTrue(metric["task_completed"])
            self.assertEqual(metric["collision_count"], 0)
            self.assertEqual(metric["path_length"], 110)
            delta = np.diff(log[["x", "y"]].to_numpy(), axis=0)
            self.assertTrue(((delta[:, 0] == 0) | (delta[:, 1] == 0)).all())
        pd.testing.assert_frame_equal(pair["A"][0].drop(columns=["strategy"]),
                                      pair["B"][0].drop(columns=["strategy"]))

    def test_transient_case_actual_hold_and_collision(self):
        settings = dict(self.noise, **self.experiment["cases"]["transient_bias"])
        pair = self.run_pair(settings)
        self.assertEqual(pair["A"][1]["collision_count"], 1)
        self.assertTrue(pair["B"][1]["task_completed"])
        log = pair["B"][0]
        stopped = log[log["action"].isin(["HOLD", "OBSERVE"])]
        self.assertGreater(len(stopped), 0)
        self.assertTrue((stopped[["vx", "vy"]].to_numpy() == 0).all())
        self.assertGreater(pair["B"][1]["execution_time"], 110)

    def test_persistent_case_returns_and_does_not_claim_success(self):
        settings = dict(self.noise, **self.experiment["cases"]["persistent_bias"])
        log, metric = self.run_pair(settings)["B"]
        self.assertTrue(metric["returned_to_start"])
        self.assertFalse(metric["task_completed"])
        self.assertEqual(metric["failure_reason"], "safe_return")
        returning = log[log["action"] == "RETURN"]
        self.assertGreater(len(returning), 0)
        self.assertTrue((returning["vy"] < 0).any())
        np.testing.assert_allclose(log.iloc[-1][["x", "y"]].astype(float), self.mission["start"])

    def test_action_speed_really_reduces(self):
        # 已知中等残差，通过配置阈值确保动作属于减速区间。
        settings = dict(self.noise, position_noise_std=0, missing_rate=0, outlier_start=0,
                        outlier_length=2, outlier_offset=[0.25, 0])
        tape = SensorTape.create(settings, self.experiment["max_steps"], 1)
        thresholds = dict(self.thresholds, continue_threshold=0.99, slow_down_threshold=0.5)
        log, _ = run_strategy(self.mission, settings, thresholds, self.experiment, tape, "B")
        rows = log[log["action"] == "SLOW_DOWN"]
        self.assertGreater(len(rows), 0)
        speeds = np.linalg.norm(rows[["vx", "vy"]], axis=1)
        self.assertTrue((speeds <= self.mission["speed"] * self.experiment["slow_speed_ratio"] + 1e-9).all())

    def test_future_observations_cannot_change_past_actions(self):
        original = SensorTape.create(self.noise, self.experiment["max_steps"], 10)
        changed = copy.deepcopy(original)
        changed.bias[20:] += 1000
        first, _ = run_strategy(self.mission, self.noise, self.thresholds, self.experiment, original, "B")
        second, _ = run_strategy(self.mission, self.noise, self.thresholds, self.experiment, changed, "B")
        pd.testing.assert_frame_equal(first[first["time"] < 20], second[second["time"] < 20])

    def test_single_missing_tick_not_counted_twice_by_observe(self):
        settings = dict(self.noise, position_noise_std=0, missing_rate=0, outlier_length=0)
        tape = SensorTape.create(settings, self.experiment["max_steps"], 1)
        tape.missing[3] = True
        log, _ = run_strategy(self.mission, settings, self.thresholds, self.experiment, tape, "B")
        self.assertEqual(log.loc[log["time"] == 3, "missing_count"].iloc[0], 1)


if __name__ == "__main__":
    unittest.main()
