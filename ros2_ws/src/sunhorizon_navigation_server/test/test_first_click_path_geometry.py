"""Replay captured Robot A XY geometry; negative cases are explicit mutations.

The capture contains no path quaternions. This test makes no claim about captured
orientations or live navigation execution. A separate later heading capture
contains actual path quaternions and TF, and exercises measured-pose goal guards.
"""
import copy
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest


TEST_ROOT = Path(__file__).resolve().parent


class FirstClickPathGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.executable = Path(cls.directory.name) / "first_click_path_geometry"
        command = [os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror"]
        if (platform.system() == "Darwin" and not Path(
                "/Library/Developer/CommandLineTools/usr/include/c++/v1/string").is_file()):
            sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
            command += ["-isystem", f"{sdk}/usr/include/c++/v1"]
        command += [f"-I{TEST_ROOT.parent / 'include'}",
                    str(TEST_ROOT / "first_click_path_geometry_harness.cpp"),
                    "-o", str(cls.executable)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        cls.capture = json.loads((TEST_ROOT / "fixtures/robot_a_compute_path_20260909.json").read_text())

    def validate(self, points=None, goal=None, max_length=0.45):
        points = self.capture["path"] if points is None else points
        goal = self.capture["goal"] if goal is None else goal
        values = [*goal, max_length, len(points), *(v for p in points for v in p)]
        result = subprocess.run([str(self.executable)], input=" ".join(map(str, values)),
                                text=True, capture_output=True, check=False)
        self.assertIn(result.returncode, (0, 1), result.stderr)
        return result.stdout.strip()

    def test_actual_seven_point_path_passes_despite_quantized_endpoint(self):
        path, goal = self.capture["path"], self.capture["goal"]
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        self.assertEqual(len(path), 7)
        self.assertEqual(length, self.capture["path_length_m"])
        self.assertLess(length, math.dist(path[0], goal))  # The old guard rejected this.
        self.assertLess(math.dist(path[-1], goal), 0.05 / math.sqrt(2))
        self.assertEqual(self.validate(), "accepted")

    def test_early_termination_is_rejected(self):
        self.assertEqual(self.validate(self.capture["path"][:-1]),
                         "first_click_path_endpoint_out_of_range")

    def test_endpoint_allowance_is_bounded_by_five_cm_grid(self):
        endpoint = self.capture["path"][-1]
        self.assertEqual(self.validate(goal=[endpoint[0] + 0.036, endpoint[1]]),
                         "first_click_path_endpoint_out_of_range")

    def test_returning_to_goal_does_not_hide_reverse_segment(self):
        path = copy.deepcopy(self.capture["path"])
        path.insert(3, path[1])
        self.assertEqual(self.validate(path), "first_click_path_must_not_reverse")

    def test_forward_detour_above_unchanged_length_limit_is_rejected(self):
        path = copy.deepcopy(self.capture["path"])
        path[3][1] -= 0.2
        self.assertEqual(self.validate(path), "first_click_path_length_out_of_range")

    def test_original_maximum_length_remains_enforced(self):
        self.assertEqual(self.validate(max_length=0.299),
                         "first_click_path_length_out_of_range")

    def test_short_or_stationary_paths_are_rejected(self):
        self.assertEqual(self.validate([]), "first_click_no_feasible_path")
        self.assertEqual(self.validate([self.capture["goal"]] * 2),
                         "first_click_path_length_out_of_range")

    def run_guard(self, mode, values):
        result = subprocess.run([str(self.executable), mode],
                                input=" ".join(map(str, values)), text=True,
                                capture_output=True, check=False)
        self.assertIn(result.returncode, (0, 1), result.stderr)
        return result.stdout.strip()

    def test_actual_heading_capture_uses_tf_not_grid_pose(self):
        capture = json.loads((TEST_ROOT / "fixtures/robot_a_first_click_heading_20260909.json").read_text())
        parent, child = capture["latest_tf"]["odom"], capture["latest_tf"]["base_footprint"]
        self.assertEqual(parent["stamp_ns"], child["stamp_ns"])
        c, s = math.cos(parent["yaw"]), math.sin(parent["yaw"])
        start = [parent["xy"][0] + c * child["xy"][0] - s * child["xy"][1],
                 parent["xy"][1] + s * child["xy"][0] + c * child["xy"][1],
                 parent["yaw"] + child["yaw"]]
        goal_pose = capture["requested_goal"]["pose"]
        goal = [goal_pose["position"]["x"], goal_pose["position"]["y"],
                capture["trial_start"]["yaw"]]
        limits = [0.25, 0.35, math.radians(5), math.radians(5)]
        first = capture["poses"][0]
        old_heading = math.atan2(goal[1] - first["xy"][1], goal[0] - first["xy"][0])
        self.assertAlmostEqual(old_heading, 0.118644, places=5)
        self.assertEqual(self.run_guard("goal", [*first["xy"], first["yaw"], *goal, *limits]),
                         "first_click_requires_straight_forward_goal")
        self.assertAlmostEqual(start[2], 0.07045519714537496)
        self.assertEqual(self.run_guard("goal", [*start, *goal, *limits]), "accepted")
        self.assertEqual(self.validate([p["xy"] for p in capture["poses"]], goal[:2]), "accepted")

    def test_measured_pose_does_not_relax_goal_direction_distance_or_yaw(self):
        limits = [0.25, 0.35, math.radians(5), math.radians(5)]
        for goal, reason in [
                ([0, .3, 0], "first_click_requires_straight_forward_goal"),
                ([-.3, 0, 0], "first_click_requires_straight_forward_goal"),
                ([.24, 0, 0], "first_click_distance_out_of_range"),
                ([.36, 0, 0], "first_click_distance_out_of_range"),
                ([.3, 0, .1], "first_click_goal_yaw_must_match_start")]:
            with self.subTest(goal=goal):
                self.assertEqual(self.run_guard("goal", [0, 0, 0, *goal, *limits]), reason)

    def test_measurement_time_rejects_stale_future_and_missing_without_restamping(self):
        now = 1788951596547215879
        for stamp, age, accepted in [
                (1788951596521568536, .85, True),
                (now - 850000000, .85, True),
                (now - 850000001, .85, False),
                (now + 1, .85, False), (0, .85, False),
                (now, 0, False), (now, -1, False)]:
            with self.subTest(stamp=stamp, age=age):
                self.assertEqual(self.run_guard("time", [now, stamp, age]),
                                 "accepted" if accepted else "invalid")


if __name__ == "__main__":
    unittest.main()
