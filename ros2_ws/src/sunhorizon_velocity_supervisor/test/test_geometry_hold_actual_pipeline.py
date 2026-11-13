"""One bounded actual DWB controller -> collision -> shadow supervisor trial.

All three ELF paths are explicit; missing actors are SKIPPED, never substituted.
Only TF/odometry/platform/localization/task/coverage and collision sensor data are
fixtures. Static odometry and a 1 m reference exercise computation, not motion.
"""

import copy
import hashlib
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest

pytest.importorskip("rclpy")
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import NavigationStatus, VelocityCommandEvidence
from unique_identifier_msgs.msg import UUID

from test_geometry_hold_supervisor_node import (
    COMMAND_TOPIC, CONTROLLER_TOPIC, Harness, is_zero, qos, twist_dict,
)


PARAMETERS = """
controller_server:
  ros__parameters:
    use_sim_time: false
    sunhorizon_command_generation_evidence_enabled: true
    odom_topic: /geometry_pipeline/odom
    controller_frequency: 20.0
    failure_tolerance: 0.0
    progress_checker_plugin: progress_checker
    goal_checker_plugins: [goal_checker]
    controller_plugins: [FollowPath]
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.10
      movement_time_allowance: 10.0
    goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      stateful: true
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.10
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      min_vel_x: 0.0
      max_vel_x: 0.05
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 0.05
      min_speed_xy: 0.0
      max_speed_xy: 0.05
      min_speed_theta: 0.0
      acc_lim_x: 0.20
      acc_lim_y: 0.0
      acc_lim_theta: 0.20
      decel_lim_x: -0.20
      decel_lim_y: 0.0
      decel_lim_theta: -0.20
      vx_samples: 5
      vy_samples: 1
      vtheta_samples: 3
      sim_time: 1.0
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.10
      trans_stopped_velocity: 0.01
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics: [BaseObstacle, PathDist, GoalDist]
      BaseObstacle.scale: 0.02
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      update_frequency: 20.0
      publish_frequency: 5.0
      transform_tolerance: 0.10
      track_unknown_space: false
      robot_radius: 0.22
      plugins: [inflation_layer]
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.30
        cost_scaling_factor: 3.0
collision_monitor:
  ros__parameters:
    use_sim_time: false
    sunhorizon_command_generation_evidence_enabled: true
    sunhorizon_command_generation_timeout_sec: 0.25
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: /sunhorizon/navigation/cmd_vel_nav2_raw
    cmd_vel_out_topic: /sunhorizon/navigation/cmd_vel_candidate
    transform_tolerance: 0.10
    source_timeout: 0.30
    base_shift_correction: false
    stop_pub_timeout: 0.50
    polygons: [StopZone]
    StopZone:
      type: polygon
      action_type: stop
      points: [0.60, 0.40, 0.60, -0.40, -0.20, -0.40, -0.20, 0.40]
      max_points: 1
      visualize: false
      enabled: true
    observation_sources: [points]
    points:
      type: pointcloud
      topic: /geometry_pipeline/obstacles
      min_height: 0.02
      max_height: 2.0
      enabled: true
"""


ORIGIN_FIELDS = (
    "schema_version", "controller_epoch_uuid", "command_sequence", "controller_goal_sequence",
    "clock_boot_id", "frame_id", "compute_begin_ros_ns", "compute_end_ros_ns",
    "compute_begin_steady_ns", "compute_end_steady_ns", "generation_state", "origin_velocity",
)


def envelope_dict(message):
    result = {field: getattr(message, field) for field in (
        "schema_version", "command_sequence", "controller_goal_sequence", "clock_boot_id",
        "frame_id", "compute_begin_ros_ns", "compute_end_ros_ns", "compute_begin_steady_ns",
        "compute_end_steady_ns", "generation_state", "collision_state", "collision_sequence",
        "collision_begin_ros_ns", "collision_end_ros_ns", "collision_begin_steady_ns",
        "collision_end_steady_ns")}
    result.update(controller_epoch_uuid=list(bytes(message.controller_epoch_uuid)),
                  collision_epoch_uuid=list(bytes(message.collision_epoch_uuid)),
                  origin_velocity=twist_dict(message.origin_velocity),
                  velocity=twist_dict(message.velocity))
    return result


def key(message):
    return bytes(message.controller_epoch_uuid).hex(), message.command_sequence


class Pipeline(Harness):
    def __init__(self, tmp_path, executables):
        super().__init__(tmp_path, fake_command_actors=False,
                         isolated_domain=208 + os.getpid() % 8)
        self.extra_trace.update(executables=executables, progress_budget_sec=10.0,
                                trial_budget_sec=8.0, lifecycle=[], owned_goal={})
        self.send_tf = False  # Publish ONLY map -> odom -> base_footprint below.
        self.origins, self.filtered = {}, {}
        self.children, self.child_logs = {}, []
        self.active_actors = set()
        self.goal_uuid = None
        self.send_future = None
        self.goal_deadline = None
        self.cleaning = False
        self.stop_points = False
        self.odom_pub = self.driver.create_publisher(Odometry, "/geometry_pipeline/odom", qos())
        self.points_pub = self.driver.create_publisher(
            PointCloud2, "/geometry_pipeline/obstacles", qos())
        self.driver.create_subscription(VelocityCommandEvidence, CONTROLLER_TOPIC,
                                        lambda message: self.receive("controller", message), qos())
        self.driver.create_subscription(VelocityCommandEvidence, COMMAND_TOPIC,
                                        lambda message: self.receive("collision", message), qos())
        self.action = ActionClient(self.driver, FollowPath, "/follow_path")
        self.cancel = self.driver.create_client(CancelGoal, "/follow_path/_action/cancel_goal")
        self.result = self.driver.create_client(
            FollowPath.Impl.GetResultService, "/follow_path/_action/get_result")
        parameters = tmp_path / "actual_pipeline.yaml"
        parameters.write_text(PARAMETERS)
        for actor, variable in (("controller_server", "CONTROLLER_EXECUTABLE"),
                                ("collision_monitor", "COLLISION_EXECUTABLE")):
            log = (tmp_path / (actor + ".log")).open("w+")
            self.child_logs.append(log)
            arguments = [executables[variable], "--ros-args", "--params-file", str(parameters)]
            if actor == "controller_server":
                arguments += ["-r", "cmd_vel:=/sunhorizon/navigation/cmd_vel_nav2_raw"]
            self.children[actor] = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT)
        self.extra_trace["actor_pids"] = {actor: process.pid for actor, process in self.children.items()}

    def receive(self, actor, message):
        receipt = self.clocks()
        target = self.origins if actor == "controller" else self.filtered
        assert key(message) not in target, "observed duplicate actual command identity"
        if target:
            previous = next(reversed(target.values()))
            assert bytes(message.controller_epoch_uuid) == bytes(previous.controller_epoch_uuid)
            assert message.command_sequence > previous.command_sequence
            if actor == "collision":
                assert bytes(message.collision_epoch_uuid) == bytes(previous.collision_epoch_uuid)
                assert message.collision_sequence > previous.collision_sequence
        target[key(message)] = message
        self.command_trace.append({"actor": actor, "received": receipt,
                                   "envelope": envelope_dict(message)})

    def publish_inputs(self):
        super().publish_inputs()
        now = self.driver.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        transforms = []
        for parent, child in (("map", "odom"), ("odom", "base_footprint")):
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        cloud = PointCloud2()
        cloud.header.stamp = now.to_msg()
        cloud.header.frame_id = "base_footprint"
        values = ((.10, -.10, .30), (.10, .10, .30), (.20, -.10, .30), (.20, .10, .30)) \
            if self.stop_points else ((2.0, 0.0, .30),)
        cloud.height, cloud.width = 1, len(values)
        cloud.point_step, cloud.row_step = 12, len(values) * 12
        cloud.fields = [PointField(name=axis, offset=4 * index,
                                   datatype=PointField.FLOAT32, count=1)
                        for index, axis in enumerate("xyz")]
        cloud.data = b"".join(struct.pack("<fff", *point) for point in values)
        self.points_pub.publish(cloud)

    def pump(self):
        if self.goal_deadline is not None and not self.cleaning:
            assert time.monotonic() < self.goal_deadline, "8 second actual-goal trial budget expired"
        for actor, child in self.children.items():
            assert child.poll() is None, actor + " exited"
        super().pump()

    def wait(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, "actual pipeline readiness timeout"
            self.pump()

    def lifecycle(self, actor, transition, expected):
        client = self.driver.create_client(ChangeState, "/" + actor + "/change_state")
        state_client = self.driver.create_client(GetState, "/" + actor + "/get_state")
        self.wait(lambda: client.service_is_ready() and state_client.service_is_ready())
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        self.wait(future.done)
        assert future.result().success, actor + " lifecycle transition rejected"
        state = state_client.call_async(GetState.Request())
        self.wait(state.done)
        assert state.result().current_state.id == expected
        if expected == State.PRIMARY_STATE_ACTIVE:
            self.active_actors.add(actor)
        else:
            self.active_actors.discard(actor)
        self.extra_trace["lifecycle"].append({"actor": actor, "transition": transition,
                                               "state": expected, "observed": self.clocks()})

    def unique_owner(self, topic, expected):
        endpoints = self.driver.get_publishers_info_by_topic(topic)
        return len(endpoints) == 1 and (
            endpoints[0].node_namespace.rstrip("/") + "/" + endpoints[0].node_name) == expected \
            and endpoints[0].topic_type == "sunhorizon_navigation_interfaces/msg/VelocityCommandEvidence" \
            and any(endpoints[0].endpoint_gid)

    def verify_loaded_actors(self):
        records = {}
        for actor, variable, library in (
                ("controller_server", "CONTROLLER_EXECUTABLE", "libcontroller_server_core.so"),
                ("collision_monitor", "COLLISION_EXECUTABLE", "libcollision_monitor_core.so")):
            process = self.children[actor]
            executable = Path(self.extra_trace["executables"][variable]).resolve()
            assert Path("/proc/{}/exe".format(process.pid)).resolve() == executable
            lines = Path("/proc/{}/maps".format(process.pid)).read_text().splitlines()
            mapped = {line.split(maxsplit=5)[5] for line in lines
                      if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith("/")}
            cores = {Path(path).resolve() for path in mapped if Path(path).name == library}
            # Both ordinary colcon build and install layouts keep the paired
            # core next to the ELF or in its immediately enclosing lib directory.
            adjacent = {candidate.resolve() for candidate in (
                executable.parent / library, executable.parent.parent / library)
                        if candidate.is_file()}
            assert len(cores) == 1 and cores.issubset(adjacent), (
                actor + " loaded an unpaired core (default Humble fallback is forbidden)")
            records[actor] = {"pid": process.pid, "executable": str(executable),
                              "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
                              "mapped_paths": sorted(mapped),
                              "core": str(next(iter(cores))),
                              "core_sha256": hashlib.sha256(next(iter(cores)).read_bytes()).hexdigest()}
        self.extra_trace["loaded_actors"] = records

    def start(self):
        self.lifecycle("controller_server", Transition.TRANSITION_CONFIGURE,
                       State.PRIMARY_STATE_INACTIVE)
        self.lifecycle("controller_server", Transition.TRANSITION_ACTIVATE,
                       State.PRIMARY_STATE_ACTIVE)
        self.wait(lambda: self.unique_owner(CONTROLLER_TOPIC, "/controller_server"))
        self.lifecycle("collision_monitor", Transition.TRANSITION_CONFIGURE,
                       State.PRIMARY_STATE_INACTIVE)
        self.lifecycle("collision_monitor", Transition.TRANSITION_ACTIVATE,
                       State.PRIMARY_STATE_ACTIVE)
        self.wait(lambda: self.unique_owner(COMMAND_TOPIC, "/collision_monitor"))
        self.verify_loaded_actors()
        self.arm()

    def send_path(self):
        self.wait(lambda: self.action.server_is_ready() and self.cancel.service_is_ready()
                  and self.result.service_is_ready())
        self.task_id = "actual-pipeline-task-1"
        self.navigation_state = NavigationStatus.STATE_NAVIGATING
        goal = FollowPath.Goal()
        goal.controller_id, goal.goal_checker_id = "FollowPath", "goal_checker"
        goal.path = NavPath()
        goal.path.header.frame_id = "map"
        goal.path.header.stamp = self.driver.get_clock().now().to_msg()
        for index in range(51):
            pose = PoseStamped()
            pose.header = copy.deepcopy(goal.path.header)
            pose.pose.position.x = index / 50.0
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        self.goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        self.extra_trace["owned_goal"].update(uuid=bytes(self.goal_uuid.uuid).hex(),
                                              send_attempted=True, sent=self.clocks())
        self.goal_deadline = time.monotonic() + 8.0
        self.send_future = self.action.send_goal_async(goal, goal_uuid=self.goal_uuid)
        self.wait(self.send_future.done)
        handle = self.send_future.result()
        assert bytes(handle.goal_id.uuid) == bytes(self.goal_uuid.uuid)
        self.extra_trace["owned_goal"]["accepted"] = handle.accepted
        assert handle.accepted, "actual FollowPath rejected"

    def joined(self):
        pairs = []
        for identity in self.origins.keys() & self.filtered.keys():
            origin, result = self.origins[identity], self.filtered[identity]
            for field in ORIGIN_FIELDS:
                if field == "controller_epoch_uuid":
                    assert bytes(origin.controller_epoch_uuid) == bytes(result.controller_epoch_uuid)
                else:
                    assert getattr(origin, field) == getattr(result, field), field + " was rewritten"
            assert origin.schema_version == 1 and origin.command_sequence > 0
            assert origin.clock_boot_id == self.boot_id and origin.frame_id == "base_footprint"
            assert origin.collision_state == origin.COLLISION_NOT_PROCESSED
            assert origin.collision_sequence == 0 and not any(origin.collision_epoch_uuid)
            assert origin.velocity == origin.origin_velocity
            assert result.collision_sequence > 0 and any(result.collision_epoch_uuid)
            assert 0 < origin.compute_begin_ros_ns <= origin.compute_end_ros_ns
            assert 0 < origin.compute_begin_steady_ns <= origin.compute_end_steady_ns
            assert result.collision_begin_ros_ns >= origin.compute_end_ros_ns
            assert result.collision_begin_steady_ns >= origin.compute_end_steady_ns
            assert result.collision_end_ros_ns >= result.collision_begin_ros_ns
            assert result.collision_end_steady_ns >= result.collision_begin_steady_ns
            if result.collision_state == result.COLLISION_CLEAR:
                assert result.velocity == origin.origin_velocity
            if result.collision_state == result.COLLISION_STOP:
                assert is_zero(result.velocity)
            pairs.append((origin, result))
        return pairs

    def cleanup_goal(self):
        self.cleaning = True
        if self.goal_uuid is None:
            return
        if self.send_future is not None and self.send_future.done():
            try:
                handle = self.send_future.result()
            except BaseException as error:
                # A failed/unknown acknowledgement is NOT proof no goal exists.
                self.extra_trace["owned_goal"]["ack_error"] = repr(error)
            else:
                if not handle.accepted:
                    self.extra_trace["owned_goal"].update(cleanup_complete=True,
                                                          terminal_evidence="explicit_goal_rejection")
                    return
        cancel_request = CancelGoal.Request()
        cancel_request.goal_info.goal_id = self.goal_uuid  # Stamp zero, ONLY this UUID.
        result_request = FollowPath.Impl.GetResultService.Request()
        result_request.goal_id = self.goal_uuid
        cancel_future = self.cancel.call_async(cancel_request)
        result_future = self.result.call_async(result_request)
        self.wait(lambda: cancel_future.done() and result_future.done(), timeout=3.0)
        status = result_future.result().status
        cancel_ids = [bytes(goal.goal_id.uuid).hex()
                      for goal in cancel_future.result().goals_canceling]
        assert all(identity == bytes(self.goal_uuid.uuid).hex() for identity in cancel_ids)
        self.extra_trace["owned_goal"].update(cancel_return_code=cancel_future.result().return_code,
                                              cancel_uuid=bytes(self.goal_uuid.uuid).hex(),
                                              result_uuid=bytes(self.goal_uuid.uuid).hex(),
                                              cancel_goal_ids=cancel_ids,
                                              terminal_status=status, cleanup_complete=status in (4, 5, 6),
                                              terminal_evidence="exact_uuid_get_result")
        assert status in (4, 5, 6), "owned FollowPath terminal remains unknown"

    def close(self):
        cleanup_error = None
        try:
            self.cleanup_goal()
            for actor in tuple(self.active_actors):
                self.lifecycle(actor, Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
        except BaseException as error:
            cleanup_error = error
            self.record_failure("actual_pipeline_cleanup:" + repr(error))
        finally:
            for child in self.children.values():
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)
            for log in self.child_logs:
                log.seek(0)
                print(log.read())
                log.close()
            super().close()
        if cleanup_error is not None:
            raise cleanup_error
        assert all(child.returncode == 0 for child in self.children.values())


def test_actual_follow_path_collision_geometry_hold_recovery_and_stop(tmp_path):
    required = ("SUPERVISOR_EXECUTABLE", "CONTROLLER_EXECUTABLE", "COLLISION_EXECUTABLE")
    missing = [variable for variable in required if not os.environ.get(variable)]
    if missing:
        pytest.skip("Missing actual actor ELF paths: " + ", ".join(missing))
    executables = {variable: os.environ[variable] for variable in required}
    for variable, executable in executables.items():
        path = Path(executable)
        assert path.is_absolute() and path.is_file(), variable
        with path.open("rb") as stream:
            assert stream.read(4) == b"\x7fELF", variable + " must identify an actual ELF"
    h = Pipeline(tmp_path, executables)
    try:
        h.start()
        h.send_path()
        h.until(lambda status: status.command_allowed and status.supervised.linear.x > 0)
        h.wait(lambda: any(origin.generation_state == origin.GENERATED_CONTROLLER and
                          origin.origin_velocity.linear.x > 0 and
                          result.collision_state == result.COLLISION_CLEAR
                          for origin, result in h.joined()))
        h.enter_hold()
        h.auto_frames = False
        h.new_source(1)
        h.assert_zero_window(.06)
        h.new_source(1)
        h.assert_zero_window(.06)
        h.new_source(1)
        h.auto_frames = True
        third_clear = h.source_ns
        h.until(lambda status: status.command_allowed and status.supervised.linear.x > 0)
        assert any(origin.compute_begin_ros_ns > third_clear and
                   result.collision_state == result.COLLISION_CLEAR for origin, result in h.joined())
        h.enter_hold()
        h.stop_points = True  # Healthy, fresh actual sensor geometry triggers Stop.
        fault = h.until(lambda status: status.latched and status.reason == "collision_stop")
        assert is_zero(fault.supervised)
        h.wait(lambda: any(result.collision_state == result.COLLISION_STOP
                          for _, result in h.joined()))
        h.stop_points = False
        h.new_source(1)
        assert all(status.latched for status in h.assert_zero_window(.06))
        # Every observed collision envelope must have its actual controller origin.
        h.wait(lambda: set(h.filtered).issubset(h.origins))
        h.joined()
    except BaseException as error:
        h.record_failure("actual_pipeline_test:" + repr(error))
        raise
    finally:
        h.close()
