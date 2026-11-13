"""Actual supervisor ELF in an isolated localhost DDS domain, always shadow.

Run on a Linux ROS overlay with generated interfaces and set
SUPERVISOR_EXECUTABLE to the exact newly built velocity_supervisor_node ELF.
This file is deliberately independent of launch files/deployed parameters. Its
fake nodes own the exact production FQNs/topics, so graph and MessageInfo GIDs
are real DDS identities; command/coverage payloads are synthetic fixtures.
No final command publisher may exist, including during successful recovery.
"""

import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from sunhorizon_navigation_interfaces.msg import (
    LocalizationStatus,
    NavigationStatus,
    ObstacleCoverageDecision,
    PlatformSafetyState,
    VelocityCommandEvidence,
    VelocitySupervisorStatus,
)
from tf2_ros import TransformBroadcaster


CONTRACT = "geometry-hold-supervisor-fixture-typed-v1"
STATUS_TOPIC = "/sunhorizon/navigation/velocity_supervisor_status"
CONTROLLER_TOPIC = "/sunhorizon/navigation/controller_command_v1"
COMMAND_TOPIC = "/sunhorizon/navigation/collision_command_v1"
COVERAGE_TOPIC = "/sunhorizon/obstacle_coverage/decision_v1"
SPATIAL_TOPIC = "/sunhorizon/obstacle_coverage/spatial_decision_v2"
SPATIAL_CONTRACT = "spatial-supervisor-fixture-v1"
FINAL_TOPIC = "/navigation/cmd_vel"


def qos(transient=False, depth=10):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient
                    else DurabilityPolicy.VOLATILE),
    )


def stamp_ns(value):
    return value.sec * 10**9 + value.nanosec


def is_zero(value):
    return all(getattr(value.linear, axis) == 0.0 for axis in "xyz") and all(
        getattr(value.angular, axis) == 0.0 for axis in "xyz"
    )


def twist_dict(value):
    return {part: {axis: getattr(getattr(value, part), axis) for axis in "xyz"}
            for part in ("linear", "angular")}


def status_dict(value):
    """Complete VelocitySupervisorStatus, serialized only at failure/teardown."""
    return {
        "header": {"stamp": {"sec": value.header.stamp.sec,
                              "nanosec": value.header.stamp.nanosec},
                   "frame_id": value.header.frame_id},
        "state": value.state,
        "motion_output_connected": value.motion_output_connected,
        "candidate_seen": value.candidate_seen,
        "command_allowed": value.command_allowed,
        "latched": value.latched,
        "reason": value.reason,
        "candidate": twist_dict(value.candidate),
        "supervised": twist_dict(value.supervised),
    }


class Harness:
    def __init__(self, tmp_path, fake_command_actors=True, isolated_domain=None, spatial=False):
        assert isolated_domain is None or 208 <= isolated_domain <= 215
        executable = os.environ.get("SUPERVISOR_EXECUTABLE")
        if not executable:
            pytest.skip("SUPERVISOR_EXECUTABLE must name the actual built node ELF")
        if not Path("/proc/sys/kernel/random/boot_id").is_file():
            pytest.skip("Linux CLOCK_MONOTONIC/boot_id is required")
        assert Path(executable).is_file(), executable
        assert Path(executable).read_bytes()[:4] == b"\x7fELF", "actual Linux ELF required"
        self.saved_env = {key: os.environ.get(key) for key in (
            "ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID")}
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["ROS_DOMAIN_ID"] = str(
            185 + os.getpid() % 20 if isolated_domain is None else isolated_domain)
        rclpy.init()
        names = ["geometry_hold_supervisor_test", "sunhorizon_obstacle_coverage",
                 "sunhorizon_navigation_server"]
        if fake_command_actors:
            names = [names[0], "controller_server", "collision_monitor", *names[1:]]
        self.nodes = [rclpy.create_node(name, namespace="/") for name in names]
        if fake_command_actors:
            self.driver, self.controller, self.collision, self.coverage, self.navigation = self.nodes
        else:
            self.driver, self.coverage, self.navigation = self.nodes
            self.controller = self.collision = None
        self.executor = SingleThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)
        self.statuses = []
        self.status_receipt_ros = []
        self.command_trace = []
        self.source_trace = []
        self.failure_trace = None
        self.extra_trace = {}
        self.trace_path = tmp_path / "geometry_hold_supervisor_trace.json"
        self.status_sub = self.driver.create_subscription(
            VelocitySupervisorStatus, STATUS_TOPIC, self.on_status, qos(True))
        self.platform_pub = self.driver.create_publisher(
            PlatformSafetyState, "/sunhorizon/platform/safety_state", qos())
        self.localization_pub = self.driver.create_publisher(
            LocalizationStatus, "/sunhorizon/localization/status", qos())
        self.spatial = spatial
        if spatial:
            from sunhorizon_navigation_interfaces.msg import ObstacleCoverageSpatialDecision
            self.coverage_pub = self.coverage.create_publisher(
                ObstacleCoverageSpatialDecision, SPATIAL_TOPIC, qos(depth=1))
        else:
            self.coverage_pub = self.coverage.create_publisher(
                ObstacleCoverageDecision, COVERAGE_TOPIC, qos(depth=1))
        self.navigation_pub = self.navigation.create_publisher(
            NavigationStatus, "/sunhorizon/navigation/status", qos(True))
        if fake_command_actors:
            # Match the production typed publishers: reliable, volatile,
            # KeepLast(1). A deeper fixture writer is a different transport load.
            self.controller_pub = self.controller.create_publisher(
                VelocityCommandEvidence, CONTROLLER_TOPIC, qos(depth=1))
            self.command_pub = self.collision.create_publisher(
                VelocityCommandEvidence, COMMAND_TOPIC, qos(depth=1))
            # Typed mode still validates graph ownership of legacy outputs.
            self.candidate_pub = self.collision.create_publisher(
                Twist, "/sunhorizon/navigation/cmd_vel_candidate", qos())
            self.audit_pub = self.collision.create_publisher(
                TwistStamped, "/sunhorizon/navigation/collision_audit", qos())
        self.tf = TransformBroadcaster(self.driver)
        self.reset_client = self.driver.create_client(
            Trigger, "/sunhorizon/navigation/reset_velocity_latch")
        self.stop_client = self.driver.create_client(
            Trigger, "/sunhorizon/navigation/latch_zero_velocity")
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.frame = self.evaluation = self.sequence = 0
        self.source_ns = 0
        self.classification = 1
        self.task_id = ""
        self.navigation_state = NavigationStatus.STATE_READY
        self.generation = VelocityCommandEvidence.GENERATED_EXPLICIT_ZERO
        self.command_style = "fresh"
        self.frozen_interval = None
        self.vx = 0.0
        self.auto_frames = True
        self.send_commands = fake_command_actors
        self.send_navigation = self.send_tf = True
        self.send_platform = self.send_localization = True
        self.platform_fault = False
        self.localization_state = LocalizationStatus.STATE_HEALTHY
        self.collision_state = VelocityCommandEvidence.COLLISION_CLEAR
        self.next_publish = self.next_frame = 0.0
        self.new_source(1)
        self.log = (tmp_path / "geometry_hold_supervisor_node.log").open("w+")
        command = [executable, "--ros-args"]
        parameters = {
            "motion_output_enabled": "false",
            "geometry_hold_enabled": "true",
            "geometry_hold_region_contract": SPATIAL_CONTRACT if spatial else CONTRACT,
            "endpoint_stability_sec": "0.05",
            "publisher_discovery_grace_sec": "0.05",
            "control_rate_hz": "100.0",
        }
        if spatial:
            parameters.update({
                "spatial_visibility_enabled": "true",
                "spatial_query_digest": "spatial-query-sha",
                "spatial_producer_epoch": "spatial-fixture-epoch",
                "spatial_source_ids": "[0, 1]",
                "spatial_source_epochs": "[camera-epoch, chest-epoch]",
                "spatial_source_frames": "[base_footprint, base_footprint]",
            })
        for key, value in parameters.items():
            command += ["-p", key + ":=" + value]
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)

    def clocks(self):
        return {"ros_ns": self.driver.get_clock().now().nanoseconds,
                "steady_ns": time.monotonic_ns()}

    def on_status(self, message):
        receipt = self.clocks()
        # Retain the unmodified received message; serialize once at teardown.
        self.statuses.append((receipt["steady_ns"], message))
        self.status_receipt_ros.append(receipt["ros_ns"])

    def record_failure(self, stage, status_index=None):
        if self.failure_trace is not None:
            return
        index = status_index if status_index is not None else len(self.statuses) - 1
        self.failure_trace = {
            "stage": stage,
            "detected": self.clocks(),
            "status_index": index,
            "status": status_dict(self.statuses[index][1]) if index >= 0 else None,
            "last_published_sequence": self.sequence,
        }

    def save_trace(self):
        # No synchronous disk writes occur in callbacks, pumps or failed waits.
        # This is the single teardown write, after the ELF has stopped.
        statuses = [{"received_ros_ns": self.status_receipt_ros[index],
                     "received_steady_ns": receipt,
                     "message": status_dict(message)}
                    for index, (receipt, message) in enumerate(self.statuses)]
        self.trace_path.write_text(json.dumps({
            "schema": 1,
            "executable": str(Path(os.environ["SUPERVISOR_EXECUTABLE"]).resolve()),
            "boot_id": self.boot_id,
            "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
            "failure": self.failure_trace,
            "extra": self.extra_trace,
            "source_frames": self.source_trace,
            "commands": self.command_trace,
            "statuses": statuses,
        }, indent=2, allow_nan=False) + "\n")
        print("geometry supervisor trace: " + str(self.trace_path))

    def new_source(self, classification=None):
        if classification is not None:
            self.classification = classification
        self.frame += 1
        self.evaluation += 1
        self.source_ns = self.driver.get_clock().now().nanoseconds
        self.next_frame = time.monotonic() + .06
        self.source_trace.append({"frame_sequence": self.frame,
                                  "evaluation_sequence": self.evaluation,
                                  "source_ros_ns": self.source_ns,
                                  "classification": self.classification,
                                  "sampled": self.clocks()})

    def command(self):
        ros = self.driver.get_clock().now().nanoseconds
        steady = time.monotonic_ns()
        if self.command_style == "fresh":
            interval = (ros - 2000000, ros - 1000000,
                        steady - 2000000, steady - 1000000)
        else:
            assert self.frozen_interval is not None
            interval = self.frozen_interval
            if self.command_style == "cross":
                interval = (interval[0], ros - 1000000,
                            interval[2], steady - 1000000)
            elif self.command_style == "ros_before":
                interval = (interval[0], interval[1],
                            steady - 2000000, steady - 1000000)
            elif self.command_style == "steady_before":
                interval = (ros - 2000000, ros - 1000000,
                            interval[2], interval[3])
        self.sequence += 1
        origin = VelocityCommandEvidence()
        origin.schema_version = 1
        origin.controller_epoch_uuid = list(range(1, 17))
        origin.command_sequence = self.sequence
        origin.controller_goal_sequence = 1
        origin.clock_boot_id = self.boot_id
        origin.frame_id = "base_footprint"
        (origin.compute_begin_ros_ns, origin.compute_end_ros_ns,
         origin.compute_begin_steady_ns, origin.compute_end_steady_ns) = interval
        origin.generation_state = self.generation
        origin.origin_velocity.linear.x = self.vx
        origin.velocity = copy.deepcopy(origin.origin_velocity)
        controller_publish_begin = self.clocks()
        self.controller_pub.publish(origin)
        controller_publish_end = self.clocks()
        result = copy.deepcopy(origin)
        result.collision_epoch_uuid = list(range(17, 33))
        result.collision_sequence = self.sequence
        result.collision_state = self.collision_state
        result.collision_begin_ros_ns = ros - 500000
        result.collision_end_ros_ns = ros - 100000
        result.collision_begin_steady_ns = steady - 500000
        result.collision_end_steady_ns = steady - 100000
        if self.collision_state in (result.COLLISION_STOP, result.COLLISION_SOURCE_INVALID,
                                    result.COLLISION_ORIGIN_INVALID):
            result.velocity = Twist()
        result_publish_begin = self.clocks()
        self.command_pub.publish(result)
        result_publish_end = self.clocks()
        self.command_trace.append({
            "sequence": self.sequence,
            "generation_state": origin.generation_state,
            "command_style": self.command_style,
            "task_id": self.task_id,
            "navigation_state": self.navigation_state,
            "compute_begin_ros_ns": origin.compute_begin_ros_ns,
            "compute_end_ros_ns": origin.compute_end_ros_ns,
            "compute_begin_steady_ns": origin.compute_begin_steady_ns,
            "compute_end_steady_ns": origin.compute_end_steady_ns,
            "origin_velocity": twist_dict(origin.origin_velocity),
            "collision_state": result.collision_state,
            "collision_begin_ros_ns": result.collision_begin_ros_ns,
            "collision_end_ros_ns": result.collision_end_ros_ns,
            "collision_begin_steady_ns": result.collision_begin_steady_ns,
            "collision_end_steady_ns": result.collision_end_steady_ns,
            "velocity": twist_dict(result.velocity),
            "controller_publish_begin": controller_publish_begin,
            "controller_publish_end": controller_publish_end,
            "result_publish_begin": result_publish_begin,
            "result_publish_end": result_publish_end,
        })
        self.candidate_pub.publish(result.velocity)
        audit = TwistStamped()
        audit.header.stamp = self.driver.get_clock().now().to_msg()
        audit.header.frame_id = "collision_clear"
        audit.twist = copy.deepcopy(result.velocity)
        self.audit_pub.publish(audit)
        return result

    def freeze_generation(self):
        # Begin/end are an actual fixture sampling BEFORE the barrier. The
        # later collision receipt/process times never replace these fields.
        ros = self.driver.get_clock().now().nanoseconds
        steady = time.monotonic_ns()
        self.frozen_interval = (ros - 2000000, ros - 1000000,
                                steady - 2000000, steady - 1000000)
        self.command_style = "frozen"

    def publish_inputs(self):
        now = self.driver.get_clock().now()
        if self.send_platform:
            platform = PlatformSafetyState()
            platform.header.stamp = now.to_msg()
            for field in ("mode_allowed", "ethercat_op", "wkc_normal", "joint_feedback_fresh",
                          "leg_joints_error_free", "leg_joints_operation_enabled",
                          "motion_ready", "navigation_mode_discovered"):
                setattr(platform, field, True)
            platform.platform_fault = self.platform_fault
            self.platform_pub.publish(platform)
        if self.send_localization:
            localization = LocalizationStatus()
            localization.header.stamp = now.to_msg()
            localization.state = self.localization_state
            self.localization_pub.publish(localization)
        if self.send_navigation:
            navigation = NavigationStatus()
            navigation.header.stamp = now.to_msg()
            navigation.task_id = self.task_id
            navigation.state = self.navigation_state
            self.navigation_pub.publish(navigation)
        if self.send_tf:
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "base_footprint"
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)
        if self.auto_frames and time.monotonic() >= self.next_frame:
            self.new_source()
        # Sample the aggregate header AFTER possibly creating a newer source.
        # source > header would be a malformed fixture, not the intended test.
        now = self.driver.get_clock().now()
        coverage = ObstacleCoverageDecision()
        coverage.coverage.header.stamp = now.to_msg()
        coverage.coverage.header.frame_id = "base_footprint"
        coverage.coverage.profile = "fusion_shadow"
        clear = self.classification == 1
        coverage.coverage.healthy = clear
        coverage.coverage.forward_coverage = clear
        coverage.coverage.near_field_coverage = clear
        coverage.coverage.required_sources = ["airy", "chest", "belly"]
        coverage.coverage.healthy_sources = ["airy", "chest", "belly"]
        coverage.schema_version = 1
        coverage.disposition = self.classification
        coverage.non_region_healthy = True
        coverage.region_evaluation_complete = True
        coverage.region_evaluation_code = 1 if clear else 3
        coverage.region_disposition = self.classification
        coverage.region_source_stamp_ns = self.source_ns
        coverage.region_frame_sequence = self.frame
        coverage.region_evaluation_sequence = self.evaluation
        coverage.region_contract = CONTRACT
        self.publish_coverage(coverage)
        if self.send_commands:
            self.command()

    def publish_coverage(self, coverage):
        self.coverage_pub.publish(coverage)

    def pump(self):
        if time.monotonic() >= self.next_publish:
            self.next_publish = time.monotonic() + .015
            self.publish_inputs()
        self.executor.spin_once(timeout_sec=.002)
        assert self.process.poll() is None, "supervisor exited; see node log"
        assert not self.driver.get_publishers_info_by_topic(FINAL_TOPIC), (
            "shadow test must NEVER create a final velocity publisher")

    def until(self, predicate, timeout=4.0, after=None):
        after = len(self.statuses) if after is None else after
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            for index, (_, status) in enumerate(self.statuses[after:], start=after):
                assert not status.motion_output_connected
                # Expected hard-fault waits must match their predicate FIRST.
                if predicate(status):
                    return status
                if status.state == status.STATE_FAULT_LATCHED:
                    self.record_failure("unexpected_fault_in_until", index)
                    raise AssertionError("unexpected latched status: " + str(status) +
                                         "; teardown trace=" + str(self.trace_path))
        last = self.statuses[-1][1] if self.statuses else None
        self.record_failure("predicate_timeout")
        raise AssertionError("status predicate timed out; last=" + str(last))

    def assert_zero_window(self, seconds=.035):
        after = len(self.statuses)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()
        messages = [status for _, status in self.statuses[after:]]
        assert messages, "must observe actual timer output"
        assert all(is_zero(status.supervised) for status in messages)
        assert all(not status.motion_output_connected for status in messages)
        return messages

    def reset(self):
        return self.call(self.reset_client)

    def call(self, client):
        deadline = time.monotonic() + 5.0
        while not client.service_is_ready():
            assert time.monotonic() < deadline, "service not ready"
            self.pump()
        future = client.call_async(Trigger.Request())
        while not future.done():
            assert time.monotonic() < deadline, "service timed out"
            self.pump()
        return future.result()

    def arm(self):
        self.until(lambda status: status.state == status.STATE_DISARMED and
                   status.reason == "disarmed", timeout=8.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            response = self.reset()
            if response.success:
                break
            self.assert_zero_window(.025)
        else:
            raise AssertionError("could not arm: " + response.message)
        self.until(lambda status: status.state == status.STATE_ARMED_IDLE and not status.latched)

    def goal(self):
        self.task_id = "original-task-1"
        self.navigation_state = NavigationStatus.STATE_NAVIGATING
        self.generation = VelocityCommandEvidence.GENERATED_CONTROLLER
        self.vx = .03
        return self.until(lambda status: status.command_allowed and status.supervised.linear.x > 0)

    def enter_hold(self):
        self.new_source(2)
        return self.until(lambda status: status.reason == "geometry_hold" and
                          not status.command_allowed and not status.latched)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.save_trace()
        self.log.seek(0)
        print(self.log.read())
        self.log.close()
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        for key, value in self.saved_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        assert self.process.returncode == 0, "node did not exit cleanly after SIGINT"


@pytest.fixture
def harness(tmp_path):
    instance = Harness(tmp_path)
    try:
        yield instance
    except BaseException as error:
        instance.record_failure("test_exception:" + repr(error))
        raise
    finally:
        instance.close()


def test_real_node_hold_three_distinct_clear_frames_and_post_barrier_generation(harness):
    h = harness
    h.arm()
    h.goal()
    held = h.enter_hold()
    assert is_zero(held.supervised)
    h.auto_frames = False
    h.new_source(1)
    # Repeat status ticks from ONE new Clear frame cannot qualify recovery.
    h.assert_zero_window(.06)
    h.new_source(1)
    h.assert_zero_window(.06)
    # Freeze compute BEFORE publishing the third Clear; later result messages
    # carry increasing sequences and fresh collision times, but old generation.
    h.freeze_generation()
    h.new_source(1)
    h.auto_frames = True
    h.assert_zero_window(.03)
    h.command_style = "cross"
    h.assert_zero_window(.03)
    # A fresh synthetic zero also cannot discharge the Hold barrier.
    h.command_style = "fresh"
    h.generation = VelocityCommandEvidence.GENERATED_EXPLICIT_ZERO
    h.vx = 0.0
    synthetic = h.assert_zero_window(.025)
    assert any(status.reason == "geometry_hold" for status in synthetic)
    h.generation = VelocityCommandEvidence.GENERATED_CONTROLLER
    h.vx = .03
    resumed = h.until(lambda status: status.command_allowed and
                     status.supervised.linear.x > 0 and not status.latched)
    assert resumed.state == resumed.STATE_RUNNING
    # A real platform hard fault must override Hold in the current task.
    h.enter_hold()
    h.platform_fault = True
    fault = h.until(lambda status: status.state == status.STATE_FAULT_LATCHED)
    assert fault.latched and is_zero(fault.supervised)
    assert fault.reason == "platform_fault"
    h.platform_fault = False
    h.new_source(1)
    assert all(status.latched for status in h.assert_zero_window(.08))


@pytest.mark.parametrize("command_style", ["frozen", "ros_before", "steady_before"])
def test_pre_reset_pre_task_command_cannot_execute_under_a_new_task(harness, command_style):
    h = harness
    # This generation sampling precedes BOTH successful arm and task activation;
    # synthetic fresh zero startup messages do not change its origin timestamps.
    h.arm()
    h.freeze_generation()
    assert h.reset().success  # Explicit idle reset after the retained generation.
    h.command_style = command_style
    h.task_id = "new-task-after-arm"
    h.navigation_state = NavigationStatus.STATE_NAVIGATING
    h.generation = VelocityCommandEvidence.GENERATED_CONTROLLER
    h.vx = .03
    messages = h.assert_zero_window(.07)
    assert all(not status.command_allowed or is_zero(status.supervised) for status in messages)
    assert all(not status.latched for status in messages), "old generation is startup waiting"
    assert any(status.reason == "waiting_for_post_task_command" for status in messages)


def test_idle_reset_requires_complete_clear_and_no_active_task(harness):
    h = harness
    h.until(lambda status: status.state == status.STATE_DISARMED and
            status.reason == "disarmed", timeout=8.0)
    h.new_source(2)
    h.assert_zero_window(.08)
    response = h.reset()
    assert not response.success
    assert response.message == "geometry_coverage_must_be_fresh_clear_before_reset"
    h.new_source(1)
    h.arm()
    h.goal()
    response = h.reset()
    assert not response.success
    assert response.message == "navigation_must_be_idle_without_task_before_reset"


def test_normal_goal_end_zero_and_explicit_stop_remain_zero(harness):
    h = harness
    h.arm()
    h.goal()
    # Stop the controller before ending the navigation lease, as real orderly
    # completion requires; an in-flight nonzero after lease loss must fault.
    h.vx = 0.0
    h.generation = VelocityCommandEvidence.GENERATED_EXPLICIT_ZERO
    h.until(lambda status: status.state == status.STATE_ARMED_IDLE and
            not status.latched and is_zero(status.supervised))
    h.task_id = ""
    h.navigation_state = NavigationStatus.STATE_READY
    idle = h.until(lambda status: status.state == status.STATE_ARMED_IDLE and
                   not status.latched and is_zero(status.supervised))
    assert not idle.motion_output_connected
    response = h.call(h.stop_client)
    assert response.success
    stopped = h.until(lambda status: status.latched and status.reason == "operator_stop")
    assert is_zero(stopped.supervised)
    assert all(status.latched for status in h.assert_zero_window(.04))


@pytest.mark.parametrize("failure", [
    "coverage_hard_fault", "collision_stop", "platform_fault", "localization_lost",
    "tf_timeout", "command_timeout", "lease_timeout", "task_changed",
])
def test_hard_gates_remain_active_while_geometry_hold(harness, failure):
    h = harness
    h.arm()
    h.goal()
    h.enter_hold()
    if failure == "coverage_hard_fault":
        h.new_source(0)
    elif failure == "collision_stop":
        h.collision_state = VelocityCommandEvidence.COLLISION_STOP
    elif failure == "platform_fault":
        h.platform_fault = True
    elif failure == "localization_lost":
        h.localization_state = LocalizationStatus.STATE_LOST
    elif failure == "tf_timeout":
        h.send_tf = False
    elif failure == "command_timeout":
        h.send_commands = False
    elif failure == "lease_timeout":
        h.send_navigation = False
    elif failure == "task_changed":
        h.task_id = "unaccepted-replacement-task"
    fault = h.until(lambda status: status.state == status.STATE_FAULT_LATCHED, timeout=2.0)
    assert fault.latched and is_zero(fault.supervised)
    # An unrelated transport timeout must never count as the injected gate.
    expected_reasons = {
        "coverage_hard_fault": {"coverage_decision_hard_fault"},
        "collision_stop": {"collision_stop"},
        "platform_fault": {"platform_fault"},
        "localization_lost": {"localization_unhealthy"},
        "tf_timeout": {"tf_unhealthy"},
        # The steady receipt backstop runs before generation-age validation;
        # either can detect this same deliberate cessation on a 100 Hz tick.
        "command_timeout": {"geometry_recovery:candidate_stale_or_future",
                            "candidate_stale_steady_clock"},
        "lease_timeout": {"geometry_recovery:task_budget_invalid"},
        "task_changed": {"geometry_navigation_task_changed"},
    }[failure]
    assert fault.reason in expected_reasons, (failure, fault.reason, expected_reasons)
    assert all(is_zero(status.supervised) for status in h.assert_zero_window(.04))
