"""Actual schema-2 supervisor path, localhost DDS and final output disconnected.

Only synthetic inputs are published. Reuse the existing command/lease fixtures,
but freeze the COMPLETE spatial payload once per belly anchor. A timer updates
only the aggregate publication header, exactly as the real coverage node does.
"""
import copy
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from sunhorizon_navigation_interfaces.msg import (
    NavigationStatus, ObstacleCoverageDecision, ObstacleCoverageSpatialDecision,
    SpatialCoverageEvidence, SpatialSourceEvidence, VelocityCommandEvidence,
)
from test_geometry_hold_supervisor_node import (
    Harness, SPATIAL_CONTRACT, SPATIAL_TOPIC, is_zero, qos,
)

AGE_NS = 300_000_000


class SpatialHarness(Harness):
    def __init__(self, tmp_path):
        self.spatial_wire = None
        self.chest_snapshot = None
        self.chest_only_free = False
        self.upstream_healthy = True
        self.wrapper_schema = 2
        self.wrong_topic_type = False
        self.coverage_publications = []
        super().__init__(tmp_path, spatial=True)

    def source(self, source_id, source_ns, sequence):
        clocks = self.clocks()
        value = SpatialSourceEvidence()
        value.source_id = source_id
        value.producer_epoch = "camera-epoch" if source_id == 0 else "chest-epoch"
        value.frame_id = "base_footprint"
        value.frame_sequence = sequence
        value.source_ros_ns = source_ns
        value.first_receipt_ros_ns = clocks["ros_ns"]
        value.first_receipt_steady_ns = clocks["steady_ns"]
        value.valid_until_ros_ns = source_ns + AGE_NS
        value.valid_until_steady_ns = clocks["steady_ns"] + (
            value.valid_until_ros_ns - clocks["ros_ns"])
        value.contribution = value.CONTRIBUTES_FREE
        value.geometry_valid = True
        return value

    def new_source(self, classification=None):
        super().new_source(classification)
        # This object is never rebuilt on status-timer publications.
        value = SpatialCoverageEvidence()
        value.schema_version = 1
        value.contract = SPATIAL_CONTRACT
        value.query_digest = "spatial-query-sha"
        value.producer_epoch = "spatial-fixture-epoch"
        value.clock_boot_id = self.boot_id
        value.header.frame_id = "base_footprint"
        value.anchor_source_ros_ns = self.source_ns
        value.anchor_frame_sequence = self.frame
        value.evaluation_sequence = self.evaluation
        value.input_generation = self.frame
        value.evaluation_complete = True
        value.total_cells = 4
        value.free_cells = 4 if self.classification == 1 else 3
        value.unknown_cells = 1 if self.classification == 2 else 0
        value.occupied_cells = 1 if self.classification == 0 else 0
        belly = self.source(0, self.source_ns, self.frame)
        if self.chest_only_free:
            belly.contribution = 0
        if value.occupied_cells:
            belly.contribution = belly.CONTRIBUTES_FREE | belly.CONTRIBUTES_OCCUPIED
        value.sources = [belly]
        if self.chest_snapshot is not None:
            value.sources.append(copy.deepcopy(self.chest_snapshot))
        clocks = self.clocks()
        value.header.stamp.sec, value.header.stamp.nanosec = divmod(clocks["ros_ns"], 10**9)
        value.evaluation_steady_ns = clocks["steady_ns"]
        value.reason = "clear" if self.classification == 1 else "occupied" if self.classification == 0 else "unknown"
        self.spatial_wire = value
        self.source_trace[-1]["spatial_dependencies"] = [{
            "id": s.source_id, "sequence": s.frame_sequence,
            "source_ros_ns": s.source_ros_ns,
            "first_receipt_ros_ns": s.first_receipt_ros_ns,
            "first_receipt_steady_ns": s.first_receipt_steady_ns,
            "valid_until_ros_ns": s.valid_until_ros_ns,
            "valid_until_steady_ns": s.valid_until_steady_ns,
            "contribution": s.contribution,
        } for s in value.sources]

    def publish_coverage(self, old):
        if self.wrong_topic_type:
            self.coverage_pub.publish(old)
            return
        envelope = ObstacleCoverageSpatialDecision()
        # Match the coverage implementation: all obsolete region fields stay
        # empty. The wrapper alone carries the distinct spatial contract.
        envelope.decision.coverage = copy.deepcopy(old.coverage)
        envelope.decision.schema_version = self.wrapper_schema
        envelope.decision.non_region_healthy = self.upstream_healthy
        envelope.decision.disposition = self.classification if self.upstream_healthy else 0
        if not self.upstream_healthy:
            envelope.decision.coverage.healthy = False
            envelope.decision.coverage.forward_coverage = False
            envelope.decision.coverage.near_field_coverage = False
            envelope.decision.coverage.healthy_sources = ["airy", "chest"]
        envelope.spatial = copy.deepcopy(self.spatial_wire)
        self.coverage_publications.append({
            "publication_ros_ns": self.driver.get_clock().now().nanoseconds,
            "spatial_header_ros_ns": self.spatial_wire.header.stamp.sec * 10**9 + self.spatial_wire.header.stamp.nanosec,
            "anchor_sequence": self.spatial_wire.anchor_frame_sequence,
            "disposition": envelope.decision.disposition,
        })
        self.coverage_pub.publish(envelope)

    def save_trace(self):
        self.extra_trace["spatial_publications"] = self.coverage_publications
        super().save_trace()


@pytest.fixture
def spatial_harness(tmp_path):
    h = SpatialHarness(tmp_path)
    try:
        yield h
    except BaseException as error:
        h.record_failure("test_exception:" + repr(error))
        raise
    finally:
        h.close()


def test_spatial_wait_requires_three_clear_frames_and_new_controller_generation(spatial_harness):
    h = spatial_harness
    h.arm(); h.goal()
    held = h.enter_hold()
    assert is_zero(held.supervised) and not held.latched
    original_task = h.task_id
    h.auto_frames = False
    h.new_source(1); h.assert_zero_window(.06)
    h.new_source(1); h.assert_zero_window(.06)
    h.freeze_generation()
    h.new_source(1); h.auto_frames = True
    h.assert_zero_window(.035)
    h.command_style = "cross"; h.assert_zero_window(.025)
    h.command_style = "fresh"
    h.generation = VelocityCommandEvidence.GENERATED_CONTROLLER
    resumed = h.until(lambda s: s.command_allowed and s.supervised.linear.x > 0 and not s.latched)
    assert resumed.state == resumed.STATE_RUNNING and h.task_id == original_task


def test_fresh_belly_and_publication_headers_cannot_renew_contributing_chest(spatial_harness):
    h = spatial_harness
    h.arm(); h.goal()
    chest_source = h.driver.get_clock().now().nanoseconds
    h.chest_snapshot = h.source(1, chest_source, 1)
    h.chest_only_free = True
    h.new_source(1)
    begin = len(h.coverage_publications)
    expiry = h.chest_snapshot.valid_until_steady_ns
    fault = h.until(lambda s: s.state == s.STATE_FAULT_LATCHED)
    assert fault.latched and is_zero(fault.supervised)
    assert "spatial_" in fault.reason and ("expired" in fault.reason or "stale" in fault.reason)
    assert time.monotonic_ns() >= expiry
    records = h.coverage_publications[begin:]
    assert len({r["anchor_sequence"] for r in records}) >= 2
    assert len({r["publication_ros_ns"] for r in records}) >= 3
    # Fresh replacement evidence cannot clear the fault in the active task.
    h.chest_snapshot = None; h.chest_only_free = False; h.new_source(1)
    assert all(s.latched and is_zero(s.supervised) for s in h.assert_zero_window(.05))


def test_recovered_fresh_coverage_requires_explicit_idle_reset(spatial_harness):
    h = spatial_harness
    h.arm(); h.goal()
    h.auto_frames = False
    fault = h.until(lambda s: s.state == s.STATE_FAULT_LATCHED)
    assert "spatial_frozen_evidence_expired" in fault.reason
    assert fault.latched and is_zero(fault.supervised)
    h.new_source(1); h.auto_frames = True
    assert all(s.latched and is_zero(s.supervised) for s in h.assert_zero_window(.08))
    response = h.reset()
    assert not response.success
    assert response.message == "navigation_must_be_idle_without_task_before_reset"
    h.vx = 0.0
    h.generation = VelocityCommandEvidence.GENERATED_EXPLICIT_ZERO
    h.task_id = ""
    h.navigation_state = NavigationStatus.STATE_READY
    assert all(s.latched and is_zero(s.supervised) for s in h.assert_zero_window(.08))
    response = h.reset()
    assert response.success and response.message == "armed_idle"
    idle = h.until(lambda s: s.state == s.STATE_ARMED_IDLE and not s.latched)
    assert is_zero(idle.supervised) and not idle.motion_output_connected


@pytest.mark.parametrize("fault_kind", ["upstream", "occupied", "legacy_schema", "terminal_revoke"])
def test_spatial_real_hard_conditions_latch_and_do_not_resume(spatial_harness, fault_kind):
    h = spatial_harness
    h.arm(); h.goal(); h.enter_hold()
    if fault_kind == "upstream":
        h.upstream_healthy = False
    elif fault_kind == "occupied":
        h.new_source(0)
    elif fault_kind == "legacy_schema":
        h.wrapper_schema = 1
    else:
        h.auto_frames = False
        h.spatial_wire.evaluation_complete = False
        h.spatial_wire.reason = "exact_info_conflict"
    fault = h.until(lambda s: s.state == s.STATE_FAULT_LATCHED)
    assert fault.latched and is_zero(fault.supervised)
    expected = {"upstream": "spatial_upstream_hard_fault",
                "occupied": "spatial_occupied",
                "legacy_schema": "spatial_wrapper_identity_mismatch",
                "terminal_revoke": "spatial_structure_or_evaluation_invalid"}[fault_kind]
    assert expected in fault.reason
    h.upstream_healthy = True; h.wrapper_schema = 2; h.auto_frames = True; h.new_source(1)
    assert all(s.latched and is_zero(s.supervised) for s in h.assert_zero_window(.05))


def test_schema_one_dds_type_is_not_accepted_on_spatial_topic(spatial_harness):
    h = spatial_harness
    h.arm(); h.goal()
    h.coverage.destroy_publisher(h.coverage_pub)
    h.coverage_pub = h.coverage.create_publisher(ObstacleCoverageDecision, SPATIAL_TOPIC, qos(depth=1))
    h.wrong_topic_type = True
    fault = h.until(lambda s: s.state == s.STATE_FAULT_LATCHED)
    assert is_zero(fault.supervised) and fault.latched
    assert ("typed_publisher_contract:" + SPATIAL_TOPIC in fault.reason or
            "spatial_frozen_evidence_expired" in fault.reason)


def test_attended_command_generation_and_stream_timeouts(tmp_path, monkeypatch):
    import os
    import subprocess
    from pathlib import Path
    from rcl_interfaces.srv import GetParameters

    executable = os.environ['SUPERVISOR_EXECUTABLE']
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', str(185 + os.getpid() % 20))
    profile = Path(__file__).resolve().parents[1] / 'config/attended_heading_correction_navigation.yaml'
    args = [executable, '--ros-args', '--params-file', str(profile),
            '-r', '/navigation/cmd_vel:=/test/unused_output']
    for value in ('motion_output_enabled:=true', 'spatial_visibility_enabled:=true',
                  'geometry_hold_enabled:=true', 'geometry_hold_region_contract:=test',
                  'spatial_obstacle_source_mode:=rgbd-only-v1', 'spatial_query_digest:=test',
                  'spatial_producer_epoch:=test', 'spatial_source_ids:=[0]',
                  'spatial_source_epochs:=[test]', 'spatial_source_frames:=[base_footprint]',
                  'camera_freshness_profile:=attended-low-speed-1000ms-v1',
                  'command_generation_timeout_sec:=0.5',
                  'final_output_lock_path:=' + str(tmp_path / 'output.lock')):
        args.extend(['-p', value])
    rclpy.init()
    node = rclpy.create_node('read_command_timeouts')
    child = subprocess.Popen(args)
    try:
        client = node.create_client(GetParameters, '/sunhorizon_velocity_supervisor/get_parameters')
        assert client.wait_for_service(timeout_sec=5)
        request = GetParameters.Request()
        request.names = ['command_generation_timeout_sec', 'candidate_timeout_sec']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert [v.double_value for v in future.result().values] == [.5, .25]
        assert node.count_publishers('/navigation/cmd_vel') == 0
    finally:
        child.terminate()
        child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
