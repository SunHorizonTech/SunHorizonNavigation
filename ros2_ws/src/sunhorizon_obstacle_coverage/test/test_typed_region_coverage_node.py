"""Real isolated ROS node; typed evidence never changes the legacy healthy gate."""
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import (
    RgbdStatus, ObstacleCoverageStatus, ObstacleCoverageDecision,
)
from tf2_ros import StaticTransformBroadcaster

TOPIC = '/sunhorizon/obstacle_coverage/decision_v1'
MARKER = 'typed-region-evaluation-diagnostics-v1'
CONTRACT = '10:fixture-v1' + str(len(MARKER)) + ':' + MARKER


def params(typed=True):
    return {'profile': 'fusion_shadow', 'output_topic': '/typed_region_test/status',
            'rgbd_status_topic': '/typed_region_test/rgbd',
            'airy_points_topic': '/typed_region_test/airy',
            'belly_freshness_sec': '0.3', 'belly_obstacle_freshness_sec': '0.3',
            'belly_obstacle_points_topic': '/typed_region_test/belly',
            'region_support_required': 'true', 'region_support_id': 'test-region',
            'region_support_status_name': 'test/region_support',
            'region_support_contract': CONTRACT,
            'region_support_diagnostics_topic': '/typed_region_test/region',
            **({'region_typed_evaluation_required': 'true'} if typed else {})}


def command(values, remap=None):
    args = [os.environ['COVERAGE_EXECUTABLE'], '--ros-args']
    for key, value in values.items():
        args += ['-p', key + ':=' + value]
    if remap:
        args += ['-r', remap]
    return args


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Harness:
    def __init__(self, tmp_path, typed, overrides=None):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['ROS_DOMAIN_ID'] = str(208 + os.getpid() % 8)
        rclpy.init()
        self.node = rclpy.create_node('typed_region_coverage_test')
        self.old, self.new = [], []
        self.node.create_subscription(ObstacleCoverageStatus, '/typed_region_test/status', self.old.append, 10)
        self.node.create_subscription(ObstacleCoverageDecision, TOPIC, self.new.append, 10)
        self.rgbd = self.node.create_publisher(RgbdStatus, '/typed_region_test/rgbd', 10)
        self.belly = self.node.create_publisher(PointCloud2, '/typed_region_test/belly', 10)
        self.airy = self.node.create_publisher(PointCloud2, '/typed_region_test/airy', 10)
        self.region = self.node.create_publisher(DiagnosticArray, '/typed_region_test/region', 10)
        self.broadcaster = StaticTransformBroadcaster(self.node)
        transforms = []
        for child in ('camera_chest_color_optical_frame', 'camera_belly_depth_optical_frame'):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base_footprint', child
            tf.header.stamp = self.node.get_clock().now().to_msg()
            tf.transform.rotation.w = 1.
            transforms.append(tf)
        self.broadcaster.sendTransform(transforms)
        self.log = (tmp_path / 'node.log').open('w+')
        values = params(typed)
        values.update(overrides or {})
        self.process = subprocess.Popen(command(values), stdout=self.log, stderr=subprocess.STDOUT)
        self.sequence = 0
        self.next_publish = 0.
        self.last_region = None
        self.phase_first_input_ns = None

    def pump(self, kind='clear', send_region=True, send_belly=True, send_airy=True,
             raw_age=0., repeat_region=False, send_rgbd=True, processed_age=0.,
             region_source_age=0.):
        if time.monotonic() >= self.next_publish:
            self.next_publish = time.monotonic() + .025
            now = self.node.get_clock().now()
            if self.phase_first_input_ns is None:
                self.phase_first_input_ns = now.nanoseconds
            status = RgbdStatus()
            status.header.stamp = now.to_msg()
            for name in ('chest_pair_fresh', 'chest_camera_info_valid', 'chest_alignment_accepted',
                         'chest_depth_scale_accepted', 'chest_pointcloud_healthy', 'belly_depth_fresh',
                         'belly_camera_info_valid', 'belly_pointcloud_healthy', 'belly_depth_scale_accepted'):
                setattr(status, name, True)
            status.belly_depth_age_sec = raw_age
            status.belly_pointcloud_age_sec = raw_age
            if send_rgbd:
                self.rgbd.publish(status)
            for pub, frame, enabled in ((self.belly, 'camera_belly_depth_optical_frame', send_belly),
                                        (self.airy, 'base_footprint', send_airy)):
                if not enabled:
                    continue
                cloud = PointCloud2()
                cloud.header.stamp, cloud.header.frame_id = now.to_msg(), frame
                if pub is self.belly and processed_age:
                    n = now.nanoseconds - int(processed_age * 10**9)
                    cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(n, 10**9)
                cloud.height = cloud.width = 1
                cloud.point_step = cloud.row_step = 12
                cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                for i, n in enumerate(('x', 'y', 'z'))]
                cloud.data = struct.pack('<fff', .1, .1, 1.)
                pub.publish(cloud)
            if send_region:
                if repeat_region:
                    assert self.last_region is not None
                    self.region.publish(self.last_region)
                else:
                    self.sequence += 1
                    n, steady = now.nanoseconds, time.monotonic_ns()
                    source = n - int(region_source_age * 10**9)
                    clear, obstacle = kind == 'clear', kind == 'obstacle'
                    mixed = kind == 'mixed'
                    fields = dict(diagnostic_event='region_depth_support', schema_version=1,
                        contract_encoding='length-prefixed-ascii-v1', contract=CONTRACT,
                        region_id='test-region', base_frame_id='base_footprint',
                        image_frame_id='camera_belly_depth_optical_frame',
                        region_frame_sequence=self.sequence, region_evaluation_sequence=self.sequence,
                        source_stamp_ns=source, image_receipt_ros_ns=n, image_receipt_steady_ns=steady,
                        evaluation_ros_ns=n, evaluation_steady_ns=steady,
                        paired_info_stamp_ns=source, tf_requested_stamp_ns=source,
                        state='ClearSupported' if clear else ('ObstacleSupported' if obstacle else 'Unknown'),
                        reason='human_readable_only', tested_pixels=10, intersecting_rays=10,
                        clear_rays=10 if clear else (9 if obstacle else (8 if mixed else 9)),
                        obstacle_rays=1 if obstacle else 0,
                        invalid_depth_rays=1 if mixed else 0, front_occluded_rays=0,
                        margin_band_rays=0 if clear or obstacle else 1,
                        region_wall_ns=0, region_thread_cpu_ns=0, evaluation_complete='true',
                        evaluation_code=1 if clear else (2 if obstacle else 3),
                        disposition=1 if clear else (0 if obstacle or mixed else 2))
                    entry = DiagnosticStatus(name='test/region_support', level=bytes([0 if clear else 1]))
                    entry.values = [KeyValue(key=k, value=str(v)) for k, v in fields.items()]
                    msg = DiagnosticArray()
                    msg.header.stamp = now.to_msg()
                    msg.status = [entry]
                    self.last_region = msg
                    self.region.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=.005)
        assert self.process.poll() is None, 'coverage process exited'

    def matching(self, after):
        old = {stamp_ns(x.header.stamp): x for x in self.old}
        for message in reversed(self.new[after:]):
            stamp = stamp_ns(message.coverage.header.stamp)
            if stamp in old:
                # ROS message equality compares every field recursively. CDR
                # alignment padding can differ between serializations even
                # when both snapshots contain exactly the same field values.
                assert message.coverage == old[stamp]
                assert message.schema_version == 1 and message.region_contract == CONTRACT
                assert message.region_source_stamp_ns <= stamp
                return message
        return None

    def until(self, predicate, timeout=3., **inputs):
        self.phase_first_input_ns = None
        after, deadline = len(self.new), time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump(**inputs)
            message = self.matching(after)
            if message is not None and predicate(message):
                return message
        raise AssertionError('typed decision did not meet predicate; last=' + str(self.matching(after)))

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.seek(0)
        print(self.log.read())
        self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        assert self.process.returncode == 0, 'coverage cleanup failed'


def test_same_calculation_typed_decision_preserves_legacy_stop_and_source_priority(tmp_path):
    h = Harness(tmp_path, True)
    try:
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy, timeout=12.)
        hold = h.until(lambda d: d.disposition == 2, kind='margin')
        assert hold.non_region_healthy and hold.region_evaluation_complete
        assert hold.region_evaluation_code == 3 and hold.region_disposition == 2
        assert not hold.coverage.healthy and not hold.coverage.forward_coverage
        assert not hold.coverage.near_field_coverage and not hold.coverage.rear_coverage
        assert hold.region_source_stamp_ns > 0
        # Timer publishes of one cached measurement retain its source/frame/evaluation.
        # Drain in-flight publisher frames until at least two new outputs share the last sent sequence.
        expected = h.sequence
        after, deadline = len(h.new), time.monotonic() + .24
        repeated = []
        while time.monotonic() < deadline:
            h.pump(kind='margin', send_region=False)
            repeated = [d for d in h.new[after:] if d.region_frame_sequence == expected and d.disposition == 2]
            if len(repeated) >= 2:
                break
        assert len(repeated) >= 2
        assert len({(d.region_source_stamp_ns, d.region_frame_sequence, d.region_evaluation_sequence)
                    for d in repeated}) == 1
        # Source callbacks and region reports arrive independently. HardFault
        # also covers a temporarily unhealthy raw/processed source; isolate
        # these region failures only after all non-region gates are healthy.
        hard = h.until(lambda d: d.non_region_healthy and d.region_evaluation_complete
                       and d.disposition == 0 and d.region_evaluation_code == 3,
                       kind='mixed')
        assert hard.non_region_healthy and hard.region_evaluation_complete
        assert hard.region_disposition == 0 and not hard.coverage.healthy
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        obstacle = h.until(lambda d: d.non_region_healthy and d.region_evaluation_complete
                           and d.disposition == 0 and d.region_evaluation_code == 2,
                           kind='obstacle')
        assert obstacle.non_region_healthy and not obstacle.coverage.healthy
        # New fresh pure margin cannot override stale original raw health or missing processed input.
        for bad in ({'raw_age': .4}, {'send_belly': False}, {'send_airy': False}):
            d = h.until(lambda d: not d.non_region_healthy and d.region_disposition == 2,
                        kind='margin', **bad)
            assert d.disposition == 0 and not d.coverage.healthy
            h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        stale = h.until(lambda d: not d.region_evaluation_complete, send_region=False)
        assert stale.disposition == 0 and stale.region_disposition == 0
        assert stale.region_evaluation_code == 0 and not stale.coverage.healthy
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        # Replayed reports (unlike timer snapshots) must remain a sticky protocol fault.
        h.until(lambda d: not d.region_evaluation_complete, repeat_region=True)
        last = h.until(lambda d: not d.region_evaluation_complete)
        assert last.disposition == 0 and not last.coverage.healthy
    finally:
        h.close()


def test_default_does_not_create_typed_publisher(tmp_path):
    h = Harness(tmp_path, False)
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            h.pump()
            if h.old and h.old[-1].healthy:
                break
        assert h.old and h.old[-1].healthy
        assert h.node.count_publishers(TOPIC) == 0 and not h.new
    finally:
        h.close()


@pytest.mark.parametrize('change,remap', [
    ({'region_support_required': 'false'}, None),
    ({'profile': 'visual'}, None),
    ({'belly_obstacle_points_topic': ''}, None),
    ({'region_support_contract': 'not-typed'}, None),
    ({}, TOPIC + ':=/typed_region_test/foreign_decision'),
    ({'output_topic': TOPIC}, None),
])
def test_invalid_typed_configuration_fails_before_any_decision(tmp_path, change, remap):
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(208 + os.getpid() % 8))
    p = params()
    p.update(change)
    # ROS CLI treats an empty value as syntax; pass an explicit YAML empty string.
    if p['belly_obstacle_points_topic'] == '':
        p['belly_obstacle_points_topic'] = "''"
    with (tmp_path / 'rejected.log').open('w+') as log:
        child = subprocess.Popen(command(p, remap), stdout=log, stderr=subprocess.STDOUT, env=environment)
        try:
            rc = child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)
            pytest.fail('invalid typed configuration did not exit')
        log.seek(0)
        text = log.read()
        print(text)
        assert rc != 0
        assert ('typed region evidence requires' in text or 'region support requires' in text or
                'typed_region_contract_suffix_missing' in text or 'typed decision topic must remain fixed' in text)
