"""Real isolated ROS candidate, exact pairing/TF/immutable expiry and revocation.

No production services, motion topics, or claims of physical model calibration.
"""
import copy
import hashlib
import itertools
import os
import re
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from sunhorizon_navigation_interfaces.msg import SpatialCoverageEvidence, ObstacleCoverageStatus
import yaml
from test_region_depth_producer_node import canonical, number

IMAGE = '/sunhorizon/raw/rgbd/belly/depth/image'
INFO = '/sunhorizon/raw/rgbd/belly/depth/camera_info'
PRIVATE = '/sunhorizon/navigation/belly_self/tf'
OUTPUT = '/sunhorizon/obstacle_coverage/spatial_evidence_v1'
# Reserved CI domain range for this file: 100..121. Dynamic filter tests
# reserve 200..219; parallel CTest targets must never share those domains.
DOMAINS = itertools.count(100)


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def calibration(info):
    fields = ['region-camera-v1', info.header.frame_id, str(info.width), str(info.height),
              info.distortion_model, str(len(info.d))]
    fields += [number(x) for array in (info.d, info.k, info.r, info.p) for x in array]
    fields += ['0', '0', '0', '0', '0', '0', 'false']
    return canonical(fields)


class Harness:
    def __init__(self, root, params=None, enabled=True, publish_global_tf=True):
        self.process = self.log = self.node = None
        self.initialized = False
        try:
            self._setup(root, params, enabled, publish_global_tf)
        except BaseException:
            self.close()
            raise

    def _setup(self, root, params, enabled, publish_global_tf):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        domain = next(DOMAINS)
        assert domain <= 121
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        print('spatial synthetic domain:', domain)
        rclpy.init()
        self.initialized = True
        self.node = rclpy.create_node('spatial_candidate_test')
        self.rows = []
        self.identities = []
        self.node.create_subscription(DiagnosticArray, '/sunhorizon/obstacle_coverage/spatial_identity_v1',
            lambda report: self.identities.extend({item.key: item.value for item in status.values} for status in report.status),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(SpatialCoverageEvidence, OUTPUT, self.rows.append, 10)
        self.image_pub = self.node.create_publisher(Image, IMAGE, 10)
        self.info_pub = self.node.create_publisher(CameraInfo, INFO, 10)
        self.tf_pub = self.node.create_publisher(TFMessage, PRIVATE, 10)
        self.global_tf = StaticTransformBroadcaster(self.node)
        self.spin(.1)
        def gid(topic):
            self.spin(3, lambda: len(self.node.get_publishers_info_by_topic(topic)) == 1)
            entries = self.node.get_publishers_info_by_topic(topic)
            assert len(entries) == 1
            return bytes(entries[0].endpoint_gid).hex()
        info = CameraInfo(); info.header.frame_id = 'spatial_optical'; info.width = info.height = 5
        info.distortion_model = 'plumb_bob'; info.d = [0.] * 5
        info.k = [10., 0., 2., 0., 10., 2., 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [10., 0., 2., 0., 0., 10., 2., 0., 0., 0., 1., 0.]
        self.info = info
        stl = root / 'triangle.stl'
        vertices = (-10., -10., .5, 10., -10., .5, 0., 10., .5)
        data = bytes(80) + struct.pack('<I', 1) + struct.pack('<12fH', 0., 0., 1., *vertices, 0)
        stl.write_bytes(data)
        query_text = 'spatial-query-v1;' + canonical(['base_link'])
        for minimum, maximum in zip((-.1, -.1, 1.), (.1, .1, 2.)):
            query_text += canonical([canonical([number(minimum), number(maximum)])])
        settings = dict(enabled=enabled, contract='c' * 64, query_digest=hashlib.sha256(query_text.encode()).hexdigest(),
            producer_epoch='producer-test', clock_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            common_frame='base_link', cell_min=[-.1, -.1, 1.], cell_max=[.1, .1, 2.], self_tolerance_m=.02, clear_margin_m=.01,
            model_mesh_paths=[str(stl.resolve())] * 12, model_mesh_sha256=[hashlib.sha256(data).hexdigest()] * 12,
            model_triangle_counts=[1] * 12, model_link_names=['spatial_leg_%d' % i for i in range(12)],
            model_parent_frames=['base_link'] * 12, private_tf_publisher='/spatial_candidate_test', private_tf_gid=gid(PRIVATE),
            belly_frame='spatial_optical', belly_epoch='camera-test', belly_calibration=calibration(info),
            belly_image_publisher='/spatial_candidate_test', belly_info_publisher='/spatial_candidate_test',
            belly_image_gid=gid(IMAGE), belly_info_gid=gid(INFO), belly_depth_scale_m=.001,
            belly_minimum_depth_m=.1, belly_maximum_depth_m=5.)
        for flag in ('rectified_pinhole_verified', 'metric_axial_depth_verified', 'depth_intrinsics_registration_verified',
                     'optical_to_base_extrinsics_verified', 'source_time_and_transform_verified'):
            settings['belly_' + flag] = True
        settings.update(params or {})
        if settings.get('publisher_identity_mode') == 'startup-pin-once-v1':
            for key in ('belly_image_gid', 'belly_info_gid', 'private_tf_gid'):
                settings[key] = ''
        config = root / 'params.yaml'; config.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
        self.log = (root / 'node.log').open('w+')
        self.process = subprocess.Popen([os.environ['SPATIAL_VISIBILITY_EXECUTABLE'], '--ros-args', '--params-file', str(config)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.spin(8, lambda: self.process.poll() is not None or (
            self.image_pub.get_subscription_count() == (1 if enabled else 0) and
            self.info_pub.get_subscription_count() == (1 if enabled else 0) and
            self.tf_pub.get_subscription_count() == (1 if enabled else 0)))
        if self.process.poll() is not None:
            raise AssertionError(self.text())
        if enabled:
            assert self.image_pub.get_subscription_count() == 1 and self.info_pub.get_subscription_count() == 1
            assert self.tf_pub.get_subscription_count() == 1
        # A matched DDS subscription can precede ROS graph name metadata.
        # These single-shot measurement cases start after discovery warmup;
        # unqualified discovery itself must never count as accepted data.
        if publish_global_tf:
            self.publish_global_transforms()
        self.spin(.5)

    def publish_global_transforms(self):
        tf = TransformStamped(); tf.header.frame_id = 'base_link'; tf.child_frame_id = 'spatial_optical'
        tf.header.stamp = self.node.get_clock().now().to_msg(); tf.transform.rotation.w = 1.
        base = TransformStamped(); base.header.frame_id = 'odom'; base.child_frame_id = 'base_link'
        base.header.stamp = copy.deepcopy(tf.header.stamp); base.transform.rotation.w = 1.
        self.global_tf.sendTransform([base, tf])

    def text(self):
        self.log.flush(); return Path(self.log.name).read_text()

    def spin(self, duration, done=lambda: False):
        end = time.monotonic() + duration
        while time.monotonic() < end and not done():
            rclpy.spin_once(self.node, timeout_sec=.002)

    def transforms(self, stamp, missing=False, lag_ns=0):
        transforms = []
        for delta in (-1000000, 1000000):
            value = ns(stamp) + delta - lag_ns
            for i in range(11 if missing else 12):
                tf = TransformStamped(); tf.header.frame_id = 'base_link'; tf.child_frame_id = 'spatial_leg_%d' % i
                tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(value, 10**9)
                tf.transform.rotation.w = 1.; transforms.append(tf)
        return TFMessage(transforms=transforms)

    def pair(self, raw=3000, missing_tf=False, delayed_tf=False, delay_info=0, source_offset_ns=0,
             tf_lag_ns=0, publish_tf=True, wait=.25):
        image = Image(); image.header.frame_id = 'spatial_optical'; image.header.stamp = self.node.get_clock().now().to_msg()
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(ns(image.header.stamp) + source_offset_ns, 10**9)
        image.width = image.height = 5; image.step = 10; image.encoding = '16UC1'; image.data = struct.pack('<25H', *([raw] * 25))
        info = copy.deepcopy(self.info); info.header.stamp = copy.deepcopy(image.header.stamp)
        if not delayed_tf and publish_tf:
            self.tf_pub.publish(self.transforms(image.header.stamp, missing_tf, tf_lag_ns)); self.spin(.005)
        if not delay_info:
            self.info_pub.publish(info); self.spin(.003)
        self.image_pub.publish(image)
        worker = None
        if delayed_tf and publish_tf:
            def deliver():
                time.sleep(.004); self.tf_pub.publish(self.transforms(image.header.stamp, missing_tf))
            worker = threading.Thread(target=deliver); worker.start()
        if delay_info:
            self.spin(delay_info); self.info_pub.publish(info)
        self.spin(wait, lambda: any(row.anchor_source_ros_ns == ns(image.header.stamp) for row in self.rows) or 'spatial terminal:' in self.text())
        if worker: worker.join(timeout=1); assert not worker.is_alive()
        return image, info

    def row(self, image):
        values = [row for row in self.rows if row.anchor_source_ros_ns == ns(image.header.stamp)]
        assert values, self.text()
        return values[-1]

    def close(self):
        # Construction failures also arrive here: only this owned child is
        # interrupted, and a partially initialized context is always released.
        timeout = False
        try:
            if self.process is not None and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try: self.process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=2); timeout = True
        finally:
            if self.log is not None:
                self.log.close(); self.log = None
            if self.node is not None:
                self.node.destroy_node(); self.node = None
            if self.initialized:
                rclpy.shutdown(); self.initialized = False
        if timeout: pytest.fail('owned synthetic node did not drain after SIGINT')


@pytest.mark.parametrize('raw,counts', [(3000, (1, 0, 0)), (1500, (0, 1, 0)), (500, (0, 0, 1)), (0, (0, 0, 1))])
def test_exact_anchor_clear_occupied_self_and_invalid(tmp_path, raw, counts):
    h = Harness(tmp_path)
    try:
        image, _ = h.pair(raw)
        row = h.row(image)
        assert row.evaluation_complete and (row.free_cells, row.occupied_cells, row.unknown_cells) == counts
        assert row.input_generation == row.anchor_frame_sequence == row.sources[0].frame_sequence
        source = row.sources[0]
        assert source.source_ros_ns == row.anchor_source_ros_ns == ns(image.header.stamp)
        assert source.geometry_valid and source.first_receipt_ros_ns >= source.source_ros_ns
        assert source.valid_until_ros_ns == min(source.source_ros_ns + 300000000, source.first_receipt_ros_ns + 300000000)
        assert source.valid_until_steady_ns-source.first_receipt_steady_ns == source.valid_until_ros_ns-source.first_receipt_ros_ns
        old = len(h.rows); h.spin(.36)
        assert len(h.rows) == old  # No timer publication/expiry extension.
    finally: h.close()


def test_private_tf_callback_can_progress_during_image_lookup(tmp_path):
    h = Harness(tmp_path, {'publisher_identity_mode': 'startup-pin-once-v1'})
    try:
        # Prime only publisher ownership with an EARLIER transform sample.
        # It cannot cover the future test anchor; the actual later source TF
        # must still arrive on the independent callback during bounded lookup.
        deadline = time.monotonic() + 1.
        while not any(row.get('private.tf.gid') for row in h.identities) and time.monotonic() < deadline:
            earlier = h.node.get_clock().now().to_msg()
            value = ns(earlier) - 100000000
            earlier.sec, earlier.nanosec = divmod(value, 10**9)
            h.tf_pub.publish(h.transforms(earlier))
            h.spin(.02, lambda: any(row.get('private.tf.gid') for row in h.identities))
        assert any(row.get('private.tf.gid') for row in h.identities), h.text()
        image, _ = h.pair(delayed_tf=True)
        assert ns(image.header.stamp) > ns(earlier) + 1000000
        assert h.row(image).evaluation_complete and h.row(image).free_cells == 1
    finally: h.close()


def test_late_conflicting_info_revokes_same_original_anchor(tmp_path):
    h = Harness(tmp_path)
    try:
        image, info = h.pair(); original = copy.deepcopy(h.row(image)); assert original.free_cells == 1
        info.k[0] += 1.; h.info_pub.publish(info)
        h.spin(.3, lambda: len(h.rows) > 1)
        revoked = h.rows[-1]
        assert not revoked.evaluation_complete and revoked.reason.startswith('terminal_revocation:')
        assert revoked.anchor_source_ros_ns == original.anchor_source_ros_ns
        assert revoked.input_generation == original.input_generation
        assert revoked.sources == original.sources and revoked.evaluation_sequence == original.evaluation_sequence + 1
        before = len(h.rows); h.pair(); h.spin(.05); assert len(h.rows) == before
    finally: h.close()


def test_all_invalid_depth_does_not_bypass_missing_leg_tf(tmp_path):
    h = Harness(tmp_path)
    try:
        first, _ = h.pair(); assert h.row(first).evaluation_complete
        before = len(h.rows)
        h.pair(0, missing_tf=True)
        h.spin(.3, lambda: len(h.rows) > before and not h.rows[-1].evaluation_complete)
        assert len(h.rows) == before + 1 and not h.rows[-1].evaluation_complete
        assert h.rows[-1].anchor_source_ros_ns == ns(first.header.stamp)
        assert 'spatial terminal:' in h.text()
    finally: h.close()


def test_belly_before_global_tf_waits_without_evidence_and_uses_fresh_new_anchor(tmp_path):
    h = Harness(tmp_path, {'publisher_identity_mode': 'startup-pin-once-v1'}, publish_global_tf=False)
    try:
        initial, _ = h.pair()
        assert not h.rows and 'spatial pending required TF' in h.text()
        assert 'spatial terminal:' not in h.text() and h.process.poll() is None
        # The failed anchor is consumed, never re-stamped or replayed when TF
        # appears. Wait past its expiry before supplying the global chain.
        h.spin(.32); h.publish_global_transforms(); h.spin(.05)
        assert not h.rows
        fresh, _ = h.pair(); row = h.row(fresh)
        assert row.evaluation_complete and row.free_cells == 1
        assert row.anchor_source_ros_ns == ns(fresh.header.stamp) > ns(initial.header.stamp)
        assert row.sources[0].source_ros_ns == ns(fresh.header.stamp)
        assert row.sources[0].first_receipt_ros_ns >= ns(fresh.header.stamp)
        assert not any(value.anchor_source_ros_ns == ns(initial.header.stamp) for value in h.rows)
        assert 'spatial required source-time TF qualified' in h.text() and 'spatial terminal:' not in h.text()
    finally: h.close()


def test_initial_missing_leg_tf_can_qualify_unknown_then_tf_failure_is_permanent(tmp_path):
    h = Harness(tmp_path)
    try:
        initial, _ = h.pair(0, missing_tf=True)
        assert not h.rows and 'spatial pending required TF' in h.text()
        assert 'spatial terminal:' not in h.text()
        # Qualification requires all twelve source-time links even when every
        # depth pixel is invalid, and is independent of a first Clear result.
        fresh, _ = h.pair(0); original = copy.deepcopy(h.row(fresh))
        assert original.evaluation_complete and original.free_cells == 0 and original.unknown_cells == 1
        assert original.anchor_source_ros_ns > ns(initial.header.stamp)
        failed, _ = h.pair(0, missing_tf=True)
        h.spin(.3, lambda: h.rows and not h.rows[-1].evaluation_complete)
        assert 'spatial terminal:' in h.text()
        revoked = h.rows[-1]
        assert not revoked.evaluation_complete and revoked.reason.startswith('terminal_revocation:')
        assert revoked.anchor_source_ros_ns == original.anchor_source_ros_ns != ns(failed.header.stamp)
        assert revoked.sources == original.sources and revoked.input_generation == original.input_generation
        count = len(h.rows); h.pair(); h.spin(.1)
        assert len(h.rows) == count and h.text().count('spatial required source-time TF qualified') == 1
    finally: h.close()


def test_qualified_future_leg_tf_waits_for_same_anchor_without_new_clear_or_expiry(tmp_path):
    h = Harness(tmp_path)
    try:
        first, _ = h.pair(); previous = copy.deepcopy(h.row(first)); h.spin(.08)
        held, _ = h.pair(tf_lag_ns=40_000_000, wait=.04)
        assert 'spatial pending future leg TF' in h.text() and len(h.rows) == 1
        new, _ = h.pair(publish_tf=False, wait=.04)
        assert len(h.rows) == 1 and h.rows[0] == previous
        assert 'spatial pending anchor full dropped=1' in h.text()
        h.tf_pub.publish(h.transforms(held.header.stamp))
        h.spin(.2, lambda: any(row.anchor_source_ros_ns == ns(held.header.stamp) for row in h.rows))
        result = h.row(held)
        assert result.evaluation_complete and result.free_cells == 1
        assert result.anchor_frame_sequence == previous.anchor_frame_sequence + 1
        source = result.sources[0]
        assert source.source_ros_ns == ns(held.header.stamp)
        assert source.first_receipt_ros_ns < ns(new.header.stamp)
        assert source.valid_until_ros_ns == ns(held.header.stamp) + 300_000_000
        assert source.valid_until_steady_ns - source.first_receipt_steady_ns == source.valid_until_ros_ns - source.first_receipt_ros_ns
        assert not any(row.anchor_source_ros_ns == ns(new.header.stamp) for row in h.rows)
        assert 'spatial terminal:' not in h.text()
    finally: h.close()


def test_qualified_pending_anchor_cannot_be_replaced_or_renewed_by_new_pairs(tmp_path):
    h = Harness(tmp_path)
    try:
        first, _ = h.pair(); previous = copy.deepcopy(h.row(first)); h.spin(.08)
        held, _ = h.pair(tf_lag_ns=40_000_000, wait=.03)
        assert len(h.rows) == 1 and 'spatial pending future leg TF' in h.text()
        for _ in range(6):
            h.pair(publish_tf=False, wait=.02)
            assert len(h.rows) == 1 and h.rows[0] == previous
        h.spin(.3, lambda: 'spatial terminal:' in h.text())
        assert 'anchor_stale_source_age' in h.text() and 'spatial terminal:' in h.text()
        h.spin(.1, lambda: len(h.rows) == 2)
        revoked = h.rows[-1]
        assert not revoked.evaluation_complete and revoked.reason.startswith('terminal_revocation:')
        assert revoked.anchor_source_ros_ns == previous.anchor_source_ros_ns != ns(held.header.stamp)
        assert revoked.sources == previous.sources and revoked.input_generation == previous.input_generation
        h.spin(.1, lambda: 'spatial timing end kind=producer_terminal' in h.text())
        timing = [dict((key, int(value)) for key, value in re.findall(r'(\w+)=(-?\d+)', line))
                  for line in re.findall(r'spatial timing frame ([^\n]+)', h.text())]
        pending = next(row for row in timing if row['source_ns'] == ns(held.header.stamp))
        assert pending['last_future_link_index'] >= 0
        assert 0 < pending['last_future_latest_upper_ns'] < pending['last_future_source_query_ns'] == ns(held.header.stamp)
        assert pending['last_future_wait_ns'] > 0  # Historical query survives the final stale-before-lookup attempt.
        count = len(h.rows)
        h.tf_pub.publish(h.transforms(held.header.stamp)); h.pair(); h.spin(.1)
        assert len(h.rows) == count
    finally: h.close()


@pytest.mark.parametrize('failure', ['parent', 'owner'])
def test_private_fault_revokes_even_while_qualified_anchor_waits(tmp_path, failure):
    h = Harness(tmp_path)
    try:
        first, _ = h.pair(); previous = copy.deepcopy(h.row(first)); h.spin(.08)
        held, _ = h.pair(tf_lag_ns=40_000_000, wait=.03)
        assert len(h.rows) == 1 and 'spatial pending future leg TF' in h.text()
        if failure == 'parent':
            wrong = h.transforms(held.header.stamp)
            wrong.transforms[0].header.frame_id = 'unapproved_parent'
            h.tf_pub.publish(wrong)
        else:
            duplicate = h.node.create_publisher(TFMessage, PRIVATE, QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            h.spin(.03)
            duplicate.publish(h.transforms(held.header.stamp))
        h.spin(.2, lambda: len(h.rows) == 2)
        if failure == 'parent':
            assert 'private_leg_tf_identity' in h.text()
        else:
            assert 'publisher_identity' in h.text() or 'frozen_publisher_missing_or_changed' in h.text()
        assert 'spatial terminal:' in h.text()
        revoked = h.rows[-1]
        assert not revoked.evaluation_complete and revoked.anchor_source_ros_ns == previous.anchor_source_ros_ns
        assert revoked.sources == previous.sources
    finally: h.close()


def test_timing_hint_is_observational_and_terminal_dumps_frozen_window_and_live_tail_once(tmp_path):
    h = Harness(tmp_path)
    try:
        publisher = h.node.create_publisher(ObstacleCoverageStatus, '/sunhorizon/obstacle_coverage/status', 10)
        h.spin(.5, lambda: publisher.get_subscription_count() == 1)
        assert publisher.get_subscription_count() == 1
        hint = ObstacleCoverageStatus(); hint.reason = 'spatial_visibility:spatial_frozen_evidence_expired'
        hint.header.stamp = h.node.get_clock().now().to_msg()
        publisher.publish(hint); h.spin(.03)  # No complete spatial output yet: ignored.
        first, _ = h.pair(); previous = copy.deepcopy(h.row(first))
        hint.header.stamp.sec, hint.header.stamp.nanosec = divmod(ns(previous.header.stamp) - 1, 10**9)
        publisher.publish(hint); h.spin(.03)  # Old consumer sample: ignored.
        hint.header.stamp = h.node.get_clock().now().to_msg(); trigger = ns(hint.header.stamp)
        publisher.publish(hint); h.spin(.04)
        assert h.rows == [previous] and 'spatial timing begin' not in h.text()
        second, _ = h.pair(); second_original = copy.deepcopy(h.row(second)); assert second_original.evaluation_complete
        hint.header.stamp = h.node.get_clock().now().to_msg()
        publisher.publish(hint); h.spin(.03)  # Cannot replace the frozen hint.
        bad = h.transforms(second.header.stamp); bad.transforms[0].header.frame_id = 'unapproved_parent'
        h.tf_pub.publish(bad)
        # stderr serialization can finish before the terminal DDS callback is
        # dispatched in this harness. Both channels must complete within the
        # same bounded window; a previous complete row is not terminal proof.
        def terminal_received():
            return (bool(h.rows) and not h.rows[-1].evaluation_complete and
                    h.rows[-1].anchor_source_ros_ns == second_original.anchor_source_ros_ns and
                    h.rows[-1].anchor_frame_sequence == second_original.anchor_frame_sequence and
                    h.rows[-1].sources == second_original.sources)
        h.spin(.3, lambda: h.text().count('spatial timing end') == 2 and terminal_received())
        assert terminal_received(), (h.rows, h.text())
        text = h.text()
        assert text.count('spatial timing begin') == text.count('spatial timing end') == 2
        headers = re.findall(r'spatial timing begin ([^\n]+)', text)
        assert 'kind=first_observed_expiry_hint records=1' in headers[0]
        assert 'trigger_d_ns=%d' % trigger in headers[0]
        assert 'kind=producer_terminal records=2' in headers[1]
        frames = [dict((key, int(value)) for key, value in re.findall(r'(\w+)=(-?\d+)', line))
                  for line in re.findall(r'spatial timing frame ([^\n]+)', text)]
        row = frames[0]
        assert row['source_ns'] == ns(first.header.stamp) and row['seq'] == previous.anchor_frame_sequence
        assert row['callback_ros_ns'] == previous.sources[0].first_receipt_ros_ns
        assert row['callback_steady_ns'] == previous.sources[0].first_receipt_steady_ns
        assert 'rmw_source_ns' in row and 'rmw_received_ns' in row
        assert row['paired_ns'] <= row['first_evaluate_ns'] <= row['model_start_ns'] <= row['model_end_ns']
        assert row['sampling_end_ns'] <= row['owners_start_ns'] <= row['owners_end_ns'] <= row['publish_start_ns'] <= row['publish_return_ns']
        assert not h.rows[-1].evaluation_complete and h.rows[-1].sources == second_original.sources
        h.spin(.1, lambda: h.text().count('spatial private_tf timing end') == 2)
        private_text = h.text()
        assert private_text.count('spatial private_tf timing begin') == private_text.count('spatial private_tf timing end') == 2
        private_headers = re.findall(r'spatial private_tf timing begin ([^\n]+)', private_text)
        assert 'kind=first_observed_expiry_hint' in private_headers[0] and 'trigger_d_ns=%d' % trigger in private_headers[0]
        assert 'link0=spatial_leg_0' in private_headers[0]
        private_rows = [dict((key, int(value)) for key, value in re.findall(r'(\w+)=(-?\d+)', line))
                        for line in re.findall(r'spatial private_tf timing frame ([^\n]+)', private_text)]
        complete = next(row for row in private_rows if row['callback_end_ns'] and row['inserted_count'] >= 12)
        assert 'rmw_source_ns' in complete and 'rmw_received_ns' in complete  # Vendor may report 0.
        assert complete['callback_steady_ns'] <= complete['owner_start_ns'] <= complete['owner_end_ns'] <= complete['inserted_end_ns'] <= complete['callback_end_ns']
        assert complete['link0_count'] > 0 and complete['link0_min_source_ns'] <= complete['link0_max_source_ns']
        assert complete['message_count'] == complete['scanned_count']
        h.pair(); h.spin(.03)
        assert h.text().count('spatial timing begin') == 2
        assert h.text().count('spatial private_tf timing begin') == 2
    finally: h.close()


def test_old_pending_image_receipt_does_not_refresh_when_info_arrives(tmp_path):
    h = Harness(tmp_path)
    try:
        image, _ = h.pair(delay_info=.32)
        assert not h.rows and 'anchor_stale' in h.text()
        match = re.search(r'spatial anchor rejection phase=evaluate_entry predicate=anchor_stale_source_age ([^\n]+)', h.text())
        assert match, h.text()
        values = dict(re.findall(r'(\w+)=(-?\d+)', match.group(1)))
        assert int(values['source_ns']) == ns(image.header.stamp)
        assert int(values['source_age_ns']) == int(values['now_ros_ns']) - ns(image.header.stamp) > 300000000
        assert int(values['receipt_ros_age_ns']) == int(values['now_ros_ns']) - int(values['image_receipt_ros_ns'])
        assert int(values['receipt_steady_age_ns']) == int(values['now_steady_ns']) - int(values['image_receipt_steady_ns'])
        assert values['required_tf_qualified'] == values['private_fault'] == '0'
    finally: h.close()


def test_future_anchor_keeps_time_order_rejection_and_original_integer_times(tmp_path):
    h = Harness(tmp_path)
    try:
        image, _ = h.pair(source_offset_ns=2000000000)
        assert not h.rows and 'spatial terminal: evaluate_entry:anchor_stale_time_order' in h.text()
        match = re.search(r'spatial anchor rejection phase=evaluate_entry predicate=anchor_stale_time_order ([^\n]+)', h.text())
        assert match, h.text()
        values = dict(re.findall(r'(\w+)=(-?\d+)', match.group(1)))
        assert int(values['source_ns']) == ns(image.header.stamp) > int(values['image_receipt_ros_ns'])
        assert int(values['source_age_ns']) == int(values['now_ros_ns']) - ns(image.header.stamp) < 0
    finally: h.close()


def test_private_tf_error_retains_original_link_stamp_on_terminal(tmp_path):
    h = Harness(tmp_path)
    try:
        stamp = h.node.get_clock().now().to_msg()
        message = h.transforms(stamp)
        message.transforms[0].header.frame_id = 'contradictory_parent'
        expected = 'checked_child=spatial_leg_0 checked_parent=contradictory_parent checked_sec=%d checked_nanosec=%d history_stamp_ns=-1' % (
            message.transforms[0].header.stamp.sec, message.transforms[0].header.stamp.nanosec)
        deadline = time.monotonic() + 1.
        while 'spatial private TF rejection' not in h.text() and time.monotonic() < deadline:
            h.tf_pub.publish(message); h.spin(.02)
        text = h.text()
        assert 'spatial private TF rejection reason=private_leg_tf_identity is_static=0' in text
        assert expected in text and 'spatial terminal:' in text and not h.rows
        assert h.process.poll() is None  # Terminal evidence owner stays available for explicit cleanup.
        before = text.count('spatial private TF rejection'); h.pair(); h.spin(.05)
        assert not h.rows and h.text().count('spatial private TF rejection') == before
    finally: h.close()


def test_duplicate_image_publisher_cannot_supply_evidence(tmp_path):
    h = Harness(tmp_path)
    extra = h.node.create_publisher(Image, IMAGE, 10)
    try:
        h.spin(.15); h.pair()
        assert not h.rows and 'publisher_identity' in h.text()
    finally:
        h.node.destroy_publisher(extra); h.close()


def test_default_disabled_has_no_observation_endpoints(tmp_path):
    h = Harness(tmp_path, enabled=False)
    try:
        assert h.image_pub.get_subscription_count() == h.info_pub.get_subscription_count() == 0
        assert not h.node.get_publishers_info_by_topic(OUTPUT)
    finally: h.close()


@pytest.mark.parametrize('replacement', [False, True])
def test_startup_freezes_callback_owners_once_and_never_repins(tmp_path, replacement):
    h = Harness(tmp_path, {'publisher_identity_mode': 'startup-pin-once-v1'})
    extra = None
    try:
        image, _ = h.pair()
        original = copy.deepcopy(h.row(image)); assert original.evaluation_complete
        h.spin(.2, lambda: any(row.get('binding_complete') == 'true' for row in h.identities))
        bound = [row for row in h.identities if row.get('binding_complete') == 'true'][-1]
        assert bound['terminal'] == 'false' and int(bound['pid']) == h.process.pid
        frozen = bound['belly.image.gid']
        assert frozen == bytes(h.node.get_publishers_info_by_topic(IMAGE)[0].endpoint_gid).hex()
        assert bound['producer.evidence.gid'] == bytes(h.node.get_publishers_info_by_topic(OUTPUT)[0].endpoint_gid).hex()
        if replacement:
            h.node.destroy_publisher(h.image_pub)
            h.spin(.15)
            h.image_pub = h.node.create_publisher(Image, IMAGE, 10)
            h.spin(.2, lambda: len(h.node.get_publishers_info_by_topic(IMAGE)) == 1 and
                   bytes(h.node.get_publishers_info_by_topic(IMAGE)[0].endpoint_gid).hex() != frozen)
        else:
            extra = h.node.create_publisher(Image, IMAGE, 10)
            h.spin(.2, lambda: len(h.node.get_publishers_info_by_topic(IMAGE)) == 2)
        h.pair()
        h.spin(.2, lambda: any(row.get('terminal') == 'true' for row in h.identities) and
               any(not row.evaluation_complete for row in h.rows))
        assert not h.rows[-1].evaluation_complete
        terminal = [row for row in h.identities if row.get('terminal') == 'true'][-1]
        assert terminal['binding_complete'] == 'false' and terminal['belly.image.gid'] == frozen
        assert h.rows[-1].sources == original.sources
    finally:
        if extra is not None: h.node.destroy_publisher(extra)
        h.close()


def test_frozen_owner_silence_revokes_without_new_measurement(tmp_path):
    h = Harness(tmp_path, {'publisher_identity_mode': 'startup-pin-once-v1'})
    try:
        image, _ = h.pair(); original = copy.deepcopy(h.row(image)); assert original.evaluation_complete
        h.node.destroy_publisher(h.image_pub)
        # No image, Info or private TF is published after owner disappearance.
        h.spin(1., lambda: any(row.get('terminal') == 'true' for row in h.identities) and
               any(not row.evaluation_complete for row in h.rows))
        revoked = h.rows[-1]
        assert not revoked.evaluation_complete and revoked.anchor_source_ros_ns == original.anchor_source_ros_ns
        assert revoked.sources == original.sources and revoked.input_generation == original.input_generation
        terminal = [row for row in h.identities if row.get('terminal') == 'true'][-1]
        assert terminal['binding_complete'] == 'false'
        before = len(h.rows); h.spin(.15); assert len(h.rows) == before
    finally: h.close()
