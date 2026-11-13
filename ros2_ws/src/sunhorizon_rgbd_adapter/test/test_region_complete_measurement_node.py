"""Completed measurements hold their original deadlines during input assembly.

Synthetic small rasters only; localhost domain, no motion publishers/services.
"""
import copy
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.serialization import serialize_message
from rclpy.executors import SingleThreadedExecutor
import yaml
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import RgbdStatus, ObstacleCoverageStatus, ObstacleCoverageDecision
from tf2_ros import StaticTransformBroadcaster
from test_region_depth_producer_node import canonical, parameters, number

COUNTS = ('tested_pixels', 'intersecting_rays', 'clear_rays', 'obstacle_rays',
          'invalid_depth_rays', 'front_occluded_rays', 'margin_band_rays')
FLAGS = ('rectified_pinhole', 'metric_axial_depth', 'depth_intrinsics_registration',
         'optical_to_base_extrinsics', 'source_time_transform')


def matching_coverage(states, start, expected, reason=None):
    """Wait for the entire assertion, not any unrelated unhealthy timer output."""
    if len(states) <= start:
        return None
    state = states[-1]
    if (state.healthy != expected or
            state.required_sources != state.healthy_sources or
            (reason is not None and reason not in state.reason)):
        return None
    return state


@pytest.mark.parametrize("typed_evaluation", [False, True])
def test_complete_measurement_hold_with_real_consumer(tmp_path, typed_evaluation):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # Honor an explicitly isolated owner domain; retain214 only as default.
    os.environ.setdefault('ROS_DOMAIN_ID', '214')
    rclpy.init()
    node = rclpy.create_node('region_hold_test')
    heartbeat_node = rclpy.create_node('region_hold_test_heartbeat')
    reports, states, decisions = [], [], []
    image_pub = node.create_publisher(Image, '/region_test/image', 10)
    info_pub = node.create_publisher(CameraInfo, '/region_test/info', 10)
    processed_pub = heartbeat_node.create_publisher(PointCloud2, '/region_test/processed_health', 10)
    rgbd_pub = heartbeat_node.create_publisher(RgbdStatus, '/region_test/rgbd', 10)
    airy_pub = heartbeat_node.create_publisher(PointCloud2, '/region_test/airy', 10)
    node.create_subscription(DiagnosticArray, '/region_test/support',
        lambda a: reports.extend({v.key: v.value for v in s.values} for s in a.status), 10)
    node.create_subscription(ObstacleCoverageStatus, '/region_test/coverage', states.append, 10)
    node.create_subscription(ObstacleCoverageDecision, '/sunhorizon/obstacle_coverage/decision_v1', decisions.append, 10)
    info = CameraInfo(); info.header.frame_id = 'camera'; info.width = info.height = 16
    info.distortion_model = 'plumb_bob'; info.d = [0.] * 5
    info.k = [20., 0., 8., 0., 20., 8., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.]
    params = parameters(info)
    params['region_hold_complete_measurement_enabled'] = True
    params['region_expected_contract'] += canonical(['complete-measurement-hold-v1'])
    if typed_evaluation:
        params['region_typed_evaluation_diagnostics_enabled'] = True
        params['region_expected_contract'] += canonical(['typed-region-evaluation-diagnostics-v1'])
    config = tmp_path / 'producer.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    consumer_params = dict(profile='fusion_shadow', frame_id='base', chest_frame_id='chest',
        belly_frame_id='camera', output_topic='/region_test/coverage',
        rgbd_status_topic='/region_test/rgbd', airy_points_topic='/region_test/airy',
        belly_obstacle_points_topic='/region_test/processed_health', belly_freshness_sec=.3,
        belly_obstacle_freshness_sec=.3, region_support_required=True,
        region_support_id='test_region',
        region_support_status_name='sunhorizon_depth_obstacle_cloud/region_support',
        region_support_contract=params['region_expected_contract'],
        region_support_diagnostics_topic='/region_test/support')
    if typed_evaluation:
        consumer_params['region_typed_evaluation_required'] = True
    consumer_config = tmp_path / 'coverage.yaml'
    consumer_config.write_text(yaml.safe_dump({'/**': {'ros__parameters': consumer_params}}))
    coverage_exe = os.environ.get('COVERAGE_EXECUTABLE') or str(Path(get_package_prefix(
        'sunhorizon_obstacle_coverage')) / 'lib/sunhorizon_obstacle_coverage/obstacle_coverage_node')
    processes, logs = [], []
    # Independent fixture producer: scenario waits and assertions must not
    # suspend the non-region health inputs whose freshness this test assumes.
    # Original25ms cadence and every production deadline remain unchanged.
    def publish_health():
        status = RgbdStatus()
        status.header.stamp = heartbeat_node.get_clock().now().to_msg()
        for field in ('chest_pair_fresh', 'chest_camera_info_valid', 'chest_alignment_accepted',
                      'chest_depth_scale_accepted', 'chest_pointcloud_healthy', 'belly_depth_fresh',
                      'belly_camera_info_valid', 'belly_pointcloud_healthy', 'belly_depth_scale_accepted'):
            setattr(status, field, True)
        rgbd_pub.publish(status)
        cloud = PointCloud2(); cloud.header.stamp = status.header.stamp
        cloud.header.frame_id = 'base'; cloud.width = cloud.height = 1
        cloud.point_step = cloud.row_step = 12
        cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z'))]
        cloud.data = struct.pack('<fff', 1., 1., 1.); airy_pub.publish(cloud)
        processed = copy.deepcopy(cloud); processed.header.frame_id = 'camera'
        processed_pub.publish(processed)

    heartbeat_executor = SingleThreadedExecutor(context=heartbeat_node.context)
    heartbeat_executor.add_node(heartbeat_node)
    heartbeat_timer = heartbeat_node.create_timer(.025, publish_health)
    heartbeat_thread = threading.Thread(target=heartbeat_executor.spin,
                                        name='region-test-heartbeat', daemon=True)

    def spin(seconds, done=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not done():
            assert heartbeat_thread.is_alive(), 'fixture heartbeat executor stopped'
            rclpy.spin_once(node, timeout_sec=.002)

    def image(raw=600, neighbor=None):
        msg = Image(); msg.header.frame_id = 'camera'; msg.header.stamp = node.get_clock().now().to_msg()
        msg.width = msg.height = 16; msg.step = 32; msg.encoding = '16UC1'
        data = [600] * 256; data[8*16+8] = raw
        if neighbor is not None: data[8*16+9] = neighbor
        msg.data = struct.pack('<256H', *data)
        return msg

    def source(msg): return str(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    def exact_info(msg):
        ci = copy.deepcopy(info); ci.header.stamp = copy.deepcopy(msg.header.stamp); return ci
    def result(msg, reason=None):
        def matches(r): return r['source_stamp_ns'] == source(msg) and (reason is None or r['reason'] == reason)
        spin(.2, lambda: any(matches(r) for r in reports))
        return [r for r in reports if matches(r)][-1]
    def paired(raw=600, neighbor=None):
        msg = image(raw, neighbor); info_pub.publish(exact_info(msg)); spin(.015); image_pub.publish(msg)
        return msg, result(msg)
    def healthy(expected, reason=None):
        start = len(states)
        spin(.15, lambda: matching_coverage(states, start, expected, reason) is not None)
        state = matching_coverage(states, start, expected, reason)
        assert state is not None, (
            f'Expected isolated region state healthy={expected}, reason={reason}; '
            f'latest={states[-1] if states else None}')
        return state

    def await_non_region_health():
        # A running25ms fixture publisher does not prove the consumer has
        # consumed fresh headers; queued old RGBD callbacks remain unavailable.
        start = len(states)
        after_ns = node.get_clock().now().nanoseconds
        def ready():
            if len(states) <= start:
                return False
            state = states[-1]
            stamp = state.header.stamp.sec * 10**9 + state.header.stamp.nanosec
            return (stamp >= after_ns and
                    set(state.required_sources) == {'airy', 'chest', 'belly'} and
                    state.required_sources == state.healthy_sources)
        spin(1., ready)
        assert ready(), f'Non-region inputs not fresh before typed frame: {states[-1] if states else None}'

    def typed_result(msg, report, expected_code, expected_disposition):
        # Match the actual producer source/evaluation, never just the newest timer output.
        def found():
            old = {s.header.stamp.sec * 10**9 + s.header.stamp.nanosec: s for s in states}
            ready = None
            for d in decisions:
                stamp = d.coverage.header.stamp.sec * 10**9 + d.coverage.header.stamp.nanosec
                if (d.region_source_stamp_ns == int(source(msg)) and
                    d.region_evaluation_sequence == int(report['region_evaluation_sequence']) and stamp in old):
                    # Check every matched record, including unavailable outputs;
                    # waiting for fresh health must not hide serialization/time faults.
                    assert serialize_message(d.coverage) == serialize_message(old[stamp])
                    assert d.region_source_stamp_ns <= stamp
                    if d.non_region_healthy:
                        ready = d
            return ready
        spin(.18, lambda: found() is not None)
        d = found()
        assert d is not None, (
            f'No fresh non-region Decision for source={source(msg)}, '
            f'evaluation={report["region_evaluation_sequence"]}; '
            f'latest coverage={states[-1] if states else None}; '
            f'latest decision={decisions[-1] if decisions else None}')
        assert d.schema_version == 1 and d.region_contract == params['region_expected_contract']
        assert d.non_region_healthy and d.region_evaluation_complete
        assert d.region_frame_sequence == int(report['region_frame_sequence'])
        assert d.region_evaluation_code == expected_code
        assert d.region_disposition == expected_disposition and d.disposition == expected_disposition
        assert d.coverage.healthy == (expected_disposition == 1)
        if expected_disposition != 1:
            assert not d.coverage.forward_coverage and not d.coverage.near_field_coverage
        return d

    try:
        heartbeat_thread.start()
        for exe, cfg, name in ((os.environ['DEPTH_OBSTACLE_EXECUTABLE'], config, 'producer'),
                               (coverage_exe, consumer_config, 'coverage')):
            log = (tmp_path / (name + '.log')).open('w+'); logs.append(log)
            processes.append(subprocess.Popen([exe, '--ros-args', '--params-file', str(cfg)],
                stdout=log, stderr=subprocess.STDOUT))
        spin(12, lambda: image_pub.get_subscription_count() == 1 and info_pub.get_subscription_count() == 1
             and rgbd_pub.get_subscription_count() == 1 and airy_pub.get_subscription_count() == 1)
        assert all(p.poll() is None for p in processes)
        spin(.2)
        _, missing = paired()
        assert missing['reason'] == 'exact_source_tf_unavailable' and missing['state'] == 'Unknown'
        tf_pub = StaticTransformBroadcaster(node)
        transforms = []
        for child in ('camera', 'chest'):
            tf = TransformStamped(); tf.header.frame_id = 'base'; tf.child_frame_id = child
            tf.header.stamp = node.get_clock().now().to_msg(); tf.transform.rotation.w = 1.
            transforms.append(tf)
        count = len(reports); tf_pub.sendTransform(transforms); spin(.1)
        assert len(reports) == count  # No second lookup/retry of failed pair.
        # With no completed sample, an incomplete image cannot authorize clearance.
        pending = image(); image_pub.publish(pending); before = len(reports); spin(.05)
        assert len(reports) == before
        healthy(False)
        info_pub.publish(exact_info(pending)); assert result(pending)['state'] == 'ClearSupported'
        healthy(True)

        # New incomplete input does not revoke a fresh COMPLETE measurement.
        complete, clear = paired(); assert clear['state'] == 'ClearSupported'; healthy(True)
        pending = image(425); count = len(reports); image_pub.publish(pending); spin(.04)
        wrong = exact_info(pending); wrong.header.stamp.nanosec += 1
        info_pub.publish(wrong); spin(.02)
        assert len(reports) == count
        healthy(True)
        info_pub.publish(exact_info(pending))
        assert result(pending)['state'] == 'ObstacleSupported'
        healthy(False)

        # Silence never refreshes source/local receipt clocks. Original .3s expiry
        # must close even while all non-region coverage sources remain fresh.
        complete, _ = paired(); healthy(True)
        pending = image(); count = len(reports); image_pub.publish(pending)
        for _ in range(4):
            spin(.085)
            newer = image(); image_pub.publish(newer)
        assert len(reports) == count
        assert 'region_stale' in healthy(False, 'region_stale').reason
        # An overwritten old pending pair cannot re-arm even if its Info arrives.
        info_pub.publish(exact_info(pending)); spin(.025)
        assert len(reports) == count
        pending = image(); image_pub.publish(pending); spin(.32)
        info_pub.publish(exact_info(pending))
        assert 'stale' in result(pending)['reason']
        healthy(False)

        # A newer incomplete input cannot hide a contradiction of the active pair.
        complete, _ = paired(); healthy(True)
        pending = image(); image_pub.publish(pending); spin(.025)
        bad = exact_info(complete); bad.k[0] += 1.; info_pub.publish(bad)
        assert result(complete, 'conflicting_held_camera_info')['state'] == 'Unknown'
        healthy(False)
        count = len(reports); info_pub.publish(exact_info(pending)); spin(.035)
        assert len(reports) == count  # Revocation discards the incomplete pair.
        healthy(False)

        complete, _ = paired(); healthy(True)
        invalid = image(); invalid.encoding = '8UC1'; image_pub.publish(invalid)
        assert result(invalid, 'invalid_region_image_layout')['state'] == 'Unknown'
        healthy(False)
        _, recovered = paired(); assert recovered['state'] == 'ClearSupported'; healthy(True)
        if typed_evaluation:
            # Actual 16UC1 -> producer core -> typed tracker -> complete Decision.
            # Box axial exit is .45m, clear boundary .47m; centre460mm is margin.
            for raw, neighbor, code, disposition in (
                    (600, None, 1, 1), (460, None, 3, 2),
                    (0, None, 3, 0), (460, 0, 3, 0),
                    (460, 425, 3, 0), (425, None, 2, 0), (600, None, 1, 1)):
                await_non_region_health()
                msg, actual = paired(raw, neighbor)
                assert actual['evaluation_complete'] == 'true'
                assert actual['evaluation_code'] == str(code)
                assert actual['disposition'] == str(disposition)
                typed_result(msg, actual, code, disposition)
                healthy(disposition == 1)
        else:
            assert not decisions
            assert node.count_publishers('/sunhorizon/obstacle_coverage/decision_v1') == 0
        assert all(r['reason'] != 'pending_exact_info' for r in reports)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        for log in logs: log.seek(0); print(log.read()); log.close()
        heartbeat_timer.cancel()
        heartbeat_executor.shutdown(timeout_sec=5.)
        if heartbeat_thread.ident is not None:
            heartbeat_thread.join(timeout=5.)
        assert not heartbeat_thread.is_alive(), 'fixture heartbeat thread did not stop'
        heartbeat_node.destroy_node()
        node.destroy_node(); rclpy.shutdown()
    assert all(p.returncode == 0 for p in processes), 'SIGINT cleanup must be clean for both real nodes'
