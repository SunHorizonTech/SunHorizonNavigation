"""Conditional raw-ray diagnostics never authorize the real coverage consumer.

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
from rclpy.executors import SingleThreadedExecutor
import yaml
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import RgbdStatus, ObstacleCoverageStatus
from tf2_ros import StaticTransformBroadcaster
from test_region_depth_producer_node import canonical, parameters, number

COUNTS = ('tested_pixels', 'intersecting_rays', 'clear_rays', 'obstacle_rays',
          'invalid_depth_rays', 'front_occluded_rays', 'margin_band_rays')
FLAGS = ('rectified_pinhole', 'metric_axial_depth', 'depth_intrinsics_registration',
         'optical_to_base_extrinsics', 'source_time_transform')


@pytest.mark.parametrize("typed_evaluation", [False, True])
def test_shadow_rays_revoke_and_never_release_actual_coverage(tmp_path, typed_evaluation):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 10)
    rclpy.init()
    node = rclpy.create_node('region_shadow_test')
    heartbeat_node = rclpy.create_node('region_shadow_test_heartbeat')
    reports, states = [], []
    image_pub = node.create_publisher(Image, '/region_test/image', 10)
    info_pub = node.create_publisher(CameraInfo, '/region_test/info', 10)
    rgbd_pub = heartbeat_node.create_publisher(RgbdStatus, '/region_test/rgbd', 10)
    airy_pub = heartbeat_node.create_publisher(PointCloud2, '/region_test/airy', 10)
    processed_pub = heartbeat_node.create_publisher(PointCloud2, '/region_test/processed_health', 10)
    node.create_subscription(DiagnosticArray, '/region_test/support',
        lambda a: reports.extend({v.key: v.value for v in s.values} for s in a.status), 10)
    node.create_subscription(ObstacleCoverageStatus, '/region_test/coverage', states.append, 10)
    info = CameraInfo(); info.header.frame_id = 'camera'; info.width = info.height = 16
    info.distortion_model = 'plumb_bob'; info.d = [0.] * 5
    info.k = [20., 0., 8., 0., 20., 8., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.]
    params = parameters(info)
    # Replace only the attestation sequence in this pinned synthetic contract.
    params['region_expected_contract'] = params['region_expected_contract'].replace(
        canonical(['true'] * 5), canonical(['false'] * 5))
    for flag in FLAGS: params['region_' + flag + '_accepted'] = False
    params['region_shadow_only_enabled'] = True
    params['region_tf_wait_sec'] = .02
    params['region_expected_contract'] += canonical(
        ['bounded-exact-tf-wait-v1', number(.02), 'shadow-only-ray-diagnostics-v1'])
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
    consumer_config = tmp_path / 'coverage.yaml'
    consumer_config.write_text(yaml.safe_dump({'/**': {'ros__parameters': consumer_params}}))
    coverage_exe = os.environ.get('COVERAGE_EXECUTABLE') or str(Path(get_package_prefix(
        'sunhorizon_obstacle_coverage')) / 'lib/sunhorizon_obstacle_coverage/obstacle_coverage_node')
    processes, logs = [], []
    # Keep independent non-region health live while the scenario checks failed,
    # incomplete or shadow-only measurements. Do not borrow the producer under
    # test's projected output to establish this separate test precondition.
    # The original25ms cadence and all production freshness limits stay fixed.
    def publish_health():
        status = RgbdStatus(); status.header.stamp = heartbeat_node.get_clock().now().to_msg()
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
                                        name='region-shadow-heartbeat', daemon=True)

    def spin(seconds, done=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not done():
            assert heartbeat_thread.is_alive(), 'fixture heartbeat executor stopped'
            rclpy.spin_once(node, timeout_sec=.002)

    def coverage_stamp(row):
        return row.header.stamp.sec * 10**9 + row.header.stamp.nanosec

    def non_region_sources_fresh(row):
        # The consumer derives healthy_sources only after checking original
        # header and receipt ages. A live heartbeat thread alone is insufficient:
        # an old RGBD callback can still report chest/belly queue_delay.
        return (set(row.required_sources) == {'airy', 'chest', 'belly'} and
                row.required_sources == row.healthy_sources)

    def await_non_region_health():
        start = len(states)
        after_ns = node.get_clock().now().nanoseconds
        spin(1., lambda: len(states) > start and coverage_stamp(states[-1]) >= after_ns and
             non_region_sources_fresh(states[-1]))
        assert (len(states) > start and coverage_stamp(states[-1]) >= after_ns and
                non_region_sources_fresh(states[-1])), states[-1] if states else None

    def image(raw=600):
        msg = Image(); msg.header.frame_id = 'camera'; msg.header.stamp = node.get_clock().now().to_msg()
        msg.width = msg.height = 16; msg.step = 32; msg.encoding = '16UC1'
        data = [600] * 256; data[8*16+8] = raw; msg.data = struct.pack('<256H', *data)
        return msg

    def source(msg): return str(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    def exact_info(msg):
        ci = copy.deepcopy(info); ci.header.stamp = msg.header.stamp; return ci
    def result(msg, reason=None):
        def matches(r): return r['source_stamp_ns'] == source(msg) and (reason is None or r['reason'] == reason)
        spin(.2, lambda: any(matches(r) for r in reports))
        return [r for r in reports if matches(r)][-1]
    def paired(raw=600):
        msg = image(raw); info_pub.publish(exact_info(msg)); spin(.015); image_pub.publish(msg)
        return msg, result(msg)
    def empty_shadow(report):
        assert report['shadow_evaluated'] == 'false' and report['shadow_state'] == 'Unknown'
        assert all(report['shadow_' + key] == '0' for key in COUNTS)

    try:
        heartbeat_thread.start()
        for exe, cfg, name in ((os.environ['DEPTH_OBSTACLE_EXECUTABLE'], config, 'producer'),
                               (coverage_exe, consumer_config, 'coverage')):
            log = (tmp_path / (name + '.log')).open('w+'); logs.append(log)
            processes.append(subprocess.Popen([exe, '--ros-args', '--params-file', str(cfg)],
                stdout=log, stderr=subprocess.STDOUT))
        spin(12, lambda: image_pub.get_subscription_count() == 1 and info_pub.get_subscription_count() == 1
             and rgbd_pub.get_subscription_count() == 1 and airy_pub.get_subscription_count() == 1
             and processed_pub.get_subscription_count() == 1)
        assert all(p.poll() is None for p in processes)
        spin(.2)
        _, missing = paired()
        assert missing['reason'] == 'exact_source_tf_unavailable'; empty_shadow(missing)
        tf_pub = StaticTransformBroadcaster(node)
        transforms = []
        for child in ('camera', 'chest'):
            tf = TransformStamped(); tf.header.frame_id = 'base'; tf.child_frame_id = child
            tf.header.stamp = node.get_clock().now().to_msg(); tf.transform.rotation.w = 1.
            transforms.append(tf)
        count = len(reports); tf_pub.sendTransform(transforms); spin(.1)
        assert len(reports) == count  # No second lookup/retry of failed pair.
        for raw, expected, counter in ((600, 'ClearSupported', 'clear_rays'),
                                      (425, 'ObstacleSupported', 'obstacle_rays'),
                                      (0, 'Unknown', 'invalid_depth_rays'),
                                      (300, 'Unknown', 'front_occluded_rays'),
                                      (460, 'Unknown', 'margin_band_rays')):
            # Establish the actual consumer's fresh non-region inputs before
            # starting this shadow frame; keep all production deadlines intact.
            await_non_region_health()
            msg, report = paired(raw)
            assert report['shadow_evaluated'] == 'true' and report['shadow_state'] == expected
            assert int(report['shadow_' + counter]) > 0
            assert report['reason'] == 'unverified_input_contract'
            assert report['contract'] == params['region_expected_contract']
            # Wait for a new coverage result satisfying the complete isolated
            # precondition; an unrelated old/missing-source timer is not proof
            # that region_support alone kept the actual consumer unavailable.
            start = len(states)
            def isolated_region_rejection():
                return (len(states) > start and coverage_stamp(states[-1]) >= int(source(msg)) and
                        not states[-1].healthy and
                        'region_support:region_unverified_input_contract' in states[-1].reason and
                        non_region_sources_fresh(states[-1]))
            # Observe a new consumer result, rather than treating an arbitrary
            # 80ms timer sample with queued old RGBD headers as this scenario.
            # Requiring the unverified-contract reason also rejects a stale
            # region result; no freshness or expected Unknown condition changes.
            spin(1., isolated_region_rejection)
            assert isolated_region_rejection(), states[-1] if states else None
        pending = image(); image_pub.publish(pending)
        empty_shadow(result(pending, 'pending_exact_info'))
        ci = exact_info(pending); info_pub.publish(ci)
        settled = result(pending, 'unverified_input_contract')
        assert settled['shadow_state'] == 'ClearSupported'
        bad = copy.deepcopy(ci); bad.k[0] += 1.; info_pub.publish(bad)
        empty_shadow(result(pending, 'camera_calibration_contract_mismatch'))
        previous = len(reports); info_pub.publish(ci); spin(.04)
        assert all(r['shadow_evaluated'] == 'false' for r in reports[previous:])
        _, recovered = paired(); assert recovered['shadow_state'] == 'ClearSupported'
        stale = image(); image_pub.publish(stale); spin(.35)
        info_pub.publish(exact_info(stale)); spin(.08)
        last = [r for r in reports if r['source_stamp_ns'] == source(stale)][-1]
        assert 'stale' in last['reason']; empty_shadow(last)
        invalid = image(); invalid.encoding = '8UC1'; image_pub.publish(invalid)
        empty_shadow(result(invalid, 'invalid_region_image_layout'))
        assert reports and all(r['state'] == 'Unknown' and all(r[k] == '0' for k in COUNTS) for r in reports)
        for report in reports:
            if typed_evaluation:
                assert [report[k] for k in ('evaluation_complete', 'evaluation_code', 'disposition')] == ['false', '0', '0']
            else:
                assert all(k not in report for k in ('evaluation_complete', 'evaluation_code', 'disposition'))
        assert states and not any(s.healthy or s.forward_coverage or s.near_field_coverage for s in states)
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
