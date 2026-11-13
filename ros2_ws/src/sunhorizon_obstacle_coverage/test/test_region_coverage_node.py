"""Isolated real ROS node: region evidence can only restrict original fusion gates."""
import os
import signal
import struct
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import RgbdStatus, ObstacleCoverageStatus
from tf2_ros import StaticTransformBroadcaster


def test_region_and_original_fusion_gates(tmp_path):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(220 + os.getpid() % 10)
    rclpy.init()
    node = rclpy.create_node('region_coverage_test')
    rgbd = node.create_publisher(RgbdStatus, '/region_test/rgbd', 10)
    points = node.create_publisher(PointCloud2, '/region_test/belly', 10)
    airy = node.create_publisher(PointCloud2, '/region_test/airy', 10)
    region = node.create_publisher(DiagnosticArray, '/region_test/region', 10)
    states = []
    node.create_subscription(ObstacleCoverageStatus, '/region_test/status', states.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for child in ('camera_chest_color_optical_frame', 'camera_belly_depth_optical_frame'):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_footprint', child
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.transform.rotation.w = 1.
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    args = ['--ros-args']
    params = {'profile': 'fusion_shadow', 'output_topic': '/region_test/status',
              'rgbd_status_topic': '/region_test/rgbd', 'airy_points_topic': '/region_test/airy',
              'belly_freshness_sec': '0.3', 'belly_obstacle_freshness_sec': '0.3',
              'belly_obstacle_points_topic': '/region_test/belly',
              'region_support_required': 'true', 'region_support_id': 'test-region',
              'region_support_status_name': 'test/region_support',
              'region_support_contract': 'test-canonical-contract',
              'region_support_diagnostics_topic': '/region_test/region'}
    for key, value in params.items(): args += ['-p', key + ':=' + value]
    log = (tmp_path / 'node.log').open('w+')
    process = subprocess.Popen([os.environ['COVERAGE_EXECUTABLE'], *args], stdout=log,
                               stderr=subprocess.STDOUT)
    sequence = 0

    def run(duration, state='ClearSupported', send_region=True, send_belly=True,
            send_airy=True, calibration=True, bad_contract=False):
        nonlocal sequence
        until, next_publish = time.monotonic() + duration, 0.
        while time.monotonic() < until:
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + .04
                now = node.get_clock().now()
                stamp, ns, steady = now.to_msg(), now.nanoseconds, time.monotonic_ns()
                status = RgbdStatus()
                status.header.stamp = stamp
                for name in ('chest_pair_fresh', 'chest_camera_info_valid', 'chest_alignment_accepted',
                             'chest_depth_scale_accepted', 'chest_pointcloud_healthy', 'belly_depth_fresh',
                             'belly_camera_info_valid', 'belly_pointcloud_healthy'):
                    setattr(status, name, True)
                status.belly_depth_scale_accepted = calibration
                rgbd.publish(status)
                for publisher, frame, enabled in (
                        (points, 'camera_belly_depth_optical_frame', send_belly),
                        (airy, 'base_footprint', send_airy)):
                    if not enabled: continue
                    cloud = PointCloud2()
                    cloud.header.stamp, cloud.header.frame_id = stamp, frame
                    cloud.height = cloud.width = 1
                    cloud.point_step = cloud.row_step = 12
                    cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                    for i, n in enumerate(('x', 'y', 'z'))]
                    cloud.data = struct.pack('<fff', .1, .1, 1.)
                    publisher.publish(cloud)
                if send_region:
                    sequence += 1
                    clear = state == 'ClearSupported'
                    obstacle = state == 'ObstacleSupported'
                    message = DiagnosticArray()
                    message.header.stamp = stamp
                    entry = DiagnosticStatus(name='test/region_support', level=bytes([0 if clear else 1]))
                    fields = dict(diagnostic_event='region_depth_support', schema_version='1',
                        contract_encoding='length-prefixed-ascii-v1',
                        contract='wrong' if bad_contract else 'test-canonical-contract',
                        region_id='test-region', base_frame_id='base_footprint',
                        image_frame_id='camera_belly_depth_optical_frame',
                        region_frame_sequence=sequence, region_evaluation_sequence=sequence,
                        source_stamp_ns=ns, image_receipt_ros_ns=ns, image_receipt_steady_ns=steady,
                        evaluation_ros_ns=ns, evaluation_steady_ns=steady,
                        paired_info_stamp_ns=ns, tf_requested_stamp_ns=ns,
                        state=state, reason='clear' if clear else ('obstacle' if obstacle else 'occluded'),
                        tested_pixels=10, intersecting_rays=10,
                        clear_rays=10 if clear else 0, obstacle_rays=10 if obstacle else 0,
                        invalid_depth_rays=0, front_occluded_rays=0 if clear or obstacle else 10,
                        margin_band_rays=0, region_wall_ns=0, region_thread_cpu_ns=0)
                    entry.values = [KeyValue(key=k, value=str(v)) for k, v in fields.items()]
                    message.status = [entry]
                    region.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
        assert process.poll() is None
        return states[-1] if states else None

    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            result = run(.4)
            if result is not None and result.healthy: break
        assert states and states[-1].healthy
        baseline_counts = (states[-1].required_sources, states[-1].healthy_sources)
        for kind in ('ObstacleSupported', 'Unknown'):
            result = run(.5, state=kind)
            assert not result.healthy and not result.forward_coverage and not result.near_field_coverage
            assert not result.rear_coverage and 'region_support' in result.reason
            assert (result.required_sources, result.healthy_sources) == baseline_counts
            assert run(.5).healthy
        assert not run(.8, send_region=False).healthy
        assert run(.5).healthy
        for missing in ('send_belly', 'send_airy'):
            assert not run(.8, **{missing: False}).healthy
            assert run(.5).healthy
        assert not run(.5, calibration=False).healthy
        assert run(.5).healthy
        assert not run(.5, bad_contract=True).healthy
        assert not run(.5).healthy  # Mismatched ownership contract remains latched.
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
        log.seek(0); print(log.read()); log.close()
        node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0, "coverage node did not exit cleanly after SIGINT"
