"""Actual node: raw camera health cannot hide loss of a configured filter."""
import os
import json
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sunhorizon_navigation_interfaces.msg import RgbdStatus, ObstacleCoverageStatus
from tf2_ros import StaticTransformBroadcaster


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def test_processed_belly_loss_and_upstream_calibration(tmp_path):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 15)
    rclpy.init()
    node = rclpy.create_node('coverage_native_test')
    latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    rgbd = node.create_publisher(RgbdStatus, '/coverage_test/rgbd', latest)
    points = node.create_publisher(PointCloud2, '/coverage_test/filtered', sensor)
    states, telemetry = [], []

    def receive(state):
        receipt = time.monotonic_ns()
        states.append((state, receipt))
        telemetry.append({'event': 'status', 'receipt_ns': receipt,
                          'stamp_ns': stamp_ns(state.header.stamp),
                          'healthy': state.healthy, 'reason': state.reason})

    node.create_subscription(ObstacleCoverageStatus, '/coverage_test/status', receive, latest)
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
    for key, value in {'profile': 'visual', 'output_topic': '/coverage_test/status',
                       'rgbd_status_topic': '/coverage_test/rgbd',
                       'belly_freshness_sec': '1.0', 'belly_obstacle_freshness_sec': '0.3',
                       'belly_obstacle_points_topic': '/coverage_test/filtered'}.items():
        args += ['-p', key + ':=' + value]
    log = (tmp_path / 'node.log').open('w+')
    process = subprocess.Popen([os.environ['COVERAGE_EXECUTABLE'], *args], stdout=log,
                               stderr=subprocess.STDOUT)

    def run(duration, with_cloud=True, accepted=True, frame='camera_belly_depth_optical_frame'):
        until = time.monotonic() + duration
        next_publish = 0.
        while time.monotonic() < until:
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + .04
                status = RgbdStatus()
                status.header.stamp = node.get_clock().now().to_msg()
                for name in ('chest_pair_fresh', 'chest_camera_info_valid', 'chest_alignment_accepted',
                             'chest_depth_scale_accepted', 'chest_pointcloud_healthy', 'belly_depth_fresh',
                             'belly_camera_info_valid', 'belly_pointcloud_healthy'):
                    setattr(status, name, True)
                status.belly_depth_scale_accepted = accepted
                before = time.monotonic_ns()
                rgbd.publish(status)
                telemetry.append({'event': 'rgbd_publish', 'begin_ns': before,
                                  'end_ns': time.monotonic_ns(),
                                  'stamp_ns': stamp_ns(status.header.stamp)})
                if with_cloud:
                    cloud = PointCloud2()
                    cloud.header.stamp, cloud.header.frame_id = status.header.stamp, frame
                    cloud.height = cloud.width = 1
                    cloud.point_step = cloud.row_step = 12
                    cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                                    for i, n in enumerate(('x', 'y', 'z'))]
                    cloud.data = struct.pack('<fff', .1, .1, 1.)
                    before = time.monotonic_ns()
                    points.publish(cloud)
                    telemetry.append({'event': 'cloud_publish', 'begin_ns': before,
                                      'end_ns': time.monotonic_ns(),
                                      'stamp_ns': stamp_ns(cloud.header.stamp)})
            rclpy.spin_once(node, timeout_sec=.01)
        assert process.poll() is None
        return states[-1][0] if states else None

    def phase(name, predicate, timeout=2.0, **kwargs):
        # Only three distinct, recent outputs calculated after this phase began
        # can satisfy it. A cached healthy state or a single lucky recovery
        # sample cannot satisfy the transition test. This is not a long-load test.
        watermark = node.get_clock().now().nanoseconds
        first_index = len(states)
        telemetry.append({'event': 'phase', 'name': name, 'stamp_ns': watermark})
        deadline = time.monotonic() + timeout
        streak, last_stamp, first_stamp = 0, 0, 0
        while time.monotonic() < deadline:
            run(.04, **kwargs)
            fresh = states[first_index:]
            first_index = len(states)
            for state, receipt in fresh:
                stamp = stamp_ns(state.header.stamp)
                age = node.get_clock().now().nanoseconds - stamp
                valid = (stamp > watermark and stamp > last_stamp and
                         0 <= age <= 100_000_000 and
                         time.monotonic_ns() - receipt <= 100_000_000)
                if valid and predicate(state):
                    if not streak:
                        first_stamp = stamp
                    streak += 1
                else:
                    streak = 0
                last_stamp = max(last_stamp, stamp)
                if streak >= 3 and stamp - first_stamp >= 100_000_000:
                    return state
        raise AssertionError(f'{name}: no current stable result within {timeout}s; '
                             f'last={states[-1][0] if states else None}')

    try:
        phase('startup', lambda s: s.healthy, timeout=10.)
        phase('processed_loss', lambda s: not s.healthy and
              not s.near_field_coverage and 'belly' in s.reason, with_cloud=False)
        phase('processed_recovery', lambda s: s.healthy)
        phase('calibration_rejected', lambda s: not s.healthy and
              not s.near_field_coverage and 'belly' in s.reason, accepted=False)
        phase('calibration_recovery', lambda s: s.healthy)
        # A valid TF for another frame must not impersonate the belly camera.
        phase('wrong_processed_frame', lambda s: not s.healthy and
              not s.near_field_coverage and 'belly' in s.reason, frame='base_footprint')
        phase('frame_recovery', lambda s: s.healthy)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        log.seek(0)
        print(log.read())
        log.close()
        (tmp_path / "timing.json").write_text(json.dumps(telemetry, indent=2))
        print(json.dumps(telemetry))
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0, "coverage node did not exit cleanly after SIGINT"
