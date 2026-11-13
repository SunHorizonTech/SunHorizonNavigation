"""Real depth executable, isolated localhost ROS domain; no robot topics."""
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def spin(node, seconds, done=lambda: False):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not done():
        rclpy.spin_once(node, timeout_sec=.01)


def test_frame_identity_timings_and_unchanged_output(tmp_path):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(160 + os.getpid() % 20)
    rclpy.init()
    node = rclpy.create_node('depth_timing_test')
    log = (tmp_path / 'node.log').open('w+')
    args = ['--ros-args']
    for key, value in {
        'enabled': 'true', 'projection_accepted': 'true',
        'depth_topic': '/depth_timing/image', 'camera_info_topic': '/depth_timing/info',
        'output_topic': '/depth_timing/cloud', 'diagnostics_topic': '/depth_timing/status',
        'component_min_pixels': '4', 'max_cloud_age_sec': '1.0',
        'pointcloud_pixel_stride': '2',
    }.items():
        args += ['-p', f'{key}:={value}']
    process = subprocess.Popen([os.environ['DEPTH_OBSTACLE_EXECUTABLE'], *args],
                               stdout=log, stderr=subprocess.STDOUT)
    try:
        clouds = []; reports = []
        pub = node.create_publisher(Image, '/depth_timing/image', 10)
        info_pub = node.create_publisher(CameraInfo, '/depth_timing/info', 10)
        subscriptions = [
            node.create_subscription(PointCloud2, '/depth_timing/cloud', clouds.append, 10),
            node.create_subscription(DiagnosticArray, '/depth_timing/status', reports.append, 10)]
        publisher_topics = None
        def graph_ready():
            nonlocal publisher_topics
            if (pub.get_subscription_count() != 1 or info_pub.get_subscription_count() != 1 or
                    node.count_publishers('/depth_timing/status') != 1 or
                    ('sunhorizon_depth_obstacle_cloud', '/') not in node.get_node_names_and_namespaces()):
                return False
            try:
                topics = {name for name, _ in node.get_publisher_names_and_types_by_node(
                    'sunhorizon_depth_obstacle_cloud', '/')}
            except Exception as error:
                # DDS endpoint discovery can precede the remote node-name graph
                # association. Retry only that explicit discovery state inside
                # the original deadline; never swallow an unrelated RMW error.
                if type(error).__name__ != 'NodeNameNonExistentError':
                    raise
                return False
            if not {'/depth_timing/cloud', '/depth_timing/status'} <= topics:
                return False
            publisher_topics = topics
            return True
        spin(node, 10, graph_ready)
        assert process.poll() is None
        assert pub.get_subscription_count() == 1
        assert publisher_topics is not None, node.get_node_names_and_namespaces()
        assert publisher_topics <= {'/depth_timing/cloud', '/depth_timing/status', '/rosout', '/parameter_events'}
        assert sorted(node.get_node_names()) == ['depth_timing_test', 'sunhorizon_depth_obstacle_cloud']
        info = CameraInfo(); info.header.frame_id = 'camera'; info.width = info.height = 16
        info.k = [20., 0., 8., 0., 20., 8., 0., 0., 1.]
        for _ in range(3):
            info_pub.publish(info); spin(node, .05)
        image = Image(); image.header.frame_id = 'camera'
        image.header.stamp = node.get_clock().now().to_msg()
        image.width = image.height = 16; image.step = 32; image.encoding = '16UC1'
        image.data = struct.pack('<256H', *([500] * 256))
        pub.publish(image)
        spin(node, 3, lambda: len(clouds) == 1 and len(reports) == 1)
        assert len(clouds) == len(reports) == 1
        cloud = clouds[0]
        assert cloud.header == image.header
        assert cloud.width == 64 and cloud.height == 1 and cloud.point_step == 12
        expected = b''.join(struct.pack('<fff', (u-8)*.5/20, (v-8)*.5/20, .5)
                            for v in range(0, 16, 2) for u in range(0, 16, 2))
        assert bytes(cloud.data) == expected
        first = reports[0].status[0]
        values = {v.key: v.value for v in first.values}
        assert first.message == 'filtered'
        assert values['component_mask_removed_pixels'] == '0'
        assert values['diagnostic_event'] == 'depth_callback'
        assert not any(v.key.startswith('region_') for v in first.values)
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        assert int(values['source_stamp_ns']) == stamp
        assert int(values['receipt_ros_ns']) >= stamp
        assert int(values['receipt_steady_ns']) > 0
        assert float(values['input_age_sec']) >= 0
        assert values['input_nonmonotonic'] == 'false'
        for stage in ('mask', 'project', 'publish'):
            assert values[stage+'_executed'] == 'true'
            assert int(values[stage+'_wall_ns']) > 0
            assert int(values[stage+'_thread_cpu_ns']) >= 0

        pub.publish(image)  # Duplicate is still rejected, linked to the same source.
        spin(node, 2, lambda: len(reports) == 2)
        assert len(reports) == 2 and len(clouds) == 1
        duplicate = {v.key: v.value for v in reports[1].status[0].values}
        assert reports[1].status[0].message == 'stale_or_nonmonotonic_depth'
        assert duplicate['source_stamp_ns'] == values['source_stamp_ns']
        assert int(duplicate['callback_sequence']) == int(values['callback_sequence']) + 1
        assert duplicate['input_nonmonotonic'] == 'true'
        assert all(duplicate[s+'_executed'] == 'false' for s in ('mask', 'project', 'publish'))

        image.header.stamp.sec -= 2  # Stale input also retains its own source identity.
        pub.publish(image)
        spin(node, 2, lambda: len(reports) == 3)
        assert len(reports) == 3 and len(clouds) == 1
        stale = {v.key: v.value for v in reports[2].status[0].values}
        assert float(stale['input_age_sec']) > 1
        assert int(stale['source_stamp_ns']) == stamp - 2 * 10**9
        assert all(stale[s+'_executed'] == 'false' for s in ('mask', 'project', 'publish'))
        spin(node, .3)
        assert len(reports) == 3  # No timer produces fake new-frame reports.
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
        log.seek(0); print(log.read()); log.close()
        node.destroy_node(); rclpy.shutdown()
