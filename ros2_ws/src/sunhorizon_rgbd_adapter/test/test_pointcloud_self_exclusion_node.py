"""Native TF/cloud integration in an isolated, localhost-only ROS domain."""
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


@pytest.fixture(params=[False, True], ids=['default_tf', 'private_tf'])
def runtime(tmp_path, request):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 20)
    remaps = ['-r', '/tf:=/self_test/private_tf', '-r', '/tf_static:=/self_test/private_tf_static'] if request.param else []
    rclpy.init(args=['--ros-args', *remaps])
    node = rclpy.create_node('self_exclusion_native_test')
    log = (tmp_path / 'node.log').open('w+')
    args = ['--ros-args', *remaps]
    for key, value in {
        'enabled': 'true', 'input_topic': '/self_test/input',
        'output_topic': '/self_test/output', 'base_frame': 'camera',
        'exclusion_min_xyz': '[-0.04, -0.03, -0.2]',
        'exclusion_max_xyz': '[0.04, 0.03, 0.2]',
        'additional_exclusion_min_xyz': '[-0.04, -0.03, -0.2]',
        'additional_exclusion_max_xyz': '[0.04, 0.03, 0.2]',
        'exclusion_frames': '[left_leg, right_leg]',
        'max_removed_fraction': '0.1', 'max_cloud_age_sec': '0.8',
        'tf_timeout_sec': '0.01',
    }.items():
        args += ['-p', key + ':=' + value]
    process = subprocess.Popen([os.environ['SELF_EXCLUSION_EXECUTABLE'], *args],
                               stdout=log, stderr=subprocess.STDOUT)
    try:
        yield node, process, log
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
        node.destroy_node()
        rclpy.shutdown()


def spin(node, duration, done=lambda: False):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not done():
        rclpy.spin_once(node, timeout_sec=.01)


def test_link_motion_exact_stamp_and_missing_link(runtime):
    node, process, _ = runtime
    received = []
    publisher = node.create_publisher(PointCloud2, '/self_test/input', 10)
    subscription = node.create_subscription(PointCloud2, '/self_test/output', received.append, 10)
    broadcaster = TransformBroadcaster(node)
    spin(node, 10, lambda: publisher.get_subscription_count() == 1
         and node.count_publishers('/self_test/output') == 1)
    assert process.poll() is None
    assert publisher.get_subscription_count() == 1
    assert node.count_publishers('/self_test/output') == 1
    spin(node, .2)
    points = [(.1, .15, -.4), (.1, -.15, -.4), (.1, 0., -.4),
              (2., 0., -.4), (.3, .15, -.4)] + [(2. + i * .01, .4, 0.) for i in range(100)]

    def sample(left_x, include_right=True):
        stamp = node.get_clock().now() - rclpy.duration.Duration(seconds=.04)
        transforms = []
        for offset in (-.01, .01):
            for frame, x, y in [('left_leg', left_x, .15), ('right_leg', .1, -.15)]:
                if frame == 'right_leg' and not include_right:
                    continue
                transform = TransformStamped()
                transform.header.frame_id = 'camera'
                transform.header.stamp = (stamp + rclpy.duration.Duration(seconds=offset)).to_msg()
                transform.child_frame_id = frame
                transform.transform.translation.x = x
                transform.transform.translation.y = y
                transform.transform.translation.z = -.4
                transform.transform.rotation.w = 1.
                transforms.append(transform)
        broadcaster.sendTransform(transforms)
        spin(node, .06)
        message = PointCloud2()
        message.header.frame_id = 'camera'
        message.header.stamp = stamp.to_msg()
        message.height, message.width = 1, len(points)
        message.point_step, message.row_step = 16, 16 * len(points)
        message.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
        message.fields.append(PointField(name='id', offset=12, datatype=PointField.UINT32, count=1))
        message.data = b''.join(struct.pack('<fffI', *p, i) for i, p in enumerate(points))
        publisher.publish(message)
        return message

    for left_x, removed in [(.1, {0, 1}), (.3, {1, 4})]:
        message = sample(left_x)
        spin(node, 3, lambda: any(m.header == message.header for m in received))
        output = next(m for m in received if m.header == message.header)
        assert output.fields == message.fields
        assert bytes(output.data) == b''.join(bytes(message.data[i*16:(i+1)*16])
                                      for i in range(len(points)) if i not in removed)
    # The right link's TF history is deliberately frozen. Latest-TF fallback
    # would wrongly emit this frame; exact-time filtering must emit nothing.
    spin(node, .15)
    missing = sample(.3, include_right=False)
    spin(node, .3)
    assert not any(m.header == missing.header for m in received)
    restored = sample(.3)
    spin(node, 3, lambda: any(m.header == restored.header for m in received))
    assert any(m.header == restored.header for m in received)
    assert process.poll() is None
    node.destroy_subscription(subscription)


def test_volume_frame_binding_is_read_only(runtime):
    node, process, _ = runtime
    client = node.create_client(SetParameters, '/sunhorizon_pointcloud_self_exclusion/set_parameters')
    assert client.wait_for_service(timeout_sec=10)
    value = ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                           string_array_value=['camera', 'camera'])
    request = SetParameters.Request(parameters=[Parameter(name='exclusion_frames', value=value)])
    future = client.call_async(request)
    spin(node, 3, future.done)
    assert future.done()
    assert not future.result().results[0].successful
    assert process.poll() is None
