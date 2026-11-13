"""Real opt-in producer, synthetic images/TF on localhost only; no motion entities."""
import copy
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def canonical(fields):
    return ''.join(f'{len(str(x).encode("ascii"))}:{x}' for x in fields)


def number(x):
    return format(0. if x == 0. else x, '.17e')


def parameters(info, typed_evaluation=False):
    cal = canonical(['region-camera-v1', info.header.frame_id, info.width, info.height,
        info.distortion_model, len(info.d), *map(number, info.d), *map(number, info.k),
        *map(number, info.r), *map(number, info.p), 0, 0, 0, 0, 0, 0, 'false'])
    contract = canonical(['region-producer-v1', 'region-depth-core-v1', 'test_region',
        'base', 'camera', '/region_test/image', '/region_test/info', '/region_test/support', '16UC1',
        *map(number, [-.1, .1, -.1, .1, .4, .45, .02, .001, .2, 8., .3]),
        *(['true'] * 5), cal,
        'exact-source-info-tf;pending-unknown;info-cache=8;depth-pending=1'])
    if typed_evaluation:
        contract += canonical(["typed-region-evaluation-diagnostics-v1"])
    result = dict(enabled=True, projection_accepted=True, component_min_pixels=4,
        depth_topic='/region_test/image', camera_info_topic='/region_test/info',
        output_topic='/region_test/cloud', diagnostics_topic='/region_test/diagnostics',
        region_support_enabled=True, region_support_diagnostics_topic='/region_test/support',
        region_id='test_region', region_base_frame='base',
        region_optical_frame='camera', region_box_min_xyz=[-.1, -.1, .4],
        region_box_max_xyz=[.1, .1, .45], region_expected_calibration=cal,
        region_expected_contract=contract, region_rectified_pinhole_accepted=True,
        region_metric_axial_depth_accepted=True, region_depth_intrinsics_registration_accepted=True,
        region_optical_to_base_extrinsics_accepted=True, region_source_time_transform_accepted=True)
    if typed_evaluation:
        result["region_typed_evaluation_diagnostics_enabled"] = True
    return result


@pytest.mark.parametrize("typed_evaluation", [False, True])
def test_exact_pairing_raw_obstacle_and_expiry(tmp_path, typed_evaluation):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 10)
    rclpy.init()
    node = rclpy.create_node('region_producer_test')
    reports, clouds = [], []
    image_pub = node.create_publisher(Image, '/region_test/image', 10)
    info_pub = node.create_publisher(CameraInfo, '/region_test/info', 10)
    subscriptions = [node.create_subscription(DiagnosticArray, '/region_test/support',
        lambda a: reports.extend({v.key: v.value for v in s.values} for s in a.status
            if s.name.endswith('/region_support')), 10),
        node.create_subscription(PointCloud2, '/region_test/cloud', clouds.append, 10)]
    def spin(seconds, done=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not done():
            rclpy.spin_once(node, timeout_sec=.005)
    tf_pub = StaticTransformBroadcaster(node)
    tf = TransformStamped(); tf.header.frame_id = 'base'; tf.child_frame_id = 'camera'
    tf.transform.rotation.w = 1.; tf.header.stamp = node.get_clock().now().to_msg()
    info = CameraInfo(); info.header.frame_id = 'camera'; info.width = info.height = 16
    info.distortion_model = 'plumb_bob'; info.d = [0.] * 5
    info.k = [20., 0., 8., 0., 20., 8., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.]
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters(info, typed_evaluation)}}))
    log = (tmp_path / 'producer.log').open('w+')
    process = subprocess.Popen([os.environ['DEPTH_OBSTACLE_EXECUTABLE'], '--ros-args',
        '--params-file', str(config)], stdout=log, stderr=subprocess.STDOUT)
    def make_image():
        image = Image(); image.header.frame_id = 'camera'
        image.header.stamp = node.get_clock().now().to_msg()
        image.width = image.height = 16; image.step = 32; image.encoding = '16UC1'
        image.data = struct.pack('<256H', *([600] * 256))
        return image
    def paired(image):
        ci = copy.deepcopy(info); ci.header.stamp = image.header.stamp
        info_pub.publish(ci); spin(.02); image_pub.publish(image)
        source = str(image.header.stamp.sec * 10**9 + image.header.stamp.nanosec)
        spin(.2, lambda: any(r['source_stamp_ns'] == source for r in reports))
        result = [r for r in reports if r['source_stamp_ns'] == source][-1]
        return result
    try:
        spin(10, lambda: image_pub.get_subscription_count() == 1 and info_pub.get_subscription_count() == 1)
        assert process.poll() is None
        spin(.3)  # Discovery settles with no TF supplied: first exact pair must fail closed.
        missing = paired(make_image())
        assert missing['state'] == 'Unknown' and missing['reason'] == 'exact_source_tf_unavailable'
        assert missing['tf_error_type'] == 'lookup' and 0 < len(missing['tf_error_detail']) <= 512
        assert int(missing['tf_lookup_wall_ns']) >= 0
        count = len(reports); tf_pub.sendTransform(tf); spin(.08)
        assert len(reports) == count  # TF arrival never retries the failed source.
        clear = paired(make_image())
        assert clear['tf_error_type'] == 'none' and clear['tf_error_detail'] == ''
        assert clear['tf_error_truncated'] == 'false' and int(clear['tf_lookup_wall_ns']) >= 0
        assert clear['state'] == 'ClearSupported'
        assert not any(k.startswith('shadow_') for k in clear)
        def typed(report, complete, code, disposition):
            keys = ('evaluation_complete', 'evaluation_code', 'disposition')
            if typed_evaluation:
                assert tuple(report[k] for k in keys) == (str(complete).lower(), str(code), str(disposition))
            else:
                assert all(k not in report for k in keys)
        typed(missing, False, 0, 0)
        typed(clear, True, 1, 1)
        assert clear['source_stamp_ns'] == clear['paired_info_stamp_ns'] == clear['tf_requested_stamp_ns']
        assert clear['contract'] == parameters(info, typed_evaluation)['region_expected_contract']
        # A single unfiltered pixel in R is an obstacle even though component_min_pixels=4.
        obstacle = make_image(); data = [600] * 256; data[8 * 16 + 8] = 425
        obstacle.data = struct.pack('<256H', *data)
        blocked = paired(obstacle)
        assert blocked['state'] == 'ObstacleSupported' and int(blocked['obstacle_rays']) == 1
        typed(blocked, True, 2, 0)
        spin(.03)
        processed = [c for c in clouds if c.header.stamp == obstacle.header.stamp][-1]
        assert not any(abs(z-.425) < 1.e-5 for _, _, z in struct.iter_unpack('<fff', bytes(processed.data)))
        padded = make_image(); padded.is_bigendian = 1; padded.step = 36
        padded.data = b''.join(struct.pack('>16H', *([600] * 16)) + b'\xff' * 4 for _ in range(16))
        assert paired(padded)['state'] == 'ClearSupported'
        for raw, expected_reason in [(0, 'invalid'), (300, 'unknown'), (460, 'unknown')]:
            unknown = make_image(); data = [600] * 256; data[8 * 16 + 8] = raw
            unknown.data = struct.pack('<256H', *data)
            result = paired(unknown)
            assert result['state'] == 'Unknown'
            field = {0: 'invalid_depth_rays', 300: 'front_occluded_rays', 460: 'margin_band_rays'}[raw]
            assert int(result[field]) == 1
            typed(result, True, 3, 2 if raw == 460 else 0)
        # Mixed margin + invalid/front/obstacle must never inherit GeometryHold.
        for other in (0, 300, 425):
            mixed = make_image(); data = [600] * 256
            data[8*16+8] = 460; data[8*16+9] = other
            mixed.data = struct.pack('<256H', *data)
            result = paired(mixed)
            assert result['state'] == 'Unknown' and result['margin_band_rays'] == '1'
            typed(result, True, 3, 0)
        # Image-first must invalidate previous Clear/Obstacle immediately, then same-source pair only.
        image = make_image(); image_pub.publish(image)
        source = str(image.header.stamp.sec * 10**9 + image.header.stamp.nanosec)
        spin(.15, lambda: any(r['source_stamp_ns'] == source for r in reports))
        pending = reports[-1]
        assert pending['state'] == 'Unknown' and pending['reason'] == 'pending_exact_info'
        typed(pending, False, 0, 0)
        assert pending['tf_error_type'] == 'none' and pending['tf_error_detail'] == ''
        assert pending['tf_lookup_wall_ns'] == '0'
        ci = copy.deepcopy(info); ci.header.stamp = image.header.stamp; info_pub.publish(ci)
        spin(.15, lambda: reports[-1]['state'] == 'ClearSupported')
        final = reports[-1]
        assert final['state'] == 'ClearSupported'
        for field in ('source_stamp_ns', 'region_frame_sequence', 'image_receipt_ros_ns', 'image_receipt_steady_ns'):
            assert final[field] == pending[field]
        assert int(final['region_evaluation_sequence']) > int(pending['region_evaluation_sequence'])
        # Same-key conflicting Info revokes a result; duplicate good Info cannot revive it.
        bad = copy.deepcopy(ci); bad.k[0] += 1.; info_pub.publish(bad)
        spin(.15, lambda: reports[-1]['state'] == 'Unknown')
        assert reports[-1]['state'] == 'Unknown'
        before = len(reports); info_pub.publish(ci); spin(.05)
        assert all(r['state'] == 'Unknown' for r in reports[before:])
        # Late exact Info cannot refresh source age or evaluate an expired pending image.
        stale = make_image(); image_pub.publish(stale)
        stale_source = str(stale.header.stamp.sec*10**9 + stale.header.stamp.nanosec)
        def stale_reports(reason):
            return [r for r in reports if r['source_stamp_ns'] == stale_source and reason in r['reason']]
        spin(.15, lambda: bool(stale_reports('pending_exact_info')))
        assert stale_reports('pending_exact_info'), reports[-3:]
        original = stale_reports('pending_exact_info')[-1]
        spin(.35)  # Real elapsed time beyond the unchanged300ms source/receipt gate.
        ci.header.stamp = stale.header.stamp; info_pub.publish(ci)
        # There is no expiry timer. Await this exact source's late-Info result,
        # using the same200ms diagnostic-observation budget as paired() above.
        # A pre-existing Unknown/pending report is not evidence of expiry.
        spin(.2, lambda: bool(stale_reports('stale')))
        assert stale_reports('stale'), reports[-3:]
        expired = stale_reports('stale')[-1]
        assert expired['state'] == 'Unknown'
        for field in ('source_stamp_ns', 'region_frame_sequence', 'image_receipt_ros_ns', 'image_receipt_steady_ns'):
            assert expired[field] == original[field]
        typed(expired, False, 0, 0)
        # Replaced pending cannot be evaluated by old Info arriving later.
        old = make_image(); image_pub.publish(old); spin(.015)
        new = make_image(); image_pub.publish(new); spin(.03)
        before = len(reports); ci.header.stamp = old.header.stamp; info_pub.publish(ci); spin(.04)
        assert len(reports) == before
        assert reports[-1]['source_stamp_ns'] == str(new.header.stamp.sec*10**9 + new.header.stamp.nanosec)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        log.seek(0); print(log.read()); log.close()
        node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0


def test_bounded_exact_tf_wait_and_post_wait_age(tmp_path):
    """Synthetic TF arrives after image; no latest fallback or source-age extension."""
    from tf2_ros import TransformBroadcaster
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(190 + os.getpid() % 10)
    rclpy.init()
    node = rclpy.create_node('bounded_region_tf_test')
    reports = []
    image_pub = node.create_publisher(Image, '/region_test/image', 10)
    info_pub = node.create_publisher(CameraInfo, '/region_test/info', 10)
    node.create_subscription(DiagnosticArray, '/region_test/support',
        lambda a: reports.extend({v.key: v.value for v in s.values} for s in a.status), 10)
    broadcaster = TransformBroadcaster(node)
    info = CameraInfo(); info.header.frame_id = 'camera'; info.width = info.height = 16
    info.distortion_model = 'plumb_bob'; info.d = [0.] * 5
    info.k = [20., 0., 8., 0., 20., 8., 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.]
    params = parameters(info)
    params['region_tf_wait_sec'] = .02
    params['region_expected_contract'] += canonical(['bounded-exact-tf-wait-v1', number(.02)])
    config = tmp_path / 'wait.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    log = (tmp_path / 'wait-producer.log').open('w+')
    process = subprocess.Popen([os.environ['DEPTH_OBSTACLE_EXECUTABLE'], '--ros-args',
        '--params-file', str(config)], stdout=log, stderr=subprocess.STDOUT)
    def spin(seconds, done=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not done(): rclpy.spin_once(node, timeout_sec=.001)
    def stamped(ns):
        from builtin_interfaces.msg import Time
        return Time(sec=ns//10**9, nanosec=ns%10**9)
    def image_at(ns):
        image = Image(); image.header.frame_id = 'camera'; image.header.stamp = stamped(ns)
        image.width = image.height = 16; image.step = 32; image.encoding = '16UC1'
        image.data = struct.pack('<256H', *([600] * 256))
        return image
    def tf_at(ns):
        tf = TransformStamped(); tf.header.frame_id = 'base'; tf.child_frame_id = 'camera'
        tf.header.stamp = stamped(ns); tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)
    def result(ns):
        spin(.15, lambda: any(r['source_stamp_ns'] == str(ns) and r['reason'] != 'pending_exact_info' for r in reports))
        return [r for r in reports if r['source_stamp_ns'] == str(ns)][-1]
    try:
        spin(10, lambda: image_pub.get_subscription_count() == 1 and info_pub.get_subscription_count() == 1)
        assert process.poll() is None
        spin(.3)
        ns = node.get_clock().now().nanoseconds
        info.header.stamp = stamped(ns); info_pub.publish(info); spin(.01)
        image_pub.publish(image_at(ns))
        absent = result(ns)
        assert absent['state'] == 'Unknown' and absent['tf_error_type'] == 'lookup'
        assert absent['reason'] == 'exact_source_tf_unavailable'
        # Known frames but latest TF is strictly earlier. A later exact TF must arrive
        # while the C++ callback waits; independent TF listener can service it.
        ns = node.get_clock().now().nanoseconds
        tf_at(ns-1_000_000); info.header.stamp = stamped(ns); info_pub.publish(info); spin(.015)
        image_pub.publish(image_at(ns)); time.sleep(.006); tf_at(ns)
        clear = result(ns)
        assert clear['state'] == 'ClearSupported' and clear['tf_error_type'] == 'none'
        assert int(clear['tf_lookup_wall_ns']) >= 1_000_000  # Evidence a wait actually occurred.
        assert clear['tf_requested_stamp_ns'] == str(ns)
        assert int(clear['evaluation_ros_ns']) - ns <= 300_000_000
        # Never substitute the earlier available transform when no exact TF arrives.
        ns = node.get_clock().now().nanoseconds
        info.header.stamp = stamped(ns); info_pub.publish(info); spin(.01)
        image_pub.publish(image_at(ns))
        missing = result(ns)
        assert missing['state'] == 'Unknown' and missing['reason'] == 'exact_source_tf_unavailable'
        assert missing['tf_error_type'] == 'extrapolation'
        assert int(missing['tf_lookup_wall_ns']) >= 20_000_000
        count = len(reports); tf_at(ns); spin(.03)
        assert len(reports) == count  # Arrival after timeout does not retry.
        # Pending source is fresh before lookup, but waiting crosses original .3s.
        ns = node.get_clock().now().nanoseconds
        tf_at(ns-1_000_000); spin(.01); image_pub.publish(image_at(ns))
        spin(.05, lambda: reports[-1]['source_stamp_ns'] == str(ns))
        assert reports[-1]['reason'] == 'pending_exact_info'
        while node.get_clock().now().nanoseconds - ns < 290_000_000: spin(.001)
        info.header.stamp = stamped(ns); info_pub.publish(info)
        while node.get_clock().now().nanoseconds - ns < 302_000_000: time.sleep(.0005)
        tf_at(ns)
        stale = result(ns)
        assert stale['state'] == 'Unknown'
        assert stale['reason'] == 'region_tf_became_stale_or_clock_invalid'
        assert stale['tf_error_type'] == 'none'  # TF lookup returned; freshness still vetoed it.
        assert int(stale['evaluation_ros_ns']) - ns > 300_000_000
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        log.seek(0); print(log.read()); log.close()
        node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0
