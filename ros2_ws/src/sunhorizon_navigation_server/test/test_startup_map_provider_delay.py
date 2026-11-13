"""Exercise the real server while one provider response spans three startup ticks."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sunhorizon_navigation_interfaces.msg import NavigationStatus
from sunhorizon_navigation_interfaces.srv import LoadMap

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))
import map_bundle  # noqa: E402


def test_delayed_startup_provider_keeps_response_and_status_callbacks_runnable(tmp_path):
    # Use only loopback and a separate domain; this test never starts a motion
    # provider, supervisor, Nav2, sensor driver, or final velocity publisher.
    executable = os.environ["NAVIGATION_SERVER_TEST_EXECUTABLE"]
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", str(200 + os.getpid() % 20))
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                 "FASTDDS_BUILTIN_TRANSPORTS"):
        env.pop(name, None)
    source = tmp_path / "input"
    source.mkdir()
    for name in set(map_bundle.COMMON_ASSETS) | set(map_bundle.LIDAR_ASSETS):
        (source / name).write_text("fixture:" + name)
    (source / "metadata.json").write_text('{"motion_output_connected":false}')
    map_root = tmp_path / "maps"
    map_bundle.create_bundle(source, map_root / "delayed-map", "delayed-map",
                             source / "metadata.json", "lidar")

    previous = {key: os.environ.get(key) for key in
                ("ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_BUILTIN_TRANSPORTS")}
    for key in previous:
        if key in env:
            os.environ[key] = env[key]
        else:
            os.environ.pop(key, None)
    context = Context()
    rclpy.init(context=context, domain_id=int(env["ROS_DOMAIN_ID"]))
    node = Node("startup_provider_delay_test", context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    request_start, response_ready, statuses, calls = [], [], [], []
    ready = threading.Event()

    def load(request, response):
        calls.append(request.map_id)
        request_start.append(time.monotonic())
        # Original reentrant timer exhausts the four server threads while this
        # first request waits. The fixed timer permits response/status delivery.
        # Exceed the old 2 s timeout. Robot A's provider reaches this latency
        # during a loaded Nav2 cold start and must still return a definitive
        # outcome rather than faulting as unknown.
        time.sleep(2.6)
        response.accepted = True
        response.code = 200
        response.reason = "delayed_test_provider_ready"
        response_ready.append(time.monotonic())
        return response

    def status(message):
        statuses.append((time.monotonic(), message))
        if message.map_id == "delayed-map":
            ready.set()

    prefix = "/startup_delay_test_provider/lidar"
    test_group = ReentrantCallbackGroup()
    service = node.create_service(LoadMap, prefix + "/load_map", load, callback_group=test_group)
    subscription = node.create_subscription(
        NavigationStatus, "/sunhorizon/navigation/status", status, 10, callback_group=test_group)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    log_path = tmp_path / "navigation-server.log"
    process = None
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen([
                executable, "--ros-args", "-p", f"map_root:={map_root}",
                "-p", f"provider_prefix:={prefix}",
                "-p", "startup_map_id:=delayed-map",
            ], env=env, stdout=log, stderr=subprocess.STDOUT)
            assert ready.wait(12), log_path.read_text()
            assert calls == ["delayed-map"], log_path.read_text()
            assert response_ready and request_start
            during_wait = [t for t, _ in statuses if request_start[0] <= t <= response_ready[0]]
            assert len(during_wait) >= 3, (during_wait, log_path.read_text())
            assert "map_load_outcome_unknown" not in log_path.read_text()
            assert process.poll() is None, log_path.read_text()
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        node.destroy_service(service)
        node.destroy_subscription(subscription)
        executor.shutdown(timeout_sec=2)
        spin.join(timeout=2)
        node.destroy_node()
        context.shutdown()
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
