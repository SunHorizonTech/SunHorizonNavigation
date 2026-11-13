"""Fail-closed online/ordinary Robot A LIVO backend launch."""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_contract_path = Path(__file__).with_name("livo_contract.py")
_contract_spec = importlib.util.spec_from_file_location("livo_contract", _contract_path)
contract = importlib.util.module_from_spec(_contract_spec)
_contract_spec.loader.exec_module(contract)


POINTS_TOPIC = "/sunhorizon/airy/points"
IMU_TOPIC = "/sunhorizon/airy/imu"
LIVE_IMAGE_TOPIC = "/sunhorizon/raw/rgbd/chest/color/image"
LIVE_CAMERA_INFO_TOPIC = "/sunhorizon/raw/rgbd/chest/color/camera_info"
REPLAY_IMAGE_TOPIC = "/sunhorizon/rgbd/color/image"
REPLAY_CAMERA_INFO_TOPIC = "/sunhorizon/rgbd/color/camera_info"
VENDOR_POINTS_TOPIC = "/sunhorizon/livo/vendor/points"
VENDOR_IMAGE_TOPIC = "/sunhorizon/livo/vendor/image"
VENDOR_ODOMETRY_TOPIC = "/sunhorizon/livo/vendor/odom"
VENDOR_PROPAGATED_ODOMETRY_TOPIC = "/sunhorizon/livo/vendor/imu_propagate"
ODOMETRY_CANDIDATE_TOPIC = "/sunhorizon/localization/livo/odom_candidate"
ODOMETRY_STATUS_TOPIC = "/sunhorizon/localization/livo/odometry_status"
BLOCKED_TF_TOPIC = "/sunhorizon/livo/vendor_tf_forbidden"
BLOCKED_TF_STATIC_TOPIC = "/sunhorizon/livo/vendor_tf_static_forbidden"


def _launch_backend(context):
    mode = contract.validate_mode(LaunchConfiguration("mode").perform(context))
    use_sim_time = contract.validate_boolean(
        LaunchConfiguration("use_sim_time").perform(context), "use_sim_time")
    reliable_sensor_qos = contract.validate_boolean(
        LaunchConfiguration("reliable_sensor_qos").perform(context),
        "reliable_sensor_qos")
    reliable_odometry_qos = contract.validate_boolean(
        LaunchConfiguration("reliable_odometry_qos").perform(context),
        "reliable_odometry_qos")
    reliable_input_image_qos = contract.validate_boolean(
        LaunchConfiguration("reliable_input_image_qos").perform(context),
        "reliable_input_image_qos")
    reliable_image_qos = contract.validate_boolean(
        LaunchConfiguration("reliable_image_qos").perform(context),
        "reliable_image_qos")
    direct_camera_input = contract.validate_boolean(
        LaunchConfiguration("direct_camera_input").perform(context),
        "direct_camera_input")
    use_propagated_odometry = contract.validate_boolean(
        LaunchConfiguration("use_propagated_odometry").perform(context),
        "use_propagated_odometry")
    epoch_timing = contract.validate_boolean(
        LaunchConfiguration("epoch_timing").perform(context), "epoch_timing")
    image_topic = LIVE_IMAGE_TOPIC if direct_camera_input else REPLAY_IMAGE_TOPIC
    camera_info_topic = (
        LIVE_CAMERA_INFO_TOPIC if direct_camera_input else REPLAY_CAMERA_INFO_TOPIC)
    localization_point_rate_hz = contract.validate_localization_point_rate(
        LaunchConfiguration("localization_point_rate_hz").perform(context))
    localization_image_rate_hz = contract.validate_localization_image_rate(
        LaunchConfiguration("localization_image_rate_hz").perform(context))
    localization_filter_size_surf = contract.resolve_localization_filter_size_surf(
        mode,
        LaunchConfiguration("localization_filter_size_surf").perform(context),
    )
    input_readiness_timeout_sec = contract.validate_input_readiness_timeout(
        LaunchConfiguration("input_readiness_timeout_sec").perform(context))
    paths = contract.validate_vendor_contract(
        LaunchConfiguration("vendor_root").perform(context),
        LaunchConfiguration("vendor_executable").perform(context),
        LaunchConfiguration("vendor_params_path").perform(context),
        LaunchConfiguration("camera_model_path").perform(context),
    )
    config_path = Path(LaunchConfiguration("config_path").perform(context)).resolve(strict=True)
    if not config_path.is_file():
        raise RuntimeError("livo_robot_a_config_missing")

    common_parameters = {
        "mode": mode,
        "use_sim_time": use_sim_time,
        "vendor_ros2_commit": contract.ROS2_PORT_COMMIT,
        "official_airy_reference_commit": contract.OFFICIAL_AIRY_REFERENCE_COMMIT,
        "input_points_topic": POINTS_TOPIC,
        "input_imu_topic": IMU_TOPIC,
        "input_image_topic": image_topic,
        "input_camera_info_topic": camera_info_topic,
        "vendor_points_topic": VENDOR_POINTS_TOPIC,
        "vendor_point_rate_hz": (
            localization_point_rate_hz if mode == "localization" else 10.0),
        "vendor_image_topic": VENDOR_IMAGE_TOPIC,
        "vendor_image_rate_hz": (
            localization_image_rate_hz if mode == "localization" else 15.0),
        "vendor_odometry_topic": VENDOR_ODOMETRY_TOPIC,
        "vendor_propagated_odometry_topic": VENDOR_PROPAGATED_ODOMETRY_TOPIC,
        "output_odometry_topic": ODOMETRY_CANDIDATE_TOPIC,
        "output_odometry_status_topic": ODOMETRY_STATUS_TOPIC,
        "reliable_sensor_qos": reliable_sensor_qos,
        "reliable_odometry_qos": reliable_odometry_qos,
        "reliable_input_image_qos": reliable_input_image_qos,
        "reliable_image_qos": reliable_image_qos,
        "use_propagated_odometry": (
            mode == "localization" and use_propagated_odometry),
    }
    use_sim_time_text = "true" if use_sim_time else "false"
    reliable_sensor_qos_text = "true" if reliable_sensor_qos else "false"
    reliable_image_qos_text = "true" if reliable_image_qos else "false"
    # Reliability is a transport guarantee, not an offline/replay mode. Live
    # reliable localization must still discard superseded visual epochs rather
    # than build an unbounded latency tail. Sealed replay uses simulated time
    # and retains strict FIFO plus its first corrected epoch.
    live_latest_sensor_sync_text = "false" if use_sim_time else "true"
    live_drop_initial_corrected_epoch_text = "false" if use_sim_time else "true"

    vendor_command = [
        paths["vendor_executable"],
        "--ros-args",
        "--params-file", paths["vendor_params_path"],
        "--params-file", paths["camera_model_path"],
        "-p", f"use_sim_time:={use_sim_time_text}",
        "-p", f"common.lid_topic:={VENDOR_POINTS_TOPIC}",
        "-p", f"common.imu_topic:={IMU_TOPIC}",
        "-p", f"common.img_topic:={VENDOR_IMAGE_TOPIC}",
        "-p", "common.img_en:=1",
        "-p", "common.lidar_en:=1",
        "-p", f"common.reliable_sensor_qos:={reliable_sensor_qos_text}",
        "-p", f"common.reliable_image_qos:={reliable_image_qos_text}",
        "-p", f"common.live_latest_sensor_sync:={live_latest_sensor_sync_text}",
        "-p", f"common.localization_mode:={'true' if mode == 'localization' else 'false'}",
        "-p", (
            "common.live_drop_initial_corrected_epoch:="
            f"{live_drop_initial_corrected_epoch_text}"),
        "-p", "common.live_lidar_window_sec:=0.25",
        "-p", "preprocess.lidar_type:=7",
    ]
    if mode == "localization":
        vendor_command.extend([
            "-p", "common.live_lidar_queue_max_frames:=64",
            "-p", "common.live_imu_queue_max_samples:=2048",
            "-p", "common.live_image_queue_max_frames:=48",
            "-p", "common.live_sensor_queue_max_age_sec:=4.0",
            "-p", "publish.pub_scan_num:=1",
            "-p", "publish.live_rgb_accumulator_max_points:=250000",
            "-p", "vio.visual_map_voxel_size:=0.5",
            "-p", "vio.visual_map_max_points_per_voxel:=32",
            "-p", "vio.visual_map_max_points:=100000",
            "-p", "vio.visual_map_max_reference_age_sec:=180.0",
            "-p", "vio.visual_map_half_window_xy_voxels:=30",
            "-p", "vio.visual_map_half_window_z_voxels:=10",
            "-p", "vio.visual_map_prune_interval_sec:=5.0",
            "-p", "vio.visual_map_diagnostics_interval_sec:=10.0",
            "-p", "local_map.map_sliding_en:=true",
            "-p", "local_map.half_map_size:=30",
            "-p", "local_map.sliding_thresh:=2.0",
        ])
    if localization_filter_size_surf is not None:
        vendor_command.extend([
            "-p",
            f"preprocess.filter_size_surf:={localization_filter_size_surf:.12g}",
        ])
    vendor_command.extend([
        "-p", f"common.epoch_timing_enable:={'true' if epoch_timing else 'false'}",
        "-p", (
            "uav.imu_rate_odom:="
            f"{'true' if mode == 'localization' and use_propagated_odometry else 'false'}"),
        "-p", f"pcd_save.pcd_save_en:={'false' if mode == 'localization' else 'true'}",
        "-p", f"evo.pose_output_en:={'false' if mode == 'localization' else 'true'}",
        "-p", f"publish.dense_map_en:={'false' if mode == 'localization' else 'true'}",
        "-p", "publish.tf_en:=false",
        "-r", f"/aft_mapped_to_init:={VENDOR_ODOMETRY_TOPIC}",
        "-r", f"/LIVO2/imu_propagate:={VENDOR_PROPAGATED_ODOMETRY_TOPIC}",
        "-r", f"/tf:={BLOCKED_TF_TOPIC}",
        "-r", f"/tf_static:={BLOCKED_TF_STATIC_TOPIC}",
    ])
    readiness_executable = (
        Path(get_package_prefix("sunhorizon_livo_bringup")) /
        "lib" / "sunhorizon_livo_bringup" / "livo_ready_exec.py"
    ).resolve(strict=True)
    vendor_process = ExecuteProcess(
        cmd=[
            str(readiness_executable),
            "--points-topic", VENDOR_POINTS_TOPIC,
            "--imu-topic", IMU_TOPIC,
            "--image-topic", VENDOR_IMAGE_TOPIC,
            *(
                ["--airy-diagnostics-topic", "/sunhorizon/airy/diagnostics"]
                if mode == "localization" else []
            ),
            "--timeout-sec", f"{input_readiness_timeout_sec:.12g}",
            "--maximum-age-sec", "0.5",
            "--maximum-future-sec", "0.1",
            "--stable-window-sec", "2.0",
            "--use-sim-time", use_sim_time_text,
            "--reliable-sensor-qos", reliable_sensor_qos_text,
            "--reliable-image-qos", reliable_image_qos_text,
            "--",
            *vendor_command,
        ],
        additional_env={
            "OMP_NUM_THREADS": "4",
            "OMP_PROC_BIND": "false",
        },
        output="screen",
    )
    vendor_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=vendor_process,
        on_exit=[EmitEvent(event=Shutdown(
            reason="FAST-LIVO vendor process exited"))],
    ))

    return [
        Node(
            package="sunhorizon_livo_bringup",
            executable="livo_adapter_node",
            name="sunhorizon_livo_adapter",
            parameters=[str(config_path), common_parameters],
            output="screen",
        ),
        vendor_exit_shutdown,
        vendor_process,
    ]


def generate_launch_description():
    package_share = get_package_share_directory("sunhorizon_livo_bringup")
    default_config = os.path.join(package_share, "config", "livo_robot_a.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="mapping",
            description="LIVO role: mapping or localization local-odometry input."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "reliable_sensor_qos", default_value="false",
            description=(
                "Live defaults to best-effort bounded queues; sealed offline "
                "replay explicitly enables reliable point/IMU transport")),
        DeclareLaunchArgument(
            "reliable_odometry_qos", default_value="false",
            description=(
                "Live corrected/propagated odometry is latest-only; sealed "
                "offline replay explicitly enables reliable retention.")),
        DeclareLaunchArgument(
            "reliable_input_image_qos", default_value="false",
            description=(
                "Live owned raw RGB uses latest-only best-effort so a DDS "
                "reader cannot block hardware capture; sealed replay "
                "explicitly enables reliable input transport.")),
        DeclareLaunchArgument(
            "reliable_image_qos", default_value="true",
            description=(
                "Adapter-selected private RGB uses reliable KEEP_LAST(1) at "
                "the FAST-LIVO2 process boundary.")),
        DeclareLaunchArgument(
            "direct_camera_input", default_value="true",
            description=(
                "Live Robot A consumes the owned raw chest RGB directly; "
                "sealed replay explicitly selects the normalized bag topics.")),
        DeclareLaunchArgument(
            "epoch_timing", default_value="false",
            description="Enable bounded per-epoch latency diagnostics; no algorithm change."),
        DeclareLaunchArgument(
            "use_propagated_odometry", default_value="true",
            description=(
                "Localization uses the vendor IMU-rate propagated odometry by "
                "default. Set false for an explicit corrected-only fallback; "
                "mapping remains corrected-only regardless.")),
        DeclareLaunchArgument(
            "localization_point_rate_hz", default_value="10.0",
            description=(
                "Robot A live localization point rate. 10 Hz preserves Airy "
                "coverage; 6 and 8 Hz are explicit comparisons. "
                "Mapping always remains 10 Hz.")),
        DeclareLaunchArgument(
            "localization_image_rate_hz", default_value="10.0",
            description=(
                "Robot A live RGB rate. 10 Hz is the quality-first profile; "
                "raw acquisition remains 30 Hz and mapping remains 15 Hz.")),
        DeclareLaunchArgument(
            "localization_filter_size_surf", default_value="0.15",
            description=(
                "FAST-LIVO2 surface voxel filter used only for localization; "
                "must be finite and within 0.05..0.50. Mapping keeps the "
                "vendor parameter-file value.")),
        DeclareLaunchArgument(
            "input_readiness_timeout_sec", default_value="30.0",
            description=(
                "Fail-closed timeout for fresh point, IMU, and RGB samples "
                "before FAST-LIVO2 is exec'd; finite and within 1..120 s.")),
        DeclareLaunchArgument("config_path", default_value=default_config),
        DeclareLaunchArgument(
            "vendor_root",
            description="Absolute root containing the patched pinned vendor build and receipt."),
        DeclareLaunchArgument(
            "vendor_executable",
            description="Absolute patched fastlivo_mapping executable below vendor_root."),
        DeclareLaunchArgument(
            "vendor_params_path",
            description="Absolute Robot A FAST-LIVO2 extrinsic and tuning parameters."),
        DeclareLaunchArgument(
            "camera_model_path",
            description="Absolute Robot A camera parameters loaded by FAST-LIVO."),
        OpaqueFunction(function=_launch_backend),
    ])
