import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TRUE_VALUES = {"1", "true", "yes", "on"}
EXPECTED_INPUT_FRAMES = {"airy_lidar", "airy_lidar_axis_provisional"}


def _launch_value(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_flag(context, name: str) -> bool:
    return _launch_value(context, name).lower() in TRUE_VALUES


def _validate_result_translation_step(value, attended_profile):
    ceiling = 0.10 if attended_profile else 0.05
    if not math.isfinite(value) or not 0.0 < value <= ceiling:
        raise RuntimeError(
            f"NDT maximum_result_translation_step_m must be within (0, {ceiling}] "
            "for the explicitly selected result-step profile")
    return value


def _sampling_geometry(profile, backend_name, canonical_prior, direct_tracking):
    if profile == "legacy":
        return 0.5, 2.0
    if profile not in {"detail_020", "detail_020_grid_050"}:
        raise RuntimeError("NDT sampling_profile must be legacy, detail_020 or detail_020_grid_050")
    if backend_name != "livo" or not canonical_prior or direct_tracking:
        raise RuntimeError("NDT detail_020 requires canonical LIVO prior mode")
    return 0.2, (0.5 if profile == "detail_020_grid_050" else 2.0)


def _launch_vector(context, name: str, length: int) -> list[float]:
    try:
        values = [
            float(value.strip())
            for value in _launch_value(context, name).split(",")
        ]
    except ValueError as error:
        raise RuntimeError(f"NDT {name} must contain finite comma-separated numbers") from error
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"NDT {name} must contain {length} finite values")
    return values


def _compose(context):
    map_path = _launch_value(context, "map_path")
    if (
        not os.path.isabs(map_path)
        or os.path.islink(map_path)
        or not os.path.isfile(map_path)
        or os.path.getsize(map_path) == 0
    ):
        raise RuntimeError("NDT map_path must be an absolute nonempty regular file")
    if not _launch_flag(context, "initial_pose_accepted"):
        raise RuntimeError("NDT initial pose must be explicitly accepted")
    direct_map_tracking_enabled = _launch_flag(
        context, "direct_map_tracking_enabled"
    )
    startup_alignment_enabled = _launch_flag(
        context, "startup_alignment_enabled"
    )
    startup_alignment_maximum_seed_translation_m = float(_launch_value(
        context, "startup_alignment_maximum_seed_translation_m"))
    startup_alignment_maximum_seed_yaw_rad = float(_launch_value(
        context, "startup_alignment_maximum_seed_yaw_rad"))
    startup_alignment_maximum_candidate_translation_spread_m = float(
        _launch_value(
            context,
            "startup_alignment_maximum_candidate_translation_spread_m"))
    startup_alignment_maximum_candidate_yaw_spread_rad = float(_launch_value(
        context, "startup_alignment_maximum_candidate_yaw_spread_rad"))
    startup_alignment_maximum_sample_gap_sec = float(_launch_value(
        context, "startup_alignment_maximum_sample_gap_sec"))
    try:
        startup_alignment_required_consecutive_samples = int(_launch_value(
            context, "startup_alignment_required_consecutive_samples"))
    except ValueError as error:
        raise RuntimeError(
            "NDT startup alignment required samples must be an integer") from error
    if startup_alignment_enabled and (
        not _launch_flag(context, "require_runtime_initial_pose")
        or direct_map_tracking_enabled
        or not 0.35 < startup_alignment_maximum_seed_translation_m <= 2.0
        or not 0.12 < startup_alignment_maximum_seed_yaw_rad <= math.radians(45.0)
        or not 0.0 < startup_alignment_maximum_candidate_translation_spread_m <= 0.35
        or not 0.0 < startup_alignment_maximum_candidate_yaw_spread_rad <= 0.12
        or not 0.0 < startup_alignment_maximum_sample_gap_sec <= 0.50
        or not 2 <= startup_alignment_required_consecutive_samples <= 100
    ):
        raise RuntimeError("NDT startup alignment parameters are invalid")

    input_points_topic = _launch_value(context, "input_points_topic")
    if not input_points_topic.startswith("/"):
        raise RuntimeError("NDT input_points_topic must be an absolute ROS topic")
    odometry_prior_topic = _launch_value(context, "odometry_prior_topic")
    if not direct_map_tracking_enabled and not odometry_prior_topic.startswith("/"):
        raise RuntimeError("NDT odometry_prior_topic must be an absolute ROS topic")
    for topic_name in (
        "map_pose_candidate_topic", "odom_candidate_topic", "status_topic",
        "diagnostics_topic",
    ):
        if not _launch_value(context, topic_name).startswith("/"):
            raise RuntimeError(f"NDT {topic_name} must be an absolute ROS topic")
    backend_name = _launch_value(context, "backend_name")
    if backend_name not in {"lidar", "livo"}:
        raise RuntimeError("NDT backend_name must be lidar or livo")
    sampling_leaf, registration_resolution = _sampling_geometry(
        _launch_value(context, "sampling_profile"), backend_name,
        _launch_flag(context, "canonical_base_prior"), direct_map_tracking_enabled)
    ndt_backend = _launch_value(context, "ndt_backend").lower()
    ndt_neighborhood = _launch_value(context, "ndt_neighborhood").upper()
    try:
        ndt_threads = int(_launch_value(context, "ndt_threads"))
    except ValueError as error:
        raise RuntimeError("NDT ndt_threads must be an integer") from error
    if ndt_backend not in {"pcl", "ndt_omp"}:
        raise RuntimeError("NDT ndt_backend must be pcl or ndt_omp")
    if ndt_neighborhood not in {"KDTREE", "DIRECT1", "DIRECT7"}:
        raise RuntimeError(
            "NDT ndt_neighborhood must be KDTREE, DIRECT1, or DIRECT7"
        )
    if not 1 <= ndt_threads <= 256:
        raise RuntimeError("NDT ndt_threads must be within [1, 256]")
    if ndt_backend == "pcl" and (
        ndt_threads != 1 or ndt_neighborhood != "KDTREE"
    ):
        raise RuntimeError(
            "PCL NDT requires ndt_threads=1 and ndt_neighborhood=KDTREE"
        )
    prior_sync_tolerance = float(_launch_value(
        context, "prior_sync_tolerance_sec"))
    if not math.isfinite(prior_sync_tolerance) or not (
        0.0 <= prior_sync_tolerance <= 0.06
    ):
        raise RuntimeError("NDT prior_sync_tolerance_sec must be within [0, 0.06]")
    prior_timeout = float(_launch_value(context, "prior_timeout_sec"))
    if not math.isfinite(prior_timeout) or not (0.0 < prior_timeout <= 0.65):
        raise RuntimeError("NDT prior_timeout_sec must be within (0, 0.65]")
    maximum_input_age = float(_launch_value(
        context, "maximum_input_age_sec"))
    if not math.isfinite(maximum_input_age) or not (
        0.0 < maximum_input_age <= 0.65
    ):
        raise RuntimeError(
            "NDT maximum_input_age_sec must be within (0, 0.65]")
    input_future_tolerance = float(_launch_value(
        context, "input_future_tolerance_sec"))
    if not math.isfinite(input_future_tolerance) or not (
        0.0 <= input_future_tolerance <= 0.01
    ):
        raise RuntimeError(
            "NDT input_future_tolerance_sec must be within [0, 0.01]")
    accepted_result_timeout = float(_launch_value(
        context, "accepted_result_timeout_sec"))
    if not math.isfinite(accepted_result_timeout) or not (
        0.0 < accepted_result_timeout <= 0.80
    ):
        raise RuntimeError(
            "NDT accepted_result_timeout_sec must be within (0, 0.80]")
    maximum_twist_sensor_dt = float(_launch_value(
        context, "maximum_twist_sensor_dt_sec"))
    if not math.isfinite(maximum_twist_sensor_dt) or not (
        0.0 < maximum_twist_sensor_dt <= 0.65
    ):
        raise RuntimeError(
            "NDT maximum_twist_sensor_dt_sec must be within (0, 0.65]")
    maximum_result_step_gap = float(_launch_value(
        context, "maximum_result_step_gap_sec"))
    maximum_result_translation_step = float(_launch_value(
        context, "maximum_result_translation_step_m"))
    attended_result_step_profile = _launch_flag(context, "attended_result_step_profile")
    if attended_result_step_profile and (
        backend_name != "livo" or direct_map_tracking_enabled or
        not _launch_flag(context, "canonical_base_prior") or
        not _launch_flag(context, "require_runtime_initial_pose")
    ):
        raise RuntimeError(
            "attended_result_step_profile_requires_explicit_seeded_livo_canonical_prior")
    maximum_result_yaw_step = float(_launch_value(
        context, "maximum_result_yaw_step_rad"))
    if not math.isfinite(maximum_result_step_gap) or not (
        0.0 < maximum_result_step_gap <= 0.80
    ):
        raise RuntimeError(
            "NDT maximum_result_step_gap_sec must be within (0, 0.80]"
        )
    _validate_result_translation_step(
        maximum_result_translation_step, attended_result_step_profile)
    if not math.isfinite(maximum_result_yaw_step) or not (
        0.0 < maximum_result_yaw_step <= math.radians(2.0)
    ):
        raise RuntimeError(
            "NDT maximum_result_yaw_step_rad must be within (0, 2 degrees]"
        )
    maximum_match_rate_hz = float(_launch_value(
        context, "maximum_match_rate_hz"))
    if not math.isfinite(maximum_match_rate_hz) or maximum_match_rate_hz <= 0.0:
        raise RuntimeError("NDT maximum_match_rate_hz must be finite and positive")
    diagnostic_inlier_max_distance_m = float(_launch_value(
        context, "diagnostic_inlier_max_distance_m"))
    if (
        not math.isfinite(diagnostic_inlier_max_distance_m)
        or diagnostic_inlier_max_distance_m <= 0.0
    ):
        raise RuntimeError(
            "NDT diagnostic_inlier_max_distance_m must be finite and positive"
        )
    minimum_scan_range_m = float(_launch_value(
        context, "minimum_scan_range_m"))
    maximum_scan_range_m = float(_launch_value(
        context, "maximum_scan_range_m"))
    if (
        not math.isfinite(minimum_scan_range_m)
        or minimum_scan_range_m <= 0.0
        or not math.isfinite(maximum_scan_range_m)
        or maximum_scan_range_m <= minimum_scan_range_m
    ):
        raise RuntimeError(
            "NDT scan range limits must be finite, positive, and strictly ordered"
        )
    try:
        minimum_raw_scan_points = int(_launch_value(
            context, "minimum_raw_scan_points"))
    except ValueError as error:
        raise RuntimeError(
            "NDT minimum_raw_scan_points must be an integer"
        ) from error
    if minimum_raw_scan_points < 0:
        raise RuntimeError("NDT minimum_raw_scan_points must be nonnegative")
    source_map_bounds_filter_enabled = _launch_flag(
        context, "source_map_bounds_filter_enabled")
    source_map_bounds_margin_m = float(_launch_value(
        context, "source_map_bounds_margin_m"))
    source_map_bounds_minimum_retained_fraction = float(_launch_value(
        context, "source_map_bounds_minimum_retained_fraction"))
    try:
        source_map_bounds_minimum_retained_points = int(_launch_value(
            context, "source_map_bounds_minimum_retained_points"))
    except ValueError as error:
        raise RuntimeError(
            "NDT source map-bounds minimum retained points must be an integer"
        ) from error
    if (
        not math.isfinite(source_map_bounds_margin_m)
        or source_map_bounds_margin_m < 0.0
        or not math.isfinite(source_map_bounds_minimum_retained_fraction)
        or not 0.0 < source_map_bounds_minimum_retained_fraction <= 1.0
        or source_map_bounds_minimum_retained_points < 100
    ):
        raise RuntimeError("NDT source map-bounds filter parameters are invalid")
    tracking_odometry_topic = _launch_value(
        context, "tracking_odom_candidate_topic"
    )
    if direct_map_tracking_enabled and not tracking_odometry_topic.startswith("/"):
        raise RuntimeError(
            "NDT tracking_odom_candidate_topic must be absolute in direct mode"
        )
    expected_input_frame = _launch_value(context, "expected_input_frame")
    if expected_input_frame not in EXPECTED_INPUT_FRAMES:
        raise RuntimeError(
            "NDT expected_input_frame must be airy_lidar or "
            "airy_lidar_axis_provisional"
        )
    canonical_base_prior = _launch_flag(context, "canonical_base_prior")
    if canonical_base_prior and (
        direct_map_tracking_enabled
        or _launch_value(context, "expected_prior_child_frame") != "base_footprint"
    ):
        raise RuntimeError(
            "NDT canonical_base_prior requires prior mode and base_footprint child"
        )
    source_to_base = _launch_vector(
        context, "source_to_base_xyz_qxyzw", 7)
    quaternion_norm = math.sqrt(sum(value * value for value in source_to_base[3:]))
    if not 0.999 <= quaternion_norm <= 1.001:
        raise RuntimeError("NDT source_to_base quaternion must be normalized")

    config = os.path.join(
        get_package_share_directory("sunhorizon_3d_localizer"),
        "config",
        "localizer.yaml",
    )
    return [
        Node(
            package="sunhorizon_3d_localizer",
            executable="sunhorizon_ndt_localizer_node",
            name="sunhorizon_ndt_localizer",
            parameters=[config, {
                "map_path": map_path,
                "initial_pose_accepted": True,
                "require_runtime_initial_pose": _launch_flag(
                    context, "require_runtime_initial_pose"),
                "startup_alignment_enabled": startup_alignment_enabled,
                "startup_alignment_maximum_seed_translation_m":
                    startup_alignment_maximum_seed_translation_m,
                "startup_alignment_maximum_seed_yaw_rad":
                    startup_alignment_maximum_seed_yaw_rad,
                "startup_alignment_maximum_candidate_translation_spread_m":
                    startup_alignment_maximum_candidate_translation_spread_m,
                "startup_alignment_maximum_candidate_yaw_spread_rad":
                    startup_alignment_maximum_candidate_yaw_spread_rad,
                "startup_alignment_maximum_sample_gap_sec":
                    startup_alignment_maximum_sample_gap_sec,
                "startup_alignment_required_consecutive_samples":
                    startup_alignment_required_consecutive_samples,
                "initial_pose_topic": _launch_value(
                    context, "initial_pose_topic"),
                "direct_map_tracking_enabled": direct_map_tracking_enabled,
                "input_points_topic": input_points_topic,
                "reliable_input_points_qos": _launch_flag(
                    context, "reliable_input_points_qos"),
                "odometry_prior_topic": odometry_prior_topic,
                "map_pose_candidate_topic": _launch_value(
                    context, "map_pose_candidate_topic"),
                "odom_candidate_topic": _launch_value(
                    context, "odom_candidate_topic"),
                "status_topic": _launch_value(context, "status_topic"),
                "diagnostics_topic": _launch_value(
                    context, "diagnostics_topic"),
                "backend_name": backend_name,
                "ndt_backend": ndt_backend,
                "ndt_threads": ndt_threads,
                "ndt_neighborhood": ndt_neighborhood,
                "tracking_odom_candidate_topic": tracking_odometry_topic,
                "expected_input_frame": expected_input_frame,
                "expected_prior_frame": _launch_value(
                    context, "expected_prior_frame"),
                "expected_prior_child_frame": _launch_value(
                    context, "expected_prior_child_frame"),
                "prior_sync_tolerance_sec": prior_sync_tolerance,
                "prior_timeout_sec": prior_timeout,
                "maximum_input_age_sec": maximum_input_age,
                "input_future_tolerance_sec": input_future_tolerance,
                "accepted_result_timeout_sec": accepted_result_timeout,
                "maximum_twist_sensor_dt_sec": maximum_twist_sensor_dt,
                "maximum_result_step_gap_sec": maximum_result_step_gap,
                "attended_result_step_profile": attended_result_step_profile,
                "maximum_result_translation_step_m":
                    maximum_result_translation_step,
                "maximum_result_yaw_step_rad": maximum_result_yaw_step,
                "maximum_match_rate_hz": maximum_match_rate_hz,
                "diagnostic_inlier_max_distance_m":
                    diagnostic_inlier_max_distance_m,
                "minimum_scan_range_m": minimum_scan_range_m,
                "maximum_scan_range_m": maximum_scan_range_m,
                "minimum_raw_scan_points": minimum_raw_scan_points,
                "map_leaf_m": sampling_leaf,
                "scan_leaf_m": sampling_leaf,
                "resolution_m": registration_resolution,
                "source_map_bounds_filter_enabled":
                    source_map_bounds_filter_enabled,
                "source_map_bounds_margin_m": source_map_bounds_margin_m,
                "source_map_bounds_minimum_retained_fraction":
                    source_map_bounds_minimum_retained_fraction,
                "source_map_bounds_minimum_retained_points":
                    source_map_bounds_minimum_retained_points,
                "use_receipt_time_for_output": _launch_flag(
                    context, "use_receipt_time_for_output"),
                "source_to_base_transform_accepted": _launch_flag(
                    context, "source_to_base_transform_accepted"),
                "source_to_base_xyz_qxyzw": source_to_base,
                "prior_basis_transform_accepted": _launch_flag(
                    context, "prior_basis_transform_accepted"),
                "canonical_base_prior": canonical_base_prior,
            }],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "sampling_profile", default_value="legacy",
            description=("Explicit point sampling selection: legacy 0.50 m or "
                         "detail_020 0.20 m. NDT resolution and admission limits are unchanged.")),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="Absolute accepted NDT target map.pcd path",
        ),
        DeclareLaunchArgument(
            "initial_pose_accepted",
            default_value="false",
            description="Explicit acceptance gate for the configured NDT initial pose",
        ),
        DeclareLaunchArgument(
            "require_runtime_initial_pose",
            default_value="false",
            description=(
                "Remain INITIALIZING until an accepted map-frame /initialpose arrives"
            ),
        ),
        DeclareLaunchArgument(
            "startup_alignment_enabled",
            default_value="false",
            description=(
                "Allow bounded multi-frame cold-start alignment outside the "
                "normal per-result tracking correction limit"
            ),
        ),
        DeclareLaunchArgument(
            "startup_alignment_maximum_seed_translation_m",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "startup_alignment_maximum_seed_yaw_rad",
            default_value="0.2617993877991494",
        ),
        DeclareLaunchArgument(
            "startup_alignment_maximum_candidate_translation_spread_m",
            default_value="0.08",
        ),
        DeclareLaunchArgument(
            "startup_alignment_maximum_candidate_yaw_spread_rad",
            default_value="0.03490658503988659",
        ),
        DeclareLaunchArgument(
            "startup_alignment_maximum_sample_gap_sec",
            default_value="0.30",
        ),
        DeclareLaunchArgument(
            "startup_alignment_required_consecutive_samples",
            default_value="10",
        ),
        DeclareLaunchArgument(
            "initial_pose_topic",
            default_value="/sunhorizon/localization/initial_guess",
            description="Internal accepted initial-pose topic from the map provider",
        ),
        DeclareLaunchArgument(
            "direct_map_tracking_enabled",
            default_value="false",
            description=(
                "Use accepted fixed-map NDT results as consecutive initial guesses; "
                "mutually exclusive with the legacy odometry-prior processing path"
            ),
        ),
        DeclareLaunchArgument(
            "input_points_topic",
            default_value="/sunhorizon/airy/points",
            description="Owned Airy point cloud; override only for recorded shadow input",
        ),
        DeclareLaunchArgument(
            "reliable_input_points_qos",
            default_value="false",
            description=(
                "Request reliable latest-only point input; enable only when the "
                "authoritative publisher is proven reliable"
            ),
        ),
        DeclareLaunchArgument(
            "odometry_prior_topic",
            default_value="/sunhorizon/glim/odom_prior",
            description="Project-owned monotonic GLIM odometry prior",
        ),
        DeclareLaunchArgument(
            "map_pose_candidate_topic",
            default_value="/sunhorizon/localization/lidar/map_pose_candidate",
        ),
        DeclareLaunchArgument(
            "odom_candidate_topic",
            default_value="/sunhorizon/localization/lidar/odom_candidate",
        ),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/sunhorizon/localization/lidar/status",
        ),
        DeclareLaunchArgument(
            "diagnostics_topic",
            default_value="/sunhorizon/localization/lidar/ndt_diagnostics",
            description=(
                "Per-match all-point fitness-m2, source inlier fraction, "
                "source-to-target inlier RMSE, and split runtimes"
            ),
        ),
        DeclareLaunchArgument("backend_name", default_value="lidar"),
        DeclareLaunchArgument(
            "ndt_backend",
            default_value="pcl",
            description="Registration backend: pcl or the optional ndt_omp",
        ),
        DeclareLaunchArgument(
            "ndt_threads",
            default_value="1",
            description="Registration thread count; PCL requires exactly one",
        ),
        DeclareLaunchArgument(
            "ndt_neighborhood",
            default_value="KDTREE",
            description="Registration neighborhood: KDTREE, DIRECT1, or DIRECT7",
        ),
        DeclareLaunchArgument(
            "tracking_odom_candidate_topic",
            default_value=(
                "/sunhorizon/localization/lidar/tracking_odom_candidate"
            ),
            description=(
                "Independent manager tracking evidence; published only in direct mode"
            ),
        ),
        DeclareLaunchArgument(
            "expected_input_frame",
            default_value="airy_lidar",
            description="Expected PointCloud2 frame; axis-provisional is shadow-only",
        ),
        DeclareLaunchArgument("expected_prior_frame", default_value="odom"),
        DeclareLaunchArgument(
            "expected_prior_child_frame", default_value="rslidar"),
        DeclareLaunchArgument(
            "prior_sync_tolerance_sec",
            default_value="0.001",
            description="Maximum point-cloud to odometry-prior timestamp skew",
        ),
        DeclareLaunchArgument(
            "prior_timeout_sec",
            default_value="0.25",
            description="Maximum continuity gap between odometry-prior samples",
        ),
        DeclareLaunchArgument(
            "maximum_input_age_sec",
            default_value="0.50",
            description=(
                "Maximum sensor/header age when a completed NDT result is accepted"
            ),
        ),
        DeclareLaunchArgument(
            "input_future_tolerance_sec",
            default_value="0.01",
            description=(
                "Maximum positive sensor clock lead tolerated at result acceptance"
            ),
        ),
        DeclareLaunchArgument(
            "accepted_result_timeout_sec",
            default_value="0.50",
            description=(
                "Maximum local receipt gap since the latest accepted NDT result"
            ),
        ),
        DeclareLaunchArgument(
            "maximum_twist_sensor_dt_sec",
            default_value="0.50",
            description="Maximum sensor-time interval used to derive NDT twist",
        ),
        DeclareLaunchArgument(
            "maximum_result_step_gap_sec",
            default_value="0.50",
            description=(
                "Maximum sensor-time interval between admitted NDT result-step "
                "samples; a larger gap requires explicit reinitialization"
            ),
        ),
        DeclareLaunchArgument(
            "attended_result_step_profile",
            default_value="false",
            description=(
                "Explicit attended trial opt-in permitting a translation cap up to 0.10 m; "
                "does not change the default 0.05 m or authorize motion"),
        ),
        DeclareLaunchArgument(
            "maximum_result_translation_step_m",
            default_value="0.05",
            description=(
                "Maximum planar translation between admitted raw NDT results"
            ),
        ),
        DeclareLaunchArgument(
            "maximum_result_yaw_step_rad",
            default_value="0.03490658503988659",
            description="Maximum wrapped yaw step (2 degrees) between NDT results",
        ),
        DeclareLaunchArgument(
            "maximum_match_rate_hz",
            default_value="1000000000.0",
            description=(
                "Maximum NDT align attempts per sensor-time second; the 1e9 Hz "
                "compatibility default passes every strictly increasing timestamp"
            ),
        ),
        DeclareLaunchArgument(
            "diagnostic_inlier_max_distance_m",
            default_value="1.0",
            description=(
                "One-way source-to-target nearest-neighbor inlier radius for "
                "source_inlier_fraction and source_to_target_inlier_rmse_m; "
                "does not change the all-point fitness-m2 acceptance gate"
            ),
        ),
        DeclareLaunchArgument(
            "minimum_scan_range_m",
            default_value="0.5",
            description="Inclusive minimum source-point range before voxel filtering",
        ),
        DeclareLaunchArgument(
            "maximum_scan_range_m",
            default_value="30.0",
            description="Inclusive maximum source-point range before voxel filtering",
        ),
        DeclareLaunchArgument(
            "minimum_raw_scan_points",
            default_value="0",
            description=(
                "Minimum raw width-times-height source cardinality; zero disables "
                "the incomplete-frame skip gate"
            ),
        ),
        DeclareLaunchArgument(
            "source_map_bounds_filter_enabled",
            default_value="false",
            description=(
                "Filter alignment source by the active map target AABB using the "
                "initial map-to-source estimate; robot profiles opt in explicitly"
            ),
        ),
        DeclareLaunchArgument(
            "source_map_bounds_margin_m",
            default_value="1.0",
            description="Expansion margin around the active target AABB",
        ),
        DeclareLaunchArgument(
            "source_map_bounds_minimum_retained_fraction",
            default_value="0.6",
            description=(
                "Minimum fraction of range-and-voxel source points retained by "
                "the map-bounds filter"
            ),
        ),
        DeclareLaunchArgument(
            "source_map_bounds_minimum_retained_points",
            default_value="500",
            description=(
                "Minimum retained source cardinality required before alignment"
            ),
        ),
        DeclareLaunchArgument(
            "use_receipt_time_for_output",
            default_value="false",
            description="Use current ROS time for recorded shadow replay outputs",
        ),
        DeclareLaunchArgument(
            "source_to_base_transform_accepted",
            default_value="false",
            description="Fail-closed gate for provisional T_source_base geometry",
        ),
        DeclareLaunchArgument(
            "source_to_base_xyz_qxyzw",
            default_value="-0.096,0.0,-0.21926,0.0,0.0,0.0,1.0",
            description=(
                "T_source_base as comma-separated xyz and quaternion xyzw; "
                "the caller must bind this to accepted geometry"),
        ),
        DeclareLaunchArgument(
            "prior_basis_transform_accepted",
            default_value="false",
            description="Fail-closed gate for raw GLIM to project-axis basis",
        ),
        DeclareLaunchArgument(
            "canonical_base_prior",
            default_value="false",
            description=(
                "Treat odometry prior as canonical odom to base_footprint; "
                "disables the legacy GLIM axis-basis conversion"
            ),
        ),
        OpaqueFunction(function=_compose),
    ])
