#pragma once

namespace sunhorizon_localization_manager::topics {

inline constexpr char kLidarOdometryCandidate[] =
    "/sunhorizon/localization/lidar/odom_candidate";
inline constexpr char kLidarTrackingOdometryCandidate[] =
    "/sunhorizon/localization/lidar/tracking_odom_candidate";
inline constexpr char kLidarMapPoseCandidate[] =
    "/sunhorizon/localization/lidar/map_pose_candidate";
inline constexpr char kLidarStatus[] = "/sunhorizon/localization/lidar/status";
inline constexpr char kVisualOdometryCandidate[] =
    "/sunhorizon/localization/visual/odom_candidate";
inline constexpr char kVisualMapPoseCandidate[] =
    "/sunhorizon/localization/visual/map_pose_candidate";
inline constexpr char kVisualStatus[] = "/sunhorizon/localization/visual/status";
inline constexpr char kLivoOdometryCandidate[] =
    "/sunhorizon/localization/livo/odom_candidate";
inline constexpr char kLivoNdtOdometryCandidate[] =
    "/sunhorizon/localization/livo/ndt_odom_candidate";
inline constexpr char kLivoMapPoseCandidate[] =
    "/sunhorizon/localization/livo/map_pose_candidate";
inline constexpr char kLivoStatus[] = "/sunhorizon/localization/livo/status";
inline constexpr char kCanonicalOdometry[] = "/sunhorizon/localization/odom";
inline constexpr char kCanonicalStatus[] = "/sunhorizon/localization/status";
inline constexpr char kStatusMarker[] =
    "/sunhorizon/localization/status_marker";
inline constexpr char kComparisonStatus[] = "/sunhorizon/localization/comparison_status";
inline constexpr char kResetService[] = "/sunhorizon/localization/reset";

}  // namespace sunhorizon_localization_manager::topics
