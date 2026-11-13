#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>

namespace sunhorizon_3d_localizer {

struct NdtDirectStamp {
  int64_t source_stamp_ns{0};
  int64_t receipt_ros_ns{0};
  int64_t receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string source_identity;
};

struct NdtDirectResult {
  NdtDirectStamp input;
  std::optional<NdtDirectStamp> prior_input;
  int64_t result_receipt_ros_ns{0};
  int64_t result_receipt_steady_ns{0};
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr map_pose;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  bool independent_tracking_evidence{false};
  // Accepted p_PCD = T_registration_source * p_source, before the planar
  // base projection. Header uses the original scan time. Even when its
  // parent is named "map", this datum is distinct from canonical planar TF;
  // consumers must not substitute public TF for this complete 3D transform.
  // Shared immutable geometry only: this does not publish or alter canonical TF.
  geometry_msgs::msg::TransformStamped::ConstSharedPtr map_to_source;
  // Actual registration PCD path, not a content hash or asset qualification.
  std::string registration_map_path;
};

struct NdtDirectSinks {
  // Nonblocking bounded-mailbox callbacks. They must not reenter the localizer
  // or perform recording, DDS publication, registration or navigation work.
  std::function<void(std::shared_ptr<const NdtDirectResult>)> accepted;
  std::function<void(sunhorizon_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr)>
      status;
  // Keep the existing manager/supervisor egress until those owners migrate.
  bool publish_ros_candidates{true};
  // Opt-in low-rate legacy status only when candidate ROS egress is disabled.
  // False preserves existing callers' candidate/status coupling by default.
  bool publish_ros_status_compatibility{false};
};

struct NdtDirectCounters {
  uint64_t clouds_submitted{0};
  uint64_t clouds_overwritten{0};
  uint64_t priors_submitted{0};
  uint64_t priors_overwritten{0};
};

class NdtDirectRuntime {
 public:
  virtual ~NdtDirectRuntime() = default;
  virtual std::shared_ptr<rclcpp::Node> node() = 0;
  // Digest of the stable file actually loaded into this registration owner.
  virtual std::string registration_map_sha256() const = 0;
  // Immutable shared inputs, one pending cloud and at most 100 pending priors.
  // The separate worker applies the unchanged raw-scan/prior synchronizer and
  // complete online NDT acceptance path. Caller acquisition never runs NDT.
  virtual bool submit_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr input,
                             const NdtDirectStamp& stamp) = 0;
  virtual bool submit_prior(nav_msgs::msg::Odometry::ConstSharedPtr input,
                            const NdtDirectStamp& stamp) = 0;
  virtual NdtDirectCounters counters() const = 0;
  virtual void stop() = 0;
};

// This replaces the existing NDT owner; it must not run alongside that owner.
// The returned node retains initial-pose and liveness callbacks, but creates no
// raw-cloud or odometry-prior DDS subscription. Constructing it alone does not
// connect the vendor LIVO estimator or switch the production runtime.
std::shared_ptr<NdtDirectRuntime> make_ndt_direct_runtime(
    const rclcpp::NodeOptions& options, NdtDirectSinks sinks);

}  // namespace sunhorizon_3d_localizer
