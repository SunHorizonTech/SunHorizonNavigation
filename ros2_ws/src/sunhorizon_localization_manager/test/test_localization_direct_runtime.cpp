#include <chrono>
#include <memory>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sunhorizon_localization_manager/localization_manager_logic.hpp"
#include "sunhorizon_localization_manager/localization_direct_runtime.hpp"

namespace {
namespace manager = sunhorizon_localization_manager;
using Status = sunhorizon_navigation_interfaces::msg::LocalizationStatus;

class DirectManager : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) { int argc = 0; char** argv = nullptr; rclcpp::init(argc, argv); }
  }
  int64_t reset_barrier{0};
  void SetUp() override { create(false); }
  void create(bool alignment,double maximum_age=.50,double receipt_timeout=.50,
      double tracking_timeout=.25,bool preserve_6d=false) {
    runtime.reset(); states.clear(); health.clear();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("active_backend", "livo"),
        rclcpp::Parameter("livo_ndt_alignment_enabled", alignment),
        rclcpp::Parameter("preserve_6d_map_alignment",preserve_6d),
        rclcpp::Parameter("maximum_measurement_age_sec",maximum_age),
        rclcpp::Parameter("map_pose_timeout_sec",receipt_timeout),
        rclcpp::Parameter("tracking_odometry_timeout_sec",tracking_timeout)});
    manager::LocalizationDirectSinks sinks;
    sinks.tracking_reset=[this](int64_t value){reset_barrier=value;};
    sinks.publish_ros_compatibility = false;
    sinks.canonical = [this](auto state) { states.push_back(std::move(state)); };
    sinks.status = [this](auto status) { health.push_back(std::move(status)); };
    runtime = manager::make_localization_direct_runtime(options, std::move(sinks));
    source = runtime->node()->get_clock()->now().nanoseconds() - 10000000;
  }
  manager::LocalizationInputStamp input(int64_t source_ns, uint64_t seq = 1,
      const std::string& identity = "trusted-livo") const {
    return {source_ns, runtime->node()->get_clock()->now().nanoseconds(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), seq, identity};
  }
  std::shared_ptr<nav_msgs::msg::Odometry> odom(int64_t source_ns) const {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp = rclcpp::Time(source_ns);
    msg->header.frame_id = "odom"; msg->child_frame_id = "base_footprint";
    msg->pose.pose.orientation.w = 1.0; return msg;
  }
  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> pose(int64_t source_ns,
      double x = 0.0) const {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = rclcpp::Time(source_ns); msg->header.frame_id = "map";
    msg->pose.pose.position.x = x; msg->pose.pose.orientation.w = 1.0; return msg;
  }
  std::shared_ptr<Status> status(int64_t source_ns, bool good = true) const {
    auto msg = std::make_shared<Status>();
    msg->header.stamp = rclcpp::Time(source_ns); msg->header.frame_id = "map";
    msg->backend = "livo"; msg->state = good ? Status::STATE_HEALTHY : Status::STATE_LOST;
    msg->measurement_accepted = good; msg->reason = good ? "ok" : "injected_backend_failure";
    return msg;
  }
  void healthy() {
    ASSERT_TRUE(runtime->receive_odometry("livo", odom(source), input(source)));
    ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source), input(source)));
    ASSERT_TRUE(runtime->receive_status("livo", status(source), input(source)));
  }
  void healthy_with_recorded_gap(double maximum_age = .65, bool early_receipt = false,
      double tracking_timeout = .25) {
    create(false, maximum_age, .8, tracking_timeout, true);
    source -= 340000000;  // Keep every original epoch in the past.
    // Real X-run deltas: the new NDT target falls 74.791239 ms after
    // its lower sample and 40.254785 ms before its upper sample.
    const int64_t offsets[] = {0, 4982949, 29979945, 145025969};
    for (uint64_t i = 0; i < 4; ++i)
      ASSERT_TRUE(runtime->receive_odometry("livo", odom(source+offsets[i]),
          input(source+offsets[i], i+1)));
    const auto accepted = source+4693872;
    auto acquisition = input(accepted);
    if (early_receipt) {
      const auto elapsed = acquisition.first_receipt_ros_ns-(accepted+1000000);
      acquisition.first_receipt_ros_ns -= elapsed;
      acquisition.first_receipt_steady_ns -= elapsed;
    }
    ASSERT_TRUE(runtime->receive_map_pose("livo", pose(accepted), acquisition));
    ASSERT_TRUE(runtime->receive_status("livo", status(accepted), acquisition));
    ASSERT_FALSE(states.empty());
    ASSERT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  }
  geometry_msgs::msg::TransformStamped configure_pcd() {
    geometry_msgs::msg::TransformStamped extrinsic;
    extrinsic.header.frame_id="rslidar";extrinsic.child_frame_id="base_footprint";
    extrinsic.transform.translation.x=-.096;extrinsic.transform.translation.z=-.21926;
    tf2::Quaternion rotation;rotation.setRPY(.03,-.02,.1);
    extrinsic.transform.rotation=tf2::toMsg(rotation);
    runtime->configure_pcd_registration_source(extrinsic,"map","/actual/registration.pcd","ndt#registration");
    return extrinsic;
  }
  std::shared_ptr<sunhorizon_3d_localizer::NdtDirectResult> registration(int64_t stamp,uint64_t seq=10) const {
    auto output=std::make_shared<sunhorizon_3d_localizer::NdtDirectResult>();
    const auto acquired=input(stamp,seq,"airy-owner");
    output->input={stamp,acquired.first_receipt_ros_ns,acquired.first_receipt_steady_ns,seq,"airy-owner"};
    const auto completed=input(stamp,seq,"ndt#registration");
    output->result_receipt_ros_ns=completed.first_receipt_ros_ns;
    output->result_receipt_steady_ns=completed.first_receipt_steady_ns;
    auto transform=std::make_shared<geometry_msgs::msg::TransformStamped>();
    transform->header.stamp=rclcpp::Time(stamp);transform->header.frame_id="map";transform->child_frame_id="rslidar";
    transform->transform.translation.x=1.;transform->transform.translation.y=-2.;transform->transform.translation.z=-.3;
    tf2::Quaternion rotation;rotation.setRPY(.08,-.06,.4);transform->transform.rotation=tf2::toMsg(rotation);
    output->map_to_source=transform;output->registration_map_path="/actual/registration.pcd";
    return output;
  }
  manager::LocalizationInputStamp delivery(const sunhorizon_3d_localizer::NdtDirectResult& result) const {
    return {result.input.source_stamp_ns,result.result_receipt_ros_ns,result.result_receipt_steady_ns,
      result.input.sequence,"ndt#registration"};
  }
  void expect_compact_equivalent(const std::vector<int64_t>& times,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& scan={}) const {
    const auto full=runtime->lookup_pcd_from_airy_times(times,scan);
    const auto compact=runtime->lookup_pcd_from_airy_geometry_times(times,scan);
    ASSERT_EQ(compact.state,full.state)<<full.reason<<" / "<<compact.reason;
    EXPECT_EQ(compact.reason,full.reason);
    EXPECT_EQ(compact.reset_revoked,full.reset_revoked);
    EXPECT_EQ(compact.reset_barrier_steady_ns,full.reset_barrier_steady_ns);
    ASSERT_EQ(compact.geometries.size(),full.snapshots.size());
    if(full.state!=manager::PcdGeometryState::Ready){
      EXPECT_TRUE(compact.geometries.empty());EXPECT_FALSE(compact.context);return;
    }
    ASSERT_TRUE(compact.context);ASSERT_EQ(compact.geometries.size(),times.size());
    const auto& common=*compact.context;
    for(std::size_t i=0;i<times.size();++i){
      const auto& expected=*full.snapshots[i];const auto& actual=compact.geometries[i];
      EXPECT_EQ(actual.source_ns,times[i]);
      EXPECT_EQ(rclcpp::Time(expected.pcd_from_airy.header.stamp).nanoseconds(),actual.source_ns);
      EXPECT_EQ(actual.pcd_from_airy,expected.pcd_from_airy.transform);
      EXPECT_EQ(actual.same_scan,expected.same_scan);
      EXPECT_EQ(actual.anchor_interpolated,expected.anchor_interpolated);
      EXPECT_EQ(actual.query_interpolated,expected.query_interpolated);
      EXPECT_EQ(common.anchor,expected.anchor);
      EXPECT_EQ(common.anchor->map_to_source->header.frame_id,expected.pcd_from_airy.header.frame_id);
      EXPECT_EQ(common.anchor->map_to_source->child_frame_id,expected.pcd_from_airy.child_frame_id);
      EXPECT_EQ(common.source_from_nominal_base,expected.source_from_nominal_base);
      EXPECT_EQ(common.registration_map_path,expected.registration_map_path);
      EXPECT_EQ(common.reset_epoch,expected.reset_epoch);EXPECT_EQ(common.reset_revoked,expected.reset_revoked);
      EXPECT_EQ(common.geometry_valid_until_ros_ns,expected.geometry_valid_until_ros_ns);
      EXPECT_EQ(common.geometry_valid_until_steady_ns,expected.geometry_valid_until_steady_ns);
      const auto check_receipt=[](const auto& a,const auto& b){
        EXPECT_EQ(a.source_ns,b.source_ns);EXPECT_EQ(a.first_receipt_ros_ns,b.first_receipt_ros_ns);
        EXPECT_EQ(a.first_receipt_steady_ns,b.first_receipt_steady_ns);
        EXPECT_EQ(a.sequence,b.sequence);EXPECT_EQ(a.identity,b.identity);
      };
      check_receipt(common.registration_delivery,expected.registration_delivery);
      ASSERT_EQ(bool(common.query_scan),bool(expected.query_scan));
      if(common.query_scan){
        EXPECT_EQ(common.query_scan->source_stamp_ns,expected.query_scan->source_stamp_ns);
        EXPECT_EQ(common.query_scan->receipt_ros_ns,expected.query_scan->receipt_ros_ns);
        EXPECT_EQ(common.query_scan->receipt_steady_ns,expected.query_scan->receipt_steady_ns);
        EXPECT_EQ(common.query_scan->sequence,expected.query_scan->sequence);
        EXPECT_EQ(common.query_scan->source_identity,expected.query_scan->source_identity);
      }
      if(!expected.anchor_tracking_inputs.empty()){
        ASSERT_EQ(common.anchor_tracking_inputs.size(),expected.anchor_tracking_inputs.size());
        for(std::size_t j=0;j<common.anchor_tracking_inputs.size();++j)
          check_receipt(common.anchor_tracking_inputs[j],expected.anchor_tracking_inputs[j]);
      }
      EXPECT_FALSE(expected.translation_error_bound_m);EXPECT_FALSE(expected.rotation_error_bound_rad);
    }
  }
  std::shared_ptr<manager::LocalizationDirectRuntime> runtime;
  std::vector<std::shared_ptr<const manager::CanonicalLocalizationState>> states;
  std::vector<std::shared_ptr<const manager::CanonicalLocalizationHealth>> health;
  int64_t source{};
};

TEST_F(DirectManager, CanonicalOwnerPreservesAcquisitionLineageAndSourceTf) {
  const auto tracking_receipt = input(source);
  const auto pose_receipt = input(source);
  const auto status_receipt = input(source);
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source), tracking_receipt));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source), pose_receipt));
  ASSERT_TRUE(states.empty());
  ASSERT_TRUE(runtime->receive_status("livo", status(source), status_receipt));
  ASSERT_EQ(states.size(), 1U);
  const auto first = states.front();
  EXPECT_EQ(rclcpp::Time(first->odometry->header.stamp).nanoseconds(), source);
  EXPECT_EQ(rclcpp::Time(first->map_to_odom.header.stamp).nanoseconds(), source);
  EXPECT_EQ(rclcpp::Time(first->odom_to_base_footprint.header.stamp).nanoseconds(), source);
  EXPECT_EQ(first->tracking.first_receipt_ros_ns, tracking_receipt.first_receipt_ros_ns);
  EXPECT_EQ(first->tracking.first_receipt_steady_ns, tracking_receipt.first_receipt_steady_ns);
  EXPECT_EQ(first->map_pose.first_receipt_ros_ns, pose_receipt.first_receipt_ros_ns);
  EXPECT_EQ(first->backend_status.first_receipt_ros_ns, status_receipt.first_receipt_ros_ns);
  ASSERT_EQ(first->alignment_inputs.size(), 1U);
  EXPECT_EQ(first->alignment_inputs.front().source_ns, source);
  ASSERT_FALSE(health.empty());
  EXPECT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  EXPECT_EQ(health.back()->last_state, first);

  // Later callback sequences for an already committed epoch cannot renew it.
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source), input(source, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(source), input(source, 2)));
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source+1000000), input(source+1000000, 2)));
  ASSERT_EQ(states.size(), 2U);
  EXPECT_EQ(states.back()->map_pose.first_receipt_ros_ns, pose_receipt.first_receipt_ros_ns);
  EXPECT_EQ(states.back()->backend_status.first_receipt_steady_ns, status_receipt.first_receipt_steady_ns);
  EXPECT_EQ(first->tracking.source_ns, source);  // Already delivered owner stays immutable.
}

TEST_F(DirectManager, UnalignedNewHealthyCorrectionCannotReplaceQualifiedOriginalPair) {
  healthy_with_recorded_gap();
  const auto original = states.back();
  const auto unavailable = source+104771184;
  for (uint64_t seq = 2; seq <= 3; ++seq) {
    ASSERT_TRUE(runtime->receive_map_pose("livo", pose(unavailable), input(unavailable, seq)));
    ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), input(unavailable, seq)));
    EXPECT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
    EXPECT_EQ(health.back()->last_state, original);
  }
  // The next genuinely supported epoch commits; no source/receipt renewal
  // or synthetic interpolation was needed to bridge the rejected epoch.
  const auto next = source+304704880;
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(next), input(next, 5)));
  EXPECT_EQ(states.back()->map_pose.source_ns, original->map_pose.source_ns);
  EXPECT_EQ(states.back()->map_pose.first_receipt_ros_ns, original->map_pose.first_receipt_ros_ns);
  EXPECT_EQ(states.back()->backend_status.first_receipt_steady_ns,
      original->backend_status.first_receipt_steady_ns);
  const auto acquired = input(next, 4);
  ASSERT_TRUE(runtime->receive_status("livo", status(next), acquired));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(next), acquired));
  ASSERT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  // The correction commits at the current tracking epoch, which must not be
  // published twice. Observe it on the next original tracking measurement.
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(next+1000000), input(next+1000000, 6)));
  EXPECT_EQ(states.back()->map_pose.source_ns, next);
  EXPECT_EQ(states.back()->map_pose.first_receipt_ros_ns, acquired.first_receipt_ros_ns);
  EXPECT_EQ(states.back()->alignment_inputs.size(), 1U);
  EXPECT_EQ(states.back()->alignment_inputs.front().source_ns, next);
}

TEST_F(DirectManager, MissingAlignmentCannotConcealReportedFailureOrMalformedPair) {
  for (int fault = 0; fault < 4; ++fault) {
    healthy_with_recorded_gap();
    const auto unavailable = source+104771184;
    auto candidate = pose(unavailable);
    auto reported = status(unavailable);
    if (fault == 0) reported = status(unavailable, false);
    if (fault == 1) candidate->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    if (fault == 2) reported->score = std::numeric_limits<double>::quiet_NaN();
    if (fault == 3) candidate->header.frame_id = "wrong-map";
    ASSERT_TRUE(runtime->receive_map_pose("livo", candidate, input(unavailable, 2)));
    ASSERT_TRUE(runtime->receive_status("livo", reported, input(unavailable, 2)));
    EXPECT_FALSE(health.back()->status->measurement_accepted) << fault;
  }
}

TEST_F(DirectManager, NewCorrectionCannotHideFutureAtOriginalAcquisition) {
  healthy_with_recorded_gap(.65, true);
  const auto unavailable = source+104771184;
  auto acquired = input(unavailable, 2);
  const auto elapsed = acquired.first_receipt_ros_ns-(unavailable-20000000);
  acquired.first_receipt_ros_ns -= elapsed;
  acquired.first_receipt_steady_ns -= elapsed;
  // Receipts advance relative to the accepted pair, but this measurement
  // was twenty milliseconds in their future. It must not be hidden by defer.
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(unavailable), acquired));
  ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), acquired));
  EXPECT_FALSE(health.back()->status->measurement_accepted);
}

TEST_F(DirectManager, RetainedCorrectionUsesOriginalReceiptInsteadOfSourceAge) {
  healthy_with_recorded_gap(.38);
  const auto unavailable = source+104771184;
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(unavailable), input(unavailable, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), input(unavailable, 2)));
  ASSERT_TRUE(health.back()->status->measurement_accepted);
  // A completed inactive backend pair must not clear the active livo lease.
  auto other = status(unavailable); other->backend = "visual";
  ASSERT_TRUE(runtime->receive_map_pose("visual", pose(unavailable), input(unavailable)));
  ASSERT_TRUE(runtime->receive_status("visual", other, input(unavailable)));
  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  // Crossing the old source deadline with live original receipts is allowed.
  // Keep the missing target's original history bracket.
  const auto fresh = runtime->node()->get_clock()->now().nanoseconds()-1000000;
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(fresh), input(fresh, 5)));
  EXPECT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), input(unavailable, 3)));
  EXPECT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
}

TEST_F(DirectManager, DelayedLiveTrackingKeepsOriginalSourceAndRejectsReplay) {
  source-=2000000000;
  healthy();
  ASSERT_FALSE(states.empty());
  ASSERT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  EXPECT_EQ(states.back()->tracking.source_ns,source);
  EXPECT_EQ(rclcpp::Time(states.back()->odometry->header.stamp).nanoseconds(),source);
  source+=5000000;
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(source),input(source,2)));
  ASSERT_TRUE(health.back()->status->measurement_accepted) << health.back()->status->reason;
  EXPECT_EQ(states.back()->tracking.source_ns,source);
  auto replay=input(source,2);
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source),replay));
}

TEST_F(DirectManager, QualifiedCorrectionExpiryAndTrackingTimeoutStillClose) {
  // A short original receipt lease makes this deterministic without changing
  // production limits. Repeated unaligned epochs cannot refresh that lease.
  create(false, .65, .035, .25, true);
  source -= 190000000;
  const auto lower = source;
  const auto upper = source+115046024;
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(lower), input(lower)));
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(upper), input(upper, 2)));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(lower), input(lower)));
  ASSERT_TRUE(runtime->receive_status("livo", status(lower), input(lower)));
  ASSERT_TRUE(health.back()->status->measurement_accepted);
  const auto unavailable = lower+74791239;
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(unavailable), input(unavailable, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), input(unavailable, 2)));
  ASSERT_TRUE(health.back()->status->measurement_accepted);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(runtime->receive_status("livo", status(unavailable), input(unavailable, 3)));
  EXPECT_FALSE(health.back()->status->measurement_accepted);

  healthy_with_recorded_gap(.65, false, .035);
  std::this_thread::sleep_for(std::chrono::milliseconds(65));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source+104771184), input(source+104771184, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(source+104771184), input(source+104771184, 2)));
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  EXPECT_EQ(health.back()->status->reason,
      "active_backend_rejected:localization_direct_tracking_receipt_stale");
}

TEST_F(DirectManager, SupportedCorrectionJumpAndResetKeepTheirOriginalLatches) {
  healthy_with_recorded_gap();
  const auto next = source+304704880;
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(next), input(next, 5)));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(next, 1.), input(next, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(next), input(next, 2)));
  EXPECT_TRUE(runtime->failed());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  runtime->reset();
  EXPECT_FALSE(health.back()->last_state);
  EXPECT_FALSE(health.back()->status->measurement_accepted);
}

TEST_F(DirectManager, OldFirstReceiptCannotBeMadeHealthyByFreshDelivery) {
  auto old = input(source);
  old.first_receipt_steady_ns -= 1000000000LL;
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source), old));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source), input(source)));
  ASSERT_TRUE(runtime->receive_status("livo", status(source), input(source)));
  EXPECT_TRUE(states.empty());
  ASSERT_FALSE(health.empty());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  EXPECT_NE(health.back()->status->reason.find("tracking_receipt_stale"), std::string::npos);
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source), input(source, 2)));
  EXPECT_TRUE(states.empty());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
}

TEST_F(DirectManager, StatusFailureWithdrawsHealthAndCorrectionJumpLatches) {
  healthy(); ASSERT_EQ(states.size(), 1U);
  ASSERT_TRUE(runtime->receive_status("livo", status(source, false), input(source, 2)));
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  EXPECT_EQ(health.back()->last_state, states.front());
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source+1000000), input(source+1000000, 2)));
  ASSERT_TRUE(runtime->receive_map_pose("livo", pose(source+1000000, 1.0), input(source+1000000, 2)));
  ASSERT_TRUE(runtime->receive_status("livo", status(source+1000000), input(source+1000000, 3)));
  EXPECT_EQ(states.size(), 1U);
  EXPECT_TRUE(runtime->failed());
  EXPECT_NE(health.back()->status->reason.find("explicit_reinitialization_required"), std::string::npos);
}

TEST_F(DirectManager, HistoryRegressionAndOwnerChangesRequireExplicitReset) {
  healthy(); ASSERT_EQ(states.size(), 1U);
  ASSERT_TRUE(runtime->receive_odometry("livo", odom(source-1000000), input(source-1000000, 2)));
  EXPECT_TRUE(runtime->failed());
  runtime->reset(); EXPECT_FALSE(runtime->failed());
  EXPECT_EQ(health.back()->reset_epoch, 1U);
  EXPECT_FALSE(health.back()->last_state);
  EXPECT_FALSE(runtime->receive_odometry("livo", odom(source+1000000),
      input(source+1000000, 3, "replacement-owner")));
  EXPECT_TRUE(runtime->failed());  // Reset never authorizes a different producer.
}

TEST_F(DirectManager, LivoNdtRequiresExactAlignmentAndCarriesItsReceipt) {
  create(true); healthy(); EXPECT_TRUE(states.empty());
  const auto alignment = input(source);
  ASSERT_TRUE(runtime->receive_livo_ndt_alignment(odom(source), alignment));
  ASSERT_EQ(states.size(), 1U);
  ASSERT_EQ(states.front()->alignment_inputs.size(), 1U);
  EXPECT_EQ(states.front()->alignment_inputs.front().first_receipt_steady_ns,
            alignment.first_receipt_steady_ns);
  EXPECT_EQ(states.front()->alignment_inputs.front().source_ns, source);
}

TEST_F(DirectManager, FatalSourceRevocationImmediatelyWithdrawsHealthAndSurvivesReset) {
  healthy(); ASSERT_EQ(states.size(),1U);
  runtime->revoke("airy_sensor_owner_failed");
  ASSERT_TRUE(runtime->failed());
  ASSERT_FALSE(health.empty());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  EXPECT_NE(health.back()->status->reason.find("airy_sensor_owner_failed"),std::string::npos);
  EXPECT_FALSE(health.back()->last_state);  // Reentrant revoke never reads the gate-protected geometry.
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source+1000000),input(source+1000000,2)));
  runtime->reset();
  EXPECT_TRUE(runtime->failed());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source+2000000),input(source+2000000,3)));
  EXPECT_EQ(states.size(),1U);
}

TEST_F(DirectManager, SinkCanRevokeWithoutRecursivelyAcquiringGateMutex) {
  runtime.reset();states.clear();health.clear();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("active_backend","livo")});
  manager::LocalizationDirectSinks sinks;sinks.publish_ros_compatibility=false;
  sinks.canonical=[this](auto){runtime->revoke("downstream_queue_overflow");};
  sinks.status=[this](auto report){const bool fatal=report->status->reason.find("revoked")!=std::string::npos;
    health.push_back(std::move(report));if(fatal)runtime->revoke("nested_fault");};
  runtime=manager::make_localization_direct_runtime(options,std::move(sinks));
  healthy();
  EXPECT_TRUE(runtime->failed());ASSERT_FALSE(health.empty());
  EXPECT_FALSE(health.back()->status->measurement_accepted);
}

TEST_F(DirectManager, RegistrationBatchUsesOneOriginalAnchorAndFullLivoHistory) {
  const auto extrinsic=configure_pcd();
  source-=90000000;const auto anchor_time=source+10000000;
  auto lower=odom(source),upper=odom(source+20000000);
  lower->pose.pose.position.z=.7;upper->pose.pose.position.x=.02;upper->pose.pose.position.z=.8;
  tf2::Quaternion lo,hi;lo.setRPY(.1,-.2,.3);hi.setRPY(.12,-.18,.32);
  lower->pose.pose.orientation=tf2::toMsg(lo);upper->pose.pose.orientation=tf2::toMsg(hi);
  const auto lower_receipt=input(source),upper_receipt=input(source+20000000,2);
  ASSERT_TRUE(runtime->receive_odometry("livo",lower,lower_receipt));
  ASSERT_TRUE(runtime->receive_odometry("livo",upper,upper_receipt));
  const auto original=registration(anchor_time);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));
  EXPECT_EQ(runtime->lookup_pcd_from_airy(anchor_time,original->input).state,manager::PcdGeometryState::WaitingForManager);
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time),input(anchor_time)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time),input(anchor_time)));
  ASSERT_FALSE(states.empty());
  EXPECT_DOUBLE_EQ(states.back()->odometry->pose.pose.position.z,0.);
  EXPECT_DOUBLE_EQ(states.back()->odometry->pose.pose.orientation.x,0.);
  EXPECT_DOUBLE_EQ(states.back()->odometry->pose.pose.orientation.y,0.);
  const auto exact=runtime->lookup_pcd_from_airy(anchor_time,original->input);
  ASSERT_EQ(exact.state,manager::PcdGeometryState::Ready)<<exact.reason;
  ASSERT_TRUE(exact.snapshot);EXPECT_TRUE(exact.snapshot->same_scan);
  EXPECT_DOUBLE_EQ(exact.snapshot->pcd_from_airy.transform.translation.z,-.3);
  EXPECT_DOUBLE_EQ(exact.snapshot->pcd_from_airy.transform.rotation.x,original->map_to_source->transform.rotation.x);
  EXPECT_EQ(exact.snapshot->anchor,original);
  EXPECT_GT(exact.snapshot->geometry_valid_until_ros_ns,original->input.source_stamp_ns);
  EXPECT_GT(exact.snapshot->geometry_valid_until_steady_ns,original->input.receipt_steady_ns);
  EXPECT_FALSE(exact.snapshot->translation_error_bound_m);
  EXPECT_FALSE(exact.snapshot->rotation_error_bound_rad);
  expect_compact_equivalent({anchor_time},original->input); // Exact source uses original Q unchanged.
  expect_compact_equivalent({anchor_time+5000000,anchor_time,anchor_time+5000000},original->input);

  // A pending later scan cannot replace accepted evidence. Once accepted, one
  // latest anchor is frozen across the whole batch, including earlier queries.
  const auto later=registration(anchor_time+8000000,11);
  ASSERT_TRUE(runtime->receive_registration(later,delivery(*later)));
  const auto pending=runtime->lookup_pcd_from_airy(anchor_time,original->input);
  ASSERT_EQ(pending.state,manager::PcdGeometryState::Ready)<<pending.reason;
  EXPECT_EQ(pending.snapshot->anchor,original);
  EXPECT_EQ(runtime->lookup_pcd_from_airy(anchor_time+8000000,later->input).state,
    manager::PcdGeometryState::WaitingForManager);
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time+8000000),input(anchor_time+8000000,2)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time+8000000),input(anchor_time+8000000,2)));
  const auto batch=runtime->lookup_pcd_from_airy_times({anchor_time,anchor_time+5000000,anchor_time+10000000},original->input);
  ASSERT_EQ(batch.state,manager::PcdGeometryState::Ready)<<batch.reason;
  ASSERT_EQ(batch.snapshots.size(),3U);
  for(const auto& snapshot:batch.snapshots){
    EXPECT_EQ(snapshot->anchor,later);EXPECT_EQ(snapshot->reset_revoked,exact.snapshot->reset_revoked);
    EXPECT_EQ(snapshot->query_scan->source_stamp_ns,anchor_time);
    EXPECT_EQ(snapshot->query_scan->receipt_steady_ns,original->input.receipt_steady_ns);
    EXPECT_EQ(snapshot->anchor->input.receipt_steady_ns,later->input.receipt_steady_ns);
    EXPECT_EQ(snapshot->registration_delivery.first_receipt_steady_ns,later->result_receipt_steady_ns);
    EXPECT_FALSE(snapshot->same_scan);
    EXPECT_LE(snapshot->geometry_valid_until_ros_ns,anchor_time+500000000);
  }
  const auto& transported=*batch.snapshots[1];
  EXPECT_TRUE(transported.anchor_interpolated);EXPECT_TRUE(transported.query_interpolated);
  ASSERT_EQ(transported.anchor_tracking_inputs.size(),2U);
  EXPECT_EQ(transported.anchor_tracking_inputs.front().first_receipt_steady_ns,lower_receipt.first_receipt_steady_ns);
  EXPECT_EQ(transported.query_tracking_inputs.back().first_receipt_steady_ns,upper_receipt.first_receipt_steady_ns);
  manager::OdometryHistory history;ASSERT_TRUE(history.insert(*lower).accepted);ASSERT_TRUE(history.insert(*upper).accepted);
  const auto expected=manager::transport_pcd_from_airy(later->map_to_source->transform,
    history.lookup(anchor_time+8000000).odometry.pose.pose,history.lookup(anchor_time+5000000).odometry.pose.pose,extrinsic.transform);
  EXPECT_NEAR(transported.pcd_from_airy.transform.translation.z,expected.translation.z,1.e-12);
  EXPECT_NEAR(transported.pcd_from_airy.transform.rotation.x,expected.rotation.x,1.e-12);
  EXPECT_EQ(rclcpp::Time(transported.pcd_from_airy.header.stamp).nanoseconds(),anchor_time+5000000);
  const auto missing=runtime->lookup_pcd_from_airy_times({anchor_time,source+21000000},original->input);
  EXPECT_EQ(missing.state,manager::PcdGeometryState::WaitingForHistory);EXPECT_TRUE(missing.snapshots.empty());
  EXPECT_EQ(missing.reset_revoked,batch.reset_revoked);EXPECT_EQ(batch.reset_revoked,exact.snapshot->reset_revoked);
  EXPECT_EQ(missing.reset_barrier_steady_ns,batch.reset_barrier_steady_ns);
  expect_compact_equivalent({anchor_time,source+21000000,anchor_time+5000000},original->input);
  auto changed=original->input;++changed.sequence;
  EXPECT_EQ(runtime->lookup_pcd_from_airy_times({anchor_time+5000000},changed).state,manager::PcdGeometryState::IdentityMismatch);
  changed=original->input;++changed.receipt_steady_ns;
  EXPECT_EQ(runtime->lookup_pcd_from_airy(anchor_time,changed).state,manager::PcdGeometryState::IdentityMismatch);
  const auto compact_before_reset=runtime->lookup_pcd_from_airy_geometry_times({anchor_time},original->input);
  ASSERT_EQ(compact_before_reset.state,manager::PcdGeometryState::Ready);
  runtime->reset();EXPECT_TRUE(exact.snapshot->reset_revoked->load());
  EXPECT_TRUE(compact_before_reset.context->reset_revoked->load());
  expect_compact_equivalent({anchor_time},original->input);
  ASSERT_TRUE(missing.reset_revoked);EXPECT_TRUE(missing.reset_revoked->load());
  EXPECT_FALSE(runtime->receive_registration(original,delivery(*original)));
  EXPECT_TRUE(runtime->lookup_pcd_from_airy(anchor_time,original->input).snapshot==nullptr);
}

TEST_F(DirectManager, RegistrationMapMutationPermanentlyRevokesIssuedGeometry) {
  configure_pcd();const auto original=registration(source);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));healthy();
  const auto ready=runtime->lookup_pcd_from_airy(source,original->input);
  ASSERT_EQ(ready.state,manager::PcdGeometryState::Ready)<<ready.reason;
  auto wrong_map=registration(source+1000000,11);wrong_map->registration_map_path="/different/map.pcd";
  EXPECT_FALSE(runtime->receive_registration(wrong_map,delivery(*wrong_map)));
  EXPECT_TRUE(ready.snapshot->reset_revoked->load());
  EXPECT_EQ(runtime->lookup_pcd_from_airy(source,original->input).state,manager::PcdGeometryState::Fault);
  expect_compact_equivalent({source},original->input);
  runtime->reset();EXPECT_TRUE(ready.snapshot->reset_revoked->load());
  source=runtime->node()->get_clock()->now().nanoseconds()-1000000;
  const auto fresh=registration(source,12);
  ASSERT_TRUE(runtime->receive_registration(fresh,delivery(*fresh)));
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(source),input(source,2)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(source),input(source,2)));
  ASSERT_TRUE(runtime->receive_status("livo",status(source),input(source,2)));
  const auto renewed=runtime->lookup_pcd_from_airy(source,fresh->input);
  ASSERT_EQ(renewed.state,manager::PcdGeometryState::Ready)<<renewed.reason;
  EXPECT_EQ(renewed.snapshot->reset_epoch,1U);
  EXPECT_NE(renewed.snapshot->reset_revoked,ready.snapshot->reset_revoked);
  EXPECT_FALSE(renewed.snapshot->reset_revoked->load());
  const auto renewed_batch=runtime->lookup_pcd_from_airy_times({source},fresh->input);
  ASSERT_EQ(renewed_batch.state,manager::PcdGeometryState::Ready)<<renewed_batch.reason;
  EXPECT_EQ(renewed_batch.reset_revoked,renewed.snapshot->reset_revoked);
  EXPECT_GT(renewed_batch.reset_barrier_steady_ns,ready.snapshot->anchor->input.receipt_steady_ns);
  EXPECT_LT(renewed_batch.reset_barrier_steady_ns,fresh->input.receipt_steady_ns);
  runtime->revoke("source_owner_failed");
  EXPECT_TRUE(renewed.snapshot->reset_revoked->load());
}

TEST_F(DirectManager, HistoricalRegistrationTransportUsesLiveTrackingDeadline) {
  configure_pcd();
  const auto anchor_time=source-380000000;
  const auto lower_time=anchor_time-10000000,upper_time=anchor_time+10000000;
  auto lower_receipt=input(lower_time,1);auto upper_receipt=input(upper_time,2);
  lower_receipt.first_receipt_ros_ns-=380000000;
  lower_receipt.first_receipt_steady_ns-=380000000;
  upper_receipt.first_receipt_ros_ns-=360000000;
  upper_receipt.first_receipt_steady_ns-=360000000;
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(lower_time),lower_receipt));
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(upper_time),upper_receipt));
  const auto live_receipt=input(source,3);
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(source),live_receipt));
  const auto original=registration(anchor_time);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time),input(anchor_time)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time),input(anchor_time)));
  const auto transported=runtime->lookup_pcd_from_airy_times({anchor_time+5000000},original->input);
  ASSERT_EQ(transported.state,manager::PcdGeometryState::Ready)<<transported.reason;
  ASSERT_EQ(transported.snapshots.size(),1U);
  const auto snapshot=transported.snapshots.front();
  EXPECT_EQ(snapshot->anchor_tracking_inputs.front().first_receipt_steady_ns,lower_receipt.first_receipt_steady_ns);
  // The old interpolation receipt is retained, but does not expire again as
  // though it were the latest stream heartbeat. Neither anchor nor live input
  // acquires a new receipt or an extended deadline.
  EXPECT_GT(snapshot->geometry_valid_until_steady_ns,upper_receipt.first_receipt_steady_ns+250000000);
  EXPECT_LE(snapshot->geometry_valid_until_steady_ns,live_receipt.first_receipt_steady_ns+250000000);
  EXPECT_LE(snapshot->geometry_valid_until_ros_ns,anchor_time+500000000);
  expect_compact_equivalent({anchor_time+5000000,anchor_time,anchor_time+5000000},original->input);
  const auto gap=runtime->lookup_pcd_from_airy_times({anchor_time+150000000},original->input);
  EXPECT_EQ(gap.state,manager::PcdGeometryState::HistoryExpired);
  EXPECT_EQ(gap.reason,"odometry_history_gap_exceeds_tolerance");
  expect_compact_equivalent({anchor_time+150000000,source+1000000},original->input);
  expect_compact_equivalent({source+1000000,anchor_time+150000000},original->input);
  EXPECT_EQ(runtime->lookup_pcd_from_airy_geometry_times(
    {anchor_time+150000000,source+1000000},original->input).state,manager::PcdGeometryState::HistoryExpired);
  EXPECT_EQ(runtime->lookup_pcd_from_airy_geometry_times(
    {source+1000000,anchor_time+150000000},original->input).state,manager::PcdGeometryState::WaitingForHistory);
  runtime->reset();EXPECT_TRUE(snapshot->reset_revoked->load());
  EXPECT_NE(runtime->lookup_pcd_from_airy_times({anchor_time+5000000},original->input).state,
    manager::PcdGeometryState::Ready);
}

TEST_F(DirectManager, LatestAcceptedAnchorTransportsBackwardWithoutRenewingOldQuery) {
  create(false,.65,.80);const auto extrinsic=configure_pcd();
  const auto query_time=source-180000000,anchor_time=source;
  auto query_lower=odom(query_time-5000000),query_upper=odom(query_time+5000000);
  auto anchor_lower=odom(anchor_time-5000000),anchor_upper=odom(anchor_time+5000000);
  query_lower->pose.pose.position.z=.70;query_upper->pose.pose.position.z=.72;
  anchor_lower->pose.pose.position.z=.80;anchor_upper->pose.pose.position.z=.82;
  tf2::Quaternion before,after;before.setRPY(.10,-.20,.30);after.setRPY(.12,-.18,.32);
  query_lower->pose.pose.orientation=query_upper->pose.pose.orientation=tf2::toMsg(before);
  anchor_lower->pose.pose.orientation=anchor_upper->pose.pose.orientation=tf2::toMsg(after);
  uint64_t sequence=0;
  for(const auto& sample:{query_lower,query_upper,anchor_lower,anchor_upper})
    ASSERT_TRUE(runtime->receive_odometry("livo",sample,input(rclcpp::Time(sample->header.stamp).nanoseconds(),++sequence)));
  const auto original=registration(query_time,9);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(query_time),input(query_time,1)));
  ASSERT_TRUE(runtime->receive_status("livo",status(query_time),input(query_time,1)));
  const auto pending=registration(query_time+50000000,10);
  ASSERT_TRUE(runtime->receive_registration(pending,delivery(*pending)));
  const auto latest=registration(anchor_time,11);
  ASSERT_TRUE(runtime->receive_registration(latest,delivery(*latest)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time),input(anchor_time,2)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time),input(anchor_time,2)));
  const auto batch=runtime->lookup_pcd_from_airy_times({query_time,query_time+1000000},original->input);
  ASSERT_EQ(batch.state,manager::PcdGeometryState::Ready)<<batch.reason;
  ASSERT_EQ(batch.snapshots.size(),2U);
  manager::OdometryHistory history;
  for(const auto& sample:{query_lower,query_upper,anchor_lower,anchor_upper})ASSERT_TRUE(history.insert(*sample).accepted);
  for(std::size_t i=0;i<batch.snapshots.size();++i) {
    const auto& actual=*batch.snapshots[i];const auto stamp=query_time+int64_t(i)*1000000;
    EXPECT_EQ(actual.anchor,latest);EXPECT_FALSE(actual.same_scan);
    ASSERT_TRUE(actual.query_scan);
    EXPECT_EQ(actual.query_scan->source_stamp_ns,original->input.source_stamp_ns);
    EXPECT_EQ(actual.query_scan->receipt_ros_ns,original->input.receipt_ros_ns);
    EXPECT_EQ(actual.query_scan->receipt_steady_ns,original->input.receipt_steady_ns);
    EXPECT_EQ(actual.registration_delivery.first_receipt_ros_ns,latest->result_receipt_ros_ns);
    EXPECT_LE(actual.geometry_valid_until_ros_ns,query_time+650000000);
    EXPECT_LE(actual.geometry_valid_until_ros_ns,latest->input.source_stamp_ns+650000000);
    EXPECT_LE(actual.geometry_valid_until_steady_ns,latest->input.receipt_steady_ns+800000000);
    const auto expected=manager::transport_pcd_from_airy(latest->map_to_source->transform,
      history.lookup(anchor_time).odometry.pose.pose,history.lookup(stamp).odometry.pose.pose,extrinsic.transform);
    EXPECT_NEAR(actual.pcd_from_airy.transform.translation.z,expected.translation.z,1.e-12);
    EXPECT_NEAR(actual.pcd_from_airy.transform.rotation.x,expected.rotation.x,1.e-12);
    EXPECT_EQ(rclcpp::Time(actual.pcd_from_airy.header.stamp).nanoseconds(),stamp);
  }
  // A fresh latest anchor never approves a known unaccepted exact scan.
  EXPECT_EQ(runtime->lookup_pcd_from_airy(query_time+50000000,pending->input).state,
    manager::PcdGeometryState::WaitingForManager);
  const auto old=runtime->lookup_pcd_from_airy(source-650000000,std::nullopt);
  EXPECT_EQ(old.state,manager::PcdGeometryState::AnchorExpired);
  EXPECT_EQ(old.reason,"pcd_query_original_measurement_expired");
  expect_compact_equivalent({source-650000000},std::nullopt);
  const auto gap=runtime->lookup_pcd_from_airy(query_time+90000000,std::nullopt);
  EXPECT_EQ(gap.state,manager::PcdGeometryState::HistoryExpired);
  EXPECT_EQ(gap.reason,"odometry_history_gap_exceeds_tolerance");
  runtime->reset();EXPECT_TRUE(batch.snapshots.front()->reset_revoked->load());
}

TEST_F(DirectManager, LatestAnchorCannotRenewExpiredOriginalAcquisitionReceipt) {
  create(false,.65,.80);configure_pcd();
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(source),input(source)));
  auto latest=registration(source);
  latest->input.receipt_steady_ns-=810000000; // Original acquisition is already expired.
  ASSERT_TRUE(runtime->receive_registration(latest,delivery(*latest)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(source),input(source)));
  ASSERT_TRUE(runtime->receive_status("livo",status(source),input(source)));
  const auto lookup=runtime->lookup_pcd_from_airy(source,latest->input);
  EXPECT_EQ(lookup.state,manager::PcdGeometryState::AnchorExpired);
  EXPECT_EQ(lookup.reason,"pcd_registration_original_evidence_expired");
  expect_compact_equivalent({source},latest->input);
  EXPECT_FALSE(lookup.snapshot);
}

TEST_F(DirectManager, HistoricalRegistrationDoesNotHideStaleLiveTracking) {
  configure_pcd();
  const auto anchor_time=source-10000000;
  auto lower_receipt=input(anchor_time-10000000,1);
  auto upper_receipt=input(anchor_time+10000000,2);
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(anchor_time-10000000),lower_receipt));
  ASSERT_TRUE(runtime->receive_odometry("livo",odom(anchor_time+10000000),upper_receipt));
  const auto original=registration(anchor_time);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time),input(anchor_time)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time),input(anchor_time)));
  const auto accepted=runtime->lookup_pcd_from_airy(anchor_time,original->input);
  ASSERT_EQ(accepted.state,manager::PcdGeometryState::Ready)<<accepted.reason;
  ASSERT_TRUE(accepted.snapshot);
  const auto initially_live=runtime->lookup_pcd_from_airy_times({anchor_time+5000000},original->input);
  ASSERT_EQ(initially_live.state,manager::PcdGeometryState::Ready)<<initially_live.reason;
  // Expire the real latest tracking receipt after the anchor has committed.
  // Its source-age and correction leases remain longer than this 260ms wait.
  const auto stale_at=upper_receipt.first_receipt_steady_ns+260000000;
  ASSERT_GT(accepted.snapshot->geometry_valid_until_steady_ns,stale_at);
  std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(stale_at)));
  const auto stale=runtime->lookup_pcd_from_airy_times({anchor_time+5000000},original->input);
  EXPECT_NE(stale.state,manager::PcdGeometryState::Ready);
  EXPECT_EQ(stale.reason,"localization_direct_tracking_receipt_stale");
  EXPECT_TRUE(stale.snapshots.empty());
  expect_compact_equivalent({anchor_time+5000000},original->input);
}

TEST_F(DirectManager, CompactRegistrationBatchRetainsEveryOriginalNanosecondQuery) {
  // This is an equivalence/allocation-shape test, not a wall-time benchmark.
  // Leave deterministic expiry coverage to the original short-deadline cases.
  create(false,10.,10.,10.);configure_pcd();source-=90000000;
  const auto anchor_time=source;
  auto lower=odom(source),upper=odom(source+20000000);
  lower->pose.pose.position.z=.7;upper->pose.pose.position.x=.02;upper->pose.pose.position.z=.8;
  tf2::Quaternion lo,hi;lo.setRPY(.1,-.2,.3);hi.setRPY(.12,-.18,.32);
  lower->pose.pose.orientation=tf2::toMsg(lo);upper->pose.pose.orientation=tf2::toMsg(hi);
  ASSERT_TRUE(runtime->receive_odometry("livo",lower,input(source)));
  ASSERT_TRUE(runtime->receive_odometry("livo",upper,input(source+20000000,2)));
  const auto original=registration(anchor_time);
  ASSERT_TRUE(runtime->receive_registration(original,delivery(*original)));
  ASSERT_TRUE(runtime->receive_map_pose("livo",pose(anchor_time),input(anchor_time)));
  ASSERT_TRUE(runtime->receive_status("livo",status(anchor_time),input(anchor_time)));
  std::vector<int64_t> times;times.reserve(10586);
  for(int64_t i=0;i<10586;++i)times.push_back(anchor_time+i*1001); // No timestamp binning.
  expect_compact_equivalent(times,original->input);
  const auto compact=runtime->lookup_pcd_from_airy_geometry_times(times,original->input);
  ASSERT_EQ(compact.state,manager::PcdGeometryState::Ready)<<compact.reason;
  ASSERT_EQ(compact.geometries.size(),times.size());ASSERT_TRUE(compact.context);
  EXPECT_TRUE(compact.geometries.front().same_scan);
  EXPECT_NE(compact.geometries[1].pcd_from_airy,compact.geometries[2].pcd_from_airy);
  // A bad interior group must discard the whole batch, even with valid ends.
  times[5000]=source+21000000;
  expect_compact_equivalent(times,original->input);
  const auto missing=runtime->lookup_pcd_from_airy_geometry_times(times,original->input);
  EXPECT_EQ(missing.state,manager::PcdGeometryState::WaitingForHistory);
  EXPECT_TRUE(missing.geometries.empty());EXPECT_FALSE(missing.context);
}

TEST_F(DirectManager, SourceMismatchFailsBeforeMutatingCanonicalState) {
  healthy(); const auto count = states.size();
  EXPECT_FALSE(runtime->receive_odometry("livo", odom(source), input(source+1, 2)));
  EXPECT_TRUE(runtime->failed()); EXPECT_EQ(states.size(), count);
  EXPECT_FALSE(health.back()->status->measurement_accepted);
}

TEST_F(DirectManager, BodyVelocityEvidenceAndTiltResidualKeepOriginalSourceAndResetBarrier) {
  create(false,.50,.50,.25,true);
  auto original=odom(source);original->pose.pose.position.z=-.215;
  tf2::Quaternion body;body.setRPY(.05,-.04,.2);original->pose.pose.orientation=tf2::toMsg(body);
  original->twist.twist.linear.x=.23;original->twist.twist.linear.y=-.04;original->twist.twist.angular.z=.12;
  auto metadata=input(source);
  sunhorizon_navigation_core::BodyVelocityEvidence evidence;
  evidence.valid=true;evidence.measured_waist=true;evidence.sample_count=6;evidence.reset_epoch=3;
  evidence.window_begin_ns=source-500000000;evidence.window_end_ns=source;evidence.effective_source_ns=source-250000000;
  evidence.variance_vx=.0005;evidence.variance_vy=.0006;evidence.variance_wz=.0007;
  evidence.odom_z_origin_source_ns=source-500000000;evidence.odom_z_origin_epoch=1;evidence.odom_z_origin_m=-.214;
  metadata.body_velocity=evidence;
  ASSERT_TRUE(runtime->receive_odometry("livo",original,metadata));
  auto mapped=pose(source);mapped->pose.pose=original->pose.pose;
  mapped->pose.pose.position.z=.6;
  ASSERT_TRUE(runtime->receive_map_pose("livo",mapped,input(source)));
  ASSERT_TRUE(runtime->receive_status("livo",status(source),input(source)));
  ASSERT_EQ(states.size(),1U);const auto& state=*states.back();
  ASSERT_TRUE(state.tracking.body_velocity);EXPECT_EQ(state.tracking.first_receipt_steady_ns,metadata.first_receipt_steady_ns);
  EXPECT_EQ(state.tracking.body_velocity->effective_source_ns,source-250000000);
  EXPECT_EQ(state.tracking.body_velocity->reset_epoch,3U);
  EXPECT_DOUBLE_EQ(state.odometry->twist.twist.linear.x,.23);
  EXPECT_DOUBLE_EQ(state.odometry->twist.twist.linear.y,-.04);
  EXPECT_DOUBLE_EQ(state.odometry->twist.twist.angular.z,.12);
  ASSERT_TRUE(state.base_footprint_to_base_link);
  tf2::Transform level,residual,full;tf2::fromMsg(state.odom_to_base_footprint.transform,level);
  tf2::fromMsg(state.base_footprint_to_base_link->transform,residual);tf2::fromMsg(original->pose.pose,full);
  EXPECT_LT(((level*residual).getOrigin()-full.getOrigin()).length(),1e-12);
  EXPECT_LT((level*residual).getRotation().angleShortestPath(full.getRotation()),1e-7);
  // This virtual level frame follows the body origin's z; it is not the floor.
  EXPECT_DOUBLE_EQ(state.odometry->pose.pose.position.z,-.215);
  EXPECT_LT(residual.getOrigin().length(),1e-12);
  runtime->reset();EXPECT_GT(reset_barrier,metadata.first_receipt_steady_ns);
  EXPECT_FALSE(runtime->receive_odometry("livo",original,metadata));
}
TEST_F(DirectManager, InvalidOrMismatchedBodyVelocityCannotBeAdmittedAsStationary) {
  auto metadata=input(source);sunhorizon_navigation_core::BodyVelocityEvidence evidence;
  evidence.window_begin_ns=source-400000000;evidence.window_end_ns=source;evidence.effective_source_ns=source-200000000;
  evidence.sample_count=5;evidence.reset_epoch=1;evidence.valid=false;metadata.body_velocity=evidence;
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source),metadata));EXPECT_TRUE(states.empty());
  metadata.body_velocity->valid=true;metadata.body_velocity->window_end_ns=source-1;
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source),metadata));EXPECT_TRUE(states.empty());
  metadata.body_velocity->window_end_ns=source;metadata.body_velocity->variance_vx=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(runtime->receive_odometry("livo",odom(source),metadata));EXPECT_TRUE(states.empty());
}
}  // namespace
