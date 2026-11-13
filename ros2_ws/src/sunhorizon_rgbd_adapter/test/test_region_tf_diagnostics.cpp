#include <gtest/gtest.h>
#include "sunhorizon_rgbd_adapter/region_tf_diagnostics.hpp"
using sunhorizon_rgbd_adapter::RegionTfDiagnostics;
TEST(RegionTfDiagnostics, ClassifiesActualExceptionTypesWithoutParsingText) {
  RegionTfDiagnostics d;
  const auto check = [&](const tf2::TransformException& e, const char* expected) {
    d = {}; d.failure(e, 200, 207);
    EXPECT_EQ(d.error_type, expected); EXPECT_EQ(d.error_detail, "same payload");
    EXPECT_EQ(d.lookup_wall_ns, 7);
  };
  check(tf2::LookupException("same payload"), "lookup");
  check(tf2::ConnectivityException("same payload"), "connectivity");
  check(tf2::ExtrapolationException("same payload"), "extrapolation");
  check(tf2::InvalidArgumentException("same payload"), "invalid_argument");
  check(tf2::TimeoutException("same payload"), "timeout");
  check(tf2::TransformException("same payload"), "transform");
}
TEST(RegionTfDiagnostics, BoundsAndSanitizesWithoutRetainingPreviousFrame) {
  RegionTfDiagnostics d;
  d.failure(tf2::LookupException(std::string(100000, 'a')), 1, 2);
  EXPECT_EQ(d.error_detail, std::string(512, 'a')); EXPECT_TRUE(d.error_truncated);
  d = {}; d.failure(tf2::LookupException("a\nb\t\x80"), 3, 4);
  EXPECT_EQ(d.error_detail, "a?b??"); EXPECT_FALSE(d.error_truncated);
  d = {}; d.failure(tf2::LookupException(std::string(512, 'b')), 5, 6);
  EXPECT_EQ(d.error_detail.size(), 512u); EXPECT_FALSE(d.error_truncated);
  d = {}; d.finish(7, 8);
  EXPECT_EQ(d.error_type, "none"); EXPECT_TRUE(d.error_detail.empty());
  EXPECT_FALSE(d.error_truncated); EXPECT_EQ(d.lookup_wall_ns, 1);
  d = {}; EXPECT_EQ(d.lookup_wall_ns, 0); EXPECT_TRUE(d.error_detail.empty());
}
TEST(RegionTfDiagnostics, InvalidClockOrderIsOnlyDiagnostic) {
  RegionTfDiagnostics d; d.finish(10, 9);
  EXPECT_EQ(d.lookup_wall_ns, -1); EXPECT_EQ(d.error_type, "none");
  d.failure(tf2::ExtrapolationException("future"), 20, 19);
  EXPECT_EQ(d.lookup_wall_ns, -1); EXPECT_EQ(d.error_type, "extrapolation");
}

#include <limits>
#include <vector>
#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"
TEST(RegionTfDiagnostics, BoundedWaitContractPreservesZeroAndRejectsInvalidTimeouts) {
  using namespace sunhorizon_rgbd_adapter;
  RegionProducerOptions o;
  o.region_id = "r"; o.base_frame = "base"; o.optical_frame = "camera";
  o.depth_topic = "/image"; o.info_topic = "/info"; o.diagnostics_topic = "/region";
  o.expected_calibration = "calibration"; o.box_min = {0., 0., 0.}; o.box_max = {1., 1., 1.};
  const auto original = region_producer_contract(o);
  o.tf_wait_sec = -0.; EXPECT_EQ(region_producer_contract(o), original);
  o.tf_wait_sec = .02;
  EXPECT_EQ(region_producer_contract(o), original + "24:bounded-exact-tf-wait-v1" +
      std::string("23:2.00000000000000004e-02"));
  for (double invalid : {-0.001, .020000001, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    o.tf_wait_sec = invalid;
    EXPECT_THROW(region_producer_contract(o), std::invalid_argument);
  }
}

TEST(RegionTfDiagnostics, ShadowContractIsExplicitAndRejectsEveryAcceptedFlag) {
  using namespace sunhorizon_rgbd_adapter;
  RegionProducerOptions o;
  o.region_id = "r"; o.base_frame = "base"; o.optical_frame = "camera";
  o.depth_topic = "/image"; o.info_topic = "/info"; o.diagnostics_topic = "/region";
  o.expected_calibration = "calibration"; o.box_min = {0., 0., 0.}; o.box_max = {1., 1., 1.};
  const auto original = region_producer_contract(o);
  o.shadow_only_enabled = true;
  EXPECT_EQ(region_producer_contract(o), original + "30:shadow-only-ray-diagnostics-v1");
  o.shadow_only_enabled = false; o.tf_wait_sec = .02;
  const auto wait_only = region_producer_contract(o);
  o.shadow_only_enabled = true;
  EXPECT_EQ(region_producer_contract(o), wait_only + "30:shadow-only-ray-diagnostics-v1");
  for (bool* flag : {&o.accepted.rectified_pinhole_verified, &o.accepted.metric_axial_depth_verified,
      &o.accepted.depth_intrinsics_registration_verified, &o.accepted.optical_to_base_extrinsics_verified,
      &o.accepted.source_time_and_transform_verified}) {
    *flag = true; EXPECT_THROW(region_producer_contract(o), std::invalid_argument); *flag = false;
  }
}


TEST(RegionTfDiagnostics, TypedEvidenceContractIsExplicitAndLastWithDefaultsUnchanged) {
  using namespace sunhorizon_rgbd_adapter;
  RegionProducerOptions o;
  o.region_id = "r"; o.base_frame = "base"; o.optical_frame = "camera";
  o.depth_topic = "/image"; o.info_topic = "/info"; o.diagnostics_topic = "/region";
  o.expected_calibration = "calibration"; o.box_min = {0., 0., 0.}; o.box_max = {1., 1., 1.};
  EXPECT_FALSE(o.typed_evaluation_diagnostics_enabled);
  for (bool extended : {false, true}) {
    o.tf_wait_sec = extended ? .02 : 0.;
    o.shadow_only_enabled = extended; o.hold_complete_measurement_enabled = extended;
    const auto original = region_producer_contract(o);
    o.typed_evaluation_diagnostics_enabled = true;
    RegionCanonical suffix; suffix.add("typed-region-evaluation-diagnostics-v1");
    EXPECT_EQ(region_producer_contract(o), original + suffix.str());
    o.typed_evaluation_diagnostics_enabled = false;
    EXPECT_EQ(region_producer_contract(o), original);
  }
}

#include <cstdlib>
#include <map>
namespace sunhorizon_rgbd_adapter {
struct RegionDepthProducerTestPeer {
  static void report(RegionDepthProducer& p,
      sunhorizon_obstacle_coverage::RegionDepthSupport support, bool expired) {
    const auto now = p.clock_->now().nanoseconds();
    const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // A fully evaluated result stalled before entering report(). No sleep or
    // race against a callback is used to cross the publication deadline.
    p.current_.key = {now - (expired ? 1000000000 : 0), "camera", 16, 16};
    p.current_.sequence++;
    p.current_.receipt_ros_ns = p.current_.key.stamp_ns;
    p.current_.receipt_steady_ns = steady - (expired ? 1000000000 : 0);
    p.report(support.reason, support);
  }
};
}
TEST(RegionTfDiagnostics, CompleteUnknownExpiresAtReportAndShadowCannotExportTypedSupport) {
  using namespace sunhorizon_rgbd_adapter;
  using namespace sunhorizon_obstacle_coverage;
  setenv("ROS_LOCALHOST_ONLY", "1", 1); setenv("ROS_DOMAIN_ID", "216", 1);
  rclcpp::init(0, nullptr);
  struct Shutdown { ~Shutdown() { rclcpp::shutdown(); } } shutdown;
  std::vector<float> raster(256, .6f); raster[8*16+8] = .46f;
  RegionDepthInput input;
  input.box_min = {-.1, -.1, .4}; input.box_max = {.1, .1, .45};
  input.width = input.height = input.row_stride_elements = 16;
  input.fx = input.fy = 20.; input.cx = input.cy = 8.;
  input.depth_m = raster.data(); input.depth_elements = raster.size();
  input.optical_to_base_rotation = {1.,0.,0.,0.,1.,0.,0.,0.,1.};
  input.minimum_depth_m = .2; input.maximum_depth_m = 8.; input.clear_margin_m = .02;
  input.contract = {true, true, true, true, true};
  const auto margin = evaluate_region_depth_support(input);
  ASSERT_TRUE(margin.evaluation_complete);
  ASSERT_EQ(margin.state, RegionSupportState::kUnknown);
  ASSERT_EQ(margin.disposition, RegionDisposition::kGeometryHold);
  for (bool typed : {false, true}) for (bool shadow : {false, true}) {
    RegionProducerOptions o;
    o.region_id = "r"; o.base_frame = "base"; o.optical_frame = "camera";
    o.depth_topic = "/image"; o.info_topic = "/info"; o.diagnostics_topic = "/region";
    o.expected_calibration = "calibration"; o.box_min = input.box_min; o.box_max = input.box_max;
    o.typed_evaluation_diagnostics_enabled = typed; o.shadow_only_enabled = shadow;
    o.expected_contract = region_producer_contract(o);
    std::map<std::string, std::string> report;
    // report() is synchronous: do not create an unrelated private TF executor.
    unsigned lookups = 0;
    RegionDepthProducer producer(o, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME), "test",
        [&](const diagnostic_msgs::msg::DiagnosticArray& a) {
          report.clear();
          for (const auto& kv : a.status.at(0).values) report.emplace(kv.key, kv.value);
          EXPECT_EQ(a.status.at(0).level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        },
        [&](const std::string&, const std::string&, int64_t, double) {
          ++lookups;
          ADD_FAILURE() << "report boundary must not perform a TF lookup";
          return geometry_msgs::msg::TransformStamped{};
        });
    RegionDepthProducerTestPeer::report(producer, margin, false);
    EXPECT_EQ(report.at("state"), "Unknown");
    if (typed) {
      EXPECT_EQ(report.at("evaluation_complete"), shadow ? "false" : "true");
      EXPECT_EQ(report.at("evaluation_code"), shadow ? "0" : "3");
      EXPECT_EQ(report.at("disposition"), shadow ? "0" : "2");
    } else {
      EXPECT_EQ(report.count("evaluation_complete"), 0u);
      EXPECT_EQ(report.count("evaluation_code"), 0u); EXPECT_EQ(report.count("disposition"), 0u);
    }
    RegionDepthProducerTestPeer::report(producer, margin, true);
    if (typed) {
      EXPECT_EQ(report.at("reason"), "region_stale_at_report");
      EXPECT_EQ(report.at("evaluation_complete"), "false");
      EXPECT_EQ(report.at("evaluation_code"), "0"); EXPECT_EQ(report.at("disposition"), "0");
      EXPECT_EQ(report.at("margin_band_rays"), "0");
    } else {
      EXPECT_EQ(report.at("reason"), margin.reason);  // Preserve legacy Unknown report bytes.
      EXPECT_EQ(report.at("margin_band_rays"), shadow ? "0" : "1");
    }
    RegionDepthProducerTestPeer::report(producer, {}, false);
    if (typed) {
      EXPECT_EQ(report.at("evaluation_complete"), "false");
      EXPECT_EQ(report.at("evaluation_code"), "0"); EXPECT_EQ(report.at("disposition"), "0");
    }
    EXPECT_EQ(lookups, 0U);
  }
}

namespace {
std::map<std::string, std::string> region_fields(
    const diagnostic_msgs::msg::DiagnosticArray& report) {
  std::map<std::string, std::string> fields;
  for (const auto& entry : report.status.front().values) fields[entry.key] = entry.value;
  return fields;
}
struct PreparedRegionFixture {
  using Producer = sunhorizon_rgbd_adapter::RegionDepthProducer;
  sunhorizon_rgbd_adapter::RegionProducerOptions options;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  std::shared_ptr<sensor_msgs::msg::Image> image = std::make_shared<sensor_msgs::msg::Image>();
  std::shared_ptr<sensor_msgs::msg::CameraInfo> info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  std::vector<float> raw_depth = std::vector<float>(256, 3.F);
  diagnostic_msgs::msg::DiagnosticArray report;
  int lookups{0};
  std::unique_ptr<Producer> producer;
  PreparedRegionFixture() {
    using namespace sunhorizon_rgbd_adapter;
    options.region_id="R"; options.base_frame="base_footprint"; options.optical_frame="camera";
    options.depth_topic="/raw/depth"; options.info_topic="/raw/info"; options.diagnostics_topic="/original/region";
    options.box_min={-.5,-.5,1.}; options.box_max={.5,.5,2.};
    options.accepted={true,true,true,true,true};
    options.hold_complete_measurement_enabled=true;
    options.typed_evaluation_diagnostics_enabled=true;
    info->header.frame_id="camera"; info->width=info->height=16;
    info->distortion_model="plumb_bob"; info->d.assign(5,0.);
    info->k={8.,0.,7.5,0.,8.,7.5,0.,0.,1.};
    info->r={1.,0.,0.,0.,1.,0.,0.,0.,1.};
    info->p={8.,0.,7.5,0.,0.,8.,7.5,0.,0.,0.,1.,0.};
    image->header.frame_id="camera"; image->width=image->height=16;
    image->encoding="16UC1"; image->step=32; image->data.resize(512);
    for (size_t at=0;at<256;++at) { image->data[2*at]=0xb8; image->data[2*at+1]=0x0b; }
    options.expected_calibration=region_calibration_contract(*info);
    options.expected_contract=region_producer_contract(options);
    producer=std::make_unique<Producer>(options,clock,"original_depth_owner",
      [this](const auto& value){report=value;},
      [this](const auto& target,const auto& source,int64_t stamp,double){
        ++lookups; geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id=target;tf.child_frame_id=source;tf.header.stamp=rclcpp::Time(stamp);
        tf.transform.rotation.w=1.;return tf;
      });
  }
  std::pair<int64_t,int64_t> evaluate(uint64_t sequence=1,bool expired=false) {
    const auto now=clock->now().nanoseconds();
    const auto steady=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    image->header.stamp=info->header.stamp=rclcpp::Time(now-1000000);
    producer->prepared(image,info,raw_depth.data(),raw_depth.size(),sequence,now-500000,
      steady-(expired?1000000000:1000000));
    return {now-500000,steady-(expired?1000000000:1000000)};
  }
};
}

TEST(RegionTfDiagnostics, PreparedRasterReusesOriginalCoreReportAndFrozenSdkReceipt) {
  using namespace sunhorizon_obstacle_coverage;
  PreparedRegionFixture fixture;
  const auto receipt=fixture.evaluate(37);
  ASSERT_EQ(fixture.report.status.size(),1U);
  const auto fields=region_fields(fixture.report);
  EXPECT_EQ(fixture.report.status.front().name,"original_depth_owner/region_support");
  EXPECT_EQ(fields.at("contract"),fixture.options.expected_contract);
  EXPECT_EQ(fields.at("region_frame_sequence"),"37");
  EXPECT_EQ(fields.at("image_receipt_ros_ns"),std::to_string(receipt.first));
  EXPECT_EQ(fields.at("image_receipt_steady_ns"),std::to_string(receipt.second));
  EXPECT_EQ(fields.at("state"),"ClearSupported");
  EXPECT_EQ(fields.at("evaluation_complete"),"true");
  EXPECT_EQ(fields.at("disposition"),"1");
  EXPECT_EQ(fixture.lookups,1);

  RegionDepthInput original;
  original.box_min=fixture.options.box_min;original.box_max=fixture.options.box_max;
  original.width=original.height=16;original.fx=original.fy=8.;original.cx=original.cy=7.5;
  original.depth_m=fixture.raw_depth.data();original.depth_elements=256;original.row_stride_elements=16;
  original.optical_to_base_rotation={1.,0.,0.,0.,1.,0.,0.,0.,1.};
  original.minimum_depth_m=.2;original.maximum_depth_m=8.;original.clear_margin_m=.02;
  original.contract=fixture.options.accepted;
  const auto expected=evaluate_region_depth_support(original);
  EXPECT_EQ(fields.at("reason"),expected.reason);
  EXPECT_EQ(fields.at("tested_pixels"),std::to_string(expected.tested_pixels));
  EXPECT_EQ(fields.at("clear_rays"),std::to_string(expected.clear_rays));
  EXPECT_GT(std::stoll(fields.at("region_thread_cpu_ns")),0);
}

TEST(RegionTfDiagnostics, PreparedRawRasterKeepsLegOcclusionAndZeroDepthUnknown) {
  PreparedRegionFixture leg;
  leg.raw_depth[8*16+8]=.5F; // Raw foreground remains even if body output self-filters it.
  leg.image->data[2*(8*16+8)]=0xf4;leg.image->data[2*(8*16+8)+1]=0x01;
  leg.evaluate();
  auto fields=region_fields(leg.report);
  EXPECT_EQ(fields.at("state"),"Unknown");
  EXPECT_GT(std::stoll(fields.at("front_occluded_rays")),0);
  EXPECT_EQ(fields.at("evaluation_complete"),"true");
  PreparedRegionFixture invalid;
  invalid.raw_depth[8*16+8]=0.F;
  invalid.image->data[2*(8*16+8)]=invalid.image->data[2*(8*16+8)+1]=0;
  invalid.evaluate();
  fields=region_fields(invalid.report);
  EXPECT_EQ(fields.at("state"),"Unknown");
  EXPECT_GT(std::stoll(fields.at("invalid_depth_rays")),0);
}

TEST(RegionTfDiagnostics, PreparedStaleSdkReceiptCannotRefreshAtComputation) {
  PreparedRegionFixture fixture;const auto receipt=fixture.evaluate(3,true);
  const auto fields=region_fields(fixture.report);
  EXPECT_EQ(fields.at("state"),"Unknown");
  EXPECT_EQ(fields.at("evaluation_complete"),"false");
  EXPECT_EQ(fields.at("image_receipt_steady_ns"),std::to_string(receipt.second));
  EXPECT_EQ(fields.at("reason"),"region_image_stale_or_clock_invalid");
  EXPECT_EQ(fixture.lookups,0);
}
