#include <gtest/gtest.h>

#include <limits>

#include "sunhorizon_3d_localizer/ndt_freshness.hpp"

namespace sunhorizon_3d_localizer {
namespace {

constexpr std::int64_t kSecond = 1000000000LL;

TEST(NdtFreshness, InputAgeAcceptsRobotAObservedMaximum) {
  const auto decision = evaluate_ndt_input_age(
      100 * kSecond, 100 * kSecond + 494000000LL, 0.65, 0.01);
  EXPECT_TRUE(decision.accepted);
  EXPECT_NEAR(decision.age_sec, 0.494, 1e-12);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(NdtFreshness, InputAgeBoundaryIsInclusiveAndOlderInputFails) {
  EXPECT_TRUE(evaluate_ndt_input_age(
      100 * kSecond, 100 * kSecond + 650000000LL, 0.65, 0.01).accepted);
  const auto stale = evaluate_ndt_input_age(
      100 * kSecond, 100 * kSecond + 650000001LL, 0.65, 0.01);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.reason, "ndt_input_measurement_stale");
}

TEST(NdtFreshness, InputTimestampInvalidOrFutureFailsClosed) {
  EXPECT_EQ(
      evaluate_ndt_input_age(0, 100 * kSecond, 0.65, 0.01).reason,
      "ndt_input_timestamp_invalid");
  EXPECT_EQ(
      evaluate_ndt_input_age(
          101 * kSecond, 100 * kSecond, 0.65, 0.01).reason,
      "ndt_input_timestamp_future");
  EXPECT_EQ(
      evaluate_ndt_input_age(
          100 * kSecond, 101 * kSecond,
          std::numeric_limits<double>::quiet_NaN(), 0.01).reason,
      "ndt_input_timestamp_invalid");
}

TEST(NdtFreshness, InputFutureToleranceBoundaryIsInclusive) {
  const auto within = evaluate_ndt_input_age(
      100 * kSecond + 10000000LL, 100 * kSecond, 0.65, 0.01);
  EXPECT_TRUE(within.accepted);
  EXPECT_DOUBLE_EQ(within.age_sec, 0.0);
  const auto beyond = evaluate_ndt_input_age(
      100 * kSecond + 10000001LL, 100 * kSecond, 0.65, 0.01);
  EXPECT_FALSE(beyond.accepted);
  EXPECT_EQ(beyond.reason, "ndt_input_timestamp_future");
}

TEST(NdtFreshness, ResultStreamAcceptsObservedGapAndTimesOutIndependently) {
  const auto observed = evaluate_ndt_result_stream_age(
      100 * kSecond, 100 * kSecond + 715000000LL, 0.80);
  EXPECT_TRUE(observed.accepted);
  EXPECT_NEAR(observed.age_sec, 0.715, 1e-12);
  EXPECT_TRUE(evaluate_ndt_result_stream_age(
      100 * kSecond, 100 * kSecond + 800000000LL, 0.80).accepted);
  const auto stale = evaluate_ndt_result_stream_age(
      100 * kSecond, 100 * kSecond + 800000001LL, 0.80);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.reason, "ndt_result_stream_stale");
}

TEST(NdtFreshness, ResultReceiptInvalidOrFutureFailsClosed) {
  EXPECT_EQ(
      evaluate_ndt_result_stream_age(0, 100 * kSecond, 0.80).reason,
      "ndt_accepted_result_receipt_invalid");
  EXPECT_EQ(
      evaluate_ndt_result_stream_age(
          101 * kSecond, 100 * kSecond, 0.80).reason,
      "ndt_accepted_result_receipt_future");
}

}  // namespace
}  // namespace sunhorizon_3d_localizer
