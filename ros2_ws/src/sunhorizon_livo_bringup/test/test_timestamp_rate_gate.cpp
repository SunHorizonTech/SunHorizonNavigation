#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "sunhorizon_livo_bringup/timestamp_rate_gate.hpp"

using sunhorizon_livo_bringup::TimestampRateGate;

TEST(TimestampRateGateTest, ReducesFifteenHertzInputToTenHertzWithoutDrift)
{
  TimestampRateGate gate(10.0);
  std::vector<std::int64_t> published;
  constexpr std::int64_t start = 1'000'000'000LL;
  constexpr std::int64_t input_period = 66'744'000LL;
  for (std::int64_t index = 0; index < 150; ++index) {
    const auto stamp = start + index * input_period;
    if (gate.should_publish(stamp)) {
      published.push_back(stamp);
    }
  }

  ASSERT_GE(published.size(), 99U);
  ASSERT_LE(published.size(), 101U);
  for (std::size_t index = 1; index < published.size(); ++index) {
    EXPECT_GT(published[index], published[index - 1]);
    EXPECT_LE(published[index] - published[index - 1], 2 * input_period);
  }
}

TEST(TimestampRateGateTest, ReducesTenHertzInputToEightHertzWithoutDrift)
{
  TimestampRateGate gate(8.0);
  std::vector<std::int64_t> published;
  constexpr std::int64_t start = 1'000'000'000LL;
  constexpr std::int64_t input_period = 100'000'000LL;
  for (std::int64_t index = 0; index < 100; ++index) {
    const auto stamp = start + index * input_period;
    if (gate.should_publish(stamp)) {
      published.push_back(stamp);
    }
  }

  ASSERT_EQ(published.size(), 80U);
  for (std::size_t index = 1; index < published.size(); ++index) {
    EXPECT_GT(published[index], published[index - 1]);
    EXPECT_LE(published[index] - published[index - 1], 2 * input_period);
  }
}

TEST(TimestampRateGateTest, ReducesTenHertzInputToSixHertzWithoutDrift)
{
  TimestampRateGate gate(6.0);
  std::vector<std::int64_t> published;
  constexpr std::int64_t start = 1'000'000'000LL;
  constexpr std::int64_t input_period = 100'000'000LL;
  for (std::int64_t index = 0; index < 100; ++index) {
    const auto stamp = start + index * input_period;
    if (gate.should_publish(stamp)) {
      published.push_back(stamp);
    }
  }

  ASSERT_GE(published.size(), 59U);
  ASSERT_LE(published.size(), 61U);
  for (std::size_t index = 1; index < published.size(); ++index) {
    EXPECT_GT(published[index], published[index - 1]);
    EXPECT_LE(published[index] - published[index - 1], 2 * input_period);
  }
}

TEST(TimestampRateGateTest, RejectsInvalidAndNonMonotonicTimestamps)
{
  TimestampRateGate gate(10.0);
  EXPECT_FALSE(gate.should_publish(0));
  EXPECT_TRUE(gate.should_publish(1'000'000'000LL));
  EXPECT_FALSE(gate.should_publish(999'000'000LL));
  EXPECT_FALSE(gate.should_publish(1'050'000'000LL));
  EXPECT_TRUE(gate.should_publish(1'100'000'000LL));
}

TEST(TimestampRateGateTest, ResetStartsANewRateEpoch)
{
  TimestampRateGate gate(10.0);
  EXPECT_TRUE(gate.should_publish(1'000'000'000LL));
  EXPECT_FALSE(gate.should_publish(1'050'000'000LL));
  gate.reset();
  EXPECT_TRUE(gate.should_publish(1'050'000'000LL));
}

TEST(TimestampRateGateTest, RejectsInvalidRate)
{
  EXPECT_THROW(TimestampRateGate(0.0), std::invalid_argument);
  EXPECT_THROW(TimestampRateGate(-1.0), std::invalid_argument);
}
