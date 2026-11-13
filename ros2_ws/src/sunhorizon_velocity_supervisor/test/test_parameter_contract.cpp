#include <gtest/gtest.h>

#include <limits>

#include <sunhorizon_velocity_supervisor/motion_authority_contract.hpp>
#include <sunhorizon_velocity_supervisor/parameter_contract.hpp>
#include <sunhorizon_velocity_supervisor/tf_continuity.hpp>

namespace {

sunhorizon_velocity_supervisor::MotionAuthorityContract qualification_contract() {
  using namespace sunhorizon_velocity_supervisor;
  MotionAuthorityContract contract;
  contract.mode = kAttendedFiveCmQualificationMode;
  contract.qualification_acknowledgement =
      kAttendedFiveCmQualificationAcknowledgement;
  contract.motion_output_enabled = true;
  contract.candidate_topic = "/sunhorizon/navigation/cmd_vel_candidate";
  contract.collision_audit_topic = "/sunhorizon/navigation/collision_audit";
  contract.final_topic = "/navigation/cmd_vel";
  contract.map_frame = "map";
  contract.base_frame = "base_footprint";
  contract.platform_state_topic = "/sunhorizon/platform/safety_state";
  contract.localization_status_topic = "/sunhorizon/localization/status";
  contract.obstacle_coverage_topic = "/sunhorizon/obstacle_coverage/status";
  contract.expected_candidate_publisher_fqn = kAttendedFiveCmCandidateOwner;
  contract.candidate_timeout_sec = 0.25;
  contract.control_rate_hz = 50.0;
  contract.max_vx = 0.05;
  contract.max_vy = 0.0;
  contract.max_wz = 0.0;
  return contract;
}

sunhorizon_velocity_supervisor::MotionAuthorityContract yaw_contract() {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = qualification_contract();
  contract.mode = kAttendedYawQualificationMode;
  contract.qualification_acknowledgement =
      kAttendedYawQualificationAcknowledgement;
  contract.expected_candidate_publisher_fqn = kAttendedYawCandidateOwner;
  contract.max_vx = 0.0;
  contract.max_vy = 0.0;
  contract.max_wz = 0.03;
  return contract;
}

sunhorizon_velocity_supervisor::MotionAuthorityContract
forward_calibration_contract() {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = qualification_contract();
  contract.mode = kAttendedForwardCalibrationMode;
  contract.qualification_acknowledgement =
      kAttendedForwardCalibrationAcknowledgement;
  contract.expected_candidate_publisher_fqn =
      kAttendedForwardCalibrationCandidateOwner;
  return contract;
}

sunhorizon_velocity_supervisor::MotionAuthorityContract
first_click_navigation_contract() {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = qualification_contract();
  contract.mode = kAttendedFirstClickNavigationMode;
  contract.qualification_acknowledgement =
      kAttendedFirstClickNavigationAcknowledgement;
  contract.expected_candidate_publisher_fqn = "/collision_monitor";
  contract.max_vx = 0.05;
  return contract;
}

}  // namespace

TEST(ParameterContract, ReadOnlyDescriptorRejectsRuntimeMutation) {
  const auto descriptor =
      sunhorizon_velocity_supervisor::read_only_parameter("construction-time value");

  EXPECT_TRUE(descriptor.read_only);
  EXPECT_EQ(descriptor.description, "construction-time value");
}

TEST(MotionAuthorityContract, NormalLiveStillRequiresBothReceipts) {
  using namespace sunhorizon_velocity_supervisor;
  MotionAuthorityContract contract;
  contract.motion_output_enabled = true;
  contract.velocity_calibration_accepted = true;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "live_output_requires_velocity_and_watchdog_receipts");

  contract.vendor_watchdog_accepted = true;
  EXPECT_TRUE(motion_authority_contract_failure(contract).empty());
}

TEST(MotionAuthorityContract, DedicatedQualificationBootstrapsWithoutReceipts) {
  using namespace sunhorizon_velocity_supervisor;
  const auto contract = qualification_contract();
  EXPECT_FALSE(contract.velocity_calibration_accepted);
  EXPECT_FALSE(contract.vendor_watchdog_accepted);
  EXPECT_TRUE(motion_authority_contract_failure(contract).empty());
}

TEST(MotionAuthorityContract, QualificationRequiresExactProfileAndEnvelope) {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = qualification_contract();
  contract.qualification_acknowledgement.clear();
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_five_cm_qualification_acknowledgement_required");

  contract = qualification_contract();
  contract.expected_candidate_publisher_fqn = "/collision_monitor";
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_five_cm_qualification_candidate_owner_mismatch");

  contract = qualification_contract();
  contract.max_vy = 0.01;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_five_cm_qualification_motion_envelope_invalid");

  contract = qualification_contract();
  contract.candidate_timeout_sec = 0.251;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_five_cm_qualification_timing_contract_invalid");
}

TEST(MotionAuthorityContract, QualificationCannotClaimProductionReceipts) {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = qualification_contract();
  contract.velocity_calibration_accepted = true;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_five_cm_qualification_must_not_claim_acceptance_receipts");
}

TEST(MotionAuthorityContract, DedicatedYawQualificationUsesExactZeroLinearEnvelope) {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = yaw_contract();
  EXPECT_TRUE(motion_authority_contract_failure(contract).empty());

  contract.max_vx = 0.001;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_yaw_qualification_motion_envelope_invalid");

  contract = yaw_contract();
  contract.max_wz = 0.031;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_yaw_qualification_motion_envelope_invalid");

  contract = yaw_contract();
  contract.expected_candidate_publisher_fqn = kAttendedFiveCmCandidateOwner;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_yaw_qualification_candidate_owner_mismatch");
}

TEST(MotionAuthorityContract, ForwardCalibrationUsesExactForwardOnlyEnvelope) {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = forward_calibration_contract();
  EXPECT_TRUE(motion_authority_contract_failure(contract).empty());

  contract.max_vx = 0.049;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_forward_calibration_motion_envelope_invalid");

  contract = forward_calibration_contract();
  contract.max_vy = 0.001;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_forward_calibration_motion_envelope_invalid");

  contract = forward_calibration_contract();
  contract.expected_candidate_publisher_fqn = kAttendedFiveCmCandidateOwner;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_forward_calibration_qualification_candidate_owner_mismatch");
}

TEST(MotionAuthorityContract, ForwardPulseIsDistinctAndRetainsAllContractChecks) {
  using namespace sunhorizon_velocity_supervisor;
  auto valid = qualification_contract();
  valid.mode = kAttendedForwardPulseMode;
  valid.qualification_acknowledgement = kAttendedForwardPulseAcknowledgement;
  valid.expected_candidate_publisher_fqn = kAttendedForwardPulseCandidateOwner;
  ASSERT_TRUE(motion_authority_contract_failure(valid).empty());
  auto changed = valid;
  changed.expected_candidate_publisher_fqn = kAttendedForwardCalibrationCandidateOwner;
  EXPECT_EQ(motion_authority_contract_failure(changed),
            "attended_forward_pulse_qualification_candidate_owner_mismatch");
  changed = valid;
  changed.qualification_acknowledgement = kAttendedForwardCalibrationAcknowledgement;
  EXPECT_EQ(motion_authority_contract_failure(changed),
            "attended_forward_pulse_qualification_acknowledgement_required");
  for (const double vx : {0.049, 0.051, -0.05, std::numeric_limits<double>::infinity()}) {
    changed = valid;
    changed.max_vx = vx;
    EXPECT_EQ(motion_authority_contract_failure(changed),
              "attended_forward_pulse_motion_envelope_invalid");
  }
  changed = valid;
  changed.max_vy = 0.001;
  EXPECT_FALSE(motion_authority_contract_failure(changed).empty());
  changed = valid;
  changed.max_wz = 0.001;
  EXPECT_FALSE(motion_authority_contract_failure(changed).empty());
  changed = valid;
  changed.vendor_watchdog_accepted = true;
  EXPECT_EQ(motion_authority_contract_failure(changed),
            "attended_forward_pulse_qualification_must_not_claim_acceptance_receipts");
  changed = valid;
  changed.candidate_timeout_sec = 0.251;
  EXPECT_EQ(motion_authority_contract_failure(changed),
            "attended_forward_pulse_qualification_timing_contract_invalid");
}

TEST(AttendedQualificationNonzeroBudget, ForwardPulseHasInclusiveTwoSecondOneShotLimit) {
  using Budget = sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  EXPECT_EQ(Budget::kForwardPulseHardLimit, std::chrono::milliseconds(2000));
  Budget budget(Budget::kForwardPulseHardLimit);
  const auto start = Budget::Clock::time_point{};
  EXPECT_FALSE(budget.spent());
  budget.observe_first_nonzero(start);
  budget.observe_first_nonzero(start + std::chrono::milliseconds(1999));
  EXPECT_FALSE(budget.reset_allowed());
  EXPECT_FALSE(budget.expired(start + std::chrono::milliseconds(2000) -
                              std::chrono::nanoseconds(1)));
  EXPECT_TRUE(budget.expired(start + std::chrono::milliseconds(2000)));
  EXPECT_TRUE(budget.expired(start + std::chrono::seconds(12)));
  EXPECT_EQ(Budget::kForwardCalibrationHardLimit, std::chrono::milliseconds(12000));
}

TEST(MotionAuthorityContract, FirstClickUsesCollisionMonitorAndUnacceptedAdapter) {
  using namespace sunhorizon_velocity_supervisor;
  auto contract = first_click_navigation_contract();
  EXPECT_TRUE(motion_authority_contract_failure(contract).empty());

  contract.expected_candidate_publisher_fqn = kAttendedFiveCmCandidateOwner;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_first_click_navigation_qualification_candidate_owner_mismatch");

  contract = first_click_navigation_contract();
  contract.velocity_calibration_accepted = true;
  EXPECT_EQ(
      motion_authority_contract_failure(contract),
      "attended_first_click_navigation_qualification_must_not_claim_acceptance_receipts");
}

TEST(MotionAuthorityContract, FirstClickRequiresExactAcknowledgementAndEnvelope) {
  auto contract = first_click_navigation_contract();
  contract.qualification_acknowledgement.clear();
  EXPECT_EQ(
      sunhorizon_velocity_supervisor::motion_authority_contract_failure(contract),
      "attended_first_click_navigation_qualification_acknowledgement_required");

  contract = first_click_navigation_contract();
  contract.max_vx = 0.0501;
  EXPECT_EQ(
      sunhorizon_velocity_supervisor::motion_authority_contract_failure(contract),
      "attended_first_click_navigation_motion_envelope_invalid");

  contract = first_click_navigation_contract();
  contract.max_wz = 0.001;
  EXPECT_EQ(
      sunhorizon_velocity_supervisor::motion_authority_contract_failure(contract),
      "attended_first_click_navigation_motion_envelope_invalid");
}

TEST(AttendedQualificationNonzeroBudget, ZeroDoesNotSpendOneShotBudget) {
  using Budget =
      sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  Budget budget;
  EXPECT_FALSE(budget.spent());
  EXPECT_TRUE(budget.reset_allowed());
  EXPECT_FALSE(budget.expired(Budget::Clock::time_point{} + Budget::kHardLimit));
}

TEST(AttendedQualificationNonzeroBudget, HardDeadlineIsInclusiveAndCannotReset) {
  using Budget =
      sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  Budget budget;
  const auto started = Budget::Clock::time_point{} + std::chrono::seconds(1);
  budget.observe_first_nonzero(started);
  EXPECT_TRUE(budget.spent());
  EXPECT_FALSE(budget.reset_allowed());
  EXPECT_FALSE(budget.expired(started + Budget::kHardLimit - std::chrono::nanoseconds(1)));
  EXPECT_TRUE(budget.expired(started + Budget::kHardLimit));
  EXPECT_TRUE(budget.expired(started + Budget::kHardLimit + std::chrono::seconds(1)));
}

TEST(AttendedQualificationNonzeroBudget, LaterNonzeroCannotExtendDeadline) {
  using Budget =
      sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  Budget budget;
  const auto started = Budget::Clock::time_point{} + std::chrono::seconds(1);
  budget.observe_first_nonzero(started);
  budget.observe_first_nonzero(started + std::chrono::milliseconds(399));
  EXPECT_TRUE(budget.expired(started + Budget::kHardLimit));
}

TEST(AttendedQualificationNonzeroBudget, ForwardCalibrationHasIndependentLimit) {
  using Budget =
      sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  Budget budget(Budget::kForwardCalibrationHardLimit);
  const auto started = Budget::Clock::time_point{} + std::chrono::seconds(1);
  budget.observe_first_nonzero(started);
  EXPECT_FALSE(budget.expired(
      started + Budget::kForwardCalibrationHardLimit -
      std::chrono::nanoseconds(1)));
  EXPECT_TRUE(
      budget.expired(started + Budget::kForwardCalibrationHardLimit));
}

TEST(AttendedQualificationNonzeroBudget, FirstClickHasFortyFiveSecondOneShotLimit) {
  using Budget =
      sunhorizon_velocity_supervisor::AttendedQualificationNonzeroBudget;
  EXPECT_EQ(Budget::kFirstClickNavigationHardLimit, std::chrono::milliseconds(45000));
  Budget budget(Budget::kFirstClickNavigationHardLimit);
  const auto started = Budget::Clock::time_point{} + std::chrono::seconds(1);
  budget.observe_first_nonzero(started);
  budget.observe_first_nonzero(started + std::chrono::seconds(19));
  EXPECT_FALSE(budget.reset_allowed());
  EXPECT_FALSE(budget.expired(
      started + Budget::kFirstClickNavigationHardLimit -
      std::chrono::nanoseconds(1)));
  EXPECT_TRUE(budget.expired(
      started + Budget::kFirstClickNavigationHardLimit));
}

TEST(TfContinuityTracker, ExplicitResetRebaselinesOnlyTheStagedEpoch) {
  using sunhorizon_velocity_supervisor::PlanarTfObservation;
  using sunhorizon_velocity_supervisor::TfContinuityTracker;

  TfContinuityTracker active;
  ASSERT_TRUE(active.observe(
      PlanarTfObservation{9.99, 0.0, 0.0, 0.0}, 10.0, 0.30, 0.25));

  auto proposed = active;
  ASSERT_TRUE(proposed.rebaseline(
      PlanarTfObservation{10.09, 1.20, -0.40, 0.50}, 10.10));

  // A failed reset can discard the staged tracker without weakening the live
  // epoch: the same relocation is still detected by the active tracker.
  EXPECT_FALSE(active.observe(
      PlanarTfObservation{10.09, 1.20, -0.40, 0.50}, 10.10, 0.30, 0.25));

  // Committing a successful operator reset accepts the new pose as baseline,
  // while retaining the ordinary runtime jump check for subsequent updates.
  active = proposed;
  EXPECT_TRUE(active.observe(
      PlanarTfObservation{10.19, 1.22, -0.40, 0.51}, 10.20, 0.30, 0.25));
  EXPECT_FALSE(active.observe(
      PlanarTfObservation{10.29, 1.80, -0.40, 0.51}, 10.30, 0.30, 0.25));
}

TEST(TfContinuityTracker, NonFiniteResetObservationCannotBecomeBaseline) {
  using sunhorizon_velocity_supervisor::PlanarTfObservation;
  using sunhorizon_velocity_supervisor::TfContinuityTracker;

  TfContinuityTracker tracker;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.rebaseline(
      PlanarTfObservation{10.0, nan, 0.0, 0.0}, 10.01));
  EXPECT_FALSE(tracker.rebaseline(
      PlanarTfObservation{10.0, 0.0, 0.0, nan}, 10.01));
  EXPECT_FALSE(tracker.rebaseline(
      PlanarTfObservation{0.0, 0.0, 0.0, 0.0}, 10.01));

  // No invalid attempt created a baseline, so the first valid observation is
  // accepted as the initial runtime epoch.
  EXPECT_TRUE(tracker.observe(
      PlanarTfObservation{10.0, 2.0, 0.0, 0.0}, 10.01, 0.30, 0.25));
}
