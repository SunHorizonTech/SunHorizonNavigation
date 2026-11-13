// Standalone deterministic checks; no ROS graph or motion is started.
#include <cassert>
#include <sunhorizon_velocity_supervisor/attended_travel_guard.hpp>
#include <cmath>
#include <limits>
#include <sunhorizon_velocity_supervisor/motion_authority_contract.hpp>
#include <sunhorizon_navigation_core/safety_gate.hpp>

using namespace sunhorizon_velocity_supervisor;
using namespace sunhorizon_navigation_core;

MotionAuthorityContract heading_contract() {
  MotionAuthorityContract c;
  c.mode = kAttendedHeadingCorrectionNavigationMode;
  c.qualification_acknowledgement = kAttendedHeadingCorrectionNavigationAcknowledgement;
  c.motion_output_enabled = true;
  c.candidate_topic = "/sunhorizon/navigation/cmd_vel_candidate";
  c.collision_audit_topic = "/sunhorizon/navigation/collision_audit";
  c.final_topic = "/navigation/cmd_vel";
  c.map_frame = "map";
  c.base_frame = "base_footprint";
  c.platform_state_topic = "/sunhorizon/platform/safety_state";
  c.localization_status_topic = "/sunhorizon/localization/status";
  c.obstacle_coverage_topic = "/sunhorizon/obstacle_coverage/status";
  c.expected_candidate_publisher_fqn = "/collision_monitor";
  c.candidate_timeout_sec = 0.25;
  c.control_rate_hz = 50.0;
  c.max_vx = .05;
  c.max_vy = 0.;
  c.max_wz = .03;
  c.max_ax = .10;
  c.max_ay = 0.;
  c.max_aw = .10;
  return c;
}

SafetyInputs healthy() {
  SafetyInputs i;
  i.now_sec = 10.;
  i.candidate_seen = i.obstacle_coverage_seen = i.navigation_status_seen = true;
  i.candidate_stamp_sec = i.platform_stamp_sec = i.obstacle_coverage_stamp_sec = 9.95;
  i.localization_stamp_sec = i.tf_stamp_sec = i.navigation_status_stamp_sec = 9.95;
  i.navigation_task_active = i.mode_allowed = i.ethercat_op = i.wkc_normal = true;
  i.joint_feedback_fresh = i.leg_joints_error_free = i.leg_joints_operation_enabled = true;
  i.motion_ready = i.navigation_mode_discovered = true;
  i.platform_fault = false;
  i.obstacle_coverage_healthy = i.forward_coverage = i.near_field_coverage = true;
  i.localization_healthy = i.tf_continuous = true;
  return i;
}

int main() {
  auto c = heading_contract();
  assert(motion_authority_contract_failure(c).empty());
  for (double invalid : {-1., 0., .03001, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    auto changed = c;
    changed.max_wz = invalid;
    assert(!motion_authority_contract_failure(changed).empty());
  }
  for (double invalid : {-1., 0., .10001, std::numeric_limits<double>::infinity()}) {
    auto changed = c;
    changed.max_aw = invalid;
    assert(!motion_authority_contract_failure(changed).empty());
    changed = c;
    changed.max_ax = invalid;
    assert(!motion_authority_contract_failure(changed).empty());
  }
  for (int mutation = 0; mutation < 8; ++mutation) {
    auto changed = c;
    if (mutation == 0) changed.max_vy = .001;
    if (mutation == 1) changed.max_ay = .001;
    if (mutation == 2) changed.max_vx = .05001;
    if (mutation == 3) changed.qualification_acknowledgement.clear();
    if (mutation == 4) changed.expected_candidate_publisher_fqn = "/unfiltered";
    if (mutation == 5) changed.velocity_calibration_accepted = true;
    if (mutation == 6) changed.vendor_watchdog_accepted = true;
    if (mutation == 7) changed.motion_output_enabled = false;
    assert(!motion_authority_contract_failure(changed).empty());
  }
  auto three = c;
  three.mode = kAttendedThreeMeterNavigationMode;
  assert(!motion_authority_contract_failure(three).empty());
  three.qualification_acknowledgement = kAttendedThreeMeterNavigationAcknowledgement;
  assert(motion_authority_contract_failure(three).empty());
  three.max_vx = .05001;
  assert(!motion_authority_contract_failure(three).empty());
  AttendedQualificationNonzeroBudget long_budget(AttendedQualificationNonzeroBudget::kThreeMeterHardLimit);
  const auto t = AttendedQualificationNonzeroBudget::Clock::now();
  long_budget.observe_first_nonzero(t);
  assert(!long_budget.expired(t+std::chrono::seconds(134)));
  long_budget.observe_first_nonzero(t+std::chrono::seconds(134));
  assert(long_budget.expired(t+std::chrono::seconds(135)));
  assert(!long_budget.reset_allowed());
  assert(AttendedQualificationNonzeroBudget::kFirstClickNavigationHardLimit == std::chrono::seconds(45));
  AttendedTravelGuard guard;
  guard.begin(10., -2., 0.);
  assert(guard.observe(13., -1.501).empty());
  assert(guard.observe(13., -1.5) == "attended_three_meter_cross_track_exceeded");
  guard.begin(13., -1.5, 0.); // Cannot renew the origin or clear its fault.
  assert(!guard.observe(10., -2.).empty());
  AttendedTravelGuard rotated;
  rotated.begin(0., 0., std::acos(-1.)/2);
  assert(rotated.observe(0., 3.30).empty());
  assert(rotated.observe(0., 3.301) == "attended_three_meter_displacement_exceeded");
  AttendedTravelGuard invalid;
  invalid.begin(0., 0., std::numeric_limits<double>::quiet_NaN());
  assert(!invalid.observe(0., 0.).empty());
  AttendedTravelGuard wider(true, .75);
  wider.begin(0., 0., 0.);
  assert(wider.observe(2., .60).empty());
  assert(wider.observe(2., .75) == "attended_five_meter_cross_track_exceeded");
  assert(!wider.observe(2., 0.).empty()); // Returning in bounds cannot clear a stop.
  for (double bad : {0., -1., 5.31, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { AttendedTravelGuard malformed(true, bad); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  auto five = c;
  five.mode = kAttendedFiveMeterNavigationMode;
  five.qualification_acknowledgement = kAttendedThreeMeterNavigationAcknowledgement;
  assert(!motion_authority_contract_failure(five).empty());
  five.qualification_acknowledgement = kAttendedFiveMeterNavigationAcknowledgement;
  assert(motion_authority_contract_failure(five).empty());
  auto faster = five;
  faster.mode = kAttendedFiveMeterFasterNavigationMode;
  faster.max_vx = .08;
  assert(!motion_authority_contract_failure(faster).empty()); // v1 acknowledgement
  faster.qualification_acknowledgement = kAttendedFiveMeterFasterNavigationAcknowledgement;
  assert(motion_authority_contract_failure(faster).empty());
  assert(is_five_meter_navigation_mode(faster.mode)); // Same hard travel/time guard.
  assert(is_five_meter_navigation_mode(five.mode));
  assert(!is_five_meter_navigation_mode(kAttendedThreeMeterNavigationMode));
  for (int mutation = 0; mutation < 8; ++mutation) {
    auto changed = faster;
    if (mutation == 0) changed.max_vx = .080001;
    if (mutation == 1) changed.max_ax = .100001;
    if (mutation == 2) changed.max_wz = .030001;
    if (mutation == 3) changed.max_vy = .001;
    if (mutation == 4) changed.max_aw = .100001;
    if (mutation == 5) changed.mode = kAttendedFiveMeterNavigationMode;
    if (mutation == 6) changed.expected_candidate_publisher_fqn = "/other";
    if (mutation == 7) changed.max_vx = std::numeric_limits<double>::quiet_NaN();
    assert(!motion_authority_contract_failure(changed).empty());
  }
  for (int mutation = 0; mutation < 5; ++mutation) {
    auto changed = five;
    if (mutation == 0) changed.max_vx = .05001;
    if (mutation == 1) changed.max_wz = .03001;
    if (mutation == 2) changed.max_vy = .001;
    if (mutation == 3) changed.expected_candidate_publisher_fqn = "/other";
    if (mutation == 4) changed.motion_output_enabled = false;
    assert(!motion_authority_contract_failure(changed).empty());
  }
  AttendedQualificationNonzeroBudget five_budget(AttendedQualificationNonzeroBudget::kFiveMeterHardLimit);
  five_budget.observe_first_nonzero(t);
  assert(!five_budget.expired(t+std::chrono::milliseconds(224999)));
  five_budget.observe_first_nonzero(t+std::chrono::milliseconds(224999));
  assert(five_budget.expired(t+std::chrono::seconds(225)));
  assert(!five_budget.reset_allowed());
  AttendedTravelGuard five_guard(true);
  five_guard.begin(0., 0., 0.);
  assert(five_guard.observe(5.30, 0.).empty());
  assert(five_guard.observe(5.301, 0.) == "attended_five_meter_displacement_exceeded");
  five_guard.begin(5.3, 0., 0.);
  assert(!five_guard.observe(0., 0.).empty());
  AttendedTravelGuard five_rotated(true);
  five_rotated.begin(0., 0., std::acos(-1.)/2);
  assert(five_rotated.observe(-.499, 5.).empty());
  assert(five_rotated.observe(-.501, 5.) == "attended_five_meter_cross_track_exceeded");
  auto old = c;
  old.mode = kAttendedFirstClickNavigationMode;
  old.qualification_acknowledgement = kAttendedFirstClickNavigationAcknowledgement;
  assert(!motion_authority_contract_failure(old).empty());
  old.max_wz = 0.;
  assert(motion_authority_contract_failure(old).empty());

  SafetyLimits limits;
  limits.max_vx = .05; limits.max_vy = 0.; limits.max_wz = .03;
  limits.max_ax = .10; limits.max_ay = 0.; limits.max_aw = .10;
  for (double direction : {-1., 1.}) {
    SafetyGate gate(limits);
    auto i = healthy();
    assert(gate.evaluate(i, {.045, 0., direction * .027}).allowed);
    i.now_sec += .02;
    auto output = gate.evaluate(i, {.045, 0., direction * .027});
    assert(output.allowed && output.output.wz * direction > 0.);
    assert(std::abs(output.output.wz) <= .002000001);
    assert(output.output.vx <= .002000001 && output.output.vy == 0.);
    i.platform_fault = true;
    output = gate.evaluate(i, {.045, 0., direction * .027});
    assert(output.latched && !output.allowed);
    assert(output.output.vx == 0. && output.output.wz == 0.);
    i.platform_fault = false;
    assert(gate.evaluate(i, {.045, 0., direction * .027}).latched);
  }
  assert(SafetyGate(limits).evaluate(healthy(), {-.01, 0., .02}).latched);
  assert(SafetyGate(limits).evaluate(healthy(), {.04, .001, .02}).latched);
  assert(SafetyGate(limits).evaluate(healthy(), {.04, 0., .031}).latched);
  for (int fault = 0; fault < 4; ++fault) {
    auto i = healthy();
    if (fault == 0) i.candidate_stamp_sec = 9.;
    if (fault == 1) i.localization_healthy = false;
    if (fault == 2) i.tf_continuous = false;
    if (fault == 3) i.leg_joints_error_free = false;
    const auto stopped = SafetyGate(limits).evaluate(i, {.04, 0., .02});
    assert(stopped.latched && stopped.output.vx == 0. && stopped.output.wz == 0.);
  }
  using Budget = AttendedQualificationNonzeroBudget;
  Budget budget(Budget::kFirstClickNavigationHardLimit);
  const auto start = Budget::Clock::time_point{};
  budget.observe_first_nonzero(start);
  assert(!budget.expired(start + std::chrono::milliseconds(44999)));
  assert(budget.expired(start + std::chrono::milliseconds(45000)));
  budget.observe_first_nonzero(start + std::chrono::milliseconds(44999));
  assert(budget.expired(start + std::chrono::milliseconds(45000)));
  assert(!budget.reset_allowed());
}
