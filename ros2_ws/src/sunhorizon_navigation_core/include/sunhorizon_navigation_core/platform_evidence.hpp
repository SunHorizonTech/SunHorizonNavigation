#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sunhorizon_navigation_core {

struct JointEvidence {
  int id{-1};
  uint32_t error_code{1U};
  uint16_t statusword{0U};
};

struct JointSetEvidence {
  bool parsed{false};
  bool complete{false};
  bool error_free{false};
  bool operation_enabled{false};
  int64_t source_stamp_ns{0};
  std::vector<JointEvidence> joints;
  std::string reason;
};

struct MotionStateEvidence {
  bool parsed{false};
  bool state_ready{false};
  bool stand_state_active{false};
  bool walking_state_active{false};
  bool navigation_control_active{false};
  bool remote_control_exited{false};
  std::string state_name;
  std::string reason;
};

struct MotionStateDecision {
  bool ready{false};
  bool armed{false};
  bool walking{false};
  std::string reason;
};

class MotionStateGate {
 public:
  MotionStateDecision evaluate(
      const MotionStateEvidence& evidence, bool evidence_fresh,
      bool platform_prerequisites_safe);
  void reset();

 private:
  enum class Phase {
    kWaitingForStand,
    kArmedStand,
    kWalking,
  };

  Phase phase_{Phase::kWaitingForStand};
};

struct EthercatEvidence {
  bool parsed{false};
  bool master_active{false};
  bool slave_op{false};
  bool wkc_normal{false};
  int working_counter{-1};
  int expected_working_counter{-1};
  std::string reason;
};

bool cia402_operation_enabled(uint16_t statusword);
bool joint_error_code_allows_motion(
    uint32_t error_code,
    const std::vector<uint32_t>& non_blocking_error_codes = {});
bool robot_mode_allowed(const std::string& actual_mode, const std::string& required_mode);
JointSetEvidence parse_joint_info_json(
    const std::string& json, const std::vector<int>& required_joint_ids,
    const std::vector<uint32_t>& non_blocking_error_codes = {});
MotionStateEvidence parse_motion_state(
    const std::string& text,
    const std::string& required_state_token = "State: STAND",
    const std::string& navigation_control_token = "Mode: NAVIGATION",
    const std::string& remote_control_exit_token = "Remote Control State: Exit",
    const std::string& allowed_running_state_token = "State: WALKING");
EthercatEvidence parse_ethercat_text(const std::string& text, int expected_wkc = 2);

}  // namespace sunhorizon_navigation_core
