#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "sunhorizon_perception_runtime/joint_kinematics.hpp"

namespace sunhorizon_perception_runtime {

struct JointIngressContract {
  std::string source_receipt_path;
  std::string source_receipt_sha256;
  std::string source_evidence_sha256;
};

struct JointIngressCounters {
  uint64_t received{0};
  uint64_t accepted{0};
  uint64_t proof_overwritten{0};
  uint64_t proof_published{0};
  uint64_t proof_expired{0};
};

// One accepted raw callback and the FK ingest it actually produced. Rejected
// arrivals and proof publication never modify these original clocks/identity.
struct JointAcceptedIdentity {
  std::string source_owner_fqn, source_publisher_gid;
  uint64_t accepted_sequence{0};
  JointKinematicsEvidence fk;
};

// Put this node on the runtime's dedicated joint SingleThreadedExecutor. The
// accepted raw stream feeds JointKinematics directly; a one-slot asynchronous
// egress still emits the existing platform/global-RSP proof JointState. Neither
// this class nor JointKinematics subscribes to private TF DDS.
class JointIngress final : public rclcpp::Node {
 public:
  JointIngress(std::shared_ptr<JointKinematics> kinematics,
               const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~JointIngress() override;
  JointIngressContract contract() const;
  JointIngressCounters counters() const;
  JointAcceptedIdentity accepted_identity() const;
  std::shared_ptr<JointKinematics> kinematics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

std::shared_ptr<JointIngress> make_joint_ingress(
    std::shared_ptr<JointKinematics> kinematics,
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

}  // namespace sunhorizon_perception_runtime
