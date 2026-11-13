#pragma once
#include <stdexcept>
#include <string>
#include <nav2_util/direct_observation.hpp>
namespace sunhorizon_navigation_runtime { namespace detail {
// Transient rejection does not revoke perception/localization ownership. The
// ORIGINAL consumer still owns its cache/current/lease and command checks.
inline void require_nonfatal_cloud_admission(
  const nav2_util::DirectAdmissionResult& result, const std::string& stage)
{
  if(result.state==nav2_util::DirectAdmissionState::InternalFault)
    throw std::runtime_error("nav2_original_cloud_admission_rejected:"+stage+":"+
      nav2_util::directRejectionReasonName(result.reason)+
      ":gate_ros="+std::to_string(result.snapshot_ros_ns)+
      ":gate_steady="+std::to_string(result.snapshot_steady_ns));
}
}}
