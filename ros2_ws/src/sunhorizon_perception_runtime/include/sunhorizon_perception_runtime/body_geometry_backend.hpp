#pragma once
#include <stdexcept>
#include <string>
namespace sunhorizon_perception_runtime::detail {
inline std::string resolve_body_geometry_backend(const std::string& global,const std::string& role="inherit") {
  if(global!="cpu"&&global!="cuda_arm11")throw std::invalid_argument("body_geometry_compute_backend_requires_cpu_or_cuda_arm11");
  if(role!="inherit"&&role!="cpu"&&role!="cuda_arm11")throw std::invalid_argument("body_geometry_role_backend_requires_inherit_cpu_or_cuda_arm11");
  return role=="inherit"?global:role;
}
inline bool body_geometry_cpu_parallel(const std::string& resolved,bool enabled) {
  return resolved=="cpu"&&enabled;
}
}
