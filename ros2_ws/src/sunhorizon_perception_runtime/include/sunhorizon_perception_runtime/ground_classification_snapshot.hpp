#pragma once

#include "sunhorizon_perception_runtime/ground_reference_owner.hpp"
#include <string>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace sunhorizon_perception_runtime {struct PerceptionOutput;struct RuntimeConfig;}
namespace sunhorizon_perception_runtime::detail {
// Explicit default-off replay diagnostic. Pure serialization rejects identity
// mismatches; writing uses the existing observer-only no-replace evidence path.
nlohmann::json serialize_body_replay_snapshot(const PerceptionOutput&,const RuntimeConfig&);
std::string write_body_replay_snapshot(const std::string&,const PerceptionOutput&,const RuntimeConfig&);
// Debug evidence only. An empty environment variable disables capture; path
// validation and all serialization/I/O belong to the existing observer worker.
std::string ground_snapshot_directory_from_environment();
// Actual-writer evidence only, called by the asynchronous observer worker.
// Original primary payload and complete calibration survive recorder drops;
// native/dual-IR payload availability is explicitly a separate requirement.
nlohmann::json write_rgbd_writer_primary_snapshot(const std::string& directory,
  const std::string& role,uint64_t sequence,const std::string& request,
  const std::string& consumer,const sensor_msgs::msg::Image&,
  const sensor_msgs::msg::CameraInfo&);
std::string write_rgbd_writer_status_snapshot(const std::string& directory,
  const std::string& request,std::size_t slot,const std::string& bytes);
std::string write_rgbd_writer_classification_snapshot(const std::string& directory,
  const std::string& role,uint64_t sequence,const std::string& request,
  const std::string& consumer,const std::string& bytes);

std::string serialize_ground_classification_snapshot(
  const ground_relative::ClassifiedObstacleFrame&,
  const ground_relative::GroundReferenceOwner* owner=nullptr);
// Existing directory must resolve inside /data/navigation/evidence. Writes a
// new role/sequence file without overwriting existing evidence. Throws on error;
// callers report the diagnostic failure without changing runtime qualification.
std::string write_ground_classification_snapshot(const std::string& directory,
  const std::string& role,const ground_relative::ClassifiedObstacleFrame&,
  const ground_relative::GroundReferenceOwner* owner=nullptr);
// Same atomic, no-replace evidence writer; payload is diagnostic-only and is
// serialized by the existing asynchronous observer/timing worker, never the compute callback.
std::string write_airy_geometry_audit_snapshot(const std::string& directory,
    uint64_t sequence,const std::string& bytes);
std::string write_rgbd_projection_audit_snapshot(const std::string& directory,
    const std::string& role,uint64_t sequence,const std::string& bytes,
    const std::string& consumer="global");
}
