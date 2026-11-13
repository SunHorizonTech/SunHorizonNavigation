#pragma once

#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"
#include "sunhorizon_perception_runtime/near_ground_clearance.hpp"
#include <map>

namespace sunhorizon_perception_runtime::ground_relative {
// Immutable, pinned independent evidence. No clock read, per-frame fitting,
// reference renewal, or physical parameter fallback occurs in this owner.
class GroundReferenceOwner final {
 public:
  using ConstSharedPtr=std::shared_ptr<const GroundReferenceOwner>;
  // actual_registration_map must come from hashing the loaded NDT PCD and its
  // registration datum, never by copying the expected identity from this JSON.
  // Static assets have no boot/receipt clock; per-frame alignment is separate.
  static ConstSharedPtr load(const std::string& absolute_file,const std::string& sha256,
      const std::string& current_boot_id,
      const std::optional<RegistrationMapIdentity>& actual_registration_map=std::nullopt);
  const std::string& reference_frame() const noexcept {return reference_.common_frame;}
  bool observation_only() const noexcept {return observation_only_;}
  bool supported_segmentation() const noexcept {return supported_segmentation_;}
  const SegmentationConfig& segmentation_config() const noexcept {return segmentation_config_;}
  const ClassifierConfig& config() const noexcept {return config_;}
  const std::optional<near_ground_clearance::TileGrid>& clearance_grid() const noexcept {return clearance_grid_;}
  const std::string& clock_boot_id() const noexcept {return clock_boot_id_;}
  bool registration_map_asset() const noexcept {
    return reference_.lifetime==GroundReference::Lifetime::RegistrationMapAsset;
  }
  const RegistrationMapIdentity& map_identity() const noexcept {return reference_.map_asset;}
  // to_common is reference_frame <- original sensor at identity.source_ns;
  // common_ns must equal source_ns. Failed/missing TF stays unqualified.
  // Original source/receipt clocks and source-frame points are retained.
  ClassifiedObstacleFrame::ConstSharedPtr classify(SourceFrame,
      RegistrationMapSourceBinding={}) const;
 private:
  struct Binding {
    std::string epoch,calibration_id,source_frame,measurement_provenance_id;
    bool measurement_qualified{false},transform_qualified{false};
    double point_error_m{kUnspecified},translation_error_m{kUnspecified},angle_error_rad{kUnspecified};
  };
  GroundReferenceOwner()=default;
  GroundReference reference_;
  ClassifierConfig config_;
  SegmentationConfig segmentation_config_;
  std::map<std::string,Binding> bindings_;
  bool observation_only_{true};
  bool supported_segmentation_{false};
  std::optional<near_ground_clearance::TileGrid> clearance_grid_;
  std::string clock_boot_id_;
};
}
