#pragma once
#include <sunhorizon_localization_manager/localization_direct_runtime.hpp>
#include <sunhorizon_perception_runtime/registration_ground_geometry.hpp>
#include <sunhorizon_rgbd_adapter/dynamic_self_model.hpp>

namespace sunhorizon_navigation_runtime::detail {
// One adapter shared by both camera exposures and Airy's scan-time groups.
// Localization owns full 6D/history/reset; perception retains the unique FK.
// The only TF lookup is Airy<-sensor within that sensor subtree, at source time.
class RegistrationGroundGeometryAdapter final {
 public:
  RegistrationGroundGeometryAdapter(
      std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime> localization,
      std::shared_ptr<tf2_ros::Buffer> transforms,std::string actual_map_path,
      sunhorizon_perception_runtime::ground_relative::RegistrationMapIdentity map_identity)
    :localization_(std::move(localization)),transforms_(std::move(transforms)),
     map_path_(std::move(actual_map_path)),map_identity_(std::move(map_identity)) {}
  sunhorizon_perception_runtime::RegistrationGroundGeometryBatch operator()(
      const sunhorizon_perception_runtime::ground_relative::SourceIdentity& identity,
      const std::vector<int64_t>& times,bool airy_scan) {
    namespace gr=sunhorizon_perception_runtime::ground_relative;
    namespace lm=sunhorizon_localization_manager;
    namespace dm=sunhorizon_rgbd_adapter::dynamic_self_model;
    sunhorizon_perception_runtime::RegistrationGroundGeometryBatch out;
    std::optional<sunhorizon_3d_localizer::NdtDirectStamp> scan;
    if(airy_scan)scan=sunhorizon_3d_localizer::NdtDirectStamp{identity.source_ns,
      identity.first_receipt_ros_ns,identity.first_receipt_steady_ns,identity.sequence,identity.source_id};
    const auto batch=localization_->lookup_pcd_from_airy_geometry_times(times,scan);
    out.diagnostic={static_cast<int>(batch.state),batch.reason};
    out.reset_revoked=batch.reset_revoked;out.reset_barrier_steady_ns=batch.reset_barrier_steady_ns;
    const auto unavailable=[&](const std::string& reason) {
      out.transforms.clear();out.map_binding={};out.diagnostic.reason=reason;return out;
    };
    if(identity.first_receipt_steady_ns<=batch.reset_barrier_steady_ns)
      return unavailable("registration_geometry_source_before_reset");
    if(batch.state!=lm::PcdGeometryState::Ready)return out;
    if(batch.geometries.size()!=times.size() || times.empty())
      return unavailable("registration_geometry_batch_size_mismatch");
    const auto& first=batch.context;
    if(!first || !first->anchor || !first->anchor->map_to_source ||
        first->registration_map_path!=map_path_ || !first->reset_revoked ||
        first->reset_revoked->load(std::memory_order_acquire))
      return unavailable("registration_geometry_map_or_reset_identity_mismatch");
    const auto until_ros=first->geometry_valid_until_ros_ns;
    const auto until_steady=first->geometry_valid_until_steady_ns;
    out.transforms.reserve(times.size());
    for(std::size_t i=0;i<times.size();++i) {
      const auto& point=batch.geometries[i];
      if(point.source_ns!=times[i])return unavailable("registration_geometry_batch_identity_mismatch");
      auto geometry=dm::rigid(point.pcd_from_airy);
      const auto& airy=first->anchor->map_to_source->child_frame_id;
      if(identity.source_frame!=airy) {
        try {
          const auto relative=transforms_->lookupTransform(airy,identity.source_frame,
            rclcpp::Time(times[i],localization_->node()->get_clock()->get_clock_type()),rclcpp::Duration::from_seconds(0.));
          geometry=dm::multiply(geometry,dm::rigid(relative.transform));
        }catch(const tf2::TransformException& error){
          return unavailable("registration_geometry_airy_from_source_tf_unavailable:"+
            std::string(error.what()).substr(0,384));
        }
      }
      gr::SourceTimeTransform transform;
      transform.rotation=geometry.rotation;
      transform.translation={geometry.translation.x,geometry.translation.y,geometry.translation.z};
      transform.source_frame=identity.source_frame;transform.common_frame="registration_map";
      transform.source_ns=transform.common_ns=times[i];transform.qualified=true;
      out.transforms.push_back(std::move(transform));
    }
    auto alignment=std::make_shared<const gr::RegistrationMapAlignment>(map_identity_,
      first->reset_epoch,first->reset_revoked,until_ros,until_steady);
    if(!alignment->current())return unavailable("registration_geometry_alignment_revoked");
    out.map_binding={std::move(alignment),identity.source_id,identity.epoch,identity.source_ns};
    return out;
  }
 private:
  std::shared_ptr<sunhorizon_localization_manager::LocalizationDirectRuntime> localization_;
  std::shared_ptr<tf2_ros::Buffer> transforms_;
  const std::string map_path_;
  const sunhorizon_perception_runtime::ground_relative::RegistrationMapIdentity map_identity_;
};
} // namespace sunhorizon_navigation_runtime::detail
