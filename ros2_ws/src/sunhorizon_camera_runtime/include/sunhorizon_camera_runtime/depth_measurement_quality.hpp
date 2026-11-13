#pragma once
#include "sunhorizon_camera_runtime/stereo_depth_quality.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
namespace sunhorizon_camera_runtime {
// Rejection evidence, never a validity/free-space certificate. Original depth
// and native sample IDs remain owned by the associated immutable CameraFrame.
enum class DepthMeasurementState : uint8_t {NotRejected=0,Unreliable=1,Unassessed=2,SaturationDominatedComponent=3,StereoMismatch=4};
struct DepthMeasurementQuality {
  using ConstSharedPtr=std::shared_ptr<const DepthMeasurementQuality>;
  std::string algorithm{kStereoQualityAlgorithm};
  std::string role,source_frame,opened_serial,capture_epoch;
  uint64_t source_ns{0},sequence{0};
  const void* image_identity{nullptr};const void* info_identity{nullptr};
  std::size_t width{0},height{0};
  std::array<uint64_t,3> device_us{},global_us{},frame_indices{};
  int depth_pixel_type{-1};double sdk_depth_scale_m{0.};
  bool exact_rectified_pair{false};
  double stereo_baseline_m{0.};
  std::array<double,3> right_from_depth_translation_m{};
  std::array<double,9> rectified_depth_k{};
  bool stereo_body_geometry_verified{false};
  uint64_t geometry_depth_device_us{0},geometry_depth_global_us{0},geometry_depth_frame_index{0};
  double geometry_depth_scale_m{0.};
  std::string stereo_body_geometry_reason{"startup_geometry_unavailable"};
  double body_occlusion_baseline(const std::array<double,9>& k,uint64_t device,uint64_t global,uint64_t index,double scale)const noexcept {
    if(!stereo_body_geometry_verified||rectified_depth_k!=k||depth_pixel_type!=0||
        !device||!global||global>UINT64_MAX/1000||global*1000!=source_ns||
        geometry_depth_device_us!=device||geometry_depth_global_us!=global||geometry_depth_frame_index!=index||
        !std::isfinite(scale)||!(scale>0.)||geometry_depth_scale_m!=scale||sdk_depth_scale_m!=scale||
        !std::isfinite(stereo_baseline_m)||!(stereo_baseline_m>0.)||
        right_from_depth_translation_m[0]!=-stereo_baseline_m||
        right_from_depth_translation_m[1]!=0.||right_from_depth_translation_m[2]!=0.)return 0.;
    return stereo_baseline_m;
  }
  bool component_extension_complete{false};
  std::string component_extension_reason;
  std::size_t component_checks{0};
  bool stereo_match_complete{false};std::string stereo_match_reason;std::size_t stereo_match_checks{0};
  std::string reason{"exact_ir_unavailable"};
  std::vector<uint8_t> pixels;
  bool matches(const std::string& frame,uint64_t ns,uint64_t seq,
      const void* image,const void* info,std::size_t w,std::size_t h)const noexcept {
    if(algorithm!=kStereoQualityAlgorithm||source_frame!=frame||source_ns!=ns||sequence!=seq||image_identity!=image||info_identity!=info||
        !image||!info||width!=w||height!=h||!w||!h||h>4194304/w||pixels.size()!=w*h||
        role!="belly"||opened_serial.empty()||capture_epoch.empty())return false;
    for(const auto value:pixels)if(value>3||(value==3&&(!component_extension_complete||component_extension_reason!="complete"||component_checks<4)))return false;
    // v4 never emits historical stereo-mismatch reason4 or matching work.
    if(stereo_match_checks!=0)return false;
    if(component_checks>65536)return false;
    if(!exact_rectified_pair)return true;
    if(!stereo_match_complete||stereo_match_reason!="disabled_foreground_ambiguity")return false;
    return depth_pixel_type==0&&std::isfinite(sdk_depth_scale_m)&&sdk_depth_scale_m>0.&&device_us[0]&&device_us[0]==device_us[1]&&device_us[0]==device_us[2]&&
      global_us[0]&&global_us[0]==global_us[1]&&global_us[0]==global_us[2]&&
      global_us[0]<=UINT64_MAX/1000&&global_us[0]*1000==source_ns;
  }
  bool unreliable(std::size_t sample)const noexcept {
    const auto value=state(sample);return value==DepthMeasurementState::Unreliable||value==DepthMeasurementState::SaturationDominatedComponent||value==DepthMeasurementState::StereoMismatch;
  }
  DepthMeasurementState state(std::size_t native_sample_id)const noexcept {
    if(algorithm!=kStereoQualityAlgorithm||!exact_rectified_pair||native_sample_id>=pixels.size())return DepthMeasurementState::Unassessed;
    const auto value=pixels[native_sample_id];
    if(value==3&&(!component_extension_complete||component_extension_reason!="complete"||component_checks<4))return DepthMeasurementState::Unassessed;
    if(value==4)return DepthMeasurementState::Unassessed;
    return value<=4?static_cast<DepthMeasurementState>(value):DepthMeasurementState::Unassessed;
  }
};
} // namespace sunhorizon_camera_runtime
