#include "sunhorizon_perception_runtime/perception_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <stdexcept>
#include <utility>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "sunhorizon_rgbd_adapter/spatial_visibility_sampling.hpp"

namespace sunhorizon_perception_runtime {
namespace rgbd = sunhorizon_rgbd_adapter;
namespace {
double thread_cpu_ms() {
  timespec time{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) != 0)
    throw std::runtime_error("perception_thread_cpu_clock_unavailable");
  return double(time.tv_sec) * 1000. + double(time.tv_nsec) / 1000000.;
}
struct Timer {
  std::chrono::steady_clock::time_point wall{std::chrono::steady_clock::now()};
  double cpu{thread_cpu_ms()};
  EngineDuration elapsed() const {
    return {thread_cpu_ms() - cpu,
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-wall).count()};
  }
};
bool rigid(const sv::RigidTransform& tf) {
  for (const auto x : tf.rotation) if (!std::isfinite(x)) return false;
  for (const auto x : tf.translation) if (!std::isfinite(x)) return false;
  const auto& r = tf.rotation;
  for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) {
    double dot=0.;
    for (std::size_t k=0;k<3;++k) dot+=r[k*3+i]*r[k*3+j];
    if (std::abs(dot-(i==j?1.:0.))>1.e-6) return false;
  }
  const double det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return std::isfinite(det) && std::abs(det-1.)<=1.e-6;
}
template<class Stamp> int64_t stamp_ns(const Stamp& stamp) {
  return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
}
void validate_input(const FrameInput& input, const sv::Query& query,bool coverage_enabled) {
  if (!input.image || !input.info) throw std::invalid_argument("missing_perception_image_or_info");
  const auto& id=input.identity;
  const auto& image=*input.image;
  const auto& tf=input.optical_to_common;
  if (id.source_id.empty() || id.epoch.empty() || id.calibration_id.empty() || !id.sequence ||
      id.source_stamp_ros_ns<=0 || id.receipt_ros_ns<=0 || id.receipt_steady_ns<=0 ||
      !sv::supported_evidence_age(id.expiry_budget_ns) ||
      image.header.stamp.nanosec>=1000000000U || input.info->header.stamp.nanosec>=1000000000U ||
      stamp_ns(image.header.stamp)!=id.source_stamp_ros_ns ||
      stamp_ns(input.info->header.stamp)!=id.source_stamp_ros_ns)
    throw std::invalid_argument("invalid_perception_source_identity_or_time");
  const auto contract=std::find_if(query.source_contracts.begin(),query.source_contracts.end(),
      [&](const sv::SourceContract& value){return value.source_id==id.source_id;});
  // An optional camera still computes body output when spatial fusion excludes
  // it. evaluate() continues to reject uncontracted sources if one is supplied.
  if ((contract==query.source_contracts.end() && id.required_for_evaluation) ||
      (contract!=query.source_contracts.end() && (contract->expected_epoch!=id.epoch ||
       contract->required_for_evaluation!=id.required_for_evaluation)))
    throw std::invalid_argument("perception_source_contract_mismatch");
  if (query.common_frame.empty() || query.query_stamp_ns<=0 ||
      tf.source_frame!=image.header.frame_id || tf.common_frame!=query.common_frame ||
      tf.source_stamp_ns!=id.source_stamp_ros_ns || tf.query_stamp_ns!=query.query_stamp_ns || !rigid(tf))
    throw std::invalid_argument("invalid_perception_source_to_query_transform");
  const auto& c=input.calibration;
  if (coverage_enabled && !(c.rectified_pinhole_verified && c.metric_axial_depth_verified &&
        c.depth_intrinsics_registration_verified && c.optical_to_base_extrinsics_verified &&
        c.source_time_and_transform_verified))
    throw std::invalid_argument("unverified_perception_calibration");
}
sv::ReturnKind return_kind(dg::Classification classification) {
  switch (classification) {
    case dg::Classification::SelfSurface: return sv::ReturnKind::kSelfMatched;
    case dg::Classification::ExternalSurface: return sv::ReturnKind::kExternal;
    case dg::Classification::ModelConflict: return sv::ReturnKind::kModelConflict;
    default: throw std::runtime_error("perception_self_classification_unobserved");
  }
}
}  // namespace

sv::Source DecodedSource::source() const {
  auto raster=raster_;
  raster.depth_m=depth_.data();raster.depth_elements=depth_.size();
  raster.return_kinds=kinds_.data();raster.kind_elements=kinds_.size();
  sv::Source result;
  result.source_id=identity_.source_id;result.epoch=identity_.epoch;result.frame_id=frame_id_;
  result.frame_sequence=identity_.sequence;result.source_stamp_ros_ns=identity_.source_stamp_ros_ns;
  result.local_receipt_ros_ns=identity_.receipt_ros_ns;result.local_receipt_steady_ns=identity_.receipt_steady_ns;
  result.expiry_budget_ns=identity_.expiry_budget_ns;result.required_for_evaluation=identity_.required_for_evaluation;
  result.data=raster;
  return result;
}

PerceptionEngine::PerceptionEngine(EngineConfig config):config_(std::move(config)) {
  const auto& c=config_;
  if(c.body_stereo_occlusion_enabled&&c.raw_projection_enabled)
    throw std::invalid_argument("stereo_body_occlusion_requires_raw_disabled");
  if (const auto* error=rgbd::dynamic_filter_budget_error(c.self_tolerance_m,c.body_budget))
    throw std::invalid_argument(error);
  if (!std::isfinite(c.depth_scale_m) || c.depth_scale_m<=0. ||
      !std::isfinite(c.minimum_depth_m) || c.minimum_depth_m<=0. ||
      !std::isfinite(c.maximum_depth_m) || c.maximum_depth_m<=c.minimum_depth_m || !c.body_stride ||
      !std::isfinite(c.clear_margin_m) || c.clear_margin_m<0. ||
      !c.coverage_max_node_visits || c.coverage_max_node_visits>sv::kMaximumChecks ||
      !c.coverage_max_triangle_tests || c.coverage_max_triangle_tests>sv::kMaximumChecks ||
      c.component_minimum_pixels>rgbd::kMaxDepthComponentPixels ||
      c.near_component_minimum_pixels>rgbd::kMaxDepthComponentPixels ||
      (c.raw_projection_enabled && (!c.raw_projection_stride || c.raw_projection_stride>16 ||
       c.raw_projection_max_pair_delta_ns<0 || c.raw_projection_max_pair_delta_ns>5000000 ||
       c.raw_projection_expiry_budget_ns<=0 || c.raw_projection_expiry_budget_ns>1000000000 ||
       (c.raw_projection_use_component_mask && !c.component_minimum_pixels))) ||
      (c.component_minimum_pixels &&
       (!std::isfinite(c.component_neighbor_delta_m) || c.component_neighbor_delta_m<=0. ||
        !std::isfinite(c.component_maximum_depth_m) || c.component_maximum_depth_m<c.minimum_depth_m ||
        c.component_maximum_depth_m>c.maximum_depth_m)) ||
      (c.near_component_minimum_pixels &&
       (!c.component_minimum_pixels || c.near_component_minimum_pixels<c.component_minimum_pixels ||
        !std::isfinite(c.near_component_maximum_depth_m) ||
        c.near_component_maximum_depth_m<c.minimum_depth_m || c.near_component_maximum_depth_m>c.maximum_depth_m)))
    throw std::invalid_argument("invalid_perception_engine_config");
}

namespace {
void validate_original_raw_source(const FrameInput& input,const EngineConfig& c) {
  if(!input.image || !input.info || input.identity.source_id.empty() || input.identity.epoch.empty() ||
     input.identity.calibration_id.empty() || !input.identity.sequence || input.identity.source_stamp_ros_ns<=0 ||
     input.image->header.stamp.nanosec>=1000000000U || input.info->header.stamp.nanosec>=1000000000U ||
     stamp_ns(input.image->header.stamp)!=input.identity.source_stamp_ros_ns ||
     stamp_ns(input.info->header.stamp)!=input.identity.source_stamp_ros_ns ||
     input.identity.receipt_ros_ns<=0 || input.identity.receipt_steady_ns<=0)
    throw std::invalid_argument("raw_projection_original_source_metadata_invalid");
  const auto& image=*input.image;
  if(input.identity.source_id=="chest" && !input.raw_projection_header)
    throw std::invalid_argument("raw_chest_projection_requires_original_color_header");
  const auto header=input.raw_projection_header.value_or(image.header);
  const auto normalized=stamp_ns(header.stamp);
  if(header.frame_id!=image.header.frame_id || header.stamp.nanosec>=1000000000U ||
     normalized<=0 || normalized>input.identity.receipt_ros_ns ||
     input.identity.source_stamp_ros_ns>input.identity.receipt_ros_ns ||
     std::abs(normalized-input.identity.source_stamp_ros_ns)>c.raw_projection_max_pair_delta_ns)
    throw std::invalid_argument("raw_projection_original_pair_gate_failed");
  (void)normalized;
}
bool same_header(const std::optional<std_msgs::msg::Header>& a,
    const std::optional<std_msgs::msg::Header>& b) {
  return a.has_value()==b.has_value() && (!a || (a->frame_id==b->frame_id &&
    a->stamp.sec==b->stamp.sec && a->stamp.nanosec==b->stamp.nanosec));
}
const PreparedQualityMask& checked_preparation(const FrameInput& input,const EngineConfig& c) {
  const auto& q=*input.prepared_quality;const auto& a=q.identity;const auto& b=input.identity;
  if(q.image.get()!=input.image.get() || q.info.get()!=input.info.get() ||
     a.source_id!=b.source_id || a.epoch!=b.epoch || a.calibration_id!=b.calibration_id ||
     a.sequence!=b.sequence || a.source_stamp_ros_ns!=b.source_stamp_ros_ns ||
     a.receipt_ros_ns!=b.receipt_ros_ns || a.receipt_steady_ns!=b.receipt_steady_ns ||
     a.expiry_budget_ns!=b.expiry_budget_ns || a.required_for_evaluation!=b.required_for_evaluation ||
     !same_header(q.raw_projection_header,input.raw_projection_header) ||
     q.measurement_quality.get()!=input.measurement_quality.get() ||
     q.scale!=c.depth_scale_m || q.minimum!=c.minimum_depth_m || q.maximum!=c.maximum_depth_m ||
     q.neighbor_delta!=c.component_neighbor_delta_m || q.minimum_pixels!=c.component_minimum_pixels ||
     q.near_minimum_pixels!=c.near_component_minimum_pixels || q.near_maximum!=c.near_component_maximum_depth_m ||
     q.component_maximum!=c.component_maximum_depth_m || q.body_stride!=c.body_stride || q.raw_stride!=c.raw_projection_stride ||
     q.raw_enabled!=c.raw_projection_enabled || q.raw_use_mask!=c.raw_projection_use_component_mask ||
     q.raw_pair_delta_ns!=c.raw_projection_max_pair_delta_ns || q.raw_expiry_ns!=c.raw_projection_expiry_budget_ns ||
     q.mask_prepared!=(c.raw_projection_enabled && c.raw_projection_use_component_mask))
    throw std::invalid_argument("prepared_quality_original_frame_or_contract_mismatch");
  if(q.mask_prepared && (!q.mask.valid || q.mask.keep.size()!=std::size_t(input.image->width)*input.image->height))
    throw std::invalid_argument("prepared_quality_invalid_mask");
  return q;
}
}  // namespace

void PerceptionEngine::enable_cuda_body_geometry() {
  if(body_geometry_worker_||body_geometry_cuda_)throw std::logic_error("body_geometry_already_enabled");
  body_geometry_cuda_=std::make_unique<rgbd::BodyGeometryCuda>();
}
void PerceptionEngine::request_stop_body_geometry() noexcept {
  if(body_geometry_cuda_)body_geometry_cuda_->request_stop();
}
void PerceptionEngine::enable_parallel_body_geometry(std::function<void()> owner_check) {
  if(body_geometry_worker_||body_geometry_cuda_)throw std::logic_error("body_geometry_already_enabled");
  body_geometry_worker_=std::make_unique<rgbd::BodyGeometryWorker>(std::move(owner_check));
}

std::shared_ptr<const PreparedQualityMask> PerceptionEngine::prepare(const FrameInput& input) {
  const Timer preparation;const auto& c=config_;
  // Use the original source/pair check before any branch can produce output.
  if(c.raw_projection_enabled)validate_original_raw_source(input,c);
  if(!input.image || !input.info)throw std::invalid_argument("missing_perception_image_or_info");
  const auto& id=input.identity;
  if(id.source_id.empty() || id.epoch.empty() || id.calibration_id.empty() || !id.sequence ||
     id.source_stamp_ros_ns<=0 || id.receipt_ros_ns<=0 || id.receipt_steady_ns<=0 ||
     !sv::supported_evidence_age(id.expiry_budget_ns) ||
     input.image->header.stamp.nanosec>=1000000000U || input.info->header.stamp.nanosec>=1000000000U ||
     stamp_ns(input.image->header.stamp)!=id.source_stamp_ros_ns || stamp_ns(input.info->header.stamp)!=id.source_stamp_ros_ns)
    throw std::invalid_argument("invalid_perception_source_identity_or_time");
  const rgbd::DepthProjectionTraversal layout(*input.image,*input.info,c.depth_scale_m,c.minimum_depth_m,c.maximum_depth_m,c.body_stride);
  (void)layout;
  if(input.measurement_quality&&(input.identity.source_id!="belly"||!input.measurement_quality->matches(input.image->header.frame_id,
      id.source_stamp_ros_ns,id.sequence,input.image.get(),input.info.get(),input.image->width,input.image->height)))
    throw std::invalid_argument("measurement_quality_original_frame_mismatch");
  if(input.prepared_quality){(void)checked_preparation(input,c);return input.prepared_quality;}
  auto q=make_preparation(input);q->preparation=preparation.elapsed();return q;
}

std::shared_ptr<PreparedQualityMask> PerceptionEngine::make_preparation(const FrameInput& input) {
  const auto& c=config_;
  auto q=std::make_shared<PreparedQualityMask>();q->image=input.image;q->info=input.info;q->identity=input.identity;
  q->raw_projection_header=input.raw_projection_header;q->measurement_quality=input.measurement_quality;
  if(q->measurement_quality&&(input.identity.source_id!="belly"||!q->measurement_quality->matches(input.image->header.frame_id,
      input.identity.source_stamp_ros_ns,input.identity.sequence,input.image.get(),input.info.get(),input.image->width,input.image->height)))
    throw std::invalid_argument("measurement_quality_original_frame_mismatch");
  q->scale=c.depth_scale_m;q->minimum=c.minimum_depth_m;q->maximum=c.maximum_depth_m;
  q->neighbor_delta=c.component_neighbor_delta_m;q->minimum_pixels=c.component_minimum_pixels;
  q->near_minimum_pixels=c.near_component_minimum_pixels;q->near_maximum=c.near_component_maximum_depth_m;
  q->component_maximum=c.component_maximum_depth_m;q->body_stride=c.body_stride;q->raw_stride=c.raw_projection_stride;
  q->raw_enabled=c.raw_projection_enabled;q->raw_use_mask=c.raw_projection_use_component_mask;
  q->raw_pair_delta_ns=c.raw_projection_max_pair_delta_ns;q->raw_expiry_ns=c.raw_projection_expiry_budget_ns;
  q->mask_prepared=c.raw_projection_enabled && c.raw_projection_use_component_mask;
  if(q->mask_prepared){
    const auto& image=*input.image;
    const auto& mask=rgbd::depth_component_mask({image.data.data(),image.data.size(),image.width,image.height,image.step,
      bool(image.is_bigendian)},c.depth_scale_m,c.minimum_depth_m,c.maximum_depth_m,c.component_neighbor_delta_m,
      c.component_minimum_pixels,component_workspace_,c.near_component_minimum_pixels,c.near_component_maximum_depth_m);
    if(!mask.valid)throw std::runtime_error("raw_projection_component_filter:"+mask.reason);
    q->mask=mask; // One owned immutable copy; this owner's workspace remains private.
  }
  return q;
}

FrameResult PerceptionEngine::project_raw(const FrameInput& input) {
  const Timer total;FrameResult out;
  out.decoded.identity_=input.identity;
  if(input.image)out.decoded.frame_id_=input.image->header.frame_id;
  if(config_.body_stereo_occlusion_enabled)return out; // No same-source model/mask in this independent raw lane.
  if(!config_.raw_projection_enabled)return out; // Preserve the disabled legacy API.
  const auto& c=config_;
  validate_original_raw_source(input,c);
  std::shared_ptr<const PreparedQualityMask> quality=input.prepared_quality;
  if(quality)(void)checked_preparation(input,c);
  else if(c.raw_projection_use_component_mask||input.measurement_quality){
    // Legacy raw-only callers retain their original metadata/pair contract;
    // production uses prepare() before fanout and validates the body identity.
    auto legacy=make_preparation(input);legacy->preparation=total.elapsed();quality=std::move(legacy);
  }
  const std::vector<uint8_t>* keep=nullptr;
  if(quality){
    out.prepared_quality=quality;out.timings.source_prepare=quality->preparation;
    if(c.raw_projection_use_component_mask){keep=&quality->mask.keep;++out.stats.quality_mask_reuses;}
  }
  std::vector<uint8_t> measurement_keep;
  if(input.measurement_quality&&input.measurement_quality->exact_rectified_pair){
    measurement_keep=keep?*keep:std::vector<uint8_t>(std::size_t(input.image->width)*input.image->height,1);
    for(std::size_t i=0;i<measurement_keep.size();++i)if(input.measurement_quality->unreliable(i))measurement_keep[i]=0;
    keep=&measurement_keep;
  }
  const Timer projection;
  auto cloud=rgbd::project_depth(*input.image,*input.info,c.depth_scale_m,c.minimum_depth_m,
    c.maximum_depth_m,c.raw_projection_stride,keep); // Original mask covers ALL depths.
  cloud.header=input.raw_projection_header.value_or(input.image->header);
  out.raw_projection_source_ns=stamp_ns(cloud.header.stamp);
  out.raw_projection_cloud=std::make_shared<const sensor_msgs::msg::PointCloud2>(std::move(cloud));
  out.raw_projection_raw_source_ns=input.identity.source_stamp_ros_ns;
  out.raw_projection_expiry_budget_ns=c.raw_projection_expiry_budget_ns;
  out.raw_projection_complete=true;out.timings.raw_projection=projection.elapsed();
  // Mandatory preparation is a separate stage even for the legacy fallback;
  // consumer totals must not count that same mask CPU a second time.
  out.timings.total=out.timings.raw_projection;
  return out;
}

FrameResult PerceptionEngine::process(const FrameInput& input,
    const std::vector<dg::LinkObservation>& links,const sv::Query& query) {
  const Timer total;
  if(input.measurement_quality&&(input.identity.source_id!="belly"||!input.image||!input.info||!input.measurement_quality->matches(
      input.image->header.frame_id,input.identity.source_stamp_ros_ns,input.identity.sequence,
      input.image.get(),input.info.get(),input.image->width,input.image->height)))
    throw std::invalid_argument("measurement_quality_original_frame_mismatch");
  if (links.size()!=12) throw std::invalid_argument("perception_requires_12_source_time_links");
  const Timer preparation;
  const dg::PreparedModel prepared(links,config_.body_budget.query);
  const auto duration=preparation.elapsed();
  auto result=process(input,prepared,query);
  result.stats.model_preparations=1;result.timings.model_prepare=duration;
  result.timings.total=total.elapsed();
  return result;
}

FrameResult PerceptionEngine::process(const FrameInput& input,
    const dg::PreparedModel& prepared,const sv::Query& query) {
  const Timer total;
  validate_input(input,query,config_.coverage_enabled);
  const auto& image=*input.image;const auto& info=*input.info;const auto& c=config_;
  const double stereo_baseline=c.body_stereo_occlusion_enabled&&input.identity.source_id=="belly"&&input.measurement_quality&&input.measurement_quality->matches(
    image.header.frame_id,input.identity.source_stamp_ros_ns,input.identity.sequence,input.image.get(),input.info.get(),image.width,image.height) ?
    input.measurement_quality->body_occlusion_baseline(info.k,input.sdk_depth_device_us,input.sdk_depth_global_us,input.sdk_depth_frame_index,input.sdk_depth_scale_m):0.;
  // Reuse the projection's complete layout/intrinsics validator before touching
  // image memory. Constructing the traversal does not materialize any XYZ.
  const rgbd::DepthProjectionTraversal validation(image,info,c.depth_scale_m,
      c.minimum_depth_m,c.maximum_depth_m,c.body_stride);
  (void)validation;
  if (!prepared.ready()) throw std::runtime_error(prepared.reason());
  const auto& budget=prepared.query_budget();
  if (budget.max_links!=c.body_budget.query.max_links ||
      budget.max_node_visits!=c.body_budget.query.max_node_visits ||
      budget.max_triangle_tests!=c.body_budget.query.max_triangle_tests)
    throw std::invalid_argument("prepared_model_query_budget_mismatch");
  FrameResult out;out.stats.stereo_body_baseline_m=stereo_baseline;
  if(c.body_stereo_occlusion_enabled)out.stats.stereo_body_geometry_reason=stereo_baseline>0.?"qualified":
    input.measurement_quality ? (input.measurement_quality->stereo_body_geometry_verified?"depth_geometry_binding_rejected":input.measurement_quality->stereo_body_geometry_reason):"depth_geometry_token_missing";
  // Copy only this exact prepared owner, only when diagnostic capture is enabled.
  // Allocation failure cannot change source admission or classification.
  if(input.capture_body_model_diagnostic||input.capture_body_replay_diagnostic)try {
    out.body_model_diagnostic=std::make_shared<const BodyModelDiagnostic>(
      BodyModelDiagnostic{input.identity,image.header.frame_id,c.self_tolerance_m,prepared.query_budget(),prepared.diagnostic_links()});
  }catch(...){}
  const PreparedQualityMask* shared_quality=nullptr;
  if(input.prepared_quality){
    shared_quality=&checked_preparation(input,c);out.prepared_quality=input.prepared_quality;
    out.timings.source_prepare=shared_quality->preparation;
  }
  const Timer quality;
  const std::vector<uint8_t>* keep=nullptr;
  if (c.component_minimum_pixels) {
    const rgbd::DepthComponentMask* mask_pointer=nullptr;
    if(shared_quality && shared_quality->mask_prepared){
      mask_pointer=&shared_quality->mask;++out.stats.quality_mask_reuses;
    }else{
      mask_pointer=&rgbd::depth_component_mask({image.data.data(),image.data.size(),image.width,
        image.height,image.step,bool(image.is_bigendian)},c.depth_scale_m,c.minimum_depth_m,
        c.maximum_depth_m,c.component_neighbor_delta_m,c.component_minimum_pixels,
        component_workspace_,c.near_component_minimum_pixels,c.near_component_maximum_depth_m);
    }
    const auto& mask=*mask_pointer;
    if(!mask.valid)throw std::runtime_error(mask.reason);
    keep=&mask.keep;out.stats.component_removed_pixels=mask.removed_pixels;
    out.stats.component_removed_components=mask.removed_components;
  }
  auto& decoded=out.decoded;
  decoded.identity_=input.identity;decoded.frame_id_=image.header.frame_id;
  auto& raster=decoded.raster_;
  raster.width=image.width;raster.height=image.height;raster.row_stride_elements=image.width;
  raster.fx=info.k[0];raster.fy=info.k[4];raster.cx=info.k[2];raster.cy=info.k[5];
  raster.optical_to_common=input.optical_to_common;raster.calibration=input.calibration;
  raster.minimum_depth_m=c.minimum_depth_m;raster.maximum_depth_m=c.maximum_depth_m;
  raster.clear_margin_m=c.clear_margin_m;
  out.stats.image_pixels=std::size_t(image.width)*image.height;
  auto needed=c.coverage_enabled ? rgbd::spatial_required_pixels(query,raster) :
      std::vector<uint8_t>(out.stats.image_pixels,0);
  decoded.depth_.resize(out.stats.image_pixels);
  // Unqueried pixels stay Invalid: even if a caller later changes its query,
  // an unclassified self return cannot accidentally become free-space support.
  decoded.kinds_.assign(out.stats.image_pixels,sv::ReturnKind::kInvalid);
  for (uint32_t v=0;v<image.height;++v) for (uint32_t u=0;u<image.width;++u) {
    const auto offset=std::size_t(v)*image.step+std::size_t(u)*2,at=std::size_t(v)*image.width+u;
    const uint16_t raw=image.is_bigendian ? (uint16_t(image.data[offset])<<8)|image.data[offset+1] :
        uint16_t(image.data[offset])|(uint16_t(image.data[offset+1])<<8);
    const double z=raw*c.depth_scale_m;
    decoded.depth_[at]=float(z);
    const bool masked=keep && z<=c.component_maximum_depth_m && !(*keep)[at];
    if (masked && raw && z>=c.minimum_depth_m && z<=c.maximum_depth_m) ++out.stats.quality_rejected_pixels;
    if (raw && z>=c.minimum_depth_m && z<=c.maximum_depth_m && !masked &&
        v%c.body_stride==0 && u%c.body_stride==0) ++out.stats.body_projected_pixels;
    if (needed[at]) ++out.stats.coverage_required_pixels;
    // Use the original double axial Z for the component cutoff, exactly like
    // body projection. FLOAT32 depth rounding must not resurrect a rejected
    // pixel on the cutoff boundary as free-space evidence.
    if(input.measurement_quality){
      const auto state=input.measurement_quality->state(at);
      if(state==sunhorizon_camera_runtime::DepthMeasurementState::Unreliable||state==sunhorizon_camera_runtime::DepthMeasurementState::SaturationDominatedComponent||state==sunhorizon_camera_runtime::DepthMeasurementState::StereoMismatch){needed[at]=0;++out.stats.measurement_unreliable_pixels;}
      else if(state==sunhorizon_camera_runtime::DepthMeasurementState::Unassessed)++out.stats.measurement_unassessed_pixels;
    }
    if (masked) needed[at]=0;
  }
  out.timings.quality_decode=quality.elapsed();
  const Timer body;
  auto filtered=rgbd::project_depth_filter_dynamic_self(image,info,c.depth_scale_m,c.minimum_depth_m,
      c.maximum_depth_m,c.body_stride,prepared,c.self_tolerance_m,c.body_budget,keep,
      c.component_minimum_pixels?c.component_maximum_depth_m:std::numeric_limits<double>::infinity(),body_geometry_worker_.get(),body_geometry_cuda_.get(),stereo_baseline);
  out.timings.body_geometry_helper_cpu_ms=filtered.geometry_helper_cpu_ms;
  // Preserve the original BODY result before Ground's independent support/quality
  // handling. No borrowed workspace pointer escapes; allocation failure only
  // discards this optional diagnostic and cannot alter production output.
  if(input.capture_body_replay_diagnostic && out.body_model_diagnostic)try {
    auto replay=std::make_shared<BodyReplayDiagnostic>();
    replay->config=c;replay->model=out.body_model_diagnostic;replay->stereo_baseline_m=stereo_baseline;replay->measurement_quality=input.measurement_quality;
    replay->image=input.image;replay->info=input.info;replay->query_ns=query.query_stamp_ns;
    replay->calibration=input.calibration;
    replay->cuda_backend=bool(body_geometry_cuda_);replay->parallel_cpu=bool(body_geometry_worker_);
    replay->keep_present=keep!=nullptr;if(keep)replay->keep=*keep;
    replay->retained_sample_ids=filtered.retained_sample_ids;
    replay->self_support=filtered.self_support;
    out.body_replay_diagnostic=std::move(replay);
  }catch(...) {}
  out.body_cloud=std::move(filtered.cloud);out.body_filter=std::move(filtered.filter);
  // An all-invalid/masked image has no body cloud but remains valid Unknown
  // coverage. Any genuine model/frame-budget failure revokes the entire frame.
  if (!out.body_filter.complete && out.stats.body_projected_pixels)
    throw std::runtime_error("perception_body_filter_failed:"+out.body_filter.reason);
  out.timings.body_filter=body.elapsed();
  if (c.classified_obstacles_enabled && out.body_filter.complete) {
    const Timer classification;
    // BODY operates in the original optical/sample identity. Acquire the
    // independent map geometry only when Ground needs it, so BODY processing
    // does not consume a frozen live-tracking lease.
    auto ground_transform=input.ground_source_transform;
    auto ground_binding=input.ground_map_binding;
    auto ground_diagnostic=input.ground_transform_diagnostic;
    if(input.ground_owner && input.acquire_ground_geometry) {
      auto batch=input.acquire_ground_geometry();
      if(!batch) {out.ground_query_cancelled=true;return out;}
      ground_transform.reset();
      ground_diagnostic=std::move(batch->diagnostic);
      if(batch->transforms.size()==1)ground_transform=std::move(batch->transforms.front());
      ground_binding=std::move(batch->map_binding);
    }
    ground_relative::SourceFrame source;
    const auto& id=input.identity;
    source.identity.source_id=id.source_id;source.identity.epoch=id.epoch;
    source.identity.calibration_id=id.calibration_id;
    source.identity.source_frame=out.body_cloud.header.frame_id;
    source.identity.common_frame=input.optical_to_common.common_frame;
    source.identity.sequence=id.sequence;source.identity.source_ns=id.source_stamp_ros_ns;
    source.identity.first_receipt_ros_ns=id.receipt_ros_ns;
    source.identity.first_receipt_steady_ns=id.receipt_steady_ns;
    source.identity.expiry_budget_ns=id.expiry_budget_ns;
    source.to_common.rotation=input.optical_to_common.rotation;
    source.to_common.translation=input.optical_to_common.translation;
    source.to_common.source_frame=input.optical_to_common.source_frame;
    source.to_common.common_frame=input.optical_to_common.common_frame;
    source.to_common.source_ns=input.optical_to_common.source_stamp_ns;
    source.to_common.common_ns=input.optical_to_common.query_stamp_ns;
    source.to_common.qualified=input.calibration.source_time_and_transform_verified &&
      input.calibration.optical_to_base_extrinsics_verified;
    if(input.ground_owner){
      source.transform_diagnostic=ground_diagnostic;
      source.to_common=ground_transform.value_or(ground_relative::SourceTimeTransform{});
      source.identity.common_frame=input.ground_owner->reference_frame();
      // These attestations qualify regional free-volume evidence. Measured
      // support uses the separately admitted body projection and original-time
      // registration transform; it does not promote regional flags or bounds.
      // Independently bounded classification retains its existing gate.
      if(!input.ground_owner->observation_only() &&
          !input.ground_owner->supported_segmentation()) {
        const bool calibration_verified=input.calibration.source_time_and_transform_verified&&
          input.calibration.optical_to_base_extrinsics_verified;
        if(source.to_common.qualified&&!calibration_verified)
          source.transform_diagnostic.reason+=";rgbd_regional_calibration_unverified";
        source.to_common.qualified=source.to_common.qualified&&calibration_verified;
      }
    }
    if(input.measurement_quality){
      const auto& q=*input.measurement_quality;auto& proof=source.measurement_quality;
      proof.present=true;proof.exact_rectified_pair=q.exact_rectified_pair;
      proof.algorithm=q.algorithm;proof.role=q.role;proof.source_frame=q.source_frame;
      proof.source_epoch=source.identity.epoch;proof.calibration_id=source.identity.calibration_id;
      proof.opened_serial=q.opened_serial;proof.capture_epoch=q.capture_epoch;proof.reason=q.reason;
      proof.source_ns=q.source_ns;proof.sequence=q.sequence;proof.width=q.width;proof.height=q.height;
      proof.device_us=q.device_us;proof.global_us=q.global_us;proof.frame_indices=q.frame_indices;
      proof.stereo_match_complete=q.stereo_match_complete;proof.stereo_match_reason=q.stereo_match_reason;proof.stereo_match_checks=q.stereo_match_checks;
      proof.component_extension_complete=q.component_extension_complete;proof.component_extension_reason=q.component_extension_reason;proof.component_checks=q.component_checks;
      proof.depth_pixel_type=q.depth_pixel_type;proof.sdk_depth_scale_m=q.sdk_depth_scale_m;
      for(std::size_t i=0;i<q.pixels.size();++i){
        const auto state=q.state(i);
        if(state==sunhorizon_camera_runtime::DepthMeasurementState::StereoMismatch)proof.stereo_mismatch_sample_ids.push_back(i);
        if(state==sunhorizon_camera_runtime::DepthMeasurementState::SaturationDominatedComponent)proof.component_sample_ids.push_back(i);
        if(state==sunhorizon_camera_runtime::DepthMeasurementState::Unreliable||state==sunhorizon_camera_runtime::DepthMeasurementState::SaturationDominatedComponent||state==sunhorizon_camera_runtime::DepthMeasurementState::StereoMismatch)proof.unreliable_sample_ids.push_back(i);
        else if(state==sunhorizon_camera_runtime::DepthMeasurementState::Unassessed)++proof.unassessed_pixels;
      }
    }
    // No independent point/transform error qualification has been supplied by
    // the live owner. In particular, calibration flags are not error bounds.
    source.measurement_qualified=false;
    source.point_error_bound_m=ground_relative::kUnspecified;
    source.transform_translation_error_bound_m=ground_relative::kUnspecified;
    source.transform_angle_error_bound_rad=ground_relative::kUnspecified;
    if (input.ground_measurement_bounds &&
        !(input.ground_owner && input.ground_owner->supported_segmentation())) {
      const auto& bounds=*input.ground_measurement_bounds;
      source.measurement_qualified=!bounds.provenance_id.empty();
      source.point_error_bound_m=bounds.point_m;
      source.transform_translation_error_bound_m=bounds.transform_translation_m;
      source.transform_angle_error_bound_rad=bounds.transform_angle_rad;
    }
    source.raster_geometry={image.width,image.height,c.body_stride};
    if(stereo_baseline>0.&&out.body_filter.complete){
      auto& proof=source.body_occlusion;proof.complete=true;proof.same_sdk_geometry_verified=true;
      proof.identity=source.identity;proof.width=image.width;proof.height=image.height;proof.baseline_m=stereo_baseline;
    }
    source.self_support_complete=true;
    source.original_projected_samples=out.body_filter.input_count;
    source.self_support.reserve(filtered.self_support.size());
    for(const auto& sample:filtered.self_support) {
      if(input.measurement_quality && input.measurement_quality->unreliable(sample.sample_id)) {
        ++source.self_support_quality_rejected_samples;continue;
      }
      ground_relative::PointReturn point;
      point.position={sample.xyz[0],sample.xyz[1],sample.xyz[2]};
      point.kind=ground_relative::ReturnKind::Self;point.sample_id=sample.sample_id;
      source.self_support.push_back(point);
    }
    source.points.reserve(out.body_filter.retained_count);
    if(filtered.retained_sample_ids.size()!=out.body_filter.retained_count ||
        out.body_filter.ground_blocked.size()!=out.body_filter.retained_count ||
        out.body_filter.nominal_model_conflict.size()!=out.body_filter.retained_count)
      throw std::runtime_error("classified_body_retained_evidence_layout_mismatch");
    if(out.body_filter.retained_count) {
    sensor_msgs::PointCloud2ConstIterator<float> x(out.body_cloud,"x"),y(out.body_cloud,"y"),z(out.body_cloud,"z");
    std::size_t point_index=0;
    for(;x!=x.end();++x,++y,++z,++point_index) {
      ground_relative::PointReturn point{{*x,*y,*z},ground_relative::ReturnKind::Measured};
      point.ground_blocked=out.body_filter.ground_blocked[point_index];
      // A complete finite-surface ambiguity remains blocked on its own ray.
      // It does not invalidate independently completed nominal-conflict rays;
      // unexplained Unobserved or any Invalid still revokes reconciliation.
      using ModelClass=sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification;
      point.model_conflict=point.ground_blocked&&out.body_filter.nominal_model_conflict[point_index]&&out.body_filter.complete&&
        out.body_filter.classifications[static_cast<std::size_t>(ModelClass::Invalid)]==0&&
        out.body_filter.classifications[static_cast<std::size_t>(ModelClass::Unobserved)]==
          out.body_filter.measured_reference_ambiguous_count;
      point.sample_id=filtered.retained_sample_ids[point_index];
      if(input.measurement_quality&&input.measurement_quality->unreliable(point.sample_id)){
        point.measurement_unreliable=true;
      }
      // IR retains its original independently proven rejection path, even if
      // the BODY query also found right-eye occlusion. Never rescue a bad IR
      // proof through a different provenance or relabel it as a body proof.
      if(out.body_filter.ground_blocked[point_index]==3&&!point.measurement_unreliable){
        point.measurement_body_occluded=true;point.kind=ground_relative::ReturnKind::Invalid;
        source.body_occlusion.sample_ids.push_back(point.sample_id);
      }
      // A proven measurement rejection keeps its existing Invalid/proof path.
      // A model-qualified body edge is Self. Keep its raw record and edge
      // provenance: the mixed return's depth is not a measured body surface,
      // so it must not enter self_support or acquire endpoint/clearing rights.
      if(out.body_filter.ground_blocked[point_index]==2&&!point.measurement_unreliable)
        point.kind=ground_relative::ReturnKind::Self;
      source.points.push_back(point);
    }
    }
    if(input.ground_owner){
      out.classified_obstacles=input.ground_owner->classify(std::move(source),ground_binding);
    }else if (input.ground_reference) {
      if (!input.ground_classifier_config)
        throw std::invalid_argument("ground_reference_requires_explicit_classifier_contract");
      out.classified_obstacles=ground_relative::classify_obstacle_frame(
        std::move(source),*input.ground_classifier_config,input.ground_reference);
    } else {
      out.classified_obstacles=ground_relative::conservative_unqualified_frame(std::move(source));
    }
    out.apply_near_body_mask_to_collision=c.body_proximity_filter_enabled;
    if(c.body_proximity_filter_enabled || c.map_body_proximity_filter_enabled) {
      const auto& frame=*out.classified_obstacles;
      const auto& points=frame.source().points;
      for(std::size_t i=0;i<frame.classes().size();++i) {
        const auto cls=frame.classes()[i];
        if(cls!=ground_relative::PointClass::Obstacle && cls!=ground_relative::PointClass::Uncertain) continue;
        const auto& p=points[i].position;
        if(prepared.excludes_near_body_map_mark({p[0],p[1],p[2]})) out.near_body_indices.push_back(i);
      }
    }
    out.obstacle_input_health=registration_ground_input_health(
      input.ground_owner && input.ground_owner->registration_map_asset(),
      out.classified_obstacles->ground().transform_diagnostic,out.classified_obstacles->ground().result);
    out.timings.obstacle_classify=classification.elapsed();
  }
  std::vector<uint8_t> near_needed;
  // Ground queries are in their independent fixed reference frame. Query each
  // projection separately, then share the same pixel quality and prepared FK
  // model; never concatenate cells from different frames or resurrect masks.
  if(out.classified_obstacles && input.ground_owner && !input.ground_owner->supported_segmentation() &&
      input.ground_owner->clearance_grid()) {
    const auto extra=near_ground_clearance::make_query(*out.classified_obstacles,
      input.ground_owner->config(),*input.ground_owner->clearance_grid());
    if(!extra.cells.empty()) {
      auto ground_raster=raster;const auto& tf=out.classified_obstacles->source().to_common;
      auto& view=ground_raster.optical_to_common;
      view.rotation=tf.rotation;view.translation=tf.translation;view.source_frame=tf.source_frame;
      view.common_frame=tf.common_frame;view.source_stamp_ns=tf.source_ns;view.query_stamp_ns=tf.common_ns;
      const auto extra_pixels=rgbd::spatial_required_pixels(extra,ground_raster);
      near_needed.assign(out.stats.image_pixels,0);
      for(uint32_t v=0;v<image.height;++v)for(uint32_t u=0;u<image.width;++u) {
        const auto at=std::size_t(v)*image.width+u;
        if(!extra_pixels[at]||needed[at])continue;
        if(input.measurement_quality&&input.measurement_quality->unreliable(at))continue;
        const auto offset=std::size_t(v)*image.step+std::size_t(u)*2;
        const uint16_t raw=image.is_bigendian ? (uint16_t(image.data[offset])<<8)|image.data[offset+1] :
          uint16_t(image.data[offset])|(uint16_t(image.data[offset+1])<<8);
        if(keep && raw*c.depth_scale_m<=c.component_maximum_depth_m && !(*keep)[at])continue;
        near_needed[at]=1;
      }
    }
  }
  const Timer coverage;
  const bool native_raster=rgbd::qualified_native_depth_raster(image,info,c.depth_scale_m);
  dg::NativePixelSupport coverage_pixel;
  if(native_raster)coverage_pixel={image.width,image.height,0,0,info.k[0],info.k[4],info.k[2],info.k[5],stereo_baseline};
  const auto classify=[&](const dg::Vec3& ray,double z,const dg::NativePixelSupport* pixel=nullptr) {
    auto value=dg::classify_body_surface(ray,z,c.self_tolerance_m,prepared);
    if(pixel){value=dg::apply_native_projection_edge_policy(ray,z,c.self_tolerance_m,prepared,*pixel,value);
      value=dg::apply_stereo_body_occlusion_policy(ray,z,c.self_tolerance_m,prepared,*pixel,value);}
    if (!value.complete || value.node_visits>c.coverage_max_node_visits-out.stats.coverage_node_visits ||
        value.triangle_tests>c.coverage_max_triangle_tests-out.stats.coverage_triangle_tests)
      throw std::runtime_error("perception_coverage_classification_incomplete_or_budget");
    out.stats.coverage_node_visits+=value.node_visits;
    out.stats.coverage_triangle_tests+=value.triangle_tests;++out.stats.coverage_queries;
    return (dg::projection_edge_suppressed(value)||dg::measurement_body_occluded(value))?sv::ReturnKind::kInvalid:return_kind(value.classification);
  };
  // Body's constructor already validates the model even with invalid XYZ.
  // When there are no projected points, still qualify all mesh revisions/TF.
  if (!out.body_filter.complete) (void)classify({0.,0.,1.},1.);
  for (uint32_t v=0;v<image.height;++v) for (uint32_t u=0;u<image.width;++u) {
    const auto at=std::size_t(v)*image.width+u;
    const float z=decoded.depth_[at];
    if (!needed[at] || !std::isfinite(z) || z<c.minimum_depth_m || z>c.maximum_depth_m || z<=0.) continue;
    // Deliberately keep the old spatial pixel ray and FLOAT32 axial depth.
    // Body uses FLOAT32 XYZ readback; unequal rays must not share a result.
    coverage_pixel.u=u;coverage_pixel.v=v;
    decoded.kinds_[at]=classify({(u-raster.cx)/raster.fx,(v-raster.cy)/raster.fy,1.},z,native_raster?&coverage_pixel:nullptr);
  }
  // Original R has first use of the existing geometry budget. Optional
  // near-ground work may exhaust only the remainder; incomplete pixels stay
  // Invalid and cannot revoke an obstacle. It cannot starve or fault old R.
  bool near_budget_exhausted=false;
  for(uint32_t v=0;v<image.height && !near_needed.empty() && !near_budget_exhausted;++v)
    for(uint32_t u=0;u<image.width;++u) {
      const auto at=std::size_t(v)*image.width+u;const float z=decoded.depth_[at];
      if(!near_needed[at] || !std::isfinite(z) || z<c.minimum_depth_m || z>c.maximum_depth_m || z<=0.)continue;
      coverage_pixel.u=u;coverage_pixel.v=v;
      const auto ray=dg::Vec3{(u-raster.cx)/raster.fx,(v-raster.cy)/raster.fy,1.};
      auto result=dg::classify_body_surface(ray,z,c.self_tolerance_m,prepared);
      if(native_raster){result=dg::apply_native_projection_edge_policy(ray,z,c.self_tolerance_m,prepared,coverage_pixel,result);
        result=dg::apply_stereo_body_occlusion_policy(ray,z,c.self_tolerance_m,prepared,coverage_pixel,result);}
      if(!result.complete || result.node_visits>c.coverage_max_node_visits-out.stats.coverage_node_visits ||
          result.triangle_tests>c.coverage_max_triangle_tests-out.stats.coverage_triangle_tests) {
        near_budget_exhausted=true;break;
      }
      out.stats.coverage_node_visits+=result.node_visits;out.stats.coverage_triangle_tests+=result.triangle_tests;
      ++out.stats.coverage_queries;++out.stats.coverage_required_pixels;
      decoded.kinds_[at]=(dg::projection_edge_suppressed(result)||dg::measurement_body_occluded(result))?sv::ReturnKind::kInvalid:return_kind(result.classification);
    }
  out.timings.coverage_classify=coverage.elapsed();out.timings.total=total.elapsed();
  return out;
}

EvaluationResult PerceptionEngine::evaluate(const sv::Query& query,
    const std::vector<const DecodedSource*>& decoded,int64_t now_ros_ns,int64_t now_steady_ns) {
  const Timer timer;
  std::vector<sv::Source> sources;sources.reserve(decoded.size());
  for (const auto* source:decoded) {
    if (!source) throw std::invalid_argument("null_decoded_perception_source");
    sources.push_back(source->source());
  }
  EvaluationResult out;out.visibility=sv::evaluate(query,sources,now_ros_ns,now_steady_ns);
  out.timing=timer.elapsed();return out;
}
}  // namespace sunhorizon_perception_runtime
