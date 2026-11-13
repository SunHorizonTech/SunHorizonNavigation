#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace sunhorizon_perception_runtime::ground_relative {
namespace {
bool finite(double value) {return std::isfinite(value);}
bool finite_point(const std::array<double,3>& p) {
  return finite(p[0])&&finite(p[1])&&finite(p[2]);
}
bool text(const std::string& value,std::size_t maximum=128) {
  return !value.empty()&&value.size()<=maximum&&std::all_of(value.begin(),value.end(),
    [](unsigned char c){return c>=33&&c<=126;});
}
void require_identity(const SourceIdentity& id) {
  if(!text(id.source_id)||!text(id.epoch)||(!id.calibration_id.empty()&&!text(id.calibration_id,16384))||
    !text(id.source_frame)||!text(id.common_frame)||!id.sequence||id.source_ns<=0||
    id.first_receipt_ros_ns<id.source_ns||id.first_receipt_steady_ns<=0||
    id.expiry_budget_ns<=0||id.raw_source_ns<0||id.normalization_start_steady_ns<0||
    id.normalization_end_steady_ns<id.normalization_start_steady_ns||id.normalization_thread_cpu_ns< -1)
    throw std::invalid_argument("classified_obstacle_original_source_identity_invalid");
}
constexpr double pi=3.14159265358979323846;
double norm(const std::array<double,3>& p) {return std::hypot(p[0],p[1],p[2]);}
bool nonnegative(double value) {return finite(value)&&value>=0.;}
bool parameters_specified(const ClassifierConfig& c,const SourceFrame& source,const GroundReference& ref) {
  const double values[]={c.ground_band_min_m,c.ground_band_max_m,c.maximum_support_residual_m,
    c.maximum_point_error_m,c.maximum_transform_translation_error_m,c.maximum_transform_angle_error_rad,
    c.maximum_plane_offset_error_m,c.maximum_plane_angle_error_rad,c.maximum_normal_tilt_rad,
    c.rotation_tolerance,c.normal_norm_tolerance,c.minimum_support_span_x_m,
    c.minimum_support_span_y_m,c.minimum_support_double_area_m2,source.point_error_bound_m,
    source.transform_translation_error_bound_m,source.transform_angle_error_bound_rad,
    ref.plane_offset_error_bound_m,ref.plane_angle_error_bound_rad};
  return std::all_of(std::begin(values),std::end(values),finite)&&c.maximum_points&&
    c.maximum_support_cells&&c.maximum_support_checks&&c.minimum_samples_per_cell&&
    c.minimum_training_cells&&c.minimum_holdout_cells&&c.continuous_support_radius_cells;
}
void require_config(const ClassifierConfig& c) {
  const double nonnegative_values[]={c.maximum_support_residual_m,c.maximum_point_error_m,
    c.maximum_transform_translation_error_m,c.maximum_transform_angle_error_rad,
    c.maximum_plane_offset_error_m,c.maximum_plane_angle_error_rad,c.maximum_normal_tilt_rad};
  for(const auto value:nonnegative_values)
    if(!nonnegative(value))throw std::invalid_argument("ground_classifier_explicit_bounds_required");
  if(!finite(c.ground_band_min_m)||!finite(c.ground_band_max_m)||
    c.ground_band_min_m>0.||c.ground_band_max_m<0.||c.ground_band_min_m>=c.ground_band_max_m||
    c.maximum_transform_angle_error_rad>pi||c.maximum_plane_angle_error_rad>pi||
    c.maximum_normal_tilt_rad>=pi/2.||!finite(c.rotation_tolerance)||
    c.rotation_tolerance<=0.||c.rotation_tolerance>1.e-6||!finite(c.normal_norm_tolerance)||
    c.normal_norm_tolerance<=0.||c.normal_norm_tolerance>1.e-6||
    !finite(c.minimum_support_span_x_m)||c.minimum_support_span_x_m<=0.||
    !finite(c.minimum_support_span_y_m)||c.minimum_support_span_y_m<=0.||
    !finite(c.minimum_support_double_area_m2)||c.minimum_support_double_area_m2<=0.||
    !c.minimum_samples_per_cell||c.minimum_training_cells<3||c.minimum_holdout_cells<3||
    !c.continuous_support_radius_cells||c.continuous_support_radius_cells>32||
    !c.maximum_points||c.maximum_points>kMaximumClassifiedPoints||
    c.minimum_samples_per_cell>c.maximum_points||!c.maximum_support_cells||
    c.maximum_support_cells>kMaximumSupportCells||!c.maximum_support_checks||
    c.maximum_support_checks>16*1024*1024)
    throw std::invalid_argument("ground_classifier_configuration_invalid");
}
bool valid_rotation(const SourceTimeTransform& transform,double tolerance) {
  const auto& r=transform.rotation;
  if(!finite_point(transform.translation))return false;
  for(const auto value:r)if(!finite(value))return false;
  for(std::size_t a=0;a<3;++a)for(std::size_t b=0;b<3;++b) {
    double value=0.;for(std::size_t k=0;k<3;++k)value+=r[3*k+a]*r[3*k+b];
    if(!finite(value)||std::abs(value-(a==b?1.:0.))>tolerance)return false;
  }
  const auto determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+
    r[2]*(r[3]*r[7]-r[4]*r[6]);
  return finite(determinant)&&std::abs(determinant-1.)<=tolerance;
}
bool valid_source_transform(const SourceTimeTransform& tf,const SourceIdentity& id,
    int64_t source_ns,double tolerance) {
  return tf.qualified&&tf.source_frame==id.source_frame&&tf.common_frame==id.common_frame&&
    tf.source_ns==source_ns&&tf.common_ns==source_ns&&valid_rotation(tf,tolerance);
}
bool valid_transform_groups(const SourceFrame& source,double tolerance) {
  int64_t previous=0;
  for(const auto& tf:source.source_time_transforms) {
    if(tf.source_ns<=previous||!valid_source_transform(tf,source.identity,tf.source_ns,tolerance))return false;
    previous=tf.source_ns;
  }
  return true;
}
const SourceTimeTransform* point_transform(const SourceFrame& source,const PointReturn& point) {
  if(point.source_offset_ns==0)return &source.to_common;
  const auto stamp=source.identity.source_ns,offset=point.source_offset_ns;
  if((offset>0&&stamp>std::numeric_limits<int64_t>::max()-offset)||
    (offset<0&&offset<=-stamp))return nullptr;
  const auto point_ns=stamp+offset;
  const auto& groups=source.source_time_transforms;
  const auto found=std::lower_bound(groups.begin(),groups.end(),point_ns,
    [](const SourceTimeTransform& tf,int64_t ns){return tf.source_ns<ns;});
  return found!=groups.end()&&found->source_ns==point_ns?&*found:nullptr;
}
std::array<double,3> transform_point(const SourceTimeTransform& transform,const std::array<double,3>& p) {
  std::array<double,3> result=transform.translation;
  for(std::size_t row=0;row<3;++row)
    for(std::size_t column=0;column<3;++column)result[row]+=transform.rotation[3*row+column]*p[column];
  return result;
}
std::optional<SupportCell> point_cell(const SupportGrid& grid,const std::array<double,3>& p) {
  const auto upper_x=grid.origin_x_m+grid.resolution_m*grid.width;
  const auto upper_y=grid.origin_y_m+grid.resolution_m*grid.height;
  if(p[0]<grid.origin_x_m||p[1]<grid.origin_y_m||p[0]>=upper_x||p[1]>=upper_y)return std::nullopt;
  const auto x=std::floor((p[0]-grid.origin_x_m)/grid.resolution_m);
  const auto y=std::floor((p[1]-grid.origin_y_m)/grid.resolution_m);
  if(!finite(x)||!finite(y)||x<0.||y<0.||x>=grid.width||y>=grid.height)return std::nullopt;
  return SupportCell{static_cast<uint32_t>(x),static_cast<uint32_t>(y)};
}
uint64_t cell_key(const SupportGrid& grid,const SupportCell& cell) {
  return static_cast<uint64_t>(cell.y)*grid.width+cell.x;
}
using SegmentationLatticeCell=std::pair<int64_t,int64_t>; // y, x

// Ordered iteration is unchanged. Raster-adjacent points often revisit the
// same cell; memoize only the latest exact lookup, including an absent cell.
template<class Value> class RepeatedLatticeMap : public std::map<SegmentationLatticeCell,Value> {
  using Base=std::map<SegmentationLatticeCell,Value>;
  std::optional<SegmentationLatticeCell> key_;
  typename Base::iterator found_;
public:
  RepeatedLatticeMap()=default;
  RepeatedLatticeMap(const RepeatedLatticeMap& other):Base(other){}
  RepeatedLatticeMap& operator=(const RepeatedLatticeMap&)=delete;
  void clear(){Base::clear();key_.reset();}
  typename Base::iterator find(const SegmentationLatticeCell& key) {
    if(!key_||*key_!=key){key_=key;found_=Base::find(key);}
    return found_;
  }
  std::size_t count(const SegmentationLatticeCell& key){return find(key)!=Base::end();}
  std::pair<typename Base::iterator,bool> emplace(const SegmentationLatticeCell& key,const Value& value) {
    if(key_&&*key_==key&&found_!=Base::end())return {found_,false};
    auto result=Base::emplace(key,value);key_=key;found_=result.first;return result;
  }
  Value& operator[](const SegmentationLatticeCell& key) {
    const auto at=find(key);if(at!=Base::end())return at->second;
    auto result=Base::try_emplace(key);found_=result.first;return found_->second;
  }
};

bool segmentation_holdout(uint64_t identity) {
  identity+=0x9e3779b97f4a7c15ULL;
  identity=(identity^(identity>>30))*0xbf58476d1ce4e5b9ULL;
  identity=(identity^(identity>>27))*0x94d049bb133111ebULL;
  return ((identity^(identity>>31))&1U)!=0;
}
double segmentation_median(std::vector<double> values) {
  const auto middle=values.size()/2;
  // Preserve the old ordering for signed-zero ties and nonfinite inputs.
  // All other finite equal
  // values have identical bits; selection returns the same order statistics.
  bool positive_zero=false,negative_zero=false,nonfinite=false;
  for(const auto value:values) {
    nonfinite=nonfinite||!std::isfinite(value);
    if(value==0.){if(std::signbit(value))negative_zero=true;else positive_zero=true;}
  }
  if(nonfinite||(positive_zero&&negative_zero)) {
    std::sort(values.begin(),values.end());
    return values.size()%2?values[middle]:values[middle-1]+(values[middle]-values[middle-1])/2.;
  }
  std::nth_element(values.begin(),values.begin()+middle,values.end());
  if(values.size()%2)return values[middle];
  const auto lower=*std::max_element(values.begin(),values.begin()+middle);
  return lower+(values[middle]-lower)/2.;
}
double segmentation_mad(const std::vector<double>& values,double center) {
  std::vector<double> deviations;deviations.reserve(values.size());
  for(const auto value:values)deviations.push_back(std::abs(value-center));
  return segmentation_median(std::move(deviations));
}
GroundQualification::MeasuredPlaneFit measure_ground_plane(
    const SourceFrame& source,const SegmentationConfig& config,const GroundReference& seed,
    const std::vector<std::optional<SegmentationLatticeCell>>& lattice,
    const std::vector<std::array<double,2>>& xy,const std::vector<double>& heights,std::size_t& checks,
    RepeatedLatticeMap<bool>& measured_cells) {
  using Fit=GroundQualification::MeasuredPlaneFit;using Result=Fit::Result;
  Fit out;out.result=Result::InsufficientTrainingGeometry;
  const auto spend=[&] {if(checks>=config.maximum_support_checks){out.result=Result::BudgetExceeded;return false;}
    ++checks;return true;};
  struct Samples {std::vector<double> x,y,h;std::vector<std::array<double,3>> holdout;};
  RepeatedLatticeMap<Samples> samples;
  for(std::size_t i=0;i<lattice.size();++i) {
    if(!lattice[i]||source.points[i].measurement_unreliable||source.points[i].ground_blocked||std::abs(heights[i])>config.maximum_point_residual_m)continue;
    if(!spend())return out;
    auto& cell=samples[*lattice[i]];const auto id=source.points[i].sample_id;
    if(segmentation_holdout(id==std::numeric_limits<uint64_t>::max()?uint64_t(i):id)) {
      cell.holdout.push_back({xy[i][0],xy[i][1],heights[i]});++out.holdout_points;
    }else {cell.x.push_back(xy[i][0]);cell.y.push_back(xy[i][1]);cell.h.push_back(heights[i]);++out.training_points;}
  }
  // Each observed training cell has equal weight, regardless of pixel density.
  // Holdout values never enter the coefficient solve or training rejection.
  std::vector<std::array<double,3>> centers;
  for(const auto& entry:samples)if(entry.second.h.size()>=config.minimum_samples_per_partition) {
    if(!spend())return out;const auto& cell=entry.second;
    centers.push_back({segmentation_median(cell.x),segmentation_median(cell.y),segmentation_median(cell.h)});
  }
  out.training_cells=centers.size();if(centers.size()<config.minimum_supported_cells)return out;
  std::vector<double> xs,ys;for(const auto& p:centers){xs.push_back(p[0]);ys.push_back(p[1]);}
  const std::array<double,2> center{segmentation_median(xs),segmentation_median(ys)};
  const auto scale=std::max(*std::max_element(xs.begin(),xs.end())-*std::min_element(xs.begin(),xs.end()),
    *std::max_element(ys.begin(),ys.end())-*std::min_element(ys.begin(),ys.end()));
  if(!finite(scale)||scale<=0.)return out;
  std::vector<bool> retained(centers.size(),true);std::array<double,3> coefficient{};
  const auto solve=[&] {
    std::array<std::array<double,4>,3> matrix{};
    for(std::size_t i=0;i<centers.size();++i)if(retained[i]) {
      if(!spend())return false;
      const auto& p=centers[i];const std::array<double,3> row{(p[0]-center[0])/scale,(p[1]-center[1])/scale,1.};
      for(std::size_t k=0;k<3;++k){for(std::size_t l=0;l<3;++l)matrix[k][l]+=row[k]*row[l];matrix[k][3]+=row[k]*p[2];}
    }
    // Centering/scaling keeps this small three-parameter solve well conditioned.
    // A rank-deficient geometry is not a fitted surface.
    double largest=0.;for(const auto& row:matrix)for(std::size_t k=0;k<3;++k)largest=std::max(largest,std::abs(row[k]));
    for(std::size_t k=0;k<3;++k) {
      auto pivot=k;for(std::size_t j=k+1;j<3;++j)if(std::abs(matrix[j][k])>std::abs(matrix[pivot][k]))pivot=j;
      if(!finite(matrix[pivot][k])||std::abs(matrix[pivot][k])<=64.*std::numeric_limits<double>::epsilon()*largest)return false;
      std::swap(matrix[k],matrix[pivot]);const auto divisor=matrix[k][k];for(std::size_t l=k;l<4;++l)matrix[k][l]/=divisor;
      for(std::size_t j=0;j<3;++j)if(j!=k){const auto value=matrix[j][k];for(std::size_t l=k;l<4;++l)matrix[j][l]-=value*matrix[k][l];}
    }
    coefficient={matrix[0][3]/scale,matrix[1][3]/scale,matrix[2][3]};return finite_point(coefficient);
  };
  for(unsigned iteration=0;iteration<6;++iteration) {
    if(!solve())return out;
    std::vector<double> residuals;for(std::size_t i=0;i<centers.size();++i)if(retained[i]) {
      const auto& p=centers[i];residuals.push_back(p[2]-coefficient[0]*(p[0]-center[0])-coefficient[1]*(p[1]-center[1])-coefficient[2]);
    }
    if(residuals.size()<config.minimum_supported_cells)return out;
    const auto median=segmentation_median(residuals);
    // Robust detection hyperparameters, never physical calibration bounds.
    const auto threshold=std::max(config.maximum_cell_mad_m,3.*1.4826*segmentation_mad(residuals,median));
    bool changed=false;for(std::size_t i=0;i<centers.size();++i) {
      if(!spend())return out;const auto& p=centers[i];
      const auto residual=p[2]-coefficient[0]*(p[0]-center[0])-coefficient[1]*(p[1]-center[1])-coefficient[2];
      const bool keep=std::abs(residual-median)<=threshold;changed=changed||(keep!=retained[i]);retained[i]=keep;
    }
    if(!changed)break;
  }
  if(!solve())return out;
  std::vector<std::array<double,3>> geometry;for(std::size_t i=0;i<centers.size();++i)if(retained[i])geometry.push_back(centers[i]);
  out.retained_training_cells=geometry.size();if(geometry.size()<config.minimum_supported_cells)return out;
  const auto origin=geometry.front();auto farthest=origin;double far=0.;
  auto min_x=origin[0],max_x=min_x,min_y=origin[1],max_y=min_y;
  for(const auto& p:geometry) {
    const auto distance=(p[0]-origin[0])*(p[0]-origin[0])+(p[1]-origin[1])*(p[1]-origin[1]);
    if(distance>far){far=distance;farthest=p;}min_x=std::min(min_x,p[0]);max_x=std::max(max_x,p[0]);min_y=std::min(min_y,p[1]);max_y=std::max(max_y,p[1]);
  }
  double area=0.;for(const auto& p:geometry)area=std::max(area,std::abs((farthest[0]-origin[0])*(p[1]-origin[1])-(farthest[1]-origin[1])*(p[0]-origin[0])));
  out.training_span_x_m=max_x-min_x;out.training_span_y_m=max_y-min_y;out.training_double_area_m2=area;
  if(!finite(area)||out.training_span_x_m<config.minimum_support_span_x_m||out.training_span_y_m<config.minimum_support_span_y_m||area<config.minimum_support_double_area_m2)return out;
  auto normal=seed.unit_normal;normal[0]-=coefficient[0];normal[1]-=coefficient[1];const auto length=norm(normal);
  if(!finite_point(normal)||!finite(length)||length<=0.||normal[2]/length<std::cos(config.maximum_normal_tilt_rad)) {out.result=Result::InvalidNumerics;return out;}
  const auto d=seed.unit_normal[0]*seed.anchor[0]+seed.unit_normal[1]*seed.anchor[1]+seed.unit_normal[2]*seed.anchor[2]-coefficient[0]*center[0]-coefficient[1]*center[1]+coefficient[2];
  out.anchor={center[0],center[1],(d-normal[0]*center[0]-normal[1]*center[1])/normal[2]};
  for(std::size_t k=0;k<3;++k)out.unit_normal[k]=normal[k]/length;
  if(!finite_point(out.anchor)){out.result=Result::InvalidNumerics;return out;}
  std::vector<double> holdout_medians;
  for(const auto& entry:samples)if(entry.second.holdout.size()>=config.minimum_samples_per_partition) {
    std::vector<double> residuals;for(const auto& p:entry.second.holdout) {
      if(!spend())return out;
      residuals.push_back((p[2]-coefficient[0]*(p[0]-center[0])-coefficient[1]*(p[1]-center[1])-coefficient[2])/length);
    }
    holdout_medians.push_back(segmentation_median(std::move(residuals)));
  }
  if(holdout_medians.size()<config.minimum_supported_cells){out.result=Result::HoldoutRejected;return out;}
  out.holdout_cell_median_m=segmentation_median(holdout_medians);out.holdout_cell_mad_m=segmentation_mad(holdout_medians,out.holdout_cell_median_m);
  if(std::abs(out.holdout_cell_median_m)>config.maximum_cell_median_residual_m||out.holdout_cell_mad_m>config.maximum_cell_mad_m) {out.result=Result::HoldoutRejected;return out;}
  // Original anchors require actual seed observations in both partitions.
  // The later negative-side extension must establish new measured cell
  // statistics and a shared-edge path to a qualifying original component.
  for(const auto& entry:samples) {
    const auto& cell=entry.second;
    if(cell.h.size()<config.minimum_samples_per_partition||cell.holdout.size()<config.minimum_samples_per_partition)continue;
    std::vector<double> held;for(const auto& p:cell.holdout)held.push_back(p[2]);
    const auto training_median=segmentation_median(cell.h),held_median=segmentation_median(held);
    if(segmentation_mad(cell.h,training_median)>config.maximum_cell_mad_m||
        segmentation_mad(held,held_median)>config.maximum_cell_mad_m||
        std::abs(training_median-held_median)>config.maximum_partition_median_difference_m)continue;
    measured_cells.emplace(entry.first,true);
  }
  if(measured_cells.empty()){out.result=Result::HoldoutRejected;return out;}
  out.result=Result::Accepted;return out;
}
bool map_alignment_matches(const GroundReference& ref) noexcept {
  if(ref.lifetime!=GroundReference::Lifetime::RegistrationMapAsset)return false;
  const auto& binding=ref.map_binding;
  if(!binding.alignment||!binding.alignment->current())return false;
  const auto& actual=binding.alignment->identity();
  return !ref.map_asset.pcd_sha256.empty()&&!ref.map_asset.datum_id.empty()&&
    actual.pcd_sha256==ref.map_asset.pcd_sha256&&actual.datum_id==ref.map_asset.datum_id&&
    binding.source_id==ref.expected_source_id&&binding.source_epoch==ref.expected_source_epoch&&
    binding.source_ns==ref.valid_for_source_ns;
}
}  // namespace

bool registration_map_binding_current(const GroundReference& ref,const SourceIdentity& source) noexcept {
  if(!map_alignment_matches(ref))return false;
  const auto& binding=ref.map_binding;const auto& alignment=*binding.alignment;
  if(binding.source_id!=source.source_id||binding.source_epoch!=source.epoch||
    binding.source_ns!=source.source_ns||source.expiry_budget_ns<=0)return false;
  const auto covers=[&](int64_t deadline,int64_t base) {
    return base>0&&deadline>=base;
  };
  // Sensor freshness and registration validity intersect at consumption. The
  // historical geometry must cover this original acquisition/receipt, not an
  // entire future sensor budget that may exceed the NDT anchor's own lease.
  return covers(alignment.geometry_valid_until_ros_ns(),source.source_ns)&&
    covers(alignment.geometry_valid_until_ros_ns(),source.first_receipt_ros_ns)&&
    covers(alignment.geometry_valid_until_steady_ns(),source.first_receipt_steady_ns);
}

ClassifiedObstacleFrame::ClassifiedObstacleFrame(SourceFrame source,std::optional<GroundReference> reference)
 :source_(std::move(source)),reference_(std::move(reference)) {
  require_identity(source_.identity);
  ground_.transform_diagnostic=source_.transform_diagnostic;
  if(source_.points.size()+source_.self_support.size()>kMaximumClassifiedPoints ||
     source_.original_projected_samples>kMaximumClassifiedPoints)
    throw std::length_error("classified_obstacle_point_limit");
  if(source_.self_support_complete) {
    if(source_.self_support_quality_rejected_samples>source_.original_projected_samples ||
       source_.points.size()+source_.self_support.size()!=source_.original_projected_samples-
         source_.self_support_quality_rejected_samples)
      throw std::invalid_argument("classified_self_support_incomplete");
  } else if(!source_.self_support.empty())throw std::invalid_argument("classified_self_support_without_completion");
  if(std::any_of(source_.points.begin(),source_.points.end(),[](const auto& p){return p.measurement_unreliable;})) {
    if(!measurement_rejection_proof_valid(source_))
      throw std::invalid_argument("classified_measurement_rejection_proof_mismatch");
    for(std::size_t i=0;i<source_.points.size();++i)if(source_.points[i].measurement_unreliable&&
        !measurement_rejection_proven(source_,i))
      throw std::invalid_argument("classified_measurement_rejection_sample_mismatch");
  }
  const auto body_rejected=std::count_if(source_.points.begin(),source_.points.end(),
    [](const auto& p){return p.measurement_body_occluded;});
  if(body_rejected) {
    if(!body_occlusion_proof_valid(source_)||std::size_t(body_rejected)!=source_.body_occlusion.sample_ids.size())
      throw std::invalid_argument("classified_body_occlusion_proof_mismatch");
    std::size_t witnessed=0;
    for(std::size_t i=0;i<source_.points.size();++i)if(source_.points[i].measurement_body_occluded) {
      if(!body_occlusion_rejection_proven(source_,i)||
          source_.points[i].sample_id!=source_.body_occlusion.sample_ids[witnessed++])
        throw std::invalid_argument("classified_body_occlusion_sample_mismatch");
    }
  }
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  classes_.reserve(source_.points.size());heights_.assign(source_.points.size(),nan);
  errors_.assign(source_.points.size(),nan);
  decisions_.assign(source_.points.size(),PointDecision::NotRecorded);
  for(std::size_t i=0;i<source_.points.size();++i) {
    const auto& point=source_.points[i];
    switch(point.kind) {
      case ReturnKind::Measured:
        // Rejected measurement has no trustworthy endpoint. Preserve its raw
        // source/audit identity, but grant neither obstacle nor clearing credit.
        // Unknown coverage remains independent of these endpoint indices.
        if(point.measurement_unreliable) {
          classes_.push_back(PointClass::Invalid);
          decisions_[i]=std::binary_search(source_.measurement_quality.stereo_mismatch_sample_ids.begin(),source_.measurement_quality.stereo_mismatch_sample_ids.end(),point.sample_id)?PointDecision::StereoMismatch:
            std::binary_search(source_.measurement_quality.component_sample_ids.begin(),source_.measurement_quality.component_sample_ids.end(),point.sample_id)?
            PointDecision::SaturationDominatedComponent:PointDecision::MeasurementUnreliable;
        } else if(finite_point(point.position)) {
          classes_.push_back(PointClass::Uncertain);protected_.push_back(i);
          ++ground_.measured_points;
        } else classes_.push_back(PointClass::Invalid);
        break;
      case ReturnKind::Invalid:classes_.push_back(PointClass::Invalid);
        if(point.measurement_body_occluded)decisions_[i]=PointDecision::MeasurementBodyOccluded;
        break;
      case ReturnKind::Self:
        classes_.push_back(PointClass::Self);
        // Edge Self retains the original mixed-depth record for audit only.
        // Physical body-surface denominator evidence lives in self_support.
        if(point.ground_blocked&&!point.model_conflict&&!point.measurement_unreliable)
          decisions_[i]=PointDecision::ModelProjectionEdge;
        break;
      case ReturnKind::Masked:
        classes_.push_back(PointClass::Masked);
        if(point.ground_blocked&&!point.model_conflict&&!point.measurement_unreliable)
          decisions_[i]=PointDecision::ModelProjectionEdge;
        break;
      default:throw std::invalid_argument("classified_obstacle_return_kind_invalid");
    }
  }
  complete_=ground_.measured_points!=0;
}

ClassifiedObstacleFrame::ConstSharedPtr conservative_unqualified_frame(SourceFrame source) {
  return ClassifiedObstacleFrame::ConstSharedPtr(new ClassifiedObstacleFrame(std::move(source),std::nullopt));
}

ClassifiedObstacleFrame::ConstSharedPtr classify_obstacle_frame(SourceFrame source,const ClassifierConfig& config,
  std::optional<GroundReference> reference,bool observation_only) {
  if(!reference)return conservative_unqualified_frame(std::move(source));
  const bool map_asset=reference->lifetime==GroundReference::Lifetime::RegistrationMapAsset;
  const bool parameters_complete=parameters_specified(config,source,*reference);
  const bool unbounded=map_asset&&observation_only&&
    (!parameters_complete||!registration_map_binding_current(*reference,source.identity));
  if(!unbounded||parameters_complete)require_config(config);
  auto out=std::shared_ptr<ClassifiedObstacleFrame>(new ClassifiedObstacleFrame(std::move(source),std::move(reference)));
  auto& stats=out->ground_;auto& input=out->source_;auto& ref=*out->reference_;
  stats.observation_only=observation_only;
  const auto fail=[&](Qualification reason)->ClassifiedObstacleFrame::ConstSharedPtr {
    stats.result=reason;stats.safety_qualified=false;return out;
  };
  if(!unbounded&&std::max(input.points.size()+input.self_support.size(),input.original_projected_samples)>config.maximum_points)return fail(Qualification::BudgetExceeded);
  if(!stats.measured_points)return fail(Qualification::NoMeasuredPoints);
  if(!input.measurement_qualified&&!observation_only)return fail(Qualification::InvalidSource);
  const auto& id=input.identity;const auto& tf=input.to_common;
  const double rotation_tolerance=unbounded?1.e-6:config.rotation_tolerance;
  const bool point_times=std::any_of(input.points.begin(),input.points.end(),
    [](const PointReturn& point){return point.kind==ReturnKind::Measured&&!point.measurement_unreliable&&finite_point(point.position)&&point.source_offset_ns!=0;});
  if(point_times&&input.source_time_transforms.size()>kMaximumClassifiedPoints)
    return fail(Qualification::BudgetExceeded);
  if(!valid_source_transform(tf,id,id.source_ns,rotation_tolerance)||
    (point_times&&!valid_transform_groups(input,rotation_tolerance)))
    return fail(Qualification::InvalidTransform);
  for(const auto& point:input.points)if(point.kind==ReturnKind::Measured&&!point.measurement_unreliable&&finite_point(point.position)&&
      !point_transform(input,point))return fail(Qualification::InvalidTransform);
  if(!unbounded&&(!nonnegative(input.point_error_bound_m)||input.point_error_bound_m>config.maximum_point_error_m||
    !nonnegative(input.transform_translation_error_bound_m)||
    input.transform_translation_error_bound_m>config.maximum_transform_translation_error_m||
    !nonnegative(input.transform_angle_error_bound_rad)||
    input.transform_angle_error_bound_rad>config.maximum_transform_angle_error_rad||
    !nonnegative(ref.plane_offset_error_bound_m)||ref.plane_offset_error_bound_m>config.maximum_plane_offset_error_m||
    !nonnegative(ref.plane_angle_error_bound_rad)||ref.plane_angle_error_bound_rad>config.maximum_plane_angle_error_rad))
    return fail(Qualification::ErrorBoundExceeded);
  auto& grid=ref.support;const auto normal_length=norm(ref.unit_normal);
  if(!finite_point(ref.unit_normal)||!finite_point(ref.anchor)||!finite(normal_length)||
    std::abs(normal_length-1.)>(unbounded?1.e-6:config.normal_norm_tolerance)||ref.unit_normal[2]<=0.||
    (!unbounded&&ref.unit_normal[2]/normal_length<std::cos(config.maximum_normal_tilt_rad))||
    !finite(grid.origin_x_m)||!finite(grid.origin_y_m)||!finite(grid.resolution_m)||grid.resolution_m<=0.||
    !grid.width||!grid.height||grid.width>4096||grid.height>4096||
    !finite(grid.origin_x_m+grid.resolution_m*grid.width)||
    !finite(grid.origin_y_m+grid.resolution_m*grid.height)||grid.required_cells.empty()||
    grid.required_cells.size()>(unbounded?kMaximumSupportCells:config.maximum_support_cells))
    return fail(Qualification::InvalidReference);
  // Unit normalization preserves the fixed plane's zero set; no fitting or
  // displacement of the independently supplied anchor occurs here.
  for(auto& value:ref.unit_normal)value/=normal_length;
  stats.reference_geometry_valid=true;
  if(!text(ref.provenance_id,16384)||!text(ref.calibration_id,16384)||
    id.calibration_id.empty()||
    ref.common_frame!=id.common_frame||ref.expected_source_id!=id.source_id||
    ref.expected_source_epoch!=id.epoch||ref.expected_source_calibration_id!=id.calibration_id||
    ref.valid_for_source_ns!=id.source_ns||ref.valid_for_common_ns!=tf.common_ns)
    return fail(Qualification::ReferenceIdentityMismatch);
  if(!ref.source_time_transform_qualified&&!observation_only)return fail(Qualification::InvalidTransform);
  if(map_asset) {
    if(ref.common_frame!="registration_map"||!map_alignment_matches(ref))
      return fail(Qualification::ReferenceIdentityMismatch);
    if(!observation_only&&!registration_map_binding_current(ref,id))return fail(Qualification::ReferenceExpired);
  }else {
    if(ref.source_ns<=0||ref.first_receipt_ros_ns<ref.source_ns||ref.first_receipt_steady_ns<=0||
      ref.expiry_budget_ns<=0||ref.source_ns>id.first_receipt_ros_ns||
      ref.first_receipt_ros_ns>id.first_receipt_ros_ns||ref.first_receipt_steady_ns>id.first_receipt_steady_ns||
      ref.expiry_budget_ns<id.expiry_budget_ns)
      return fail(Qualification::ReferenceExpired);
    // Cover the complete source lease on both original reference clocks.
    const auto maximum_reference_age=ref.expiry_budget_ns-id.expiry_budget_ns;
    if(id.first_receipt_ros_ns-ref.source_ns>maximum_reference_age||
      id.first_receipt_ros_ns-ref.first_receipt_ros_ns>maximum_reference_age||
      id.first_receipt_steady_ns-ref.first_receipt_steady_ns>maximum_reference_age)
      return fail(Qualification::ReferenceExpired);
  }
  if(unbounded) {
    // An immutable plane can explain geometry before physical uncertainty is
    // known. Missing bounds remain NaN, never zero-width acceptance intervals.
    for(std::size_t i=0;i<input.points.size();++i)if(out->classes_[i]==PointClass::Uncertain&&!input.points[i].measurement_unreliable) {
      const auto* at=point_transform(input,input.points[i]);
      const auto common=transform_point(*at,input.points[i].position);
      double h=0.;for(std::size_t axis=0;axis<3;++axis)h+=ref.unit_normal[axis]*(common[axis]-ref.anchor[axis]);
      if(!finite_point(common)||!finite(h))return fail(Qualification::NumericFailure);
      out->heights_[i]=h;
    }
    stats.candidate_uncertain_points=stats.measured_points;
    if(!map_alignment_matches(ref))return fail(Qualification::ReferenceIdentityMismatch);
    return fail(Qualification::Unbounded);
  }
  auto cells=grid.required_cells;
  std::sort(cells.begin(),cells.end(),[](const SupportCell& a,const SupportCell& b){
    return a.y<b.y||(a.y==b.y&&a.x<b.x);
  });
  std::map<uint64_t,std::size_t> cell_indices;
  for(const auto& cell:cells) {
    if(cell.x>=grid.width||cell.y>=grid.height||!cell_indices.emplace(cell_key(grid,cell),stats.cells.size()).second)
      return fail(Qualification::InvalidReference);
    CellSupport record;record.cell=cell;record.holdout=(cell.x+cell.y)%2!=0;stats.cells.push_back(record);
  }
  const auto spend=[&] {
    if(stats.support_checks>=config.maximum_support_checks)return false;
    ++stats.support_checks;return true;
  };
  std::vector<std::optional<SupportCell>> point_cells(input.points.size());
  for(std::size_t i=0;i<input.points.size();++i) {
    if(out->classes_[i]!=PointClass::Uncertain||input.points[i].measurement_unreliable)continue;
    if(!spend())return fail(Qualification::BudgetExceeded);
    const auto& p=input.points[i].position;
    const auto common=transform_point(*point_transform(input,input.points[i]),p);
    if(!finite_point(common))return fail(Qualification::NumericFailure);
    std::array<double,3> relative{};double h=0.;
    for(std::size_t axis=0;axis<3;++axis) {
      relative[axis]=common[axis]-ref.anchor[axis];h+=ref.unit_normal[axis]*relative[axis];
    }
    const auto e=input.point_error_bound_m+input.transform_translation_error_bound_m+
      2.*norm(p)*std::sin(input.transform_angle_error_bound_rad/2.)+
      ref.plane_offset_error_bound_m+2.*norm(relative)*std::sin(ref.plane_angle_error_bound_rad/2.);
    if(!finite_point(relative)||!finite(h)||!nonnegative(e))return fail(Qualification::NumericFailure);
    out->heights_[i]=h;out->errors_[i]=std::nextafter(e,std::numeric_limits<double>::infinity());
    const auto lower=std::nextafter(h-out->errors_[i],-std::numeric_limits<double>::infinity());
    const auto upper=std::nextafter(h+out->errors_[i],std::numeric_limits<double>::infinity());
    if(!finite(lower)||!finite(upper))return fail(Qualification::NumericFailure);
    if(input.points[i].ground_blocked)continue;
    point_cells[i]=point_cell(grid,common);
    if(!point_cells[i])continue;
    const auto found=cell_indices.find(cell_key(grid,*point_cells[i]));
    if(found==cell_indices.end())continue;
    auto& cell=stats.cells[found->second];
    if(!cell.samples) {
      cell.minimum_signed_distance_m=cell.maximum_signed_distance_m=h;
      cell.minimum_lower_bound_m=lower;cell.maximum_upper_bound_m=upper;
      cell.mean_measured_xy={common[0],common[1]};
    }
    else {cell.minimum_signed_distance_m=std::min(cell.minimum_signed_distance_m,h);
      cell.maximum_signed_distance_m=std::max(cell.maximum_signed_distance_m,h);
      cell.minimum_lower_bound_m=std::min(cell.minimum_lower_bound_m,lower);
      cell.maximum_upper_bound_m=std::max(cell.maximum_upper_bound_m,upper);
      for(std::size_t axis=0;axis<2;++axis)
        cell.mean_measured_xy[axis]+=(common[axis]-cell.mean_measured_xy[axis])/(cell.samples+1.);
    }
    if(!finite(cell.mean_measured_xy[0])||!finite(cell.mean_measured_xy[1]))
      return fail(Qualification::NumericFailure);
    ++cell.samples;
  }
  for(auto& cell:stats.cells) {
    if(cell.samples<config.minimum_samples_per_cell) {
      cell.failure=CellSupport::Failure::Missing;continue;
    }
    if(cell.minimum_lower_bound_m< -config.maximum_support_residual_m||
      cell.maximum_upper_bound_m>config.maximum_support_residual_m) {
      cell.failure=CellSupport::Failure::Contradictory;continue;
    }
    cell.passed=true;cell.failure=CellSupport::Failure::None;
  }
  // Both fixed checkerboard partitions independently cover a non-collinear XY
  // span. Holdout samples are never used to select or move the reference plane.
  for(const bool holdout:{false,true}) {
    std::vector<std::array<double,2>> points;
    for(const auto& cell:stats.cells)if(cell.passed&&cell.holdout==holdout)
      points.push_back(cell.mean_measured_xy);
    if(points.size()<(holdout?config.minimum_holdout_cells:config.minimum_training_cells))
      return fail(Qualification::InsufficientSupportGeometry);
    auto min_x=points[0][0],max_x=min_x,min_y=points[0][1],max_y=min_y;
    std::size_t farthest=0;double distance=0.;
    for(std::size_t i=0;i<points.size();++i) {
      if(!spend())return fail(Qualification::BudgetExceeded);
      min_x=std::min(min_x,points[i][0]);max_x=std::max(max_x,points[i][0]);
      min_y=std::min(min_y,points[i][1]);max_y=std::max(max_y,points[i][1]);
      const auto dx=points[i][0]-points[0][0],dy=points[i][1]-points[0][1];
      const auto squared=dx*dx+dy*dy;
      if(!finite(squared))return fail(Qualification::NumericFailure);
      if(squared>distance){distance=squared;farthest=i;}
    }
    double area=0.;
    for(const auto& p:points) {
      if(!spend())return fail(Qualification::BudgetExceeded);
      const auto value=std::abs((points[farthest][0]-points[0][0])*(p[1]-points[0][1])-
        (points[farthest][1]-points[0][1])*(p[0]-points[0][0]));
      if(!finite(value))return fail(Qualification::NumericFailure);
      area=std::max(area,value);
    }
    if(max_x-min_x<config.minimum_support_span_x_m||max_y-min_y<config.minimum_support_span_y_m||
      area<config.minimum_support_double_area_m2)
      return fail(Qualification::InsufficientSupportGeometry);
  }
  stats.support_valid=true;
  if(!ref.independently_qualified&&!observation_only)return fail(Qualification::InvalidReference);
  auto classes=out->classes_;std::vector<std::size_t> marking,clearing,protected_hits;
  bool unknown=std::any_of(stats.cells.begin(),stats.cells.end(),[](const CellSupport& cell){return !cell.passed;});
  for(std::size_t i=0;i<input.points.size();++i) {
    if(classes[i]!=PointClass::Uncertain){unknown=true;continue;}
    bool continuous=!input.points[i].ground_blocked&&point_cells[i].has_value();
    if(continuous) {
      const auto cell=*point_cells[i];const auto radius=config.continuous_support_radius_cells;
      // The neighborhood is clipped to the explicit support domain. Every
      // cell inside that neighborhood must be independently supported; this
      // never labels an endpoint outside the domain or proves ground along a
      // clearing path outside it. Consumer ray/endpoint protection still applies.
      const auto x0=cell.x>radius?cell.x-radius:0U,y0=cell.y>radius?cell.y-radius:0U;
      const auto x1=static_cast<uint32_t>(std::min<uint64_t>(static_cast<uint64_t>(cell.x)+radius,grid.width-1));
      const auto y1=static_cast<uint32_t>(std::min<uint64_t>(static_cast<uint64_t>(cell.y)+radius,grid.height-1));
      for(uint32_t y=y0;y<=y1&&continuous;++y)
        for(uint32_t x=x0;x<=x1;++x) {
          if(!spend())return fail(Qualification::BudgetExceeded);
          const auto neighbor=cell_indices.find(cell_key(grid,{x,y}));
          if(neighbor==cell_indices.end()||!stats.cells[neighbor->second].passed){continuous=false;break;}
        }
    }
    const auto lower=std::nextafter(out->heights_[i]-out->errors_[i],-std::numeric_limits<double>::infinity());
    const auto upper=std::nextafter(out->heights_[i]+out->errors_[i],std::numeric_limits<double>::infinity());
    if(!finite(lower)||!finite(upper))return fail(Qualification::NumericFailure);
    if(continuous&&lower>config.ground_band_max_m)classes[i]=PointClass::Obstacle;
    else if(continuous&&lower>=config.ground_band_min_m&&upper<=config.ground_band_max_m)
      classes[i]=PointClass::Ground;
    if(classes[i]==PointClass::Ground)++stats.candidate_ground_points;
    else if(classes[i]==PointClass::Obstacle)++stats.candidate_obstacle_points;
    else ++stats.candidate_uncertain_points;
    if(classes[i]==PointClass::Ground)clearing.push_back(i);
    else {if(classes[i]==PointClass::Obstacle)marking.push_back(i);
      protected_hits.push_back(i);unknown=unknown||classes[i]==PointClass::Uncertain;}
  }
  // Observation calculates the same geometry/support candidates without ever
  // installing those candidates in the active marking/clearing transaction.
  // It does not promote the reference's independent qualification bit.
  if(map_asset&&!registration_map_binding_current(ref,id))return fail(Qualification::ReferenceExpired);
  if(observation_only)return fail(Qualification::ObserveOnly);
  out->classes_=std::move(classes);out->marking_=std::move(marking);out->clearing_=std::move(clearing);
  out->protected_=std::move(protected_hits);out->unknown_=unknown;
  stats.result=Qualification::Qualified;stats.safety_qualified=true;
  stats.basis=GroundQualification::Basis::BoundedReference;
  return out;
}

ClassifiedObstacleFrame::ConstSharedPtr classify_supported_ground_frame(
    SourceFrame source,const SegmentationConfig& config,GroundReference reference) {
  const auto positive=[](double v){return finite(v)&&v>0.;};
  if(!positive(config.maximum_point_residual_m)||
      !positive(config.maximum_cell_median_residual_m)||
      config.maximum_cell_median_residual_m>config.maximum_point_residual_m||
      !nonnegative(config.maximum_cell_mad_m)||
      !nonnegative(config.maximum_partition_median_difference_m)||
      !nonnegative(config.maximum_normal_tilt_rad)||config.maximum_normal_tilt_rad>=pi/2.||
      !positive(config.minimum_support_span_x_m)||!positive(config.minimum_support_span_y_m)||
      !positive(config.minimum_support_double_area_m2)||!config.minimum_samples_per_partition||
      config.minimum_supported_cells<3||!config.continuous_support_radius_cells||
      config.continuous_support_radius_cells!=1||!config.maximum_points||
      config.maximum_points>kMaximumClassifiedPoints||!config.maximum_support_cells||
      config.maximum_support_cells>kMaximumSupportCells||!config.maximum_support_checks||
      config.maximum_support_checks>16*1024*1024)
    throw std::invalid_argument("ground_segmentation_detection_configuration_invalid");
  auto out=std::shared_ptr<ClassifiedObstacleFrame>(new ClassifiedObstacleFrame(
    std::move(source),std::move(reference)));
  auto& stats=out->ground_;auto& input=out->source_;auto& ref=*out->reference_;
  const auto fail=[&](Qualification reason)->ClassifiedObstacleFrame::ConstSharedPtr {
    stats.result=reason;stats.safety_qualified=false;return out;
  };
  if(std::max(input.points.size()+input.self_support.size(),input.original_projected_samples)>config.maximum_points)return fail(Qualification::BudgetExceeded);
  if(!stats.measured_points)return fail(Qualification::NoMeasuredPoints);
  const auto& id=input.identity;constexpr double numeric_tolerance=1.e-6;
  if(!valid_source_transform(input.to_common,id,id.source_ns,numeric_tolerance)||
      input.source_time_transforms.size()>kMaximumClassifiedPoints||
      !valid_transform_groups(input,numeric_tolerance))return fail(Qualification::InvalidTransform);
  // Resolve every original exact group in the same validation order, then
  // reuse that immutable address for the numerical pass. No second per-point
  // binary search, transformed-coordinate approximation or latest fallback.
  std::vector<const SourceTimeTransform*> point_transforms(input.points.size(),nullptr);
  for(std::size_t i=0;i<input.points.size();++i) {
    const auto& point=input.points[i];
    if(point.kind==ReturnKind::Measured&&!point.measurement_unreliable&&finite_point(point.position)) {
      point_transforms[i]=point_transform(input,point);
      if(!point_transforms[i])return fail(Qualification::InvalidTransform);
    }
  }
  auto& grid=ref.support;const double length=norm(ref.unit_normal);
  if(!finite_point(ref.unit_normal)||!finite_point(ref.anchor)||!finite(length)||
      std::abs(length-1.)>numeric_tolerance||ref.unit_normal[2]<=0.||
      ref.unit_normal[2]/length<std::cos(config.maximum_normal_tilt_rad)||
      !finite(grid.origin_x_m)||!finite(grid.origin_y_m)||!positive(grid.resolution_m))
    return fail(Qualification::InvalidReference);
  for(auto& value:ref.unit_normal)value/=length;
  stats.reference_geometry_valid=true;
  if(!text(ref.provenance_id,16384)||!text(ref.calibration_id,16384)||
      ref.common_frame!=id.common_frame||ref.expected_source_id!=id.source_id||
      ref.expected_source_epoch!=id.epoch||ref.expected_source_calibration_id!=id.calibration_id||
      ref.valid_for_source_ns!=id.source_ns||ref.valid_for_common_ns!=input.to_common.common_ns)
    return fail(Qualification::ReferenceIdentityMismatch);
  if(ref.lifetime==GroundReference::Lifetime::RegistrationMapAsset) {
    if(ref.common_frame!="registration_map"||!map_alignment_matches(ref))
      return fail(Qualification::ReferenceIdentityMismatch);
    if(!registration_map_binding_current(ref,id))return fail(Qualification::ReferenceExpired);
  }else {
    if(ref.source_ns<=0||ref.first_receipt_ros_ns<ref.source_ns||ref.first_receipt_steady_ns<=0||
        ref.source_ns>id.first_receipt_ros_ns||ref.first_receipt_ros_ns>id.first_receipt_ros_ns||
        ref.first_receipt_steady_ns>id.first_receipt_steady_ns||ref.expiry_budget_ns<id.expiry_budget_ns)
      return fail(Qualification::ReferenceExpired);
    const auto age=ref.expiry_budget_ns-id.expiry_budget_ns;
    if(id.first_receipt_ros_ns-ref.source_ns>age||id.first_receipt_ros_ns-ref.first_receipt_ros_ns>age||
        id.first_receipt_steady_ns-ref.first_receipt_steady_ns>age)return fail(Qualification::ReferenceExpired);
  }
  const auto spend=[&] {
    if(stats.support_checks>=config.maximum_support_checks)return false;
    ++stats.support_checks;return true;
  };
  // The fixed plane and world lattice remain unchanged. Support is sparse:
  // only this frame's actual unblocked point-gate candidates create cells.
  // Cache original signed lattice coordinates so rebasing the diagnostic grid
  // never rounds a boundary point into an adjacent cell or repeats its TF.
  using LatticeCell=SegmentationLatticeCell; // deterministic row order
  RepeatedLatticeMap<std::size_t> observed;
  std::vector<std::optional<LatticeCell>> lattice_cells(input.points.size());
  std::vector<std::array<double,2>> common_xy(input.points.size());
  for(std::size_t i=0;i<input.points.size();++i) {
    if(out->classes_[i]!=PointClass::Uncertain||input.points[i].measurement_unreliable)continue;
    if(!spend())return fail(Qualification::BudgetExceeded);
    const auto& point=input.points[i];const auto common=transform_point(*point_transforms[i],point.position);
    double height=0.;for(std::size_t k=0;k<3;++k)height+=ref.unit_normal[k]*(common[k]-ref.anchor[k]);
    if(!finite_point(common)||!finite(height))return fail(Qualification::NumericFailure);
    out->heights_[i]=height; // Physical error bounds remain NaN.
    common_xy[i]={common[0],common[1]};
    if(point.ground_blocked)continue;
    const auto x=std::floor((common[0]-grid.origin_x_m)/grid.resolution_m);
    const auto y=std::floor((common[1]-grid.origin_y_m)/grid.resolution_m);
    const auto lower=static_cast<double>(std::numeric_limits<int64_t>::min());
    const auto upper=static_cast<double>(std::numeric_limits<int64_t>::max());
    if(!finite(x)||!finite(y)||x<lower||y<lower||x>=upper||y>=upper)
      return fail(Qualification::NumericFailure);
    const LatticeCell cell{static_cast<int64_t>(y),static_cast<int64_t>(x)};
    lattice_cells[i]=cell;
    if(std::abs(height)>config.maximum_point_residual_m)continue;
    observed.emplace(*lattice_cells[i],0);
    if(observed.size()>config.maximum_support_cells)return fail(Qualification::BudgetExceeded);
  }
  out->seed_heights_=out->heights_;
  RepeatedLatticeMap<bool> measured_cells;
  stats.measured_plane_fit=measure_ground_plane(input,config,ref,lattice_cells,common_xy,
    out->seed_heights_,stats.support_checks,measured_cells);
  using FitResult=GroundQualification::MeasuredPlaneFit::Result;
  if(stats.measured_plane_fit.result==FitResult::BudgetExceeded)return fail(Qualification::BudgetExceeded);
  if(stats.measured_plane_fit.result==FitResult::Accepted) {
    const auto& fit=stats.measured_plane_fit;observed.clear();
    // Reuse each original source-time transform's cached coordinates. The
    // reference remains the immutable seed; reported heights now use the
    // explicitly recorded measured plane, only for endpoint segmentation.
    for(std::size_t i=0;i<input.points.size();++i) {
      if(out->classes_[i]!=PointClass::Uncertain||input.points[i].measurement_unreliable)continue;
      if(!spend())return fail(Qualification::BudgetExceeded);
      const auto z=(out->heights_[i]+ref.unit_normal[0]*ref.anchor[0]+ref.unit_normal[1]*ref.anchor[1]+ref.unit_normal[2]*ref.anchor[2]-
        ref.unit_normal[0]*common_xy[i][0]-ref.unit_normal[1]*common_xy[i][1])/ref.unit_normal[2];
      out->heights_[i]=fit.unit_normal[0]*(common_xy[i][0]-fit.anchor[0])+fit.unit_normal[1]*(common_xy[i][1]-fit.anchor[1])+fit.unit_normal[2]*(z-fit.anchor[2]);
      if(!finite(out->heights_[i]))return fail(Qualification::NumericFailure);
      if(!lattice_cells[i]||!measured_cells.count(*lattice_cells[i])||
          out->seed_heights_[i]>config.maximum_point_residual_m||
          std::abs(out->heights_[i])>config.maximum_point_residual_m)continue;
      observed.emplace(*lattice_cells[i],0);
      if(observed.size()>config.maximum_support_cells)return fail(Qualification::BudgetExceeded);
    }
  }
  // Preserve the original seed-supported cells as anchors. An accepted fit
  // may propose new negative-side measured cells, but cannot qualify itself
  // from them or bridge an empty lattice cell. Each proposed cell still has
  // to pass independent training/holdout statistics below.
  const auto seed_observed=observed;
  if(stats.measured_plane_fit.result==FitResult::Accepted) {
    for(std::size_t i=0;i<input.points.size();++i) {
      if(!spend())return fail(Qualification::BudgetExceeded);
      if(!lattice_cells[i]||out->seed_heights_[i]>=-config.maximum_point_residual_m||
          std::abs(out->heights_[i])>config.maximum_point_residual_m)continue;
      observed.emplace(*lattice_cells[i],0);
      if(observed.size()>config.maximum_support_cells)return fail(Qualification::BudgetExceeded);
    }
  }
  if(observed.empty())return fail(Qualification::InsufficientSupportGeometry);
  auto min_x=observed.begin()->first.second,max_x=min_x;
  auto min_y=observed.begin()->first.first,max_y=min_y;
  for(const auto& entry:observed) {
    min_x=std::min(min_x,entry.first.second);max_x=std::max(max_x,entry.first.second);
    min_y=std::min(min_y,entry.first.first);max_y=std::max(max_y,entry.first.first);
  }
  const auto width=static_cast<long double>(max_x)-min_x+1.;
  const auto height=static_cast<long double>(max_y)-min_y+1.;
  if(width>std::numeric_limits<uint32_t>::max()||height>std::numeric_limits<uint32_t>::max())
    return fail(Qualification::BudgetExceeded);
  grid.origin_x_m+=static_cast<double>(min_x)*grid.resolution_m;
  grid.origin_y_m+=static_cast<double>(min_y)*grid.resolution_m;
  grid.width=static_cast<uint32_t>(width);grid.height=static_cast<uint32_t>(height);
  if(!finite(grid.origin_x_m)||!finite(grid.origin_y_m)||
      !finite(grid.origin_x_m+grid.resolution_m*grid.width)||
      !finite(grid.origin_y_m+grid.resolution_m*grid.height))return fail(Qualification::NumericFailure);
  grid.required_cells.clear();
  std::map<uint64_t,std::size_t> indices;
  for(auto& entry:observed) {
    const SupportCell cell{static_cast<uint32_t>(entry.first.second-min_x),
      static_cast<uint32_t>(entry.first.first-min_y)};
    entry.second=stats.cells.size();indices.emplace(cell_key(grid,cell),entry.second);
    CellSupport record;record.cell=cell;stats.cells.push_back(record);grid.required_cells.push_back(cell);
  }
  struct Samples {std::vector<double> training,holdout;};
  std::vector<Samples> samples(observed.size());
  std::vector<bool> seed_cell(observed.size(),false);
  for(const auto& entry:seed_observed)seed_cell[observed.at(entry.first)]=true;
  const auto no_cell=std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> point_cells(input.points.size(),no_cell);
  const auto holdout=segmentation_holdout;
  for(std::size_t i=0;i<input.points.size();++i) {
    if(!lattice_cells[i])continue;
    if(!spend())return fail(Qualification::BudgetExceeded);
    const auto found=observed.find(*lattice_cells[i]);if(found==observed.end())continue;
    const auto at=found->second;auto& record=stats.cells[at];point_cells[i]=at;
    if(stats.measured_plane_fit.result==FitResult::Accepted&&out->seed_heights_[i]>config.maximum_point_residual_m)continue;
    if(std::abs(out->heights_[i])>config.maximum_point_residual_m)continue;
    const auto& point=input.points[i];
    const auto identity=point.sample_id==std::numeric_limits<uint64_t>::max()?uint64_t(i):point.sample_id;
    auto& partition=holdout(identity)?samples[at].holdout:samples[at].training;
    partition.push_back(out->heights_[i]);
    if(!record.samples)record.mean_measured_xy=common_xy[i];
    else for(std::size_t k=0;k<2;++k)
      record.mean_measured_xy[k]+=(common_xy[i][k]-record.mean_measured_xy[k])/(record.samples+1.);
    ++record.samples;
  }
  const auto median=[](std::vector<double> values) {
    return segmentation_median(std::move(values));
  };
  const auto mad=[&](const std::vector<double>& values,double center) {
    std::vector<double> deviations;deviations.reserve(values.size());
    for(const auto value:values)deviations.push_back(std::abs(value-center));
    return median(std::move(deviations));
  };
  for(std::size_t i=0;i<stats.cells.size();++i) {
    auto& cell=stats.cells[i];const auto& sample=samples[i];
    cell.training_samples=sample.training.size();cell.holdout_samples=sample.holdout.size();
    if(cell.training_samples<config.minimum_samples_per_partition||
        cell.holdout_samples<config.minimum_samples_per_partition)continue;
    cell.training_median_residual_m=median(sample.training);
    cell.holdout_median_residual_m=median(sample.holdout);
    cell.training_mad_m=mad(sample.training,cell.training_median_residual_m);
    cell.holdout_mad_m=mad(sample.holdout,cell.holdout_median_residual_m);
    if(std::abs(cell.training_median_residual_m)>config.maximum_cell_median_residual_m||
        std::abs(cell.holdout_median_residual_m)>config.maximum_cell_median_residual_m||
        cell.training_mad_m>config.maximum_cell_mad_m||cell.holdout_mad_m>config.maximum_cell_mad_m||
        std::abs(cell.training_median_residual_m-cell.holdout_median_residual_m)>
          config.maximum_partition_median_difference_m) {cell.failure=CellSupport::Failure::Contradictory;continue;}
    cell.passed=true;cell.failure=CellSupport::Failure::None;
  }
  // Components use shared cell edges (four neighbours), never a diagonal or
  // a jump across an unobserved cell. Missing cells remain holes. Geometry is
  // evaluated independently for each component; distant islands cannot pool
  // their samples/span to qualify one another.
  std::vector<bool> visited(stats.cells.size(),false),continuous_cell(stats.cells.size(),false);
  bool any_component=false;
  constexpr std::array<std::array<int,2>,4> neighbours{{{{1,0}},{{-1,0}},{{0,1}},{{0,-1}}}};
  for(std::size_t seed=0;seed<stats.cells.size();++seed) {
    if(visited[seed]||!stats.cells[seed].passed||!seed_cell[seed])continue;
    std::vector<std::size_t> component{seed};visited[seed]=true;
    for(std::size_t next=0;next<component.size();++next) {
      const auto cell=stats.cells[component[next]].cell;
      for(const auto& delta:neighbours) {
        if(!spend())return fail(Qualification::BudgetExceeded);
        const auto x=int64_t(cell.x)+delta[0],y=int64_t(cell.y)+delta[1];
        if(x<0||y<0||x>=grid.width||y>=grid.height)continue;
        const auto found=indices.find(cell_key(grid,{uint32_t(x),uint32_t(y)}));
        if(found==indices.end()||visited[found->second]||!stats.cells[found->second].passed||
            !seed_cell[found->second])continue;
        visited[found->second]=true;component.push_back(found->second);
      }
    }
    if(component.size()<config.minimum_supported_cells)continue;
    const auto origin=stats.cells[component.front()].mean_measured_xy;
    auto min_x=origin[0],max_x=min_x,min_y=origin[1],max_y=min_y;
    std::array<double,2> farthest=origin;double distance=0.;
    for(const auto index:component) {
      if(!spend())return fail(Qualification::BudgetExceeded);
      const auto p=stats.cells[index].mean_measured_xy;
      min_x=std::min(min_x,p[0]);max_x=std::max(max_x,p[0]);
      min_y=std::min(min_y,p[1]);max_y=std::max(max_y,p[1]);
      const auto dx=p[0]-origin[0],dy=p[1]-origin[1],squared=dx*dx+dy*dy;
      if(!finite(squared))return fail(Qualification::NumericFailure);
      if(squared>distance){distance=squared;farthest=p;}
    }
    double area=0.;for(const auto index:component) {
      if(!spend())return fail(Qualification::BudgetExceeded);
      const auto p=stats.cells[index].mean_measured_xy;
      area=std::max(area,std::abs((farthest[0]-origin[0])*(p[1]-origin[1])-
        (farthest[1]-origin[1])*(p[0]-origin[0])));
    }
    if(!finite(area))return fail(Qualification::NumericFailure);
    if(max_x-min_x<config.minimum_support_span_x_m||max_y-min_y<config.minimum_support_span_y_m||
        area<config.minimum_support_double_area_m2)continue;
    for(const auto index:component)continuous_cell[index]=true;
    any_component=true;
  }
  if(!any_component)return fail(Qualification::InsufficientSupportGeometry);
  // Flood only from independently qualifying original components. Every new
  // edge requires two actually observed, passed cells and agreement in BOTH
  // partitions. A missing cell, step, contradictory cell or one-partition
  // endpoint remains a barrier. No diagonal, synthetic sample or plane-only
  // extrapolation can turn an unseen region into support.
  if(stats.measured_plane_fit.result==FitResult::Accepted) {
    std::vector<std::size_t> frontier;
    for(std::size_t i=0;i<continuous_cell.size();++i)if(continuous_cell[i])frontier.push_back(i);
    for(std::size_t next=0;next<frontier.size();++next) {
      const auto& from=stats.cells[frontier[next]];
      for(const auto& delta:neighbours) {
        if(!spend())return fail(Qualification::BudgetExceeded);
        const auto x=int64_t(from.cell.x)+delta[0],y=int64_t(from.cell.y)+delta[1];
        if(x<0||y<0||x>=grid.width||y>=grid.height)continue;
        const auto found=indices.find(cell_key(grid,{uint32_t(x),uint32_t(y)}));
        if(found==indices.end()||continuous_cell[found->second])continue;
        const auto& to=stats.cells[found->second];
        if(!to.passed||
            std::abs(from.training_median_residual_m-to.training_median_residual_m)>
              config.maximum_partition_median_difference_m||
            std::abs(from.holdout_median_residual_m-to.holdout_median_residual_m)>
              config.maximum_partition_median_difference_m)continue;
        continuous_cell[found->second]=true;frontier.push_back(found->second);
      }
    }
  }
  // CAD-conflicted and insufficiently supported mixed returns remain
  // Uncertain. They neither create an obstacle vote nor supply ground clearing.
  stats.support_valid=true;
  // Anchor qualification is immutable during this final pass. Evaluate it once
  // per lattice cell, so repeated raised returns cannot consume the complete
  // frame budget by revisiting the same eight floor neighbors.
  RepeatedLatticeMap<bool> floor_support_cache;
  auto decisions=out->decisions_;
  auto classes=out->classes_;std::vector<std::size_t> marking,clearing,protected_hits;
  bool unknown=std::any_of(stats.cells.begin(),stats.cells.end(),[](const auto& cell){return !cell.passed;});
  for(std::size_t i=0;i<input.points.size();++i) {
    if(classes[i]!=PointClass::Uncertain){
      if(!input.points[i].measurement_unreliable&&decisions[i]!=PointDecision::ModelProjectionEdge&&
          decisions[i]!=PointDecision::MeasurementBodyOccluded)
        decisions[i]=PointDecision::NonMeasured;
      unknown=true;continue;
    }
    decisions[i]=input.points[i].ground_blocked?PointDecision::ModelBlocked:PointDecision::Unsupported;
    bool continuous=false;
    if(point_cells[i]!=no_cell&&!input.points[i].ground_blocked)
      continuous=continuous_cell[point_cells[i]];
    const auto h=out->heights_[i];
    const bool seed_admitted=stats.measured_plane_fit.result!=FitResult::Accepted||
      out->seed_heights_[i]<=config.maximum_point_residual_m;
    if(h < -config.maximum_point_residual_m&&!input.points[i].ground_blocked)decisions[i]=PointDecision::BelowGroundBand;
    if(continuous&&!seed_admitted)decisions[i]=PointDecision::SeedOutsideBand;
    if(continuous&&seed_admitted&&std::abs(h)<=config.maximum_point_residual_m) {
      classes[i]=PointClass::Ground;
      decisions[i]=PointDecision::GroundContinuous;
    }
    else if(h>config.maximum_point_residual_m&&!input.points[i].ground_blocked) {
      // A mixed cell may contradict ground without making its raised returns
      // unknowable. Borrow only independent, already continuous floor anchors
      // to establish the reference; never turn this evidence into Ground.
      bool floor_supported=continuous;
      if(!floor_supported&&lattice_cells[i]&&input.raster_geometry.width&&
          input.raster_geometry.height&&input.raster_geometry.stride&&
          stats.measured_plane_fit.result==FitResult::Accepted) {
        const auto cached=floor_support_cache.find(*lattice_cells[i]);
        if(cached!=floor_support_cache.end())floor_supported=cached->second;
        else {
          const double narrow=std::min(.015,config.maximum_point_residual_m);
          const auto cell_x=lattice_cells[i]->second-min_x,cell_y=lattice_cells[i]->first-min_y;
          std::size_t anchors=0,actual_support=0;
          for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
            if(!dx&&!dy)continue;
            if(!spend())return fail(Qualification::BudgetExceeded);
            const auto x=cell_x+dx,y=cell_y+dy;
            if(x<0||y<0||x>=grid.width||y>=grid.height)continue;
            const auto at=indices.find(cell_key(grid,{uint32_t(x),uint32_t(y)}));
            if(at==indices.end()||!continuous_cell[at->second])continue;
            const auto& anchor=stats.cells[at->second];
            if(anchor.training_samples<config.minimum_samples_per_partition||
               anchor.holdout_samples<config.minimum_samples_per_partition||
               std::abs(anchor.training_median_residual_m)>narrow||
               std::abs(anchor.holdout_median_residual_m)>narrow||
               anchor.training_mad_m>narrow||anchor.holdout_mad_m>narrow||
               std::abs(anchor.training_median_residual_m-anchor.holdout_median_residual_m)>narrow)continue;
            ++anchors;actual_support+=anchor.training_samples+anchor.holdout_samples;
          }
          floor_supported=anchors>=3&&actual_support>=32;
          floor_support_cache.emplace(*lattice_cells[i],floor_supported);
        }
      }
      if(floor_supported) {
        classes[i]=PointClass::Obstacle;
        decisions[i]=continuous?PointDecision::ObstacleContinuous:PointDecision::ObstacleAnchored;
      }
    }
    if(classes[i]==PointClass::Ground){++stats.candidate_ground_points;clearing.push_back(i);}
    else {protected_hits.push_back(i);
      if(classes[i]==PointClass::Obstacle){marking.push_back(i);++stats.candidate_obstacle_points;}
      else {++stats.candidate_uncertain_points;unknown=true;}}
  }
  if(ref.lifetime==GroundReference::Lifetime::RegistrationMapAsset&&!registration_map_binding_current(ref,id))
    return fail(Qualification::ReferenceExpired);
  out->classes_=std::move(classes);out->decisions_=std::move(decisions);out->marking_=std::move(marking);
  out->clearing_=std::move(clearing);out->protected_=std::move(protected_hits);out->unknown_=unknown;
  stats.result=Qualification::Qualified;stats.safety_qualified=true;
  stats.basis=GroundQualification::Basis::MeasuredSupport;
  return out;
}

}  // namespace sunhorizon_perception_runtime::ground_relative
