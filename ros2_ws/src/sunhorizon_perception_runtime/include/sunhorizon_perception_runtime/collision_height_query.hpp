#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"

namespace sunhorizon_navigation_runtime {class Nav2Assembly; class NavigationRuntime;}
namespace sunhorizon_perception_runtime {
struct SpatialQueryPolicy {
  std::string mode{"full-region-v1"};
  double collision_min_height_m{-.75}, collision_max_height_m{2.};
  bool height_limited() const {return mode=="planar-collision-height-v1";}
  bool operator==(const SpatialQueryPolicy& other) const {
    return mode==other.mode && (!height_limited() ||
      (collision_min_height_m==other.collision_min_height_m &&
       collision_max_height_m==other.collision_max_height_m));
  }
};
struct SpatialQueryBox {std::array<double,3> min, max;};
namespace detail {
constexpr double kCollisionHeightExpansionM=1.e-5;
inline void validate_spatial_query_policy(const SpatialQueryPolicy& policy) {
  if (policy.mode!="full-region-v1" && !policy.height_limited())
    throw std::invalid_argument("unsupported_spatial_query_mode");
  if (policy.height_limited() &&
      (policy.collision_min_height_m!=-.75 || policy.collision_max_height_m!=2.))
    throw std::invalid_argument("collision_height_requires_fixed_accepted_bounds");
}
inline std::vector<SpatialQueryBox> effective_spatial_boxes(const SpatialQueryPolicy& policy,
    const std::string& frame,const std::vector<SpatialQueryBox>& original) {
  validate_spatial_query_policy(policy);
  if (!policy.height_limited()) return original;
  if (frame!="base_footprint" || original.size()!=1 ||
      original[0].min!=std::array<double,3>{.2,.15,-.8} ||
      original[0].max!=std::array<double,3>{.3,.25,-.7})
    throw std::invalid_argument("collision_height_requires_original_fixed_region");
  auto result=original;
  result[0].min[2]=std::max(result[0].min[2],policy.collision_min_height_m-kCollisionHeightExpansionM);
  result[0].max[2]=std::min(result[0].max[2],policy.collision_max_height_m+kCollisionHeightExpansionM);
  if (result[0].min[2]>=result[0].max[2])
    throw std::invalid_argument("collision_height_empty_responsibility");
  return result;
}
inline std::string spatial_query_canonical(const SpatialQueryPolicy& policy,
    const std::string& frame,const std::vector<SpatialQueryBox>& original) {
  const auto effective=effective_spatial_boxes(policy,frame,original);
  sunhorizon_rgbd_adapter::RegionCanonical fields;
  if (policy.height_limited()) fields.add(policy.mode);
  fields.add(frame);
  if (policy.height_limited()) fields.number(policy.collision_min_height_m)
    .number(policy.collision_max_height_m).number(kCollisionHeightExpansionM);
  const auto boxes=[&](const auto& values) {
    for (const auto& box:values) for (std::size_t axis=0;axis<3;++axis) {
      sunhorizon_rgbd_adapter::RegionCanonical pair;
      pair.number(box.min[axis]).number(box.max[axis]);fields.add(pair.str());
    }
  };
  boxes(original);
  if (policy.height_limited()) boxes(effective);
  return std::string(policy.height_limited()?"spatial-query-v2;":"spatial-query-v1;")+fields.str();
}
struct CollisionHeightSource {
  std::string consumer, source, frame;
  bool marking{false};
  double min_height{0.}, max_height{0.};
};
inline void require_collision_height_sources(const SpatialQueryPolicy& policy,
    const std::vector<CollisionHeightSource>& sources,
    const std::map<std::string,std::set<std::string>>& expected) {
  validate_spatial_query_policy(policy);
  if (!policy.height_limited()) return;
  if (expected.size()!=3 || !expected.count("local") || !expected.count("global") || !expected.count("collision"))
    throw std::invalid_argument("collision_height_requires_all_three_consumers");
  std::map<std::string,std::set<std::string>> found;
  std::map<std::string,std::pair<double,double>> ranges;
  for (const auto& source:sources) {
    if (!source.marking) continue; // Clearing rays do not admit collision points.
    if (!expected.count(source.consumer) || source.source.empty() ||
        !found[source.consumer].insert(source.source).second ||
        (source.frame!="map" && source.frame!="odom" && source.frame!="base_footprint" && source.frame!="base_link") ||
        !std::isfinite(source.min_height) || !std::isfinite(source.max_height) ||
        source.min_height>source.max_height || source.min_height<policy.collision_min_height_m ||
        source.max_height>policy.collision_max_height_m)
      throw std::invalid_argument("collision_height_source_outside_bound_contract");
    auto inserted=ranges.emplace(source.consumer,std::make_pair(source.min_height,source.max_height));
    if (!inserted.second) {
      inserted.first->second.first=std::min(inserted.first->second.first,source.min_height);
      inserted.first->second.second=std::max(inserted.first->second.second,source.max_height);
    }
  }
  if (found!=expected) throw std::invalid_argument("collision_height_unbound_or_missing_marking_source");
  for (const auto& range:ranges) if (range.second.first!=policy.collision_min_height_m ||
      range.second.second!=policy.collision_max_height_m)
    throw std::invalid_argument("collision_height_consumer_union_mismatch");
}
inline void require_exact_planar_pose(double x,double y,double z,double qx,double qy,double qz,double qw) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw) ||
      z!=0. || qx!=0. || qy!=0. || std::abs(qz*qz+qw*qw-1.)>1.e-9)
    throw std::invalid_argument("collision_height_requires_exact_planar_canonical_pose");
}
inline void require_exact_base_identity(double x,double y,double z,double qx,double qy,double qz,double qw) {
  require_exact_planar_pose(x,y,z,qx,qy,qz,qw);
  if (x!=0. || y!=0. || qz!=0. || (qw!=1. && qw!=-1.))
    throw std::invalid_argument("collision_height_requires_fixed_base_identity");
}
} // namespace detail

// Only the actual configured Nav2 assembly can issue this proof. A standalone
// perception node cannot replace the responsibility check with a parameter bool.
class CollisionHeightBinding {
 public:
  const SpatialQueryPolicy& policy() const {return policy_;}
 private:
  explicit CollisionHeightBinding(SpatialQueryPolicy policy):policy_(std::move(policy)) {}
  SpatialQueryPolicy policy_;
  friend class sunhorizon_navigation_runtime::Nav2Assembly;
};
} // namespace sunhorizon_perception_runtime
