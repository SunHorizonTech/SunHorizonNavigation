#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"
namespace sunhorizon_rgbd_adapter {
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
// Conservative union of exactly the core's projected rectangles. Only actual
// ray/box intersections are classified. No striding or whole-image BVH pass.
inline std::vector<uint8_t> spatial_required_pixels(const sv::Query& query, const sv::DepthRaster& d) {
  if (!d.width || !d.height || d.width>640 || d.height>480 || query.cells.empty() || query.cells.size()>sv::kMaximumCells ||
      !std::isfinite(d.fx) || d.fx<=0. || !std::isfinite(d.fy) || d.fy<=0. || !std::isfinite(d.cx) || !std::isfinite(d.cy))
    throw std::invalid_argument("invalid_sampling_contract");
  std::vector<uint8_t> mask(std::size_t(d.width)*d.height,0);
  std::size_t checks=0;
  for (const auto& cell:query.cells) {
    for (std::size_t axis=0;axis<3;++axis)
      if (!std::isfinite(cell.min[axis]) || !std::isfinite(cell.max[axis]) || cell.min[axis]>=cell.max[axis])
        throw std::invalid_argument("invalid_sampling_cell");
    double umin=std::numeric_limits<double>::infinity(), vmin=umin, umax=-umin, vmax=-umin;
    unsigned positive=0, nonpositive=0;
    for (unsigned corner=0;corner<8;++corner) {
      std::array<double,3> p{};
      for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j)
        p[i]+=d.optical_to_common.rotation[j*3+i]*
            (((corner&(1U<<j))?cell.max[j]:cell.min[j])-d.optical_to_common.translation[j]);
      for (double x:p) if (!std::isfinite(x)) throw std::runtime_error("query_projection_nonfinite");
      if (p[2]<=0.) {++nonpositive;continue;}
      ++positive;
      const double u=d.fx*p[0]/p[2]+d.cx, v=d.fy*p[1]/p[2]+d.cy;
      if (!std::isfinite(u) || !std::isfinite(v)) throw std::runtime_error("query_projection_nonfinite");
      umin=std::min(umin,u);umax=std::max(umax,u);vmin=std::min(vmin,v);vmax=std::max(vmax,v);
    }
    if (!positive) continue;
    if (nonpositive) throw std::runtime_error("query_crosses_optical_plane");
    if (umax<-.5 || vmax<-.5 || umin>d.width-.5 || vmin>d.height-.5) continue;
    const int u0=int(std::max(0.,std::floor(umin))),u1=int(std::min(double(d.width-1),std::ceil(umax)));
    const int v0=int(std::max(0.,std::floor(vmin))),v1=int(std::min(double(d.height-1),std::ceil(vmax)));
    for (int v=v0;v<=v1;++v) for (int u=u0;u<=u1;++u) {
      if (++checks>sv::kMaximumChecks) throw std::runtime_error("query_projection_budget");
      std::array<double,3> ray{};
      const double optical[3]{(u-d.cx)/d.fx,(v-d.cy)/d.fy,1.};
      for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j)
        ray[i]+=d.optical_to_common.rotation[i*3+j]*optical[j];
      double enter=0.,exit=std::numeric_limits<double>::infinity(); bool hit=true;
      for (std::size_t i=0;i<3;++i) {
        if (!std::isfinite(ray[i])) throw std::runtime_error("query_ray_nonfinite");
        if (ray[i]==0.) {hit=hit && d.optical_to_common.translation[i]>=cell.min[i] && d.optical_to_common.translation[i]<=cell.max[i];continue;}
        double a=(cell.min[i]-d.optical_to_common.translation[i])/ray[i];
        double b=(cell.max[i]-d.optical_to_common.translation[i])/ray[i];
        if (!std::isfinite(a)||!std::isfinite(b)) throw std::runtime_error("query_ray_nonfinite");
        if (a>b) std::swap(a,b);
        enter=std::max(enter,a);exit=std::min(exit,b);hit=hit && enter<=exit;
      }
      if (hit && exit>0.) mask[std::size_t(v)*d.width+u]=1;
    }
  }
  return mask;
}
}  // namespace sunhorizon_rgbd_adapter
