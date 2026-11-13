#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include <limits>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sunhorizon_camera_runtime {
inline constexpr char kStereoQualityAlgorithm[]="dual-ir-native-quality-v4";

// A rejection test, NOT proof that remaining depth is accurate or space is free.
// Caller must establish native rectified depth/left/right geometry: same K,
// identity rotations, depth origin at left IR, right translation (-baseline,0,0).
struct RectifiedStereoSample {
  const uint16_t * depth_mm{};
  const uint8_t * left{};
  const uint8_t * right{};
  std::size_t depth_count{}, left_count{}, right_count{};
  uint32_t width{}, height{};
  uint64_t depth_us{}, left_us{}, right_us{};
  double fx{}, baseline_mm{};
  bool rectified_geometry_verified{false};
  double fy{},cx{},cy{};
  std::size_t maximum_component_checks{65536};
  std::size_t maximum_stereo_match_checks{65536};
};

enum class StereoQuality : uint8_t {
  kNotRejected = 0,  // Does not certify a measurement.
  kSaturated = 1,    // Cannot support either an obstacle or clearing ray.
  kUnassessed = 2,   // Invalid depth or insufficient image support; never clear.
  kSaturationDominatedComponent = 3, // Inherited quality rejection, not direct pixel clipping.
  kStereoMismatch = 4 // Historical v3 code; disabled in v4 due foreground ambiguity.
};

struct StereoQualityResult {
  bool valid{false};
  std::string reason;
  std::vector<StereoQuality> pixels;
  std::size_t saturated{}, unassessed{};
  bool component_extension_complete{false};
  std::string component_extension_reason;
  std::size_t component_checks{},component_added{};
  bool stereo_match_complete{false};
  std::string stereo_match_reason;
  std::size_t stereo_match_checks{},stereo_mismatch_pixels{};
};

// Scratch belongs to one serialized camera conversion owner. Never retain a
// result view here: every returned mask still owns its independent storage.
struct StereoQualityWorkspace {
  std::vector<uint16_t> depth_mm;
  std::vector<uint32_t> left_integral, right_integral;
  std::vector<uint8_t> visited;
  std::vector<std::size_t> component, extra;
};

namespace detail {
// The original direct mask is immutable evidence for component qualification.
// Additional rejection commits only after every seeded component is complete.
inline void extend_saturation_components(const RectifiedStereoSample& in,StereoQualityResult& out,
    StereoQualityWorkspace& workspace) {
  const std::size_t count=out.pixels.size();
  std::size_t budget=in.maximum_component_checks;
  if(budget>65536)budget=65536;
  if(budget>4*count)budget=4*count;
  auto& visited=workspace.visited;visited.assign(count,0);
  auto& component=workspace.component;auto& extra=workspace.extra;
  component.clear();extra.clear();component.reserve(1025);
  const auto valid=[&](std::size_t i){return in.depth_mm[i]>=200&&in.depth_mm[i]<=8000;};
  const auto adjacent=[&](std::size_t a,std::size_t b) {
    const double za=in.depth_mm[a],zb=in.depth_mm[b];
    const double dx=(double(a%in.width)-in.cx)*za/in.fx-(double(b%in.width)-in.cx)*zb/in.fx;
    const double dy=(double(a/in.width)-in.cy)*za/in.fy-(double(b/in.width)-in.cy)*zb/in.fy;
    const double dz=za-zb;
    return dx*dx+dy*dy+dz*dz<=100.; // millimetres: original 3D edge <=10mm
  };
  for(std::size_t seed=0;seed<count;++seed) {
    if(out.pixels[seed]!=StereoQuality::kSaturated||visited[seed])continue;
    component.clear();component.push_back(seed);visited[seed]=1;std::size_t saturated=0;
    for(std::size_t head=0;head<component.size();++head) {
      const auto i=component[head];saturated+=out.pixels[i]==StereoQuality::kSaturated;
      for(int direction=0;direction<4;++direction) {
        if(out.component_checks>=budget){out.component_extension_reason="component_work_budget";return;}
        ++out.component_checks;
        const auto x=i%in.width,y=i/in.width;std::size_t next;
        if(direction==0){if(!x)continue;next=i-1;}
        else if(direction==1){if(x+1==in.width)continue;next=i+1;}
        else if(direction==2){if(!y)continue;next=i-in.width;}
        else {if(y+1==in.height)continue;next=i+in.width;}
        if(visited[next]||!valid(next)||!adjacent(i,next))continue;
        visited[next]=1;component.push_back(next);
        if(component.size()>1024){out.component_extension_reason="component_oversize";return;}
      }
    }
    if(saturated*10>=component.size()*7)
      for(const auto i:component)if(out.pixels[i]!=StereoQuality::kSaturated)extra.push_back(i);
  }
  // No partially explored component or unfinished frame can publish extras.
  for(const auto i:extra){
    if(out.pixels[i]==StereoQuality::kUnassessed)--out.unassessed;
    out.pixels[i]=StereoQuality::kSaturationDominatedComponent;
  }
  out.component_added=extra.size();
  out.component_extension_complete=true;out.component_extension_reason="complete";
}
inline void extend_saturation_components(const RectifiedStereoSample& in,StereoQualityResult& out) {
  StereoQualityWorkspace workspace;
  extend_saturation_components(in,out,workspace);
}
// Disabled: a small foreground with correct native disparity can share its
// surrounding IR texture with a background, making a background-dominated5x5
// match appear uniquely better. Original foreground shape also supports the
// native disparity in recorded26912. No tested local criterion distinguishes
// these cases conservatively; do not grant rejection authority to this stage.
// Keep the result fields/schema for historical diagnostics, with zero work and
// zero new reason4. Saturation/component rejection above is unaffected.
inline void reject_native_micro_mismatches(const RectifiedStereoSample&,StereoQualityResult& out) {
  out.stereo_match_complete=true;
  out.stereo_match_reason="disabled_foreground_ambiguity";
  out.stereo_match_checks=0;
  out.stereo_mismatch_pixels=0;
}

} // namespace detail

// Direct seeds require >=13/25 pixels >=250 in BOTH corresponding 5x5 patches.
// Only a complete <=1024-pixel native depth component with >=70% original
// direct seeds may inherit rejection. This empirical quality rule grants no
// self/ground/free-space or object-identity authority.
inline StereoQualityResult assess_stereo_saturation(const RectifiedStereoSample & in,
    StereoQualityWorkspace* scratch = nullptr) {
  StereoQualityWorkspace local;
  auto& workspace=scratch ? *scratch : local;
  StereoQualityResult out;
  const uint64_t count = uint64_t(in.width) * in.height;
  if (in.width < 5 || in.height < 5 || count > 4194304 || !in.depth_mm ||
      !in.left || !in.right || in.depth_count != count || in.left_count != count ||
      in.right_count != count) {
    out.reason = "invalid_stereo_layout"; return out;
  }
  if (!in.depth_us || in.depth_us != in.left_us || in.depth_us != in.right_us) {
    out.reason = "stereo_source_stamp_mismatch"; return out;
  }
  if (!in.rectified_geometry_verified || !std::isfinite(in.fx) || in.fx <= 0. ||
      !std::isfinite(in.baseline_mm) || in.baseline_mm <= 0. ||
      in.baseline_mm > 1000. || in.fx > 100000. ||
      !std::isfinite(in.fy)||in.fy<=0.||in.fy>100000.||
      !std::isfinite(in.cx)||!std::isfinite(in.cy)) {
    out.reason = "unverified_rectified_geometry"; return out;
  }
  const std::size_t stride = std::size_t(in.width) + 1;
  const auto integral = [&](const uint8_t * image, std::vector<uint32_t>& result) {
    result.assign(stride * (std::size_t(in.height) + 1), 0);
    for (uint32_t y = 0; y < in.height; ++y) {
      uint32_t sum = 0;
      for (uint32_t x = 0; x < in.width; ++x) {
        sum += image[std::size_t(y) * in.width + x] >= 250;
        result[(y + 1) * stride + x + 1] = result[y * stride + x + 1] + sum;
      }
    }
  };
  integral(in.left,workspace.left_integral);integral(in.right,workspace.right_integral);
  const auto& left=workspace.left_integral;const auto& right=workspace.right_integral;
  const auto bright = [&](const std::vector<uint32_t> & image, int x, int y) {
    const std::size_t a = std::size_t(y - 2) * stride, b = std::size_t(y + 3) * stride;
    // Subtract nested horizontal sums to avoid unsigned intermediate underflow.
    return (image[b + x + 3] - image[b + x - 2]) -
           (image[a + x + 3] - image[a + x - 2]);
  };
  out.pixels.assign(count, StereoQuality::kUnassessed);
  for (uint32_t y = 0; y < in.height; ++y) {
    for (uint32_t x = 0; x < in.width; ++x) {
      const std::size_t i = std::size_t(y) * in.width + x;
      const double z = in.depth_mm[i];
      if (z < 200. || z > 8000. || x < 2 || x + 2 >= in.width ||
          y < 2 || y + 2 >= in.height) {
        ++out.unassessed; continue;
      }
      const double u = double(x) - in.fx * in.baseline_mm / z;
      if (!std::isfinite(u) || u < 1.5 || u >= double(in.width) - 2.5) {
        ++out.unassessed; continue;
      }
      const int rx = int(std::lround(u));
      if (bright(left,int(x),int(y))>=13 && bright(right,rx,int(y))>=13) {
        out.pixels[i] = StereoQuality::kSaturated; ++out.saturated;
      } else {
        out.pixels[i] = StereoQuality::kNotRejected;
      }
    }
  }
  detail::extend_saturation_components(in,out,workspace);
  detail::reject_native_micro_mismatches(in,out);
  out.valid = true; out.reason = "ok";
  return out;
}
}  // namespace sunhorizon_camera_runtime
