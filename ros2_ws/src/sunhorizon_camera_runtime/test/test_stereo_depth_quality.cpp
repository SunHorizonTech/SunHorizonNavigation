#include "sunhorizon_camera_runtime/stereo_depth_quality.hpp"
#include "sunhorizon_camera_runtime/depth_measurement_quality.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace sunhorizon_camera_runtime;
void require(bool value) { if (!value) throw std::runtime_error("quality test failed"); }
// Two-depth-layer rectified rendering. Hidden background remains visible in
// right-eye disocclusions; foreground rays overwrite it at their own correct
// native disparity. Background-dominated patch correspondence is not authority
// to delete these correct foreground measurements.
void stereo_mismatch_tests() {
  constexpr std::size_t w=96,h=20,u=64,v=10,at=v*w+u;
  constexpr double fx=370.9275207519531,baseline=50.09230041503906;
  std::vector<uint16_t> z(w*h);std::vector<uint8_t> left(w*h),right(w*h);
  RectifiedStereoSample in{z.data(),left.data(),right.data(),z.size(),left.size(),right.size(),w,h,
    100,100,100,fx,baseline,true,fx,47.5,9.5};
  for(uint16_t foreground:{uint16_t(400),uint16_t(500),uint16_t(600),uint16_t(800)})
    for(uint16_t step:{uint16_t(30),uint16_t(50)})for(bool shadow:{false,true}) {
      const int native_disparity=int(std::lround(fx*baseline/foreground));
      const int background_disparity=int(std::lround(fx*baseline/(foreground+step)));
      bool background_dominates=false;
      for(uint32_t seed=0;seed<100;++seed) {
        uint32_t random=seed+1;
        for(auto& a:left){random=random*1664525u+1013904223u;a=uint8_t(20+(random>>24)%200);}
        std::fill(z.begin(),z.end(),foreground+step);
        std::fill(right.begin(),right.end(),80);
        for(std::size_t y=0;y<h;++y)for(std::size_t x=0;x+background_disparity<w;++x)
          right[y*w+x]=left[y*w+x+background_disparity];
        for(const auto i:{at,at+1,at+w,at+w+1}){
          z[i]=foreground;right[i-native_disparity]=left[i];
          require(right[i-native_disparity]==left[i]); // correct foreground correspondence
        }
        if(shadow){for(auto& a:left)a=uint8_t(a/2);for(auto& a:right)a=uint8_t(a/2);}
        // Establish the counterexample's window premise independently: the
        // surrounding background can beat the correct SDK patch substantially.
        const auto ncc=[&](std::size_t index,int disparity){
          double sl=0,sr=0,sll=0,srr=0,slr=0;
          for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){
            const double l=left[index+dy*int(w)+dx],r=right[index+dy*int(w)+dx-disparity];
            sl+=l;sr+=r;sll+=l*l;srr+=r*r;slr+=l*r;
          }
          const double den=(25*sll-sl*sl)*(25*srr-sr*sr);
          return den>0?(25*slr-sl*sr)/std::sqrt(den):-2.;
        };
        for(const auto i:{at,at+1,at+w,at+w+1})
          background_dominates|=ncc(i,background_disparity)>.9&&
            ncc(i,background_disparity)-ncc(i,native_disparity)>.2;
        for(std::size_t budget:{std::size_t(0),std::size_t(224),std::size_t(65536)}){
          in.maximum_stereo_match_checks=budget;
          const auto q=assess_stereo_saturation(in);
          require(q.valid&&q.stereo_match_complete&&q.stereo_match_checks==0&&q.stereo_mismatch_pixels==0);
          require(q.stereo_match_reason=="disabled_foreground_ambiguity");
          for(const auto i:{at,at+1,at+w,at+w+1})require(q.pixels[i]==StereoQuality::kNotRejected);
        }
      }
      // At800mm small depth differences may round to the same disparity; this
      // is still a retention regression, not a forced mismatch witness.
      if(foreground<=500)require(background_dominates);
    }
  std::fill(left.begin(),left.end(),40);std::fill(right.begin(),right.end(),80);
  auto q=assess_stereo_saturation(in);require(q.valid&&q.stereo_mismatch_pixels==0);
  // The disabled stage cannot overwrite direct/component reasons or counters.
  StereoQualityResult original;original.pixels.assign(z.size(),StereoQuality::kNotRejected);
  original.pixels[0]=StereoQuality::kSaturated;original.pixels[1]=StereoQuality::kSaturationDominatedComponent;
  original.saturated=1;original.component_added=1;
  detail::reject_native_micro_mismatches(in,original);
  require(original.stereo_match_checks==0&&original.stereo_mismatch_pixels==0);
  require(original.pixels[0]==StereoQuality::kSaturated&&original.pixels[1]==StereoQuality::kSaturationDominatedComponent);
  require(original.saturated==1&&original.component_added==1);
}

int main() {
  stereo_mismatch_tests();
  std::vector<uint16_t> depth(32 * 12, 1000);
  std::vector<uint8_t> left(depth.size(), 80), right(depth.size(), 80);
  RectifiedStereoSample in{depth.data(), left.data(), right.data(), depth.size(),
    left.size(), right.size(), 32, 12, 100, 100, 100, 100., 50., true,100.,16.,6.};
  const std::size_t point = 6 * 32 + 20;
  auto result = assess_stereo_saturation(in);
  require(result.valid && result.pixels[point] == StereoQuality::kNotRejected);
  require(result.pixels[0] == StereoQuality::kUnassessed);
  // The right match is u=15, not u=20. Saturating the wrong position is insufficient.
  for (int y = 4; y <= 8; ++y) for (int x = 18; x <= 22; ++x) {
    left[y * 32 + x] = 255; right[y * 32 + x] = 255;
  }
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kNotRejected);
  for (int y = 4; y <= 8; ++y) for (int x = 13; x <= 17; ++x) right[y * 32 + x] = 255;
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kSaturated);
  std::fill(right.begin(), right.end(), 80);
  for (int i = 0; i < 12; ++i) right[(4 + i / 5) * 32 + 13 + i % 5] = 255;
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kNotRejected);
  right[6 * 32 + 15] = 255;
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kSaturated);
  // A textured bright real surface retains matching information; isolated
  // bright speckles must not trigger the majority-saturation condition.
  std::fill(left.begin(), left.end(), 80); std::fill(right.begin(), right.end(), 80);
  left[point] = 255; right[6 * 32 + 15] = 255;
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kNotRejected);
  // A 3/5 cm foreground object is not rejected by height/range/size. These
  // synthetic negatives test the policy only, not a physical sensor experiment.
  for (const uint16_t object_mm : {uint16_t(370), uint16_t(350)}) {
    std::fill(left.begin(), left.end(), 80); std::fill(right.begin(), right.end(), 80);
    depth[point] = object_mm;
    require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kNotRejected);
    // A real object can itself saturate: it must become unknown, never free or
    // self. IR alone does not establish empty space behind a rejected return.
    std::fill(left.begin(), left.end(), 255); std::fill(right.begin(), right.end(), 255);
    require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kSaturated);
  }
  depth[point] = 0;
  require(assess_stereo_saturation(in).pixels[point] == StereoQuality::kUnassessed);
  in.right_us = 101; require(!assess_stereo_saturation(in).valid); in.right_us = 100;
  in.rectified_geometry_verified = false; require(!assess_stereo_saturation(in).valid);
  in.rectified_geometry_verified = true;
  in.baseline_mm = -50.; require(!assess_stereo_saturation(in).valid);
  in.baseline_mm = 50.; in.fx = std::numeric_limits<double>::quiet_NaN();
  require(!assess_stereo_saturation(in).valid); in.fx = 100.;
  --in.left_count; require(!assess_stereo_saturation(in).valid); ++in.left_count;
  in.width = 0xffffffff; require(!assess_stereo_saturation(in).valid);
  // The bounded component stage consumes only ORIGINAL direct saturation
  // seeds. Inject that exact stage input to test fraction/size/budget boundaries
  // independently from overlapping IR patch arithmetic.
  const auto component=[](std::size_t count,std::size_t seeds,std::size_t budget=65536,
      bool discontinuity=false,bool later_oversize=false,double focal=1000.,bool unassessed_tail=false,bool zero_tail=false) {
    std::vector<uint16_t> z(80*64,0);std::vector<uint8_t> ir(z.size(),80);
    RectifiedStereoSample frame{z.data(),ir.data(),ir.data(),z.size(),z.size(),z.size(),
      80,64,100,100,100,focal,50.,true,focal,40.,32.,budget};
    StereoQualityResult q;q.pixels.assign(z.size(),StereoQuality::kUnassessed);
    for(std::size_t i=0;i<count;++i) {
      const auto at=(5+i/50)*80+5+i%50;z[at]=zero_tail&&i>=seeds?0:(discontinuity&&i>=seeds?1100:1000);
      q.pixels[at]=i<seeds?StereoQuality::kSaturated:(unassessed_tail?StereoQuality::kUnassessed:StereoQuality::kNotRejected);
    }
    if(later_oversize)for(std::size_t i=0;i<1025;++i){const auto at=(32+i/50)*80+5+i%50;z[at]=1000;q.pixels[at]=i?StereoQuality::kNotRejected:StereoQuality::kSaturated;}
    for(const auto v:q.pixels){q.saturated+=v==StereoQuality::kSaturated;q.unassessed+=v==StereoQuality::kUnassessed;}
    detail::extend_saturation_components(frame,q);return q;
  };
  auto q=component(10,7);require(q.component_extension_complete&&q.component_added==3&&q.saturated==7);
  require(q.pixels[5*80+12]==StereoQuality::kSaturationDominatedComponent);
  q=component(10,6);require(q.component_extension_complete&&q.component_added==0);
  q=component(1024,717);require(q.component_extension_complete&&q.component_added==307);
  q=component(1025,718);require(!q.component_extension_complete&&q.component_added==0&&q.component_extension_reason=="component_oversize");
  q=component(10,7,2);require(!q.component_extension_complete&&q.component_added==0&&q.component_extension_reason=="component_work_budget");
  q=component(10,7,65536,false,true);require(!q.component_extension_complete&&q.component_added==0); // no prior small-component extras leak
  q=component(10,7,65536,true);require(q.component_extension_complete&&q.component_added==0); // raw depth discontinuity
  q=component(1024,1);require(q.component_extension_complete&&q.component_added==0); // large textured floor/obstacle preserved
  q=component(10,7,40);require(q.component_extension_complete&&q.component_added==3&&q.component_checks==40);
  q=component(10,7,39);require(!q.component_extension_complete&&q.component_added==0&&q.component_checks==39);
  q=component(10,7,40,false,true);require(!q.component_extension_complete&&q.component_added==0&&q.component_extension_reason=="component_work_budget");
  q=component(10,7,65536,false,false,100.);require(q.component_extension_complete&&q.component_added==3); // <=10mm includes boundary
  q=component(10,7,65536,false,false,99.9);require(q.component_extension_complete&&q.component_added==0); // >10mm never bridges
  q=component(10,7,65536,false,false,1000.,true);require(q.component_added==3&&q.pixels[5*80+12]==StereoQuality::kSaturationDominatedComponent); // valid unassessed border
  q=component(10,0,65536,false,false,1000.,true);require(q.component_added==0&&q.pixels[5*80+5]==StereoQuality::kUnassessed);
  q=component(10,7,65536,false,false,1000.,true,true);require(q.component_added==0&&q.pixels[5*80+12]==StereoQuality::kUnassessed); // invalid depth never grows
  q=component(10,0);require(q.component_extension_complete&&q.component_added==0); // no recursive seed generation
  in.width=32;in.fy=0;require(!assess_stereo_saturation(in).valid);in.fy=100;
  in.cx=std::numeric_limits<double>::quiet_NaN();require(!assess_stereo_saturation(in).valid);in.cx=16;
  // Original immutable token must name this exact rejection algorithm. A
  // historical v1 mask cannot acquire v2 rejection authority through relabeling.
  DepthMeasurementQuality proof;proof.role="belly";proof.source_frame="optical";
  proof.opened_serial="test";proof.capture_epoch="epoch";proof.source_ns=100000;
  proof.sequence=7;proof.width=proof.height=1;proof.pixels={1};proof.exact_rectified_pair=true;
  proof.stereo_match_complete=true;proof.stereo_match_reason="disabled_foreground_ambiguity";
  proof.device_us={1,1,1};proof.global_us={100,100,100};proof.depth_pixel_type=0;proof.sdk_depth_scale_m=.001;
  proof.image_identity=depth.data();proof.info_identity=left.data();
  require(proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  const std::array<double,9> original_k{{100,0,50,0,100,50,0,0,1}};
  proof.rectified_depth_k=original_k;proof.stereo_baseline_m=.05;
  proof.geometry_depth_device_us=1;proof.geometry_depth_global_us=100;proof.geometry_depth_frame_index=42;proof.geometry_depth_scale_m=.001;
  proof.right_from_depth_translation_m={-.05,0,0};
  require(proof.body_occlusion_baseline(original_k,1,100,42,.001)==0.); // no SDK geometry attestation
  proof.stereo_body_geometry_verified=true;
  require(proof.body_occlusion_baseline(original_k,1,100,42,.001)==.05);
  auto bad_k=original_k;bad_k[0]=101;
  require(proof.body_occlusion_baseline(bad_k,1,100,42,.001)==0.);
  proof.stereo_baseline_m=.06;require(proof.body_occlusion_baseline(original_k,1,100,42,.001)==0.); // wrong baseline vs actual extrinsic
  proof.stereo_baseline_m=.05;proof.right_from_depth_translation_m[1]=.001;
  require(proof.body_occlusion_baseline(original_k,1,100,42,.001)==0.);
  proof.right_from_depth_translation_m[1]=0.;proof.exact_rectified_pair=false;
  require(proof.body_occlusion_baseline(original_k,1,100,42,.001)==.05); // IR absence does not revoke same-depth geometry
  require(proof.body_occlusion_baseline(original_k,1,100,43,.001)==0.);
  require(proof.body_occlusion_baseline(original_k,2,100,42,.001)==0.);
  require(proof.body_occlusion_baseline(original_k,1,101,42,.001)==0.);
  require(proof.body_occlusion_baseline(original_k,1,100,42,.002)==0.);proof.exact_rectified_pair=true;
  proof.stereo_match_reason="complete";
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.stereo_match_reason="disabled_foreground_ambiguity";
  proof.algorithm="dual-ir-native-quality-v3";
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.algorithm="dual-ir-5x5-saturation-v1";
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  require(proof.state(0)==DepthMeasurementState::Unassessed);
  proof.algorithm=kStereoQualityAlgorithm;proof.pixels={3};
  require(proof.state(0)==DepthMeasurementState::Unassessed); // incomplete propagation never rejects
  proof.component_extension_complete=true;proof.component_extension_reason="complete";proof.component_checks=4;
  require(proof.state(0)==DepthMeasurementState::SaturationDominatedComponent&&proof.unreliable(0));
  require(proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.pixels={4};require(proof.state(0)==DepthMeasurementState::Unassessed);
  proof.stereo_match_complete=true;proof.stereo_match_reason="complete";proof.stereo_match_checks=225;
  require(proof.state(0)==DepthMeasurementState::Unassessed&&!proof.unreliable(0));
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.stereo_match_checks=65537;
  require(proof.state(0)==DepthMeasurementState::Unassessed);
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.stereo_match_checks=224;require(proof.state(0)==DepthMeasurementState::Unassessed);
  require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  proof.pixels={5};require(!proof.matches("optical",100000,7,depth.data(),left.data(),1,1));
  require(proof.state(0)==DepthMeasurementState::Unassessed);
  std::cout << "stereo quality contract tests passed\n";
}
