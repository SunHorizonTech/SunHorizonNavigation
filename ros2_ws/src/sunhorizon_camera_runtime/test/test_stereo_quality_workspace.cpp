#include "sunhorizon_camera_runtime/stereo_depth_quality.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
namespace q=sunhorizon_camera_runtime;
void require(bool b){if(!b)throw std::runtime_error("quality workspace mismatch");}
void equal(const q::StereoQualityResult& a,const q::StereoQualityResult& b){
 require(a.valid==b.valid&&a.reason==b.reason&&a.pixels.size()==b.pixels.size());
 require(a.pixels.empty()||!std::memcmp(a.pixels.data(),b.pixels.data(),a.pixels.size()));
 require(a.saturated==b.saturated&&a.unassessed==b.unassessed);
 require(a.component_extension_complete==b.component_extension_complete&&a.component_extension_reason==b.component_extension_reason);
 require(a.component_checks==b.component_checks&&a.component_added==b.component_added);
 require(a.stereo_match_complete==b.stereo_match_complete&&a.stereo_match_reason==b.stereo_match_reason);
 require(a.stereo_match_checks==b.stereo_match_checks&&a.stereo_mismatch_pixels==b.stereo_mismatch_pixels);
}
int main(){
 q::StereoQualityWorkspace scratch;unsigned rng=7;unsigned cases=0;
 auto random=[&]{rng=rng*1664525U+1013904223U;return rng;};
 for(unsigned w:{5U,32U,96U,640U,20U})for(unsigned mode=0;mode<7;++mode){
  const unsigned h=w==640?480:20;const auto n=w*h;
  std::vector<uint16_t> depth(n);std::vector<uint8_t> l(n),r(n);
  for(unsigned i=0;i<n;++i){depth[i]=mode==0?500:mode==1?0:mode==2?8001:200+random()%7801;l[i]=mode==3?255:uint8_t(random()%256);r[i]=mode==3?255:uint8_t(random()%256);}
  q::RectifiedStereoSample in{depth.data(),l.data(),r.data(),n,n,n,w,h,100,100,100,370.9,50.09,true,370.9,double(w)/2,double(h)/2};
  q::RectifiedStereoSample old{depth.data(),l.data(),r.data(),n,n,n,w,h,100,100,100,370.9,50.09,true,370.9,double(w)/2,double(h)/2};
  for(unsigned budget:{0U,1U,17U,65536U,100000U}){
   in.maximum_component_checks=old.maximum_component_checks=budget;
   if(mode==6){in.right_us=old.right_us=101;}
   auto expected=q::assess_stereo_saturation(old);auto actual=q::assess_stereo_saturation(in,&scratch);equal(expected,actual);
   auto retained=actual;q::assess_stereo_saturation(in,&scratch);equal(expected,retained);++cases;
  }
 }
 std::vector<uint16_t> z(640*480,700);std::vector<uint8_t> l(z.size()),r(z.size());
 for(std::size_t i=0;i<z.size();++i){l[i]=uint8_t(random()%256);r[i]=uint8_t(random()%256);}
 q::RectifiedStereoSample in{z.data(),l.data(),r.data(),z.size(),z.size(),z.size(),640,480,100,100,100,370.9,50.09,true,370.9,319.5,239.5};
 scratch.depth_mm.resize(z.size());q::assess_stereo_saturation(in,&scratch);
 const auto* left=scratch.left_integral.data();const auto* right=scratch.right_integral.data();const auto* visited=scratch.visited.data();
 for(int i=0;i<5;++i){auto expected=q::assess_stereo_saturation(in);equal(expected,q::assess_stereo_saturation(in,&scratch));}
 require(left==scratch.left_integral.data()&&right==scratch.right_integral.data()&&visited==scratch.visited.data());
 // Exercise partial-work failure then success on the SAME scratch: no
 // previous visited bits, queued pixels or uncommitted extras may leak.
 for(unsigned count:{10U,1024U,1025U,10U})for(unsigned budget:{65536U,39U,0U,65536U}){
  std::vector<uint16_t> depth(80*64,0);std::vector<uint8_t> ir(depth.size(),80);
  q::RectifiedStereoSample frame{depth.data(),ir.data(),ir.data(),depth.size(),depth.size(),depth.size(),80,64,100,100,100,1000,50,true,1000,40,32,budget};
  q::StereoQualityResult before;before.pixels.assign(depth.size(),q::StereoQuality::kUnassessed);
  for(unsigned i=0;i<count;++i){auto at=(5+i/50)*80+5+i%50;depth[at]=1000;before.pixels[at]=i<count*7/10?q::StereoQuality::kSaturated:q::StereoQuality::kNotRejected;}
  auto expected=before;auto actual=before;q::detail::extend_saturation_components(frame,expected);
  q::detail::extend_saturation_components(frame,actual,scratch);equal(expected,actual);++cases;
 }
 std::cout<<"equal_cases="<<cases<<" storage_reuse_pass\n";
}
