#include "sunhorizon_navigation_runtime/obstacle_source_policy.hpp"
#include <stdexcept>
#include <iostream>
using namespace sunhorizon_navigation_runtime;
static void check(bool value) {if(!value)throw std::runtime_error("source policy check");}
int main() {
  check(obstacle_uses_airy("lidar-rgbd-v1"));
  check(!obstacle_uses_airy("rgbd-only-v1"));
  for(const auto& mode:{"lidar-rgbd-v1","rgbd-only-v1"}) {
    for(const auto& profile:{"fusion_shadow","visual","lidar"})for(bool spatial:{false,true}) {
      bool admitted=true;
      try {require_obstacle_coverage_profile(mode,profile,spatial);}catch(const std::invalid_argument&){admitted=false;}
      check(admitted==(spatial && std::string(profile)==(obstacle_uses_airy(mode)?"fusion_shadow":"visual")));
    }
  }
  bool rejected=false;try{obstacle_uses_airy("none");}catch(const std::invalid_argument&){rejected=true;}
  check(rejected);std::cout<<"obstacle source policy passed\n";
}
