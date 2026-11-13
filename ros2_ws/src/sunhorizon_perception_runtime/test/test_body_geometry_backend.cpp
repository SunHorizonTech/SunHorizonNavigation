#include "sunhorizon_perception_runtime/body_geometry_backend.hpp"
#include <cassert>
using namespace sunhorizon_perception_runtime::detail;
int main(){
 assert(resolve_body_geometry_backend("cpu")=="cpu");
 assert(resolve_body_geometry_backend("cuda_arm11")=="cuda_arm11");
 assert(resolve_body_geometry_backend("cpu","cuda_arm11")=="cuda_arm11");
 assert(resolve_body_geometry_backend("cuda_arm11","cpu")=="cpu");
 assert(body_geometry_cpu_parallel(resolve_body_geometry_backend("cpu","inherit"),true));
 assert(!body_geometry_cpu_parallel(resolve_body_geometry_backend("cpu","cuda_arm11"),true));
 assert(!body_geometry_cpu_parallel("cpu",false));
 for(const auto& pair:{std::pair<std::string,std::string>{"gpu","cpu"},{"cpu","gpu"},{"inherit","cpu"},{"cpu",""}}){
  bool threw=false;try{resolve_body_geometry_backend(pair.first,pair.second);}catch(const std::invalid_argument&){threw=true;}assert(threw);
 }
}
