#include <iostream>
#include <stdexcept>
#include "sunhorizon_rgbd_adapter/spatial_visibility_sampling.hpp"
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
#define CHECK(x) do {if (!(x)) throw std::runtime_error(#x);} while(0)
int main() {
  sv::Query q;q.common_frame="base";q.contract_id="contract";q.query_id="query";q.query_stamp_ns=1000000000;
  q.source_contracts={{"belly","epoch",true}};q.cells={sv::AxisAlignedCell{1,{-.1,-.1,1.},{.1,.1,2.}}};
  std::vector<float> values(25,3.);std::vector<sv::ReturnKind> kinds(25,sv::ReturnKind::kExternal);
  sv::DepthRaster d;d.width=d.height=5;d.fx=d.fy=10;d.cx=d.cy=2;d.depth_m=values.data();d.depth_elements=25;d.row_stride_elements=5;
  d.return_kinds=kinds.data();d.kind_elements=25;d.minimum_depth_m=.1;d.maximum_depth_m=5;d.clear_margin_m=.01;
  d.calibration={true,true,true,true,true};d.optical_to_common.source_frame="optical";d.optical_to_common.common_frame="base";
  d.optical_to_common.source_stamp_ns=d.optical_to_common.query_stamp_ns=q.query_stamp_ns;
  sv::Source s;s.source_id="belly";s.epoch="epoch";s.frame_id="optical";s.frame_sequence=1;s.source_stamp_ros_ns=q.query_stamp_ns;
  s.local_receipt_ros_ns=q.query_stamp_ns;s.local_receipt_steady_ns=2000000000;s.expiry_budget_ns=300000000;s.required_for_evaluation=true;s.data=d;
  const auto mask=sunhorizon_rgbd_adapter::spatial_required_pixels(q,d);
  CHECK(std::count(mask.begin(),mask.end(),uint8_t(1))==9);
  CHECK(sv::evaluate(q,{s},1000000001,2000000001).aggregate==sv::Aggregate::kClear);
  // Changing every unused pixel cannot change whole-cell support. Required
  // rays remain exhaustive, including boundary pixel centres and duplicates.
  for (std::size_t i=0;i<mask.size();++i) if (!mask[i]) kinds[i]=sv::ReturnKind::kSelfMatched;
  CHECK(sv::evaluate(q,{s},1000000001,2000000001).aggregate==sv::Aggregate::kClear);
  values[12]=.5f;kinds[12]=sv::ReturnKind::kSelfMatched;
  CHECK(sv::evaluate(q,{s},1000000001,2000000001).aggregate==sv::Aggregate::kVisibilityWait);
  values[12]=1.5f;kinds[12]=sv::ReturnKind::kExternal;
  CHECK(sv::evaluate(q,{s},1000000001,2000000001).aggregate==sv::Aggregate::kOccupied);
  auto outside=q;outside.cells[0].min[0]=10;outside.cells[0].max[0]=11;
  const auto empty=sunhorizon_rgbd_adapter::spatial_required_pixels(outside,d);
  CHECK(std::count(empty.begin(),empty.end(),uint8_t(1))==0);
  CHECK(sv::evaluate(outside,{s},1000000001,2000000001).aggregate==sv::Aggregate::kVisibilityWait);
  auto crossing=q;crossing.cells[0].min[2]=-.1;
  bool rejected=false;try {sunhorizon_rgbd_adapter::spatial_required_pixels(crossing,d);}catch(const std::runtime_error&){rejected=true;}CHECK(rejected);
  auto malformed=d;malformed.width=641;rejected=false;
  try {sunhorizon_rgbd_adapter::spatial_required_pixels(q,malformed);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  auto large=q;large.cells.resize(4096,q.cells.front());auto full=d;full.width=640;full.height=480;full.fx=full.fy=1;full.cx=320;full.cy=240;
  for(auto& cell:large.cells){cell.min={-1000,-1000,1};cell.max={1000,1000,2};}
  rejected=false;try {sunhorizon_rgbd_adapter::spatial_required_pixels(large,full);}catch(const std::runtime_error&){rejected=true;}CHECK(rejected);
  std::cout<<"spatial sampling: 8 groups passed (always-on CHECK)\n";
}
