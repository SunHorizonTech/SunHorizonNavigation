#include "sunhorizon_rgbd_adapter/dynamic_self_point_filter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <ctime>
#include <stdexcept>
namespace sunhorizon_rgbd_adapter {
namespace {
double worker_cpu_ms() {
#if defined(CLOCK_THREAD_CPUTIME_ID)
  timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)!=0)
    throw std::runtime_error("body_geometry_helper_cpu_clock_failed");
  return double(t.tv_sec)*1000.+double(t.tv_nsec)*1.e-6;
#else
  return 0.;
#endif
}
}
BodyGeometryWorker::BodyGeometryWorker(std::function<void()> check):owner_check_(std::move(check)) {
  if(!owner_check_)throw std::invalid_argument("body_geometry_owner_check_required");
  thread_=std::thread([this]{loop();});
  std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return started_;});
  if(startup_error_){const auto error=startup_error_;lock.unlock();thread_.join();std::rethrow_exception(error);}
}
BodyGeometryWorker::~BodyGeometryWorker() {
  {std::lock_guard<std::mutex> lock(mutex_);closed_=true;}ready_.notify_all();
  if(thread_.joinable())thread_.join();
}
void BodyGeometryWorker::loop() {
  try{owner_check_();}catch(...){startup_error_=std::current_exception();}
  {std::lock_guard<std::mutex> lock(mutex_);started_=true;}ready_.notify_all();
  if(startup_error_)return;
  for(;;){std::function<void()> job;
    {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return closed_||bool(job_);});
      if(!job_&&closed_)return;job=std::move(job_);}
    job();
  }
}
BodyGeometryWorker::Frame BodyGeometryWorker::begin_frame() {
  std::unique_lock<std::mutex> lock(frame_mutex_,std::try_to_lock);
  if(!lock.owns_lock())throw std::runtime_error("body_geometry_concurrent_frame");
  frame_cpu_ms_=0.;run(owner_check_,[]{});return Frame(std::move(lock));
}
void BodyGeometryWorker::run(std::function<void()> helper,const std::function<void()>& caller) {
  auto done=std::make_shared<std::promise<void>>();auto future=done->get_future();
  {std::lock_guard<std::mutex> lock(mutex_);
    if(worker_error_)std::rethrow_exception(worker_error_);
    if(closed_||job_)throw std::runtime_error("body_geometry_worker_closed_or_busy");
    job_=[this,helper=std::move(helper),done]{try{
      const auto before=worker_cpu_ms();helper();frame_cpu_ms_+=worker_cpu_ms()-before;done->set_value();
    }catch(...){const auto error=std::current_exception();{std::lock_guard<std::mutex> lock(mutex_);worker_error_=error;}done->set_exception(error);}};
  }ready_.notify_all();
  std::exception_ptr caller_error;try{caller();}catch(...){caller_error=std::current_exception();}
  // Always retrieve the helper outcome before releasing immutable source/model
  // storage, even when the calling half throws.
  future.get();if(caller_error)std::rethrow_exception(caller_error);
}
namespace {
namespace dsg = sunhorizon_obstacle_coverage::dynamic_self_geometry;
double value(const std::uint8_t* point, const DynamicPointField& f, bool swap) {
  std::uint8_t b[8]; std::memcpy(b,point+f.offset,f.scalar_bytes);
  if(swap)std::reverse(b,b+f.scalar_bytes);
  if(f.scalar_bytes==4){float v;std::memcpy(&v,b,4);return v;}
  double v;std::memcpy(&v,b,8);return v;
}
bool validate_cloud(const DynamicCloudView& c, double tolerance,
    const DynamicFilterBudget& budget, DynamicFilterResult& r) {
  auto fail=[&](const char* why){r.reason=why;return false;};
  if(const auto* why=dynamic_filter_budget_error(tolerance,budget))return fail(why);
  if(c.width==0||c.height==0||c.width>budget.max_points/c.height)return fail("point_count_budget_or_empty");
  r.input_count=c.width*c.height;
  if(c.point_step==0||c.point_step>budget.max_bytes/c.width||c.row_step<c.width*c.point_step||
     c.row_step>budget.max_bytes/c.height||c.size!=c.row_step*c.height||!c.data)
    return fail("invalid_pointcloud_layout");
  for(const auto& f:c.xyz)if((f.scalar_bytes!=4&&f.scalar_bytes!=8)||f.offset>c.point_step||
      f.scalar_bytes>c.point_step-f.offset)return fail("invalid_xyz_fields");
  for(std::size_t i=0;i<3;++i)for(std::size_t j=i+1;j<3;++j)
    if(c.xyz[i].offset<c.xyz[j].offset+c.xyz[j].scalar_bytes&&c.xyz[j].offset<c.xyz[i].offset+c.xyz[i].scalar_bytes)
      return fail("overlapping_xyz_fields");
  return true;
}
DynamicFilterResult filter_validated_cloud(const DynamicCloudView& c,
    const dsg::PreparedModel& model, double tolerance, const DynamicFilterBudget& budget) {
  DynamicFilterAccumulator accumulator(model,tolerance,budget,c.width*c.height,c.point_step);
  if(!accumulator.ready())return accumulator.finish();
  const std::uint16_t endian_probe=1;
  const bool swap=c.big_endian!=(*reinterpret_cast<const std::uint8_t*>(&endian_probe)==0);
  for(std::size_t row=0;row<c.height;++row)for(std::size_t col=0;col<c.width;++col){
    const auto* p=c.data+row*c.row_step+col*c.point_step;
    if(!accumulator.append(p,value(p,c.xyz[0],swap),value(p,c.xyz[1],swap),value(p,c.xyz[2],swap)))
      return accumulator.finish();
  }
  return accumulator.finish();
}
}
const char* dynamic_filter_budget_error(double tolerance, const DynamicFilterBudget& budget) {
  if(!std::isfinite(tolerance)||tolerance<0||tolerance>dsg::kSelfSurfaceToleranceM)return "invalid_self_tolerance";
  if(budget.max_points==0||budget.max_points>307200||budget.max_bytes==0||budget.max_bytes>64*1024*1024||
     budget.max_node_visits==0||budget.max_node_visits>32000000||budget.max_triangle_tests==0||budget.max_triangle_tests>16000000)
    return "invalid_frame_budget";
  return nullptr;
}
bool DynamicFilterAccumulator::fail(const char* why) {
  active_=false;result_.complete=false;result_.data.clear();result_.ground_blocked.clear();result_.nominal_model_conflict.clear();result_.measured_reference_ambiguous_count=0;result_.reason=why;return false;
}
DynamicFilterAccumulator::DynamicFilterAccumulator(const dsg::PreparedModel& model,
    double tolerance,const DynamicFilterBudget& budget,std::size_t input_count,std::size_t point_step)
    :model_(model),tolerance_(tolerance),budget_(budget),point_step_(point_step) {
  if(const auto* why=dynamic_filter_budget_error(tolerance,budget)){fail(why);return;}
  if(input_count==0||input_count>budget.max_points){fail("point_count_budget_or_empty");return;}
  result_.input_count=input_count;
  if(point_step==0||point_step>budget.max_bytes/input_count){fail("invalid_pointcloud_layout");return;}
  const auto& query=model.query_budget();
  if(query.max_links!=budget.query.max_links||query.max_node_visits!=budget.query.max_node_visits||
      query.max_triangle_tests!=budget.query.max_triangle_tests){fail("prepared_model_query_budget_mismatch");return;}
  // The legacy dummy validates the whole model even if every real XYZ is invalid.
  // Its work also counts against the frame budget, exactly once per filter call.
  const auto check=dsg::classify_body_surface({0,0,1},1,tolerance,model);
  if(!check.complete){fail(check.reason);return;}
  result_.node_visits=check.node_visits;result_.triangle_tests=check.triangle_tests;
  if(result_.node_visits>budget.max_node_visits||result_.triangle_tests>budget.max_triangle_tests){fail("frame_query_budget");return;}
  try {result_.data.reserve(input_count*point_step);}
  catch(const std::exception&){fail("filter_allocation_failure");return;}
  active_=true;
}
DynamicFilterResult DynamicFilterAccumulator::finish() {
  if(active_) {
    if(result_.retained_count+result_.removed_count!=result_.input_count)fail("incomplete_point_traversal");
    else {result_.complete=true;result_.reason="filtered";active_=false;}
  }
  return std::move(result_);
}
DynamicFilterResult filter_dynamic_self_points(const DynamicCloudView& c,
    const std::vector<dsg::LinkObservation>& links,double tolerance,const DynamicFilterBudget& budget) {
  DynamicFilterResult r;
  if(!validate_cloud(c,tolerance,budget,r))return r;
  const dsg::PreparedModel model(links,budget.query);
  return filter_validated_cloud(c,model,tolerance,budget);
}
DynamicFilterResult filter_dynamic_self_points(const DynamicCloudView& c,
    const dsg::PreparedModel& model,double tolerance,const DynamicFilterBudget& budget) {
  DynamicFilterResult r;
  if(!validate_cloud(c,tolerance,budget,r))return r;
  return filter_validated_cloud(c,model,tolerance,budget);
}
}
