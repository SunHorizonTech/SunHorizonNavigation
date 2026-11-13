#include "sunhorizon_camera_runtime/capture_thread_affinity.hpp"
#include <iostream>
#include <functional>
#include <thread>
#include <exception>
using namespace sunhorizon_camera_runtime::capture_affinity;
struct Fake {
  mutable State state{{0,1,2,3,6,7},42,0,-3};
  mutable int writes{0},reads{0};
  int fail_write{0},mismatch_read{0};
  State read() const {++reads;auto value=state;if(reads==mismatch_read)value.cpus={0};return value;}
  void setAffinity(const std::vector<int64_t>& cpus) const {
    ++writes;state.cpus=cpus;if(writes==fail_write)throw std::runtime_error("set_failure");
  }
};
void need(bool value){if(!value)throw std::runtime_error("test failure");}
void fails(const std::function<void()>& fn,const std::string& expected){
  try{fn();}catch(const std::exception& e){need(std::string(e.what()).find(expected)!=std::string::npos);return;}
  throw std::runtime_error("expected failure");
}
void native_test(){
#ifdef __linux__
  NativeOps ops;
  const auto before=ops.read();
  if(before.policy!=0)throw std::runtime_error("native test requires SCHED_OTHER");
  // Capability check changes only this thread, then immediately restores it.
  // with_affinity readback rejects online/cpuset-restricted masks precisely.
  auto child_check=[&] {
    std::exception_ptr failure;
    std::thread child([&] {
      try {
        const auto inherited=ops.read();
        need(inherited.tid!=before.tid);
        need(inherited.cpus==approvedMask());
        need(inherited.policy==before.policy && inherited.nice==before.nice);
      } catch(...) {failure=std::current_exception();}
    });
    child.join();
    if(failure)std::rethrow_exception(failure);
  };
  auto restored=[&] {
    const auto after=ops.read();
    need(after.tid==before.tid && after.cpus==before.cpus);
    need(after.policy==before.policy && after.nice==before.nice);
  };
  with_affinity(approvedMask(),ops,child_check);
  restored();
  fails([&] {
    with_affinity(approvedMask(),ops,[&] {
      child_check();
      throw std::runtime_error("native_constructor_failure");
    });
  },"native_constructor_failure");
  restored();
  std::cout<<"native thread inheritance and caller restoration passed\n";
#else
  throw std::runtime_error("native affinity test requires Linux");
#endif
}
int main(int argc,char** argv){
  if(argc==2 && std::string(argv[1])=="--native"){native_test();return 0;}
  if(argc!=1)throw std::invalid_argument("usage: test_capture_thread_affinity [--native]");
  const auto mask=approvedMask();
  {Fake f;bool called=false;with_affinity({},f,[&]{called=true;});need(called&&f.reads==0&&f.writes==0);}
  {Fake f;const auto old=f.state;with_affinity(mask,f,[&]{need(f.state.cpus==mask);need(f.state.policy==old.policy&&f.state.nice==old.nice);});need(f.state.cpus==old.cpus&&f.writes==2);}
  {Fake f;const auto old=f.state.cpus;fails([&]{with_affinity(mask,f,[]{throw std::runtime_error("constructor_failure");});},"constructor_failure");need(f.state.cpus==old&&f.writes==2);}
  {Fake f;f.fail_write=1;const auto old=f.state.cpus;fails([&]{with_affinity(mask,f,[]{throw std::runtime_error("must_not_run");});},"set_failure");need(f.state.cpus==old&&f.writes==2);}
  {Fake f;f.mismatch_read=2;const auto old=f.state.cpus;fails([&]{with_affinity(mask,f,[]{throw std::runtime_error("must_not_run");});},"readback_mismatch");need(f.state.cpus==old);}
  {Fake f;f.mismatch_read=3;const auto old=f.state.cpus;fails([&]{with_affinity(mask,f,[]{});},"readback_mismatch");need(f.state.cpus==old);}
  {Fake f;f.fail_write=2;fails([&]{with_affinity(mask,f,[]{});},"restore_failed");}
  {Fake f;f.mismatch_read=4;fails([&]{with_affinity(mask,f,[]{});},"restore_failed");}
  {Fake f;f.state.policy=1;fails([&]{with_affinity(mask,f,[]{});},"sched_other");need(f.writes==0);}
  for(const auto& bad:std::vector<std::vector<int64_t>>{{4},{5},{0,1},{0,1,2,3,6,7,8,9,10,11,11}}){Fake f;fails([&]{with_affinity(bad,f,[]{});},"fixed_robot_a_ten_core_mask");need(f.writes==0);}
  {Fake f;const auto old=f.state;const auto nine=serialLivoCompanionMask();
   with_affinity(nine,f,[&]{need(f.state.cpus==nine&&f.state.policy==old.policy&&f.state.nice==old.nice);});
   need(f.state.cpus==old.cpus&&f.writes==2);}
  for(const auto& bad:std::vector<std::vector<int64_t>>{{6},{0,1,2,3,7,8,9,10},{0,1,2,3,7,8,9,10,10},{0,1,2,3,5,7,8,9,10,11}})
   {Fake f;fails([&]{with_affinity(bad,f,[]{});},"fixed_robot_a_ten_core_mask");need(f.writes==0);}
  std::cout<<"capture affinity scope tests passed\n";
}
