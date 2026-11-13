#include "sunhorizon_camera_runtime/exposure_cap_transaction.hpp"
#include "sunhorizon_camera_runtime/ae_target_transaction.hpp"
#include <sys/mman.h>
#include <sys/wait.h>
#include <fstream>
#include <iostream>
#include <memory>

using namespace sunhorizon_camera_runtime;
namespace fs = std::filesystem;
void check(bool value) {if (!value) throw std::runtime_error("AE transaction test failed");}
template<class F> void rejected(F fn) {bool threw=false;try {fn();} catch (const std::exception &) {threw=true;}check(threw);}
struct Directory {
  fs::path path;
  Directory() {char name[]="/tmp/ae-target-test-XXXXXX";const auto p=::mkdtemp(name);check(p);path=fs::canonical(p);}
  ~Directory() {fs::remove_all(path);}
};
struct Fake {
  int target{80}, writes{}, throw_after_write{}, wrong_read_once{};
  bool read_supported{true}, write_supported{true}, ae_supported{true}, ae{true}, refuse_restore{false};
  ExposureCapRange limits{10,100,5};
  ExposureCapDevice access() {
    return {[this]{return read_supported;},[this]{return write_supported;},[this]{return ae_supported;},
      [this]{if(wrong_read_once && target==40){--wrong_read_once;return 45;}return target;},
      [this]{return limits;},[this]{return ae;},[this](int v){
        ++writes;if(refuse_restore && v==80) throw std::runtime_error("SDK restoration unavailable");
        target=v;if(throw_after_write){--throw_after_write;throw std::runtime_error("SDK failed after write");}}};
  }
};
std::string value(const ExposureCapTransaction & t,const std::string & key) {
  for(const auto & p:t.receipt())if(p.first==key)return p.second;
  throw std::runtime_error("missing receipt field");
}
int main() {
  unsigned cases=0;
  {Directory d;Fake f;{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(-1);check(f.writes==0);}check(f.writes==0);++cases;}
  {Directory d;Fake f;{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(40);check(f.target==40);check(value(t,"exposure_cap_restore_pending")=="true");}check(f.target==80 && f.writes==2);++cases;}
  {Directory d;Fake f;ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(40);t.restore();check(f.target==80);check(value(t,"exposure_cap_transaction_state")=="restored_verified");t.restore();check(f.writes==2);++cases;}
  for(int request:{9,41,105,80,85,-3,-2}) {Directory d;Fake f;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(request);});check(f.writes==0 && f.target==80);++cases;}
  for(int unsupported=0;unsupported<3;++unsupported) {Directory d;Fake f;
    if(unsupported==0)f.read_supported=false;if(unsupported==1)f.write_supported=false;if(unsupported==2)f.ae_supported=false;
    ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.writes==0);++cases;}
  {Directory d;Fake f;f.ae=false;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.writes==0 && !f.ae);++cases;}
  {Directory d;Fake f;f.limits.step=0;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.writes==0);++cases;}
  {Directory d;Fake f;f.throw_after_write=1;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.target==80 && f.writes==2);check(value(t,"exposure_cap_restore_pending")=="false");++cases;}
  {Directory d;Fake f;f.wrong_read_once=1;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.target==80);++cases;}
  {Directory d;Fake f;{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(40);f.refuse_restore=true;check(!t.restore_noexcept());check(value(t,"exposure_cap_restore_pending")=="true");}
    check(f.target==40);f.refuse_restore=false;{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(-1);check(f.target==80);check(value(t,"exposure_cap_startup_recovered")=="true");}++cases;}
  {Directory d;Fake f;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{ExposureCapTransaction other(d.path,"SERIAL1",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;rejected([&]{ExposureCapTransaction t(d.path,"../bad",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;auto raw=d.path/"SERIAL1.ir-ae-cap.txn";{std::ofstream o(raw);o<<"bad journal\n";}fs::permissions(raw,fs::perms::owner_read|fs::perms::owner_write);rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;auto raw=d.path/"SERIAL1.ir-ae-cap.txn";{std::ofstream o(raw);o<<"IR_AE_CAP_V1\nOTHER\n190\napplied\n80\n40\n1\n";}fs::permissions(raw,fs::perms::owner_read|fs::perms::owner_write);rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;fs::create_symlink(d.path/"missing",d.path/"SERIAL1.ir-ae-cap.txn");rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;ExposureCapTransaction t(d.path/"absent","SERIAL1",f.access());t.begin(-1);rejected([&]{t.begin(40);});check(f.writes==0);++cases;}
  {Directory d;Fake f;try {ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(40);throw std::runtime_error("subsequent pipeline configuration failed");} catch (const std::exception &) {}check(f.target==80 && f.writes==2);++cases;}
  {Directory d;Fake f;const auto temporary=d.path/("SERIAL1.ir-ae-cap.tmp."+std::to_string(::getpid()));fs::create_symlink(d.path/"other",temporary);
    ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(40);});check(f.target==80 && f.writes==0);++cases;}
  // Abrupt process exit bypasses every C++ destructor. The fake target is in
  // shared memory to model hardware persisting independently of its SDK owner.
  for (const bool crash_during_set : {false,true}) {Directory d;auto * target=static_cast<int *>(::mmap(nullptr,sizeof(int),PROT_READ|PROT_WRITE,MAP_SHARED|MAP_ANON,-1,0));check(target!=MAP_FAILED);*target=80;
    auto access=[&]{return ExposureCapDevice{[]{return true;},[]{return true;},[]{return true;},[=]{return *target;},[]{return ExposureCapRange{10,100,5};},[]{return true;},[=](int v){*target=v;if (crash_during_set && v==40) ::_exit(0);}};};
    const auto child=::fork();check(child>=0);
    if(child==0){ExposureCapTransaction t(d.path,"SERIAL1",access());t.begin(40);::_exit(0);}
    int status{};check(::waitpid(child,&status,0)==child && WIFEXITED(status) && WEXITSTATUS(status)==0);check(*target==40);
    {ExposureCapTransaction t(d.path,"SERIAL1",access());check(*target==80);t.begin(-1);check(value(t,"exposure_cap_transaction_state")=="recovered_before_capture");}
    ::munmap(target,sizeof(int));++cases;
  }
  {Directory d;Fake f;f.target=11;ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(-2);});check(f.writes==0);++cases;}
  {Directory d;Fake f;f.target=0;f.limits={0,100,1};ExposureCapTransaction t(d.path,"SERIAL1",f.access());rejected([&]{t.begin(-2);});check(f.writes==0);++cases;}
  {Directory d;Fake f;{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(40);f.ae=false;check(!t.restore_noexcept());check(f.target==80);}
    rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});f.ae=true;
    {ExposureCapTransaction t(d.path,"SERIAL1",f.access());check(value(t,"exposure_cap_startup_recovered")=="true");}++cases;}
  // PREPARED is equally unresolved: the hardware write might have completed
  // just before a process crash but before APPLIED could be persisted.
  {Directory d;Fake f;f.target=40;auto raw=d.path/"SERIAL1.ir-ae-cap.txn";{std::ofstream o(raw);o<<"IR_AE_CAP_V1\nSERIAL1\n190\nprepared\n80\n40\n1\n";}fs::permissions(raw,fs::perms::owner_read|fs::perms::owner_write);
    {ExposureCapTransaction t(d.path,"SERIAL1",f.access());check(f.target==80 && f.writes==1);}++cases;}
  {Directory d;Fake f;f.target=40;auto raw=d.path/"SERIAL1.ir-ae-cap.txn";{std::ofstream o(raw);o<<"IR_AE_CAP_V1\nSERIAL1\n190\napplied\n80\n40\n1\n";}fs::permissions(raw,fs::perms::owner_read|fs::perms::owner_write);
    f.write_supported=false;rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});check(f.target==40 && f.writes==0);f.write_supported=true;
    {ExposureCapTransaction t(d.path,"SERIAL1",f.access());check(f.target==80);}++cases;}
  {Directory d;Fake f;f.target=7582;f.limits={1,166000,1};{ExposureCapTransaction t(d.path,"SERIAL1",f.access());t.begin(500);check(f.target==500);}check(f.target==7582);++cases;}
  {Directory d;Fake f;AeTargetDevice old{[]{return true;},[]{return true;},[]{return true;},[]{return 60;},[]{return AeTargetRange{0,255,1};},[]{return true;},[](int){throw std::runtime_error("old property must never be written");}};
    AeTargetTransaction runtime_owner(d.path,"SERIAL1",old);rejected([&]{ExposureCapTransaction t(d.path,"SERIAL1",f.access());});check(f.writes==0);++cases;}
  {Directory d;Fake f;auto a=f.access();bool prepared=false;
    a.write_cap=[&](int v){if(v==40){std::ifstream in(d.path/"SERIAL1.ir-ae-cap.txn");std::string raw((std::istreambuf_iterator<char>(in)),{});prepared=raw=="IR_AE_CAP_V1\nSERIAL1\n190\nprepared\n80\n40\n1\n";}f.target=v;++f.writes;};
    {ExposureCapTransaction t(d.path,"SERIAL1",a);t.begin(40);check(prepared);}check(f.target==80);++cases;}
  std::cout<<"Exposure cap: "<<cases<<" fake SDK/persistent journal scenarios passed\n";
}
