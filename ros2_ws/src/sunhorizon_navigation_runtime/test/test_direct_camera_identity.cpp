#include "sunhorizon_navigation_runtime/detail/direct_camera_identity.hpp"
#include <stdexcept>
#include <thread>

using namespace sunhorizon_navigation_runtime::detail;
using sunhorizon_camera_runtime::CameraSourceIdentity;
void check(bool value) {if(!value)throw std::runtime_error("direct camera identity contract");}
DirectCameraSample sample(uint64_t seq=1) {
  return {std::make_shared<const CameraSourceIdentity>(CameraSourceIdentity{"/camera","actual-sdk-serial","capture-epoch"}),
    seq,1000+int64_t(seq),999+int64_t(seq),2000+int64_t(seq),3000+int64_t(seq)};
}
int main() {
  DirectCameraIdentity tracker;
  check(!tracker.snapshot().samples[0].owner);
  const auto original=sample();tracker.accepted("belly",original);
  tracker.accepted("chest",sample(10));
  for(int i=0;i<1000;++i) {
    const auto value=tracker.snapshot();
    check(value.samples[0].owner==original.owner && value.samples[0].receipt_ros_ns==original.receipt_ros_ns);
    check(value.samples[1].sequence==10 && value.failure.empty());
  }
  std::thread writer([&]{for(uint64_t i=2;i<1000;++i)tracker.accepted("belly",sample(i));});
  for(int i=0;i<1000;++i) {
    const auto value=tracker.snapshot().samples[0];
    check(value.receipt_steady_ns-value.depth_source_ns==2000 && value.receipt_ros_ns-value.depth_source_ns==1000);
  }
  writer.join();check(tracker.snapshot().samples[0].sequence==999);
  tracker.accepted("belly",original);
  check(tracker.snapshot().samples[0].sequence==999);
  check(tracker.snapshot().failure=="direct_camera_measurement_reordered");
  tracker.accepted("belly",sample(1001));check(tracker.snapshot().samples[0].sequence==999);
  DirectCameraIdentity replaced;replaced.accepted("belly",sample());
  auto other=sample(2);other.owner=std::make_shared<const CameraSourceIdentity>(CameraSourceIdentity{"/camera","another-device","capture-epoch"});
  replaced.accepted("belly",other);check(replaced.snapshot().failure=="direct_camera_owner_changed");
  check(replaced.snapshot().samples[0].owner->opened_serial=="actual-sdk-serial");
  DirectCameraIdentity missing;auto invalid=sample();invalid.owner.reset();missing.accepted("chest",invalid);
  check(missing.snapshot().failure=="direct_camera_identity_missing" && !missing.snapshot().samples[1].owner);
}
