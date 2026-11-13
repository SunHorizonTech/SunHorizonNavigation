#pragma once

#include <rclcpp/context.hpp>
#include <rclcpp/init_options.hpp>
#include <rclcpp/executor.hpp>
#include <exception>
#include <memory>
#include <stdexcept>

namespace sunhorizon_navigation_runtime::detail {
inline void require_joint_context_domain(const rclcpp::Context::SharedPtr& control,
    const rclcpp::Context::SharedPtr& joint) {
  if(!control || !joint || !control->is_valid() || !joint->is_valid() ||
      control->get_domain_id()!=joint->get_domain_id())
    throw std::invalid_argument("joint ingress requires valid ROS contexts in the same domain");
}

inline rclcpp::Context::SharedPtr make_joint_context(const rclcpp::Context::SharedPtr& control,
    int argc,const char* const* argv) {
  if(!control || !control->is_valid())
    throw std::invalid_argument("joint context requires initialized control context");
  auto options=control->get_init_options();
  // Copy all RMW init settings, but freeze the actual resolved domain and leave
  // signal shutdown to the main lifecycle owner after joint proof egress drains.
  options.set_domain_id(control->get_domain_id());
  options.shutdown_on_signal=false;
  auto joint=std::make_shared<rclcpp::Context>();
  try {
    joint->init(argc,argv,options); // Full global params/remaps, never init(0,nullptr).
    require_joint_context_domain(control,joint);
  } catch(...) {
    if(joint->is_valid())joint->shutdown("joint context initialization failed");
    throw;
  }
  return joint;
}

// Called only by the owned, joined joint executor thread. No context callback
// captures the runtime; an unexpected context loss/cancel cannot silently stop FK.
template<class Stopping,class Fault>
void spin_joint_executor(rclcpp::Executor& executor,const rclcpp::Context::SharedPtr& control,
    Stopping stopping,Fault fault) {
  try {
    executor.spin();
    if(!stopping() && control && control->is_valid())fault("joint_executor_stopped_unexpectedly");
  } catch(const std::exception& error) {fault(error.what());}
  catch(...) {fault("joint_executor_failure");}
}
}  // namespace sunhorizon_navigation_runtime::detail
