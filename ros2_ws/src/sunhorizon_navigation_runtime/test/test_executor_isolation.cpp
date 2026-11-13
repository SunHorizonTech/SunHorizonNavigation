#include "sunhorizon_navigation_runtime/detail/executor_options.hpp"
#include "sunhorizon_navigation_runtime/detail/joint_context.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <array>
#include <cstring>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {
using namespace std::chrono_literals;
using Message=std_msgs::msg::UInt64;
void require(bool condition,const char* reason) {
  if(!condition)throw std::runtime_error(reason); // Keep checks in Release/NDEBUG.
}
void concurrent_executors(const rclcpp::Context::SharedPtr& context,unsigned round,
    const rclcpp::Context::SharedPtr& joint_context,bool exercise_context_loss=false) {
  namespace detail=sunhorizon_navigation_runtime::detail;
  const auto control_options=detail::make_executor_options(context);
  const auto joint_options=detail::make_executor_options(joint_context);
  require(control_options.context==context && joint_options.context==joint_context,"executor context changed");
  require(control_options.memory_strategy && joint_options.memory_strategy,"missing memory strategy");
  // This fails deterministically if the original copied-options bug returns;
  // the runtime below additionally exercises real simultaneous ROS wait sets.
  require(control_options.memory_strategy!=joint_options.memory_strategy,"executors share mutable memory strategy");
  rclcpp::executors::MultiThreadedExecutor control(control_options,4);
  rclcpp::executors::SingleThreadedExecutor joints(joint_options);
  auto node_options=rclcpp::NodeOptions().context(context).use_global_arguments(exercise_context_loss)
    .enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
  const auto name="executor_isolation_"+std::to_string(getpid())+"_"+std::to_string(round);
  auto control_node=std::make_shared<rclcpp::Node>(name+"_control",node_options);
  auto joint_options_for_node=node_options;
  joint_options_for_node.context(joint_context);
  auto joint_node=std::make_shared<rclcpp::Node>(name+"_joint",joint_options_for_node);
  if(exercise_context_loss){
    require(joint_context!=context,"joint context was not isolated");
    detail::require_joint_context_domain(context,joint_context);
    require(joint_node->get_name()==name+"_joint_remapped","secondary lost full-argv node remap");
    require(joint_node->declare_parameter<int64_t>("context_argument",0)==73,"secondary lost global parameter");
    require(control_node->declare_parameter<int64_t>("context_argument",0)==73,"control lost global parameter");
  }
  std::atomic<uint64_t> control_received{0},joint_received{0};
  std::atomic<bool> gid_mismatch{false},stopping{false};
  std::array<uint8_t,RMW_GID_STORAGE_SIZE> expected_joint_gid{};
  const auto control_topic="/"+name+"/control",joint_topic="/"+name+"/joint";
  std::vector<rclcpp::Subscription<Message>::SharedPtr> subscriptions;
  // Different capacities make shared-strategy interference observable rather
  // than giving both executor wait sets the same accidental dimensions.
  for(unsigned i=0;i<11;++i)subscriptions.push_back(control_node->create_subscription<Message>(
    control_topic,rclcpp::QoS(10).best_effort(),
    [&](Message::ConstSharedPtr){++control_received;}));
  subscriptions.push_back(joint_node->create_subscription<Message>(joint_topic,rclcpp::QoS(10).best_effort(),
    [&](Message::ConstSharedPtr,const rclcpp::MessageInfo& info){
      if(std::memcmp(info.get_rmw_message_info().publisher_gid.data,expected_joint_gid.data(),
          expected_joint_gid.size())!=0)gid_mismatch.store(true);
      ++joint_received;
    }));
  auto control_pub=control_node->create_publisher<Message>(control_topic,rclcpp::QoS(10).best_effort());
  // The source belongs to the control context. The separate-context round
  // must traverse the real RMW boundary and preserve that actual publisher GID.
  auto joint_pub=control_node->create_publisher<Message>(joint_topic,rclcpp::QoS(10).best_effort());
  const auto gid=joint_pub->get_gid();
  std::memcpy(expected_joint_gid.data(),gid.data,expected_joint_gid.size());
  auto control_timer=control_node->create_wall_timer(1ms,[&]{control_pub->publish(Message{});});
  auto joint_timer=joint_node->create_wall_timer(1ms,[&]{joint_pub->publish(Message{});});
  control.add_node(control_node);joints.add_node(joint_node);
  std::mutex mutex;std::condition_variable changed;std::exception_ptr failure;std::string joint_failure;
  auto spin=[&](rclcpp::Executor& executor){
    try {executor.spin();}
    catch(...){std::lock_guard<std::mutex> lock(mutex);if(!failure)failure=std::current_exception();}
    changed.notify_all();
  };
  std::thread control_thread,joint_thread;
  auto stop=[&]{
    stopping.store(true);
    control.cancel();joints.cancel();
    if(control_thread.joinable())control_thread.join();
    if(joint_thread.joinable())joint_thread.join();
  };
  try {
    control_thread=std::thread([&]{spin(control);});
    joint_thread=std::thread([&]{
      detail::spin_joint_executor(joints,context,[&]{return stopping.load();},[&](const std::string& reason){
        {std::lock_guard<std::mutex> lock(mutex);
          joint_failure=reason;
          if(!exercise_context_loss && !failure)failure=std::make_exception_ptr(std::runtime_error(reason));}
        changed.notify_all();
      });
    });
    const auto begin=std::chrono::steady_clock::now(),deadline=begin+5s;
    std::unique_lock<std::mutex> lock(mutex);
    while(!failure && std::chrono::steady_clock::now()<deadline){
      // Keep concurrent wait-set rebuilding active after initial discovery.
      if(std::chrono::steady_clock::now()-begin>=500ms &&
         control_received.load()>=100 && joint_received.load()>=30)break;
      changed.wait_for(lock,10ms);
    }
    if(exercise_context_loss && !failure){
      require(joint_failure.empty(),"joint executor failed before intentional context loss");
      lock.unlock();joint_context->shutdown("intentional secondary-context loss");lock.lock();
      const auto loss_deadline=std::chrono::steady_clock::now()+2s;
      while(joint_failure.empty() && !failure && std::chrono::steady_clock::now()<loss_deadline)
        changed.wait_for(lock,10ms);
      require(!joint_failure.empty(),"secondary loss silently stopped the joint executor");
      require(context->is_valid(),"secondary shutdown stopped the control context");
      const auto prior=control_received.load();
      const auto progress_deadline=std::chrono::steady_clock::now()+500ms;
      while(control_received.load()<=prior && !failure && std::chrono::steady_clock::now()<progress_deadline)
        changed.wait_for(lock,10ms);
      require(control_received.load()>prior,"control callbacks stopped with secondary context");
    }
    lock.unlock();stop();
    if(failure)std::rethrow_exception(failure);
    require(!gid_mismatch.load(),"cross-context callback lost the real publisher GID");
    require(control_received.load()>=100,"control executor callbacks stopped progressing");
    require(joint_received.load()>=30,"joint executor callbacks stopped progressing");
  }catch(...){
    // Also closes a thread that had not reached spin() when its peer failed.
    if(context->is_valid())context->shutdown("executor isolation worker failure");
    if(joint_context!=context && joint_context->is_valid())joint_context->shutdown("joint executor isolation worker failure");
    stop();throw;
  }
  control.remove_node(control_node);joints.remove_node(joint_node);
}
}  // namespace
int main() {
  namespace detail=sunhorizon_navigation_runtime::detail;
  auto context=std::make_shared<rclcpp::Context>();
  rclcpp::Context::SharedPtr isolated;
  try {
    const auto joint_name="executor_isolation_"+std::to_string(getpid())+"_3_joint";
    const std::vector<std::string> arguments{"executor_isolation","--ros-args","-r",
      joint_name+":__node:="+joint_name+"_remapped","-p","context_argument:=73"};
    std::vector<const char*> argv;
    for(const auto& argument:arguments)argv.push_back(argument.c_str());
    const int argc=static_cast<int>(argv.size());
    rclcpp::InitOptions options;options.set_domain_id(221);
    context->init(argc,argv.data(),options);
    detail::require_joint_context_domain(context,context); // Legacy embedding remains supported.
    for(unsigned round=0;round<3;++round)concurrent_executors(context,round,context);
    {
      auto different=std::make_shared<rclcpp::Context>();
      auto other_options=context->get_init_options();other_options.set_domain_id(222);
      other_options.shutdown_on_signal=false;different->init(argc,argv.data(),other_options);
      bool rejected=false;
      try{detail::require_joint_context_domain(context,different);}
      catch(const std::invalid_argument&){rejected=true;}
      different->shutdown("different-domain rejection test complete");
      require(rejected,"different actual ROS domain was accepted");
    }
    const auto control_signal_setting=context->get_init_options().shutdown_on_signal;
    isolated=detail::make_joint_context(context,argc,argv.data());
    require(context->get_init_options().shutdown_on_signal==control_signal_setting,"secondary mutated control init options");
    require(isolated!=context && isolated->is_valid(),"secondary context creation failed");
    require(isolated->get_domain_id()==context->get_domain_id(),"resolved ROS domain changed");
    require(!isolated->get_init_options().shutdown_on_signal,"secondary signal shutdown was not deferred");
    concurrent_executors(context,3,isolated,true);
    require(!isolated->is_valid() && context->is_valid(),"independent shutdown was not retained");
    bool rejected=false;
    try{detail::require_joint_context_domain(context,isolated);}
    catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"invalid secondary context was accepted");
    context->shutdown("executor isolation test complete");
    std::cout<<"PASS independent strategies, three legacy rounds, full-argv isolated participant and fail-closed context loss\n";return 0;
  }catch(const std::exception& error){
    if(isolated && isolated->is_valid())isolated->shutdown("joint executor isolation test failure");
    if(context->is_valid())context->shutdown("executor isolation test failure");
    std::cerr<<"FAIL "<<error.what()<<'\n';return 1;
  }
}
