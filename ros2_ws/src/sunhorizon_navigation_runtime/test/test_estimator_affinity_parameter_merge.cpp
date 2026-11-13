#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace {
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
void write(const std::filesystem::path& path,const std::string& text) {
  std::ofstream stream(path);stream<<text;require(bool(stream),"write YAML fixture");
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int result=0;
  const auto root=std::filesystem::temp_directory_path()/("shn-estimator-merge-"+std::to_string(getpid()));
  try {
    std::filesystem::create_directory(root);
    const std::vector<int64_t> approved{0,1,2,3,6,7,8,9,10,11};
    for(int test=0;test!=3;++test) {
      // Both flattened full-runtime inputs and the original nested source style
      // must merge with the renderer's single flat-dot-key node overlay.
      write(root/"baseline.yaml",test==2?
        "laserMapping:\n  ros__parameters:\n    common:\n      estimator_thread_nice: -5\n      opencv_threads: 1\n    lio:\n      min_iterations: 5\n":
        "laserMapping:\n  ros__parameters:\n    common.estimator_thread_nice: -5\n    common.opencv_threads: 1\n    lio.min_iterations: 5\n");
      write(root/"stage.yaml",test==0?
        "other_node:\n  ros__parameters:\n    common.estimator_thread_nice: 7\n":
        "laserMapping:\n  ros__parameters:\n    common.estimator_thread_cpu_affinity: [0, 1, 2, 3, 6, 7, 8, 9, 10, 11]\n");
      rclcpp::NodeOptions options;options.use_global_arguments(false).arguments({"--ros-args",
        "--params-file",(root/"baseline.yaml").string(),"--params-file",(root/"stage.yaml").string()});
      auto node=std::make_shared<rclcpp::Node>("laserMapping",options);
      rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
      const auto mask=node->declare_parameter<std::vector<int64_t>>("common.estimator_thread_cpu_affinity",{},descriptor);
      require(mask==(test==0?std::vector<int64_t>{}:approved),"overlay/default int64 affinity value");
      require(node->get_parameter("common.estimator_thread_cpu_affinity").get_type()==
        rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY,"actual ROS typed integer-array parser");
      require(node->describe_parameter("common.estimator_thread_cpu_affinity").read_only,"read-only binding retained");
      require(node->declare_parameter<int64_t>("common.estimator_thread_nice",0)==-5,"nice retained across node overlay");
      require(node->declare_parameter<int64_t>("common.opencv_threads",0)==1,"other common parameter retained");
      require(node->declare_parameter<int64_t>("lio.min_iterations",0)==5,"algorithm parameter retained");
    }
    std::cout<<"real ROS estimator single-parameter overlay merge/default passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';result=1;}
  std::filesystem::remove_all(root);rclcpp::shutdown();return result;
}
