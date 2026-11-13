#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace {
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
void write(const std::filesystem::path& path,const std::string& text) {
  std::ofstream stream(path);stream<<text;require(bool(stream),"write YAML fixture");
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);int result=0;
  const auto root=std::filesystem::temp_directory_path()/("shn-parameter-inputs-"+std::to_string(getpid()));
  try {
    std::filesystem::create_directory(root);
    write(root/"baseline.yaml","/sunhorizon_navigation_runtime_entry:\n  ros__parameters:\n    use_sim_time: false\n");
    write(root/"inline.yaml","/**:\n  ros__parameters:\n    nav2.peripheral_lifecycle_manager: /lifecycle_manager_navigation\n");
    for(int test=0;test!=5;++test) {
      std::string producer="/sunhorizon_navigation_runtime_entry:\n  ros__parameters:\n    obstacle_source_mode: rgbd-only-v1\n";
      if(test==0||test==1)producer+="    nav2.airy.local_sources: []\n";
      if(test==0||test==2)producer+="    nav2.airy.global_sources: []\n";
      if(test==4)producer+="    nav2.airy.local_sources: [airy_points]\n    nav2.airy.global_sources: [airy_points]\n";
      write(root/"producer.yaml",producer);
      rclcpp::NodeOptions options;options.use_global_arguments(false).arguments({"--ros-args",
        "--params-file",(root/"baseline.yaml").string(),"--params-file",(root/"producer.yaml").string(),
        "--params-file",(root/"inline.yaml").string()});
      bool constructor_rejected=false;
      rclcpp::Node::SharedPtr node;
      try {node=std::make_shared<rclcpp::Node>("sunhorizon_navigation_runtime_entry",options);}
      catch(const rclcpp::exceptions::InvalidParameterValueException& e) {
        constructor_rejected=true;require(std::string(e.what())=="No parameter value set","expected exact Humble empty-array failure");
      }
      require(constructor_rejected==(test<3),"real ROS parser empty-array construction behavior");
      if(!node)continue;
      rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
      for(const auto* key:{"nav2.airy.local_sources","nav2.airy.global_sources"}) {
        // Same typed declaration as production main.cpp, including default and descriptor.
        const auto value=node->declare_parameter<std::vector<std::string>>(key,{},descriptor);
        require(node->get_parameter(key).get_type()==rclcpp::ParameterType::PARAMETER_STRING_ARRAY,"typed default retained");
        require(node->describe_parameter(key).read_only,"read-only binding retained");
        require(value==(test==4?std::vector<std::string>{"airy_points"}:std::vector<std::string>{}),"omitted defaults and retained legacy override");
      }
    }
    std::cout<<"three-layer real ROS parameter parser/default regression passed\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';result=1;}
  std::filesystem::remove_all(root);rclcpp::shutdown();return result;
}
