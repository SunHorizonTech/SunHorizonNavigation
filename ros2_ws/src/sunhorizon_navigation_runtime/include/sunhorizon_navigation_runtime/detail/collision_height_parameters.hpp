#pragma once
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace sunhorizon_navigation_runtime::detail {
// Before component configure can start any executor: freeze original values at
// the rclcpp descriptor gate. A rejected atomic batch never enters any callback,
// including callbacks installed later by activation with mutable side effects.
template<class Node>
void freeze_collision_height_parameters(Node& node,std::set<std::string> exact,
    const std::set<std::string>& prefixes,
    const std::map<std::string,rclcpp::ParameterValue>& defaults={}) {
  const auto selected=[&](const std::string& name) {
    if(exact.count(name))return true;
    for(const auto& prefix:prefixes)if(name.compare(0,prefix.size(),prefix)==0)return true;
    return false;
  };
  const auto overrides=node.get_node_parameters_interface()->get_parameter_overrides();
  for(const auto& item:overrides)if(selected(item.first))exact.insert(item.first);
  for(const auto& name:node.list_parameters({},1000).names)if(selected(name))exact.insert(name);
  for(const auto& item:defaults)exact.insert(item.first);
  for(const auto& name:exact) {
    rclcpp::ParameterValue value;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    if(node.has_parameter(name)) {
      if(!node.describe_parameter(name).read_only)
        throw std::invalid_argument("collision_height_existing_parameter_must_be_read_only:"+name);
      continue;
    } else if(overrides.count(name))value=overrides.at(name);
    else if(defaults.count(name))value=defaults.at(name);
    else throw std::invalid_argument("collision_height_explicit_parameter_required:"+name);
    if(value.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET)
      throw std::invalid_argument("collision_height_parameter_value_required:"+name);
    descriptor.read_only=true;
    node.declare_parameter(name,value,descriptor,true);
  }
}
// Shared by assembly's pre-configure path and the actual-costmap regression.
// No executor, source subscription or lifecycle configure is started here.
template<class Node>
void freeze_costmap_collision_height_configuration(Node& node,bool classified_obstacles_enabled) {
  const auto require_overrides=[](const auto& overrides,const std::vector<std::string>& keys) {
    for (const auto& key:keys) if (!overrides.count(key))
      throw std::invalid_argument("collision_height_explicit_parameter_required:"+key);
  };
  const std::map<std::string,rclcpp::ParameterValue> overrides=node.get_node_parameters_interface()->get_parameter_overrides();
  require_overrides(overrides,{"plugins","filters","global_frame","robot_base_frame"});
  std::set<std::string> prefixes;
  std::map<std::string,rclcpp::ParameterValue> defaults;
  // rclcpp::Parameter owns the arrays returned by as_string_array().
  // Copy before iteration: C++17 range-for does not extend that temporary.
  const std::vector<std::string> layers=node.get_parameter("plugins").as_string_array();
  const std::vector<std::string> filters=node.get_parameter("filters").as_string_array();
  for (const auto& layer:layers) {
    require_overrides(overrides,{layer+".plugin"});prefixes.insert(layer+".");
    const auto type=overrides.at(layer+".plugin").get<std::string>();
    if(classified_obstacles_enabled && type=="nav2_costmap_2d::InflationLayer")
      defaults.emplace(layer+".inflate_unknown",rclcpp::ParameterValue(false));
    if (type!="nav2_costmap_2d::VoxelLayer" && type!="nav2_costmap_2d::ObstacleLayer") continue;
    require_overrides(overrides,{layer+".observation_sources"});
    defaults.emplace(layer+".enabled",rclcpp::ParameterValue(true));
    defaults.emplace(layer+".min_obstacle_height",rclcpp::ParameterValue(0.));
    defaults.emplace(layer+".max_obstacle_height",rclcpp::ParameterValue(2.));
    std::istringstream names(overrides.at(layer+".observation_sources").get<std::string>());
    for (std::string source;names>>source;) {
      const auto key=layer+"."+source+".";
      require_overrides(overrides,{key+"marking",key+"min_obstacle_height",key+"max_obstacle_height",key+"data_type"});
    }
  }
  for (const auto& filter:filters) {
    require_overrides(overrides,{filter+".plugin",filter+".enabled",filter+".filter_info_topic"});
    prefixes.insert(filter+".");
  }
  freeze_collision_height_parameters(node,
    {"plugins","filters","global_frame","robot_base_frame"},prefixes,defaults);
}
} // namespace sunhorizon_navigation_runtime::detail
