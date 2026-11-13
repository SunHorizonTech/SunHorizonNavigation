#include "sunhorizon_navigation_runtime/detail/collision_height_parameters.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/direct_cloud_binding.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sunhorizon_navigation_runtime/detail/direct_cloud_admission.hpp"
#include <nav2_planner/planner_server.hpp>
#include <nav2_controller/controller_server.hpp>
#include <tf2_ros/buffer.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value) {if(!value)throw std::runtime_error("collision height parameter guard check failed");}
void run() {
  auto node=std::make_shared<rclcpp::Node>("collision_height_guard_test",
    rclcpp::NodeOptions().use_global_arguments(false)
      .parameter_overrides({{"voxel_layer.belly.min_obstacle_height",-.75}}));
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
  node->declare_parameter("global_frame","odom",descriptor);
  node->declare_parameter("voxel_layer.belly.marking",true,descriptor);
  node->declare_parameter("frequency",10.);
  int changes=0;
  auto upstream=node->add_on_set_parameters_callback([&](const auto&){
    ++changes;rcl_interfaces::msg::SetParametersResult result;result.successful=true;return result;
  });
  sunhorizon_navigation_runtime::detail::freeze_collision_height_parameters(
    *node,{"global_frame"},{"voxel_layer.","belly."});
  changes=0;
  // Even a later callback with arbitrary side effects must not run on a
  // descriptor-rejected batch. Callback ordering no longer grants authority.
  auto later=node->add_on_set_parameters_callback([&](const auto&){
    ++changes;rcl_interfaces::msg::SetParametersResult result;result.successful=true;return result;
  });
  require(!node->set_parameter({"global_frame","map"}).successful);
  require(!node->set_parameter({"voxel_layer.belly.min_obstacle_height",-.8}).successful);
  require(!node->set_parameter({"voxel_layer.belly.marking",false}).successful);
  require(changes==0 && node->get_parameter("global_frame").as_string()=="odom" &&
    node->get_parameter("voxel_layer.belly.min_obstacle_height").as_double()==-.75);
  require(node->set_parameter({"frequency",20.}).successful && changes==2);
  const auto atomic=node->set_parameters_atomically({{"frequency",30.},{"global_frame","map"}});
  require(!atomic.successful && changes==2 && node->get_parameter("frequency").as_double()==20.);
  rclcpp::Node mutable_node("mutable_configuration_test",rclcpp::NodeOptions().use_global_arguments(false));
  mutable_node.declare_parameter("global_frame","odom");
  bool rejected=false;
  try {sunhorizon_navigation_runtime::detail::freeze_collision_height_parameters(mutable_node,{"global_frame"},{});}
  catch(const std::invalid_argument& error){rejected=std::string(error.what()).find("existing_parameter_must_be_read_only")!=std::string::npos;}
  require(rejected && mutable_node.get_parameter("global_frame").as_string()=="odom" &&
    !mutable_node.describe_parameter("global_frame").read_only);
}
void actual_child_costmaps() {
  const auto options=rclcpp::NodeOptions().use_global_arguments(false);
  auto verify=[](const auto& costmap,bool expected){
    for(const std::string name:{"global_frame","robot_base_frame","plugins","filters"})
      require(costmap->describe_parameter(name).read_only==expected);
  };
  {
    auto planner=std::make_shared<nav2_planner::PlannerServer>(options,true);
    auto controller=std::make_shared<nav2_controller::ControllerServer>(options,true);
    verify(planner->getCostmapROS(),true);verify(controller->getCostmapROS(),true);
  }
  {
    // Exercise the original constructor symbols, whose default stays mutable.
    auto planner=std::make_shared<nav2_planner::PlannerServer>(options);
    auto controller=std::make_shared<nav2_controller::ControllerServer>(options);
    verify(planner->getCostmapROS(),false);verify(controller->getCostmapROS(),false);
  }
}
void actual_costmap_preconfigure_arrays() {
  // Exercise assembly's exact pre-configure traversal with heap-owned strings
  // in both arrays. Borrowing from a temporary Parameter here used to destroy
  // its vector before the first range-for iteration (C++17).
  const std::string obstacle="obstacle_layer_with_heap_owned_parameter_name";
  const std::string inflation="inflation_layer_with_heap_owned_parameter_name";
  const std::string keepout="keepout_filter_with_heap_owned_parameter_name";
  const std::vector<std::string> plugins{obstacle,inflation},filters{keepout};
  for(const bool classified:{false,true}) {
    auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
        {"plugins",plugins},{"filters",filters},{"global_frame","odom"},
        {"robot_base_frame","base_footprint"},
        {obstacle+".plugin","nav2_costmap_2d::ObstacleLayer"},
        {obstacle+".observation_sources","belly chest"},
        {obstacle+".belly.marking",true},{obstacle+".belly.min_obstacle_height",-.75},
        {obstacle+".belly.max_obstacle_height",2.},{obstacle+".belly.data_type","PointCloud2"},
        {obstacle+".chest.marking",true},{obstacle+".chest.min_obstacle_height",-.75},
        {obstacle+".chest.max_obstacle_height",2.},{obstacle+".chest.data_type","PointCloud2"},
        {inflation+".plugin","nav2_costmap_2d::InflationLayer"},
        {keepout+".plugin","nav2_costmap_2d::KeepoutFilter"},
        {keepout+".enabled",true},{keepout+".filter_info_topic","/costmap_filter_info"}}),true);
    sunhorizon_navigation_runtime::detail::freeze_costmap_collision_height_configuration(*costmap,classified);
    require(costmap->get_parameter("plugins").as_string_array()==plugins);
    require(costmap->get_parameter("filters").as_string_array()==filters);
    for(const auto& name:{obstacle+".plugin",obstacle+".belly.marking",
        obstacle+".chest.min_obstacle_height",keepout+".enabled",keepout+".filter_info_topic"})
      require(costmap->describe_parameter(name).read_only);
    require(costmap->get_parameter(obstacle+".chest.min_obstacle_height").as_double()==-.75);
    require(!costmap->set_parameters_atomically({{keepout+".enabled",false},
      {obstacle+".belly.min_obstacle_height",-.8}}).successful);
    if(classified) {
      require(!costmap->get_parameter(inflation+".inflate_unknown").as_bool());
      require(costmap->describe_parameter(inflation+".inflate_unknown").read_only);
    }
  }
}
void inactive_costmap_keeps_original_direct_ingress() {
  // ControllerServer::on_deactivate invokes this exact child transition.
  // Exercise the real plugin/buffer and the assembly's actual result classifier.
  for(const std::string mode:{"direct-v1","direct-existing-source-v1","direct-classified-v1"}) {
    const bool classified=mode=="direct-classified-v1";
    const auto kind=classified?nav2_util::DirectCloudKind::ClassifiedObstacleFrame:
      mode=="direct-existing-source-v1"?nav2_util::DirectCloudKind::RetainedIndependentSource:
      nav2_util::DirectCloudKind::SelfFilteredMarking;
    auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false)
      .parameter_overrides({{"plugins",std::vector<std::string>{"obstacle"}},
        {"filters",std::vector<std::string>{}},{"global_frame","odom"},
        {"robot_base_frame","base_footprint"},{"publish_frequency",0.},
        {"obstacle.plugin","nav2_costmap_2d::ObstacleLayer"},
        {"obstacle.observation_sources","body"},{"obstacle.body.input_mode",mode},
        {"obstacle.body.data_type","PointCloud2"},{"obstacle.body.marking",true},
        {"obstacle.body.clearing",classified},{"obstacle.body.observation_persistence",0.},
        {"obstacle.body.min_obstacle_height",0.},{"obstacle.body.max_obstacle_height",2.},
        {"obstacle.body.direct_source_id","belly"},{"obstacle.body.direct_epoch","epoch"},
        {"obstacle.body.direct_frame","base_footprint"},{"obstacle.body.direct_maximum_age_sec",.3}}));
    auto buffer=std::make_shared<tf2_ros::Buffer>(costmap->get_clock());buffer->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped edge;edge.header.frame_id="odom";edge.child_frame_id="base_footprint";
    edge.transform.rotation.w=1.;require(buffer->setTransform(edge,"test",true));
    costmap->setDirectTransformBuffer(buffer);
    require(costmap->configure().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const auto sink=nav2_costmap_2d::bindDirectCostmapResultSource(costmap,"obstacle","body",kind);
    auto frame=[&](uint64_t sequence,int64_t receipt_age_ns=0){
      const auto ros=costmap->now().nanoseconds(),steady=nav2_util::directSteadyNowNs();
      auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
      cloud->header.frame_id="base_footprint";cloud->header.stamp=rclcpp::Time(ros-1);
      sensor_msgs::PointCloud2Modifier modifier(*cloud);
      modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
      sensor_msgs::PointCloud2Iterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
      *x=.25f;*y=.25f;*z=.5f;
      nav2_util::DirectCloud value{cloud,{"belly","epoch","base_footprint",sequence,ros-1,ros,steady-receipt_age_ns},kind};
      if(classified){value.classes=std::make_shared<const std::vector<uint8_t>>(std::vector<uint8_t>{2});
        value.complete=true;value.ground_qualified=true;
        value.ground_basis=nav2_util::DirectGroundBasis::BoundedReference;}
      return value;
    };
    require(costmap->activate().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    if(classified){auto untyped=frame(0);untyped.ground_basis=nav2_util::DirectGroundBasis::None;
      require(!nav2_util::directCloudMatchesMetadata(untyped));}
    require(sink(frame(1)).accepted());
    require(costmap->deactivate().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const auto inactive=sink(frame(2));require(inactive.accepted());
    sunhorizon_navigation_runtime::detail::require_nonfatal_cloud_admission(inactive,"inactive.actual_sink");
    const auto expired=sink(frame(3,300000001));
    require(expired.state==nav2_util::DirectAdmissionState::MotionUnavailable);
    sunhorizon_navigation_runtime::detail::require_nonfatal_cloud_admission(expired,"inactive.expired_sink");
    require(sink(frame(4)).accepted());
    auto invalid=frame(5);invalid.metadata.epoch="wrong_epoch";
    const auto rejected=sink(invalid);require(rejected.state==nav2_util::DirectAdmissionState::InternalFault);
    bool fatal=false;
    try{sunhorizon_navigation_runtime::detail::require_nonfatal_cloud_admission(rejected,"inactive.invalid_sink");}
    catch(const std::runtime_error&){fatal=true;}
    require(fatal);costmap->cleanup();
  }
}
void active_costmap() {
  auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions().use_global_arguments(false)
    .parameter_overrides({{"plugins",std::vector<std::string>{}},{"filters",std::vector<std::string>{}},
      {"global_frame","odom"},{"robot_base_frame","base_footprint"},{"publish_frequency",0.},
      {"voxel_layer.min_obstacle_height",-.75}}),true);
  sunhorizon_navigation_runtime::detail::freeze_collision_height_parameters(*costmap,
    {"global_frame","robot_base_frame","plugins","filters"},{"voxel_layer."});
  auto buffer=std::make_shared<tf2_ros::Buffer>(costmap->get_clock());buffer->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped edge;edge.header.frame_id="odom";edge.child_frame_id="base_footprint";
  edge.transform.rotation.w=1.;require(buffer->setTransform(edge,"test",true));
  edge.header.frame_id="base_footprint";edge.child_frame_id="other_base";
  require(buffer->setTransform(edge,"test",true));costmap->setDirectTransformBuffer(buffer);
  require(costmap->configure().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  for(int activation=0;activation<2;++activation) {
    require(costmap->activate().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const auto footprint=costmap->getRobotFootprint();
    const auto reject=costmap->set_parameters_atomically({{"footprint_padding",0.},
      {"voxel_layer.min_obstacle_height",-.8},{"robot_base_frame","other_base"}});
    require(!reject.successful && costmap->getBaseFrameID()=="base_footprint");
    const auto after=costmap->getRobotFootprint();require(after.size()==footprint.size());
    for(std::size_t i=0;i<after.size();++i)
      require(after[i].x==footprint[i].x && after[i].y==footprint[i].y && after[i].z==footprint[i].z);
    require(costmap->get_parameter("voxel_layer.min_obstacle_height").as_double()==-.75);
    require(!costmap->set_parameter({"robot_base_frame","other_base"}).successful);
    require(costmap->getBaseFrameID()=="base_footprint");
    require(costmap->deactivate().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }
  costmap->cleanup();
}
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try{run();actual_child_costmaps();actual_costmap_preconfigure_arrays();active_costmap();inactive_costmap_keeps_original_direct_ingress();rclcpp::shutdown();std::cout<<"PASS actual preconfigure arrays and active costmap responsibility freeze\n";return 0;}
  catch(const std::exception& error){rclcpp::shutdown();std::cerr<<error.what()<<'\n';return 1;}
}
