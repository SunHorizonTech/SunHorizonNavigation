#pragma once
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace sunhorizon_perception_runtime {
// Nominal accepted source<-base at q=0, with current/neutral waist from the
// single JointKinematics owner. No new joint model, cache or TF publisher.
inline geometry_msgs::msg::Transform source_from_base_at_measured_waist(
    const geometry_msgs::msg::Transform& source_from_base0,
    const geometry_msgs::msg::Transform& base_from_waist0,
    const geometry_msgs::msg::Transform& base_from_waist) {
  tf2::Transform nominal,neutral,current;
  tf2::fromMsg(source_from_base0,nominal);tf2::fromMsg(base_from_waist0,neutral);
  tf2::fromMsg(base_from_waist,current);
  return tf2::toMsg(nominal*neutral*current.inverse());
}
} // namespace sunhorizon_perception_runtime
