#pragma once

#include <memory>
#include <string>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tesseract_environment/fwd.h>
#include <tesseract_scene_graph/collision.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/visual.h>

namespace tesseract_environment
{
class Command;
}

namespace rlc_scene_bridge
{
namespace tesseract_scene
{

struct NamingPolicy
{
  std::string world_link_prefix{ "w_" };
  std::string attached_link_prefix{ "a_" };
};
struct ObjectInfo
{
  std::string id;
  std::string frame_id;
  std::string link_name;
  std::string joint_name;
  std::string parent_link;
};
struct BuiltObject
{
  ObjectInfo info;
  tesseract_scene_graph::Link link;
  tesseract_scene_graph::Joint joint;
};

std::string worldLinkName(const NamingPolicy& naming_policy, const std::string& id);
std::string attachedLinkName(const NamingPolicy& naming_policy, const std::string& id);
std::string jointName(const std::string& link_name);

std::optional<BuiltObject>
buildWorldObject(const moveit_msgs::msg::CollisionObject& object,
                 const NamingPolicy& naming_policy);

std::optional<BuiltObject>
buildAttachedObject(const moveit_msgs::msg::AttachedCollisionObject& aco,
                    const NamingPolicy& naming_policy);
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose);
}  // namespace tesseract_scene
}  // namespace rlc_scene_bridge
