// rlc_scene_bridge/src/tesseract_cmd_builders.cpp

#include "rlc_scene_bridge/tesseract_scene_utils.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/collision.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/visual.h>
#include <shape_msgs/msg/plane.hpp>

namespace rlc_scene_bridge
{
namespace tesseract_scene
{
namespace
{

std::shared_ptr<tesseract_geometry::Geometry>
makePrimitiveGeometry(const shape_msgs::msg::SolidPrimitive& prim)
{
  using shape_msgs::msg::SolidPrimitive;
  switch (prim.type)
  {
    case SolidPrimitive::BOX:
    {
      if (prim.dimensions.size() < 3U)
      {
        return nullptr;
      }
      const double x = prim.dimensions[SolidPrimitive::BOX_X];
      const double y = prim.dimensions[SolidPrimitive::BOX_Y];
      const double z = prim.dimensions[SolidPrimitive::BOX_Z];
      return std::make_shared<tesseract_geometry::Box>(x, y, z);
    }

    case SolidPrimitive::SPHERE:
    {
      if (prim.dimensions.size() < 1U)
      {
        return nullptr;
      }
      const double r = prim.dimensions[SolidPrimitive::SPHERE_RADIUS];
      return std::make_shared<tesseract_geometry::Sphere>(r);
    }

    case SolidPrimitive::CYLINDER:
    {
      if (prim.dimensions.size() < 2U)
      {
        return nullptr;
      }
      const double h = prim.dimensions[SolidPrimitive::CYLINDER_HEIGHT];
      const double r = prim.dimensions[SolidPrimitive::CYLINDER_RADIUS];
      return std::make_shared<tesseract_geometry::Cylinder>(r, h);
    }

    case SolidPrimitive::CONE:
    {
      if (prim.dimensions.size() < 2U)
      {
        return nullptr;
      }
      const double h = prim.dimensions[SolidPrimitive::CONE_HEIGHT];
      const double r = prim.dimensions[SolidPrimitive::CONE_RADIUS];
      return std::make_shared<tesseract_geometry::Cone>(r, h);
    }

    default:
      return nullptr;
  }
}

std::shared_ptr<tesseract_geometry::Geometry>
makePlaneGeometry(const shape_msgs::msg::Plane& plane)
{
  const double a = plane.coef[0];
  const double b = plane.coef[1];
  const double c = plane.coef[2];
  const double d = plane.coef[3];

  const double n2 = a * a + b * b + c * c;
  if (n2 <= 0.0)
  {
    return nullptr;
  }

  const double inv_n = 1.0 / std::sqrt(n2);

  return std::make_shared<tesseract_geometry::Plane>(a * inv_n, b * inv_n, c * inv_n,
                                                     d * inv_n);
}

std::shared_ptr<tesseract_scene_graph::Visual>
makeVisual(const std::string& name, const Eigen::Isometry3d& origin,
           const std::shared_ptr<tesseract_geometry::Geometry>& geometry)
{
  auto visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->name = name;
  visual->origin = origin;
  visual->geometry = geometry;
  return visual;
}

std::shared_ptr<tesseract_scene_graph::Collision>
makeCollision(const std::string& name, const Eigen::Isometry3d& origin,
              const std::shared_ptr<tesseract_geometry::Geometry>& geometry)
{
  auto collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->name = name;
  collision->origin = origin;
  collision->geometry = geometry;
  return collision;
}

bool validateCollisionObject(const moveit_msgs::msg::CollisionObject& object)
{
  if (object.id.empty() || object.header.frame_id.empty())
  {
    return false;
  }
  if (!object.meshes.empty())
  {
    return false;
  }
  if (object.primitives.size() != object.primitive_poses.size())
  {
    return false;
  }
  if (object.planes.size() != object.plane_poses.size())
  {
    return false;
  }
  if (object.primitives.empty() && object.planes.empty())
  {
    return false;
  }
  return true;
}

bool fillLinkGeometry(const moveit_msgs::msg::CollisionObject& object,
                      const std::string& link_name, tesseract_scene_graph::Link& link)
{
  for (std::size_t i = 0; i < object.primitives.size(); ++i)
  {
    const auto geometry = makePrimitiveGeometry(object.primitives[i]);
    if (!geometry)
    {
      return false;
    }
    const Eigen::Isometry3d origin = poseToIsometry(object.primitive_poses[i]);
    const auto visual_name = link_name + "_v_" + std::to_string(i);
    const auto collision_name = link_name + "_c_" + std::to_string(i);
    link.visual.push_back(makeVisual(visual_name, origin, geometry));
    link.collision.push_back(makeCollision(collision_name, origin, geometry));
  }

  for (std::size_t i = 0; i < object.planes.size(); ++i)
  {
    const auto geometry = makePlaneGeometry(object.planes[i]);
    if (!geometry)
    {
      return false;
    }
    const Eigen::Isometry3d origin = poseToIsometry(object.plane_poses[i]);
    const auto visual_name = link_name + "_v_plane_" + std::to_string(i);
    const auto collision_name = link_name + "_c_plane_" + std::to_string(i);
    link.visual.push_back(makeVisual(visual_name, origin, geometry));
    link.collision.push_back(makeCollision(collision_name, origin, geometry));
  }

  return true;
}

std::optional<BuiltObject>
buildFixedLinkObject(const moveit_msgs::msg::CollisionObject& object,
                     std::string parent_link, std::string link_name,
                     std::string joint_name)
{
  tesseract_scene_graph::Link link(link_name);
  if (!fillLinkGeometry(object, link_name, link))
  {
    return std::nullopt;
  }

  tesseract_scene_graph::Joint joint(joint_name);
  joint.parent_link_name = std::move(parent_link);
  joint.child_link_name = link_name;
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.parent_to_joint_origin_transform = poseToIsometry(object.pose);

  ObjectInfo info;
  info.id = object.id;
  info.frame_id = object.header.frame_id;  // or rename to moveit_frame_id later
  info.link_name = link_name;
  info.joint_name = joint_name;
  info.parent_link = joint.parent_link_name;

  return BuiltObject{ std::move(info), std::move(link), std::move(joint) };
}

}  // namespace

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                             pose.orientation.z);

  if (q.norm() > 0.0)
  {
    out.linear() = q.normalized().toRotationMatrix();
  }

  return out;
}

std::string worldLinkName(const NamingPolicy& naming_policy, const std::string& id)
{ return naming_policy.world_link_prefix + id; }

std::string attachedLinkName(const NamingPolicy& naming_policy, const std::string& id)
{ return naming_policy.attached_link_prefix + id; }

std::string jointName(const std::string& link_name)
{ return "joint_" + link_name; }

std::optional<BuiltObject>
buildWorldObject(const moveit_msgs::msg::CollisionObject& object,
                 const NamingPolicy& naming_policy)
{
  if (!validateCollisionObject(object))
  {
    return std::nullopt;
  }
  const auto& parent_frame = object.header.frame_id;

  const std::string link_name = worldLinkName(naming_policy, object.id);
  const std::string joint_name = jointName(link_name);

  return buildFixedLinkObject(object, parent_frame, link_name, joint_name);
}

std::optional<BuiltObject>
buildAttachedObject(const moveit_msgs::msg::AttachedCollisionObject& aco,
                    const NamingPolicy& naming_policy)
{
  const auto& object = aco.object;

  if (!validateCollisionObject(object))
  {
    return std::nullopt;
  }

  const auto& parent_frame = aco.link_name;

  if (parent_frame.empty())
  {
    return std::nullopt;
  }

  const std::string link_name = attachedLinkName(naming_policy, object.id);
  const std::string joint_name = jointName(link_name);

  return buildFixedLinkObject(object, parent_frame, link_name, joint_name);
}

}  // namespace tesseract_scene
}  // namespace rlc_scene_bridge
