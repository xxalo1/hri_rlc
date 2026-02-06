#include <stdexcept>
#include <utility>
#include <tesseract_environment/environment_monitor_interface.h>
#include "tesseract_env_sync.hpp"

namespace rlc_scene_bridge
{

TesseractEnvSync::TesseractEnvSync(MonitorInterfaceConstPtr monitor_interface,
                                   std::string monitor_namespace, rclcpp::Logger logger,
                                   Options opt)
  : monitor_interface_(std::move(monitor_interface))
  , monitor_namespace_(std::move(monitor_namespace))
  , logger_(std::move(logger))
  , opt_(std::move(opt))
{
  if (!monitor_interface_)
  {
    throw std::invalid_argument("TesseractEnvSync: monitor_interface is null");
  }

  if (monitor_namespace_.empty())
  {
    throw std::invalid_argument("TesseractEnvSync: monitor_namespace is empty");
  }
}

TesseractEnvSync::EnvironmentUPtr TesseractEnvSync::snapshot() const
{ return monitor_interface_->getEnvironment(monitor_namespace_); }

bool TesseractEnvSync::applyFullScene(const PlanningSceneMsg& scene)
{
  auto commands = fromFullScene(scene);
  if (!commands)
  {
    return false;
  }
  return applyCommands(commands);
}

bool TesseractEnvSync::applyDiff(const PlanningSceneMsg& scene)
{
  auto commands = fromDiff(scene);
  if (!commands)
  {
    return false;
  }
  return applyCommands(commands);
}

bool TesseractEnvSync::applyCommands(const tesseract_environment::Commands& commands) const
{
  const bool ok =
      monitor_interface_->applyCommands(monitor_namespace_, tesseract_commands);
  if (!ok)
  {
    RCLCPP_ERROR(logger_, "applyCommands failed for namespace '%s'",
                 monitor_namespace_.c_str());
  }

  return ok;
}

bool TesseractEnvSync::fromFullScene(const PlanningSceneMsg& scene)
{
  tesseract_environment::Commands commands;
  commands.reserve(scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size());

  // Clear existing world objects
  for (const auto& kv : world_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  // Clear existing attached objects
  for (const auto& kv : attached_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  world_objects_.clear();
  attached_objects_.clear();

  // Add all world objects
  for (const auto& object : scene.world.collision_objects)
  {
    auto cmd = makeAddWorldCommand(object);
    if (!cmd)
    {
      return false;
    }
    commands.push_back(std::move(cmd));
  }

  // Add all attached objects
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    auto cmd = makeAddAttachedCommand(aco);
    if (!cmd)
    {
      return false;
    }
    commands.push_back(std::move(cmd));
  }

  return true;
}

bool TesseractEnvSync::fromDiff(const PlanningSceneMsg& scene)
{
  tesseract_environment::Commands commands;
  commands.reserve(scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size());
  // 1) World diffs
  for (const auto& object : scene.world.collision_objects)
  {
    const std::string& id = object.id;

    CommandPtr cmd;

    switch (object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        cmd = makeAddWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        cmd = makeRemoveWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        cmd = makeMoveWorldCommand(object);
        break;
      }

      case moveit_msgs::msg::CollisionObject::APPEND:
      {
        RCLCPP_ERROR(logger_, "APPEND unsupported for world objects (id='%s')",
                     id.c_str());
        return false;
      }

      default:
      {
        RCLCPP_ERROR(logger_, "unsupported world operation %d for '%s'",
                     static_cast<int>(object.operation), id.c_str());
        return false;
      }
    }

    if (!cmd)
    {
      return false;
    }
    commands.push_back(std::move(cmd));
  }
  // 2) Attached diffs
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    const std::string& id = aco.object.id;

    CommandPtr cmd;
    switch (aco.object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        cmd = makeAddAttachedCommand(aco);
        if (!cmd)
        {
          return false;
        }
        commands.push_back(std::move(cmd));
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        if (!appendRemoveAttachedCommands(aco, commands))
        {
          return false;
        }
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        RCLCPP_ERROR(logger_, "MOVE unsupported for attached objects (id='%s')",
                     aco.object.id.c_str());
        return false;
      }

      default:
      {
        RCLCPP_ERROR(logger_, "unsupported attached operation %u (id='%s')",
                     static_cast<unsigned>(aco.object.operation), aco.object.id.c_str());
        return false;
      }
    }
  }
  return true;
}

TesseractEnvSync::CommandPtr
TesseractEnvSync::makeAddWorldCommand(const moveit_msgs::msg::CollisionObject& object)
{
  const auto built = tesseract_scene::buildWorldObject(object, opt_.naming_policy);
  if (!built)
  {
    RCLCPP_ERROR(logger_, "failed to build world object '%s'", object.id.c_str());
    return nullptr;
  }

  world_objects_.insert_or_assign(built->info.id, std::move(built->info));

  auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(
      built->link, built->joint, opt_.allow_replace);

  return cmd;
}

TesseractEnvSync::CommandPtr TesseractEnvSync::makeAddAttachedCommand(
    const moveit_msgs::msg::AttachedCollisionObject& aco)
{
  const auto built = tesseract_scene::buildAttachedObject(aco, opt_.naming_policy);
  if (!built)
  {
    RCLCPP_ERROR(logger_, "failed to build attached object '%s'", aco.object.id.c_str());
    return nullptr;
  }

  attached_objects_.insert_or_assign(built->info.id, std::move(built->info));

  auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(
      built->link, built->joint, opt_.allow_replace);

  return cmd;
}

TesseractEnvSync::CommandPtr
TesseractEnvSync::makeMoveWorldCommand(const moveit_msgs::msg::CollisionObject& object)
{
  const std::string& id = object.id;
  if (id.empty())
  {
    RCLCPP_ERROR(logger_, "MOVE with empty id");
    return nullptr;
  }

  auto it = world_objects_.find(id);
  if (it == world_objects_.end())
  {
    RCLCPP_ERROR(logger_, "MOVE for unknown world object '%s'", id.c_str());
    return nullptr;
  }

  const ObjectInfo& info = it->second;

  // v1: reject frame changes (requires TF resolution / parent relinking)
  if (!object.header.frame_id.empty() && object.header.frame_id != info.frame_id)
  {
    RCLCPP_ERROR(logger_,
                 "MOVE for '%s' changed frame_id from '%s' to '%s' (unsupported v1)",
                 id.c_str(), info.frame_id.c_str(), object.header.frame_id.c_str());
    return nullptr;
  }

  const Eigen::Isometry3d origin = tesseract_scene::poseToIsometry(object.pose);

  return std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      info.joint_name, origin);
}

TesseractEnvSync::CommandPtr
TesseractEnvSync::makeRemoveWorldCommand(const moveit_msgs::msg::CollisionObject& object)
{
  const std::string& id = object.id;
  if (id.empty())
  {
    RCLCPP_ERROR(logger_, "MOVE with empty id");
    return nullptr;
  }

  auto it = world_objects_.find(id);
  if (it == world_objects_.end())
  {
    RCLCPP_ERROR(logger_, "REMOVE for unknown world object '%s'", id.c_str());
    return nullptr;
  }

  const std::string link_name = it->second.link_name;
  world_objects_.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

TesseractEnvSync::CommandPtr TesseractEnvSync::makeRemoveAttachedCommand(
    const moveit_msgs::msg::CollisionObject& object)
{
  const std::string& id = object.id;
  if (id.empty())
  {
    RCLCPP_ERROR(logger_, "MOVE with empty id");
    return nullptr;
  }

  auto it = attached_objects_.find(id);
  if (it == attached_objects_.end())
  {
    RCLCPP_ERROR(logger_, "REMOVE for unknown attached object '%s'", id.c_str());
    return nullptr;
  }

  const std::string link_name = it->second.link_name;
  attached_objects_.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

bool TesseractEnvSync::appendRemoveAttachedCommands(
    const moveit_msgs::msg::AttachedCollisionObject& aco,
    tesseract_environment::Commands& commands)
{
  if (aco.link_name.empty())
  {
    RCLCPP_ERROR(logger_, "REMOVE attached objects with empty link_name");
    return false;
  }

  const std::string& id = aco.object.id;

  if (id.empty())
  {
    bool removed_any = false;

    for (auto it = attached_objects_.begin(); it != attached_objects_.end();)
    {
      if (it->second.parent_link == aco.link_name)
      {
        commands.push_back(std::make_shared<tesseract_environment::RemoveLinkCommand>(
            it->second.link_name));
        it = attached_objects_.erase(it);
        removed_any = true;
      }
      else
      {
        ++it;
      }
    }

    if (!removed_any)
    {
      RCLCPP_WARN(logger_, "REMOVE(all) found no attached objects on link '%s'",
                  aco.link_name.c_str());
    }

    return true;
  }

  // Remove single by id
  auto cmd = makeRemoveAttachedCommand(aco.object);
  if (!cmd)
  {
    return false;
  }

  commands.push_back(std::move(cmd));
  return true;
}

}  // namespace rlc_scene_bridge
