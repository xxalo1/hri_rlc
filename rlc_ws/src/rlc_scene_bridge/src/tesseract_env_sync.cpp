#include <stdexcept>
#include <utility>
#include <tesseract_environment/environment_monitor_interface.h>
#include <rlc_scene_bridge/tesseract_env_sync.hpp>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/remove_link_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <Eigen/Geometry>

namespace rlc_scene_bridge
{

TesseractEnvSync::TesseractEnvSync(MonitorInterfaceConstPtr monitor_interface,
                                   std::string monitor_namespace,
                                   TesseractEnvSyncOptions opt)
  : monitor_interface_(std::move(monitor_interface))
  , monitor_namespace_(std::move(monitor_namespace))
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

TesseractEnvSync::~TesseractEnvSync() = default;

TesseractEnvSync::EnvironmentUPtr TesseractEnvSync::snapshot() const
{
  auto env = monitor_interface_->getEnvironment(monitor_namespace_);
  if (!env)
  {
    throw std::runtime_error("TesseractEnvSync: monitor returned null environment for "
                             "namespace '" +
                             monitor_namespace_ + "'");
  }

  return env;
}

void TesseractEnvSync::applyFullScene(const PlanningSceneMsg& scene)
{
  std::unordered_map<std::string, tesseract_scene::ObjectInfo> world_objects_next;
  std::unordered_map<std::string, tesseract_scene::ObjectInfo> attached_objects_next;

  auto commands = fromFullScene(scene, world_objects_next, attached_objects_next);
  applyCommands(commands);

  world_objects_ = std::move(world_objects_next);
  attached_objects_ = std::move(attached_objects_next);
}

void TesseractEnvSync::applyDiff(const PlanningSceneMsg& scene)
{
  auto world_objects_next = world_objects_;
  auto attached_objects_next = attached_objects_;

  auto commands = fromDiff(scene, world_objects_next, attached_objects_next);
  applyCommands(commands);

  world_objects_ = std::move(world_objects_next);
  attached_objects_ = std::move(attached_objects_next);
}

void TesseractEnvSync::applyCommands(const tesseract_environment::Commands& commands) const
{
  const bool ok = monitor_interface_->applyCommands(monitor_namespace_, commands);
  if (!ok)
  {
    throw std::runtime_error("TesseractEnvSync: applyCommands failed for namespace '" +
                             monitor_namespace_ + "'");
  }
}

tesseract_environment::Commands TesseractEnvSync::fromFullScene(
    const PlanningSceneMsg& scene,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& world_objects_next,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& attached_objects_next)
    const
{
  tesseract_environment::Commands commands;
  commands.reserve(world_objects_.size() + attached_objects_.size() +
                   scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size());

  world_objects_next.clear();
  attached_objects_next.clear();

  // Clear existing world objects.
  for (const auto& kv : world_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  // Clear existing attached objects.
  for (const auto& kv : attached_objects_)
  {
    const auto& info = kv.second;
    commands.push_back(
        std::make_shared<tesseract_environment::RemoveLinkCommand>(info.link_name));
  }

  // Add all world objects.
  for (const auto& object : scene.world.collision_objects)
  {
    auto cmd = makeAddWorldCommand(object, world_objects_next);
    commands.push_back(std::move(cmd));
  }

  // Add all attached objects.
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    auto cmd = makeAddAttachedCommand(aco, attached_objects_next);
    commands.push_back(std::move(cmd));
  }

  return commands;
}

tesseract_environment::Commands TesseractEnvSync::fromDiff(
    const PlanningSceneMsg& scene,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& world_objects_next,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& attached_objects_next)
    const
{
  tesseract_environment::Commands commands;
  commands.reserve(scene.world.collision_objects.size() +
                   scene.robot_state.attached_collision_objects.size() * 2);

  // 1) World diffs.
  for (const auto& object : scene.world.collision_objects)
  {
    tesseract_environment::Command::Ptr cmd{};

    switch (object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        cmd = makeAddWorldCommand(object, world_objects_next);
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        cmd = makeRemoveWorldCommand(object, world_objects_next);
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        cmd = makeMoveWorldCommand(object, world_objects_next);
        break;
      }

      case moveit_msgs::msg::CollisionObject::APPEND:
      {
        throw std::domain_error("TesseractEnvSync: APPEND unsupported for world objects "
                                "(id='" +
                                object.id + "')");
      }

      default:
      {
        throw std::domain_error("TesseractEnvSync: unsupported world operation " +
                                std::to_string(static_cast<int>(object.operation)) +
                                " (id='" + object.id + "')");
      }
    }

    commands.push_back(std::move(cmd));
  }

  // 2) Attached diffs.
  for (const auto& aco : scene.robot_state.attached_collision_objects)
  {
    switch (aco.object.operation)
    {
      case moveit_msgs::msg::CollisionObject::ADD:
      {
        commands.push_back(makeAddAttachedCommand(aco, attached_objects_next));
        break;
      }

      case moveit_msgs::msg::CollisionObject::REMOVE:
      {
        appendRemoveAttachedCommands(aco, attached_objects_next, commands);
        break;
      }

      case moveit_msgs::msg::CollisionObject::MOVE:
      {
        throw std::domain_error("TesseractEnvSync: MOVE unsupported for attached objects "
                                "(id='" +
                                aco.object.id + "')");
      }

      default:
      {
        throw std::domain_error(
            "TesseractEnvSync: unsupported attached operation " +
            std::to_string(static_cast<unsigned>(aco.object.operation)) + " (id='" +
            aco.object.id + "')");
      }
    }
  }

  return commands;
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeAddWorldCommand(
    const moveit_msgs::msg::CollisionObject& object,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& world_objects_next) const
{
  if (object.id.empty())
  {
    throw std::domain_error("TesseractEnvSync: ADD world object with empty id");
  }

  const auto built = tesseract_scene::buildWorldObject(object, opt_.naming_policy);
  if (!built)
  {
    throw std::domain_error("TesseractEnvSync: failed to build world object '" +
                            object.id + "'");
  }

  world_objects_next.insert_or_assign(built->info.id, built->info);

  return std::make_shared<tesseract_environment::AddLinkCommand>(
      built->link, built->joint, opt_.allow_replace);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeAddAttachedCommand(
    const moveit_msgs::msg::AttachedCollisionObject& aco,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& attached_objects_next)
    const
{
  if (aco.object.id.empty())
  {
    throw std::domain_error("TesseractEnvSync: ADD attached object with empty id");
  }

  const auto built = tesseract_scene::buildAttachedObject(aco, opt_.naming_policy);
  if (!built)
  {
    throw std::domain_error("TesseractEnvSync: failed to build attached object '" +
                            aco.object.id + "'");
  }

  attached_objects_next.insert_or_assign(built->info.id, built->info);

  return std::make_shared<tesseract_environment::AddLinkCommand>(
      built->link, built->joint, opt_.allow_replace);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeMoveWorldCommand(
    const moveit_msgs::msg::CollisionObject& object,
    const std::unordered_map<std::string, tesseract_scene::ObjectInfo>& world_objects_next)
    const
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: MOVE world object with empty id");
  }

  auto it = world_objects_next.find(id);
  if (it == world_objects_next.end())
  {
    throw std::domain_error("TesseractEnvSync: MOVE for unknown world object '" + id +
                            "'");
  }

  const tesseract_scene::ObjectInfo& info = it->second;

  // v1: reject frame changes (requires TF resolution / parent relinking)
  if (!object.header.frame_id.empty() && object.header.frame_id != info.frame_id)
  {
    throw std::domain_error("TesseractEnvSync: MOVE for '" + id +
                            "' changed frame_id from '" + info.frame_id + "' to '" +
                            object.header.frame_id + "'");
  }

  const Eigen::Isometry3d origin = tesseract_scene::poseToIsometry(object.pose);

  return std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      info.joint_name, origin);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeRemoveWorldCommand(
    const moveit_msgs::msg::CollisionObject& object,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& world_objects_next) const
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE world object with empty id");
  }

  auto it = world_objects_next.find(id);
  if (it == world_objects_next.end())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE for unknown world object '" + id +
                            "'");
  }

  const std::string link_name = it->second.link_name;
  world_objects_next.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

tesseract_environment::Command::Ptr TesseractEnvSync::makeRemoveAttachedCommand(
    const moveit_msgs::msg::CollisionObject& object,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& attached_objects_next)
    const
{
  const std::string& id = object.id;
  if (id.empty())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE attached object with empty id");
  }

  auto it = attached_objects_next.find(id);
  if (it == attached_objects_next.end())
  {
    throw std::domain_error("TesseractEnvSync: REMOVE for unknown attached object '" +
                            id + "'");
  }

  const std::string link_name = it->second.link_name;
  attached_objects_next.erase(it);

  return std::make_shared<tesseract_environment::RemoveLinkCommand>(link_name);
}

void TesseractEnvSync::appendRemoveAttachedCommands(
    const moveit_msgs::msg::AttachedCollisionObject& aco,
    std::unordered_map<std::string, tesseract_scene::ObjectInfo>& attached_objects_next,
    tesseract_environment::Commands& commands) const
{
  if (aco.link_name.empty())
  {
    throw std::domain_error(
        "TesseractEnvSync: REMOVE attached objects with empty link_name");
  }

  const std::string& id = aco.object.id;

  if (id.empty())
  {
    for (auto it = attached_objects_next.begin(); it != attached_objects_next.end();)
    {
      if (it->second.parent_link == aco.link_name)
      {
        commands.push_back(std::make_shared<tesseract_environment::RemoveLinkCommand>(
            it->second.link_name));
        it = attached_objects_next.erase(it);
      }
      else
      {
        ++it;
      }
    }
    return;
  }

  // Remove single by id
  auto cmd = makeRemoveAttachedCommand(aco.object, attached_objects_next);

  commands.push_back(std::move(cmd));
}

}  // namespace rlc_scene_bridge
