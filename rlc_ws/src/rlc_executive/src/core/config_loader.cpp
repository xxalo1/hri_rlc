#include "rlc_executive/core/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "rlc_executive/core/types.hpp"

namespace rlc_executive
{
namespace config
{
namespace
{
template <typename T>
T getOr(const YAML::Node& parent, const std::string& key, const T& fallback)
{
  const YAML::Node n = parent[key];
  if (!n || n.IsNull())
  {
    return fallback;
  }
  return n.as<T>();
}
std::string nodePath(const std::string& context, const std::string& key)
{
  if (context.empty())
  {
    return key;
  }
  return context + "." + key;
}
std::string requireString(const YAML::Node& parent, const std::string& key,
                          const std::string& context)
{
  const YAML::Node n = parent[key];
  if (!n || n.IsNull())
  {
    throw std::runtime_error("Missing required YAML key '" + nodePath(context, key) + "'");
  }

  try
  {
    return n.as<std::string>();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Invalid YAML type for key '" + nodePath(context, key) +
                             "': expected string. Details: " + std::string{ e.what() });
  }
}
void validateConfig(const ExecConfig& cfg)
{
  if (cfg.default_tree_xml.empty())
  {
    throw std::runtime_error("Executor config: 'default_tree_xml' must not be empty");
  }

  if (cfg.tick_rate_hz <= 0.0)
  {
    throw std::runtime_error("Executor config: 'tick_rate_hz' must be > 0");
  }

  if (cfg.tick_while_running_max_block.count() < 0)
  {
    throw std::runtime_error(
        "Executor config: 'tick_while_running_max_block_ms' must be >= 0");
  }

  if (cfg.tf_timeout_sec < 0.0)
  {
    throw std::runtime_error("Executor config: 'tf_timeout_sec' must be >= 0");
  }

  if (cfg.state_stale_sec < 0.0)
  {
    throw std::runtime_error("Executor config: 'state_stale_sec' must be >= 0");
  }
  return;
}

TickOption parseTickMode(const std::string& tick_mode)
{
  std::string normalized = tick_mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  if (normalized == "ONCE_UNLESS_WOKEN_UP")
  {
    return TickOption::ONCE_UNLESS_WOKEN_UP;
  }
  if (normalized == "WHILE_RUNNING")
  {
    return TickOption::WHILE_RUNNING;
  }
  if (normalized == "EXACTLY_ONCE")
  {
    return TickOption::EXACTLY_ONCE;
  }

  throw std::runtime_error("Executor config: 'tick_mode' must be one of: "
                           "ONCE_UNLESS_WOKEN_UP, WHILE_RUNNING, EXACTLY_ONCE");
}
}  // namespace

ExecConfig loadExecConfigFromFile(const std::string& yaml_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Failed to load executor config YAML: '" + yaml_path +
                             "'. Details: " + e.what());
  }

  ExecConfig cfg;

  // Optional keys with defaults
  cfg.tick_rate_hz = getOr<double>(root, "tick_rate_hz", cfg.tick_rate_hz);
  cfg.default_tree_xml =
      getOr<std::string>(root, "default_tree_xml", cfg.default_tree_xml);

  cfg.joint_states_topic =
      getOr<std::string>(root, "joint_states_topic", cfg.joint_states_topic);
  cfg.base_frame = getOr<std::string>(root, "base_frame", cfg.base_frame);
  cfg.ee_frame = getOr<std::string>(root, "ee_frame", cfg.ee_frame);

  cfg.tf_timeout_sec = getOr<double>(root, "tf_timeout_sec", cfg.tf_timeout_sec);
  cfg.state_stale_sec = getOr<double>(root, "state_stale_sec", cfg.state_stale_sec);

  cfg.move_group_action_name =
      getOr<std::string>(root, "move_group_action_name", cfg.move_group_action_name);
  cfg.execute_traj_action_name =
      getOr<std::string>(root, "execute_traj_action_name", cfg.execute_traj_action_name);
  cfg.get_planning_scene_service_name = getOr<std::string>(
      root, "get_planning_scene_service_name", cfg.get_planning_scene_service_name);

  cfg.default_planning_profile =
      getOr<std::string>(root, "default_planning_profile", cfg.default_planning_profile);

  const std::string tick_mode = getOr<std::string>(root, "tick_mode", std::string{});
  if (!tick_mode.empty())
  {
    cfg.tick_option = parseTickMode(tick_mode);
  }

  const std::int64_t max_block_ms = getOr<std::int64_t>(
      root, "tick_while_running_max_block_ms", cfg.tick_while_running_max_block.count());
  if (max_block_ms < 0)
  {
    throw std::runtime_error(
        "Executor config: 'tick_while_running_max_block_ms' must be >= 0");
  }
  cfg.tick_while_running_max_block = std::chrono::milliseconds(max_block_ms);

  validateConfig(cfg);

  return cfg;
}

void loadPlanningProfilesInto(ExecConfig& cfg, const std::string& yaml_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(yaml_path);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(std::string("Failed to load planning profiles YAML: '") +
                             yaml_path + "'. Details: " + std::string(e.what()));
  }

  const YAML::Node profiles = root["profiles"];
  if (!profiles || !profiles.IsMap())
  {
    throw std::runtime_error(
        "Planning profiles YAML must contain a map at key 'profiles'");
  }

  cfg.planning_profiles.clear();

  for (auto it = profiles.begin(); it != profiles.end(); ++it)
  {
    const std::string name = it->first.as<std::string>();
    const YAML::Node p = it->second;

    if (!p || !p.IsMap())
    {
      throw std::runtime_error(std::string("Profile '") + name + "' must be a YAML map");
    }

    PlanningProfile profile;
    profile.name = name;

    // Required selectors
    profile.pipeline_id = requireString(p, "pipeline_id", "profiles." + name);
    profile.planner_id = requireString(p, "planner_id", "profiles." + name);

    // Optional knobs
    profile.allowed_planning_time_sec =
        getOr<double>(p, "allowed_planning_time_sec", profile.allowed_planning_time_sec);

    profile.num_planning_attempts =
        getOr<int>(p, "num_planning_attempts", profile.num_planning_attempts);

    profile.executor_mode = getOr<std::string>(p, "executor_mode", profile.executor_mode);

    profile.fjt_action_name =
        getOr<std::string>(p, "fjt_action_name", profile.fjt_action_name);

    // Validate knobs
    if (profile.allowed_planning_time_sec <= 0.0)
    {
      throw std::runtime_error("profiles." + name +
                               ".allowed_planning_time_sec must be > 0");
    }

    if (profile.num_planning_attempts < 1)
    {
      throw std::runtime_error("profiles." + name + ".num_planning_attempts must be >= 1");
    }

    // Validate executor mode
    if (profile.executor_mode != consts::EXECUTOR_MODE_MOVEIT &&
        profile.executor_mode != consts::EXECUTOR_MODE_DIRECT_FJT)
    {
      throw std::runtime_error("profiles." + name + ".executor_mode must be one of: '" +
                               std::string(consts::EXECUTOR_MODE_MOVEIT) + "', '" +
                               std::string(consts::EXECUTOR_MODE_DIRECT_FJT) + "'");
    }

    if (profile.executor_mode == consts::EXECUTOR_MODE_DIRECT_FJT &&
        profile.fjt_action_name.empty())
    {
      throw std::runtime_error("profiles." + name +
                               ".fjt_action_name is required when executor_mode == '" +
                               std::string(consts::EXECUTOR_MODE_DIRECT_FJT) + "'");
    }

    cfg.planning_profiles.emplace(profile.name, std::move(profile));
  }

  // Validate default profile (if set in executor.yaml)
  if (!cfg.default_planning_profile.empty())
  {
    if (cfg.planning_profiles.find(cfg.default_planning_profile) ==
        cfg.planning_profiles.end())
    {
      throw std::runtime_error(
          "default_planning_profile '" + cfg.default_planning_profile +
          "' not found in planning profiles file '" + yaml_path + "'");
    }
  }
}

}  // namespace config
}  // namespace rlc_executive
