#include "rlc_executive/core/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
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

YAML::Node mapNode(const YAML::Node& parent, const std::string& key);

int resolveSdlAxisName(const std::string& token)
{
  if (token == "LEFTX")
  {
    return 0;
  }
  if (token == "LEFTY")
  {
    return 1;
  }
  if (token == "RIGHTX")
  {
    return 2;
  }
  if (token == "RIGHTY")
  {
    return 3;
  }
  if (token == "TRIGGERLEFT")
  {
    return 4;
  }
  if (token == "TRIGGERRIGHT")
  {
    return 5;
  }

  return -1;
}

int resolveSdlButtonName(const std::string& token)
{
  if (token == "A")
  {
    return 0;
  }
  if (token == "B")
  {
    return 1;
  }
  if (token == "X")
  {
    return 2;
  }
  if (token == "Y")
  {
    return 3;
  }
  if (token == "BACK" || token == "SELECT")
  {
    return 4;
  }
  if (token == "GUIDE")
  {
    return 5;
  }
  if (token == "START")
  {
    return 6;
  }
  if (token == "LEFTSTICK")
  {
    return 7;
  }
  if (token == "RIGHTSTICK")
  {
    return 8;
  }
  if (token == "LEFTSHOULDER")
  {
    return 9;
  }
  if (token == "RIGHTSHOULDER")
  {
    return 10;
  }
  if (token == "DPAD_UP")
  {
    return 11;
  }
  if (token == "DPAD_DOWN")
  {
    return 12;
  }
  if (token == "DPAD_LEFT")
  {
    return 13;
  }
  if (token == "DPAD_RIGHT")
  {
    return 14;
  }

  return -1;
}

std::string normalizeControllerToken(const std::string& token)
{
  std::string normalized;
  normalized.reserve(token.size());

  for (const unsigned char c : token)
  {
    if (std::isspace(c) != 0)
    {
      continue;
    }

    if (c == '-')
    {
      normalized.push_back('_');
      continue;
    }

    normalized.push_back(static_cast<char>(std::toupper(c)));
  }

  return normalized;
}

int parseControllerIndex(const YAML::Node& node, const std::string& context,
                         int (*resolve_token)(const std::string&))
{
  if (!node || node.IsNull())
  {
    throw std::runtime_error("Missing required YAML key '" + context + "'");
  }
  if (!node.IsScalar())
  {
    throw std::runtime_error("Expected scalar value at YAML key '" + context + "'");
  }

  try
  {
    const int index = node.as<int>();
    if (index < 0)
    {
      throw std::runtime_error("YAML key '" + context + "' must be >= 0");
    }
    return index;
  }
  catch (const YAML::BadConversion&)
  {
  }

  const std::string token = normalizeControllerToken(node.as<std::string>());
  const int index = resolve_token(token);
  if (index < 0)
  {
    throw std::runtime_error("Unsupported controller mapping '" + node.as<std::string>() +
                             "' at YAML key '" + context + "'");
  }

  return index;
}

void loadAxisMapping(ServoJoyAxisConfig& axis_cfg, const YAML::Node& parent,
                     const std::string& key)
{
  const YAML::Node axis_node = mapNode(parent, key);
  if (!axis_node || axis_node.IsNull())
  {
    return;
  }

  if (const YAML::Node n = axis_node["x"])
  {
    axis_cfg.x = parseControllerIndex(n, key + ".x", resolveSdlAxisName);
  }
  if (const YAML::Node n = axis_node["y"])
  {
    axis_cfg.y = parseControllerIndex(n, key + ".y", resolveSdlAxisName);
  }
  if (const YAML::Node n = axis_node["z_pos"])
  {
    axis_cfg.z_pos = parseControllerIndex(n, key + ".z_pos", resolveSdlAxisName);
  }
  if (const YAML::Node n = axis_node["z_neg"])
  {
    axis_cfg.z_neg = parseControllerIndex(n, key + ".z_neg", resolveSdlAxisName);
  }
}

void loadButtonMapping(ServoJoyButtonsConfig& button_cfg, const YAML::Node& parent,
                       const std::string& key)
{
  const YAML::Node button_node = mapNode(parent, key);
  if (!button_node || button_node.IsNull())
  {
    return;
  }

  if (const YAML::Node n = button_node["finish"])
  {
    button_cfg.finish = parseControllerIndex(n, key + ".finish", resolveSdlButtonName);
  }
  if (const YAML::Node n = button_node["abort"])
  {
    button_cfg.abort = parseControllerIndex(n, key + ".abort", resolveSdlButtonName);
  }
}

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

YAML::Node mapNode(const YAML::Node& parent, const std::string& key)
{
  const YAML::Node child = parent[key];
  if (child && !child.IsNull() && !child.IsMap())
  {
    throw std::runtime_error("Expected YAML map at key '" + key + "'");
  }
  return child;
}

std::string requireString(const YAML::Node& parent, const std::string& key,
                          const std::string& context)
{
  const YAML::Node n = parent[key];
  if (!n || n.IsNull())
  {
    throw std::runtime_error("Missing required YAML key '" + context + "." + key + "'");
  }
  return n.as<std::string>();
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

  throw std::runtime_error("Executor config: 'tick.mode' must be one of: "
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

  if (!root || root.IsNull())
  {
    return ExecConfig{};
  }
  if (!root.IsMap())
  {
    throw std::runtime_error("Executor config root must be a YAML map");
  }

  ExecConfig cfg;

  const YAML::Node tick = mapNode(root, "tick");
  cfg.tick.rate_hz = getOr<double>(tick, "rate_hz", cfg.tick.rate_hz);
  const std::string tick_mode = getOr<std::string>(tick, "mode", std::string{});
  if (!tick_mode.empty())
  {
    cfg.tick.option = parseTickMode(tick_mode);
  }
  cfg.tick.while_running_max_block = std::chrono::milliseconds(getOr<std::int64_t>(
      tick, "while_running_max_block_ms", cfg.tick.while_running_max_block.count()));

  const YAML::Node state = mapNode(root, "state");
  cfg.state.joint_states_topic =
      getOr<std::string>(state, "joint_states_topic", cfg.state.joint_states_topic);
  cfg.state.base_frame = getOr<std::string>(state, "base_frame", cfg.state.base_frame);
  cfg.state.ee_frame = getOr<std::string>(state, "ee_frame", cfg.state.ee_frame);
  cfg.state.tf_timeout_sec = getOr<double>(state, "tf_timeout_sec", cfg.state.tf_timeout_sec);
  cfg.state.state_stale_sec =
      getOr<double>(state, "state_stale_sec", cfg.state.state_stale_sec);

  const YAML::Node move_group = mapNode(root, "move_group");
  cfg.move_group.action_name =
      getOr<std::string>(move_group, "action_name", cfg.move_group.action_name);
  cfg.move_group.get_planning_scene_service_name = getOr<std::string>(
      move_group, "get_planning_scene_service_name",
      cfg.move_group.get_planning_scene_service_name);

  const YAML::Node execution = mapNode(root, "execution");
  cfg.execution.execute_traj_action_name = getOr<std::string>(
      execution, "execute_traj_action_name", cfg.execution.execute_traj_action_name);

  const YAML::Node planning = mapNode(root, "planning");
  cfg.planning.default_profile =
      getOr<std::string>(planning, "default_profile", cfg.planning.default_profile);

  const YAML::Node servo = mapNode(root, "servo");
  cfg.servo.pause_service_name =
      getOr<std::string>(servo, "pause_service_name", cfg.servo.pause_service_name);
  cfg.servo.switch_command_type_service_name = getOr<std::string>(
      servo, "switch_command_type_service_name",
      cfg.servo.switch_command_type_service_name);
  cfg.servo.status_topic =
      getOr<std::string>(servo, "status_topic", cfg.servo.status_topic);

  const YAML::Node servo_joy = mapNode(root, "servo_joy_frontend");
  cfg.servo_joy_frontend.joy_topic =
      getOr<std::string>(servo_joy, "joy_topic", cfg.servo_joy_frontend.joy_topic);
  cfg.servo_joy_frontend.twist_topic =
      getOr<std::string>(servo_joy, "twist_topic", cfg.servo_joy_frontend.twist_topic);
  cfg.servo_joy_frontend.joint_topic =
      getOr<std::string>(servo_joy, "joint_topic", cfg.servo_joy_frontend.joint_topic);
  cfg.servo_joy_frontend.pose_topic =
      getOr<std::string>(servo_joy, "pose_topic", cfg.servo_joy_frontend.pose_topic);
  cfg.servo_joy_frontend.command_frame =
      getOr<std::string>(servo_joy, "command_frame", cfg.servo_joy_frontend.command_frame);
  if (cfg.servo_joy_frontend.command_frame.empty())
  {
    cfg.servo_joy_frontend.command_frame = cfg.state.base_frame;
  }
    cfg.servo_joy_frontend.deadzone =
      getOr<double>(servo_joy, "deadzone", cfg.servo_joy_frontend.deadzone);
    cfg.servo_joy_frontend.linear_scale =
      getOr<double>(servo_joy, "linear_scale", cfg.servo_joy_frontend.linear_scale);
    cfg.servo_joy_frontend.angular_scale =
      getOr<double>(servo_joy, "angular_scale", cfg.servo_joy_frontend.angular_scale);

    loadAxisMapping(cfg.servo_joy_frontend.axis_linear, servo_joy, "axis_linear");
    loadAxisMapping(cfg.servo_joy_frontend.axis_angular, servo_joy, "axis_angular");

    if (const YAML::Node legacy_finish = servo_joy["button_finish"])
    {
    cfg.servo_joy_frontend.buttons.finish = parseControllerIndex(
        legacy_finish, "servo_joy_frontend.button_finish", resolveSdlButtonName);
    }
    if (const YAML::Node legacy_abort = servo_joy["button_abort"])
    {
    cfg.servo_joy_frontend.buttons.abort = parseControllerIndex(
        legacy_abort, "servo_joy_frontend.button_abort", resolveSdlButtonName);
    }
    loadButtonMapping(cfg.servo_joy_frontend.buttons, servo_joy, "buttons");

  const YAML::Node monitor = mapNode(root, "monitor");
  cfg.monitor.monitor_namespace =
      getOr<std::string>(monitor, "monitor_namespace", cfg.monitor.monitor_namespace);
  cfg.monitor.env_name = getOr<std::string>(monitor, "env_name", cfg.monitor.env_name);
  cfg.monitor.wait_timeout = std::chrono::milliseconds(
      getOr<std::int64_t>(monitor, "wait_timeout_ms", cfg.monitor.wait_timeout.count()));

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

  cfg.planning.profiles.clear();

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
    profile.pipeline_id = requireString(p, "pipeline_id", "profiles." + name);
    profile.planner_id = requireString(p, "planner_id", "profiles." + name);
    profile.allowed_planning_time_sec =
        getOr<double>(p, "allowed_planning_time_sec", profile.allowed_planning_time_sec);
    profile.num_planning_attempts =
        getOr<int>(p, "num_planning_attempts", profile.num_planning_attempts);
    profile.max_velocity_scaling_factor = getOr<double>(
        p, "max_velocity_scaling_factor", profile.max_velocity_scaling_factor);
    profile.max_acceleration_scaling_factor = getOr<double>(
        p, "max_acceleration_scaling_factor", profile.max_acceleration_scaling_factor);
    profile.executor_mode = getOr<std::string>(p, "executor_mode", profile.executor_mode);
    profile.fjt_action_name =
        getOr<std::string>(p, "fjt_action_name", profile.fjt_action_name);

    if (profile.allowed_planning_time_sec <= 0.0)
    {
      throw std::runtime_error("profiles." + name +
                               ".allowed_planning_time_sec must be > 0");
    }
    if (profile.num_planning_attempts < 1)
    {
      throw std::runtime_error("profiles." + name + ".num_planning_attempts must be >= 1");
    }
    if (profile.max_velocity_scaling_factor <= 0.0 ||
        profile.max_velocity_scaling_factor > 1.0)
    {
      throw std::runtime_error("profiles." + name +
                               ".max_velocity_scaling_factor must be in (0, 1]");
    }
    if (profile.max_acceleration_scaling_factor <= 0.0 ||
        profile.max_acceleration_scaling_factor > 1.0)
    {
      throw std::runtime_error("profiles." + name +
                               ".max_acceleration_scaling_factor must be in (0, 1]");
    }
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

    cfg.planning.profiles.emplace(profile.name, std::move(profile));
  }
}

}  // namespace config
}  // namespace rlc_executive
