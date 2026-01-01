#include "rlc_common/endpoints.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <string>

namespace rlc_common {
namespace {

std::string Resolve(const std::string& raw, const YAML::Node& namespaces)
{
  std::string resolved = raw;
  for (const auto& entry : namespaces) {
    const auto key = entry.first.as<std::string>();
    const auto value = entry.second.as<std::string>();
    const std::string token = "{" + key + "}";
    std::size_t pos = 0;
    while ((pos = resolved.find(token, pos)) != std::string::npos) {
      resolved.replace(pos, token.size(), value);
      pos += value.size();
    }
  }
  return resolved;
}

} // namespace

Endpoints LoadEndpoints()
{
  const auto share_dir = ament_index_cpp::get_package_share_directory("rlc_common");
  const auto root = YAML::LoadFile(share_dir + "/config/endpoints.yaml");

  const auto namespaces = root["namespaces"];
  const auto topics = root["topics"];
  const auto services = root["services"];
  const auto actions = root["actions"];

  Endpoints endpoints;
  endpoints.topics.joint_state = Resolve(topics["joint_state"].as<std::string>(), namespaces);
  endpoints.topics.joint_state_action = Resolve(topics["joint_state_action"].as<std::string>(), namespaces);
  endpoints.topics.effort_cmd = Resolve(topics["effort_cmd"].as<std::string>(), namespaces);
  endpoints.topics.planned_joint_traj = Resolve(topics["planned_joint_traj"].as<std::string>(), namespaces);
  endpoints.topics.frame_states = Resolve(topics["frame_states"].as<std::string>(), namespaces);
  endpoints.topics.planned_cart_traj = Resolve(topics["planned_cart_traj"].as<std::string>(), namespaces);
  endpoints.topics.controller_state = Resolve(topics["controller_state"].as<std::string>(), namespaces);
  endpoints.topics.current_plan = Resolve(topics["current_plan"].as<std::string>(), namespaces);

  endpoints.services.reset_sim = Resolve(services["reset_sim"].as<std::string>(), namespaces);
  endpoints.services.pause_sim = Resolve(services["pause_sim"].as<std::string>(), namespaces);
  endpoints.services.set_controller_gains = Resolve(services["set_controller_gains"].as<std::string>(), namespaces);
  endpoints.services.set_controller_mode = Resolve(services["set_controller_mode"].as<std::string>(), namespaces);
  endpoints.services.plan_quintic = Resolve(services["plan_quintic"].as<std::string>(), namespaces);
  endpoints.services.execute_traj = Resolve(services["execute_traj"].as<std::string>(), namespaces);

  endpoints.actions.follow_traj = Resolve(actions["follow_traj"].as<std::string>(), namespaces);

  return endpoints;
}

const Endpoints& GetEndpoints()
{
  static const Endpoints endpoints = LoadEndpoints();
  return endpoints;
}

const TopicEndpoints& Topics()
{
  return GetEndpoints().topics;
}

const ServiceEndpoints& Services()
{
  return GetEndpoints().services;
}

const ActionEndpoints& Actions()
{
  return GetEndpoints().actions;
}

} // namespace rlc_common
