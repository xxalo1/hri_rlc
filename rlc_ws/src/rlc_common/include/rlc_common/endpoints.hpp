#pragma once

#include <string>

namespace rlc_common {

struct TopicEndpoints {
  std::string joint_state;
  std::string joint_state_action;
  std::string effort_cmd;
  std::string planned_joint_traj;
  std::string frame_states;
  std::string planned_cart_traj;
  std::string controller_state;
  std::string current_plan;
};

struct ServiceEndpoints {
  std::string reset_sim;
  std::string pause_sim;
  std::string set_controller_gains;
  std::string set_controller_mode;
  std::string plan_quintic;
  std::string execute_traj;
};

struct ActionEndpoints {
  std::string follow_traj;
};

struct Endpoints {
  TopicEndpoints topics;
  ServiceEndpoints services;
  ActionEndpoints actions;
};

Endpoints LoadEndpoints();
const Endpoints& GetEndpoints();
const TopicEndpoints& Topics();
const ServiceEndpoints& Services();
const ActionEndpoints& Actions();

} // namespace rlc_common
