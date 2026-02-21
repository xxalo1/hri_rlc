#pragma once

#include <string>

#include "rlc_executive/core/exec_config.hpp"

namespace rlc_executive
{
namespace config
{
ExecConfig loadExecConfigFromFile(const std::string& yaml_path);
void loadPlanningProfilesInto(ExecConfig& cfg, const std::string& yaml_path);
}  // namespace config
}  // namespace rlc_executive