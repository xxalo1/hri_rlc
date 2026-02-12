#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace rlc_planner
{

class PlanningError : public std::runtime_error
{
public:
  PlanningError(int32_t code, const std::string& msg)
    : std::runtime_error(msg), code_(code)
  {
  }

  int32_t code() const noexcept { return code_; }

private:
  int32_t code_;
};

}  // namespace rlc_planner
