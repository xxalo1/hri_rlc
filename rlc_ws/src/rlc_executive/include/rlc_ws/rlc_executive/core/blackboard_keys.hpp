#pragma once

namespace rlc_executive_bt
{
namespace bb
{
// Shared runtime context injected once per tree.
inline constexpr const char* RUNTIME_CONTEXT = "ctx";

// Task scoped input (from RunTask action goal).
inline constexpr const char* TASK_CONTEXT = "task";

// Latest or snapshotted robot environment state used for planning and metrics.
inline constexpr const char* STATE_SNAPSHOT = "state";

// Planning output from MoveIt planning skill.
inline constexpr const char* PLAN_RESULT = "plan_result";

// Execution output from execution skill.
inline constexpr const char* EXEC_RESULT = "exec_result";

// Latest interaction event (human torque, contact, etc).
inline constexpr const char* INTERACTION_EVENT = "interaction_event";

// Metrics accumulator for logging and reward.
inline constexpr const char* METRICS = "metrics";
}  // namespace bb
}  // namespace rlc_executive_bt