#include "rlc_executive/bt_nodes/bt_utils.hpp"

#include <exception>
#include <utility>

#include <behaviortree_cpp/exceptions.h>

#include "rlc_executive/core/blackboard_keys.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{
namespace bt_utils
{

void ensureContextAndLogger(const BT::TreeNode& node,
                            std::shared_ptr<RuntimeContext>& ctx_cache,
                            std::optional<rclcpp::Logger>& logger_cache)
{
  if (!ctx_cache)
  {
    try
    {
      if (!node.config().blackboard)
      {
        throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                               "]: blackboard is null");
      }

      ctx_cache = node.config().blackboard->get<std::shared_ptr<RuntimeContext>>(
          bb::RUNTIME_CONTEXT);
    }
    catch (const std::exception& e)
    {
      throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                             "]: failed to read RuntimeContext: ", e.what());
    }
  }

  if (!ctx_cache)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: RuntimeContext missing on blackboard");
  }

  if (!logger_cache)
  {
    rclcpp::Logger base_logger = ctx_cache->node().get_logger();

    base_logger = base_logger.get_child(node.registrationName());
    base_logger = base_logger.get_child(sanitizeLoggerName(node.fullPath()));

    logger_cache = std::move(base_logger);
  }
}

std::string sanitizeLoggerName(std::string s)
{
  for (char& c : s)
  {
    if (c == '/')
    {
      c = '.';
      continue;
    }

    const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z') || (c == '.') || (c == '_');
    if (!ok)
    {
      c = '_';
    }
  }
  return s;
}

void setDiagnostics(BT::TreeNode& node, const std::string& error,
                    const std::string& feedback, double elapsed_sec)
{
  setOutput(node, DiagnosticPortKeys::ERROR, error);
  setOutput(node, DiagnosticPortKeys::FEEDBACK, feedback);
  setOutput(node, DiagnosticPortKeys::ELAPSED_SEC, elapsed_sec);
}

void setRunningDiagnostics(BT::TreeNode& node, const std::string& feedback,
                           double elapsed_sec)
{
  setDiagnostics(node, std::string(), feedback, elapsed_sec);
}

void setFailureDiagnostics(BT::TreeNode& node, const std::string& error,
                           double elapsed_sec)
{
  setDiagnostics(node, error, std::string(), elapsed_sec);
}

void setSuccessDiagnostics(BT::TreeNode& node, double elapsed_sec)
{
  setDiagnostics(node, std::string(), std::string(), elapsed_sec);
}

}  // namespace bt_utils
}  // namespace rlc_executive
