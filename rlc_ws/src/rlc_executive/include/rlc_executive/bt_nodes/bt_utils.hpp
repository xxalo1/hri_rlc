#pragma once

#include <exception>
#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/exceptions.h>
#include <rclcpp/logger.hpp>

namespace rlc_executive
{

class RuntimeContext;

namespace bt_utils
{

/**
 * @brief Standard diagnostic port names shared by BT nodes.
 *
 * @details
 * Use these port names consistently across nodes, and remap them in XML to avoid
 * blackboard collisions between different node instances.
 */
struct DiagnosticPortKeys
{
  static inline const std::string ERROR = "error";
  static inline const std::string FEEDBACK = "feedback";
  static inline const std::string ELAPSED_SEC = "elapsed_sec";
};

/**
 * @brief Ensure that RuntimeContext and a BT-scoped logger are initialized.
 *
 * @details
 * If `ctx_cache` is empty, this function reads it from the blackboard key
 * `bb::RUNTIME_CONTEXT`.
 *
 * If `logger_cache` is empty, it is initialized from `ctx_cache->node().get_logger()`,
 * then extended with child loggers using `node.registrationName()` and `node.fullPath()`
 * (sanitized).
 *
 * @param[in] node BT node requesting context/logging.
 * @param[in,out] ctx_cache Cached runtime context (filled on first call).
 * @param[in,out] logger_cache Cached logger (filled on first call).
 *
 * @throws BT::RuntimeError If RuntimeContext is missing/unreadable.
 */
void ensureContextAndLogger(const BT::TreeNode& node,
                            std::shared_ptr<RuntimeContext>& ctx_cache,
                            std::optional<rclcpp::Logger>& logger_cache);

/**
 * @brief Sanitize a string for use as part of an rclcpp logger name.
 *
 * @param[in] s Input name.
 * @return Sanitized name ("/" -> ".", other invalid chars -> "_").
 */
std::string sanitizeLoggerName(std::string s);

/**
 * @brief Reads a required value from the blackboard.
 *
 * @tparam T Type of the blackboard entry.
 * @param[in] node BT node reading the blackboard.
 * @param[in] key Blackboard entry key.
 * @return Blackboard value.
 * @throws BT::RuntimeError If the blackboard is null, the key is missing, or the value
 * can not be cast to `T`.
 */
template <class T>
T requireBlackboard(const BT::TreeNode& node, const std::string& key)
{
  if (!node.config().blackboard)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: blackboard is null");
  }

  try
  {
    return node.config().blackboard->get<T>(key);
  }
  catch (const std::exception& e)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to read blackboard key '", key, "': ", e.what());
  }
}

/**
 * @brief Sets an output port value and throws if it fails.
 *
 * @tparam T Type of the output value.
 * @param[in,out] node BT node setting the output.
 * @param[in] key Output port name.
 * @param[in] value Output value.
 * @throws BT::RuntimeError If setting the output port fails (e.g. port missing or not
 * remapped to a blackboard entry).
 */
template <class T>
void setOutput(BT::TreeNode& node, const std::string& key, const T& value)
{
  auto res = node.setOutput(key, value);

  if (!res)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to set output port '", key, "': ", res.error());
  }
}

/**
 * @brief Writes all standard diagnostic outputs.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] error Error message; empty to clear.
 * @param[in] feedback Feedback message; empty to clear.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setDiagnostics(BT::TreeNode& node, const std::string& error,
                    const std::string& feedback, double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a running node.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] feedback Feedback message.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setRunningDiagnostics(BT::TreeNode& node, const std::string& feedback,
                           double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a successful completion.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setSuccessDiagnostics(BT::TreeNode& node, double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a failure completion.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] error Error message.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setFailureDiagnostics(BT::TreeNode& node, const std::string& error,
                           double elapsed_sec);

/**
 * @brief Reads a required input port.
 *
 * @details
 * Wraps `BT::TreeNode::getInput(key, out)` and throws if the port is missing or
 * conversion fails.
 *
 * @tparam T Type of the input port.
 * @param[in] node BT node reading the input.
 * @param[in] key Input port name.
 * @return Input value.
 * @throws BT::RuntimeError If the input port is missing or can not be converted to `T`.
 */
template <class T>
T requireInput(const BT::TreeNode& node, const std::string& key)
{
  T out{};
  auto res = node.getInput(key, out);
  if (!res)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to read input port '", key, "': ", res.error());
  }
  return out;
}

}  // namespace bt_utils
}  // namespace rlc_executive
