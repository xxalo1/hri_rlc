#pragma once

/**
 * @file tesseract_utils.hpp
 * @brief Helpers for interacting with Tesseract monitoring services.
 */

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace tesseract_monitoring
{
class ROSEnvironmentMonitorInterface;
}  // namespace tesseract_monitoring

namespace rlc_utils::tesseract_utils
{

/**
 * @brief Discovers the Tesseract environment id hosted under a monitor namespace.
 * @param[in] node Node used to create a service client and spin for the response; must
 * not be null.
 * @param[in] logger Logger used for warnings about discovery failures.
 * @param[in] monitor_namespace Tesseract monitor namespace used to build the service
 * name `/<monitor_namespace>/get_tesseract_information`.
 * @param[in] timeout Maximum wait time for service availability and the request [ms].
 * @return Discovered environment id string. Empty if discovery fails.
 *
 * @details
 * This queries `tesseract_msgs::srv::GetEnvironmentInformation` with `flags = 0` and
 * uses `response.id` as the authoritative environment id.
 */
[[nodiscard]] std::string
discoverEnvironmentId(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                      const std::string& monitor_namespace,
                      std::chrono::milliseconds timeout);

/**
 * @brief Creates a `tesseract_monitoring::ROSEnvironmentMonitorInterface` client.
 * @param[in] node Node used for ROS interfaces; must not be null.
 * @param[in] logger Logger used for warnings/errors.
 * @param[in] monitor_namespace Monitor namespace hosting Tesseract monitor services.
 * @param[in] timeout Maximum wait time for environment id discovery and namespace connectivity [ms].
 * @param[in] configured_env_id Optional configured environment id. If discovery fails, this id is used.
 * @return Monitor interface client. Null if environment id is empty or monitor connectivity fails.
 *
 * @details
 * This function:
 * 1) tries to discover the environment id via `discoverEnvironmentId()`,
 * 2) falls back to `configured_env_id` if discovery fails,
 * 3) constructs a `ROSEnvironmentMonitorInterface` with the resolved id,
 * 4) calls `addNamespace(monitor_namespace)` and `waitForNamespace(...)`.
 */
[[nodiscard]] std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
makeMonitorInterface(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                     const std::string& monitor_namespace,
                     std::chrono::milliseconds timeout,
                     const std::string& configured_env_id = {});

/**
 * @brief Creates a `tesseract_monitoring::ROSEnvironmentMonitorInterface` backed by a dedicated auxiliary node.
 * @param[in] parent_node Parent node used to derive the ROS context, namespace, and client node name; must remain valid for the duration of this call.
 * @param[in] logger Logger used for warnings/errors.
 * @param[in] monitor_namespace Monitor namespace hosting Tesseract monitor services.
 * @param[in] timeout Maximum wait time for environment id discovery and namespace connectivity [ms].
 * @param[in] configured_env_id Optional configured environment id. If discovery fails, this id is used.
 * @return Monitor interface client. Null if the auxiliary node cannot connect to the monitor services.
 *
 * @details
 * This overload creates a dedicated client node in the same ROS context and namespace as
 * `parent_node`, disables parameter services on that auxiliary node, and passes it to
 * `makeMonitorInterface(const rclcpp::Node::SharedPtr&, ...)`. The returned monitor
 * interface stores the auxiliary node internally, so no extra node ownership is required
 * from the caller after this function returns.
 */
[[nodiscard]] std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
makeMonitorInterfaceFromParentNode(rclcpp::Node& parent_node,
                                   const rclcpp::Logger& logger,
                                   const std::string& monitor_namespace,
                                   std::chrono::milliseconds timeout,
                                   const std::string& configured_env_id = {});

}  // namespace rlc_utils::tesseract_utils
