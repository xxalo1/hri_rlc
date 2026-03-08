#include <rlc_utils/tesseract/tesseract_utils.hpp>

#include <tesseract_msgs/srv/get_environment_information.hpp>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>

namespace rlc_utils::tesseract_utils
{

namespace
{
constexpr char GET_ENV_INFO_SERVICE[] = "/get_tesseract_information";
std::atomic_uint64_t s_monitor_client_node_counter{ 0 };

std::string makeMonitorClientNodeName(const rclcpp::Node& parent_node)
{
  const auto node_index =
      s_monitor_client_node_counter.fetch_add(1, std::memory_order_relaxed);
  return std::string{ parent_node.get_name() } + "_tesseract_monitor_client_" +
         std::to_string(node_index);
}

rclcpp::Node::SharedPtr makeMonitorClientNode(rclcpp::Node& parent_node)
{
  rclcpp::NodeOptions node_options;
  node_options.context(parent_node.get_node_base_interface()->get_context());
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  return std::make_shared<rclcpp::Node>(makeMonitorClientNodeName(parent_node),
                                        parent_node.get_namespace(), node_options);
}
}  // namespace

std::string discoverEnvironmentId(const rclcpp::Node::SharedPtr& node,
                                  const rclcpp::Logger& logger,
                                  const std::string& monitor_namespace,
                                  std::chrono::milliseconds timeout)
{
  if (!node)
  {
    RCLCPP_WARN(logger, "discoverEnvironmentId(): node is null");
    return {};
  }

  const auto clamped_timeout =
      std::chrono::milliseconds{ std::max<int64_t>(0, timeout.count()) };

  const std::string info_srv =
      "/" + monitor_namespace + std::string{ GET_ENV_INFO_SERVICE };

  auto info_client =
      node->create_client<tesseract_msgs::srv::GetEnvironmentInformation>(info_srv);

  if (!info_client->wait_for_service(clamped_timeout))
  {
    RCLCPP_WARN(logger,
                "Tesseract monitor info service '%s' not available after %ld ms. "
                "Environment id discovery failed.",
                info_srv.c_str(), static_cast<long>(clamped_timeout.count()));
    return {};
  }

  try
  {
    auto req =
        std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation::Request>();
    req->flags = 0;

    auto fut = info_client->async_send_request(req);
    const auto rc = rclcpp::spin_until_future_complete(node, fut, clamped_timeout);
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(logger,
                  "Timed out requesting Tesseract environment id from '%s' after %ld ms. "
                  "Environment id discovery failed.",
                  info_srv.c_str(), static_cast<long>(clamped_timeout.count()));
      return {};
    }

    auto res = fut.get();
    if (!res || !res->success || res->id.empty())
    {
      RCLCPP_WARN(logger,
                  "Tesseract monitor '%s' returned no environment id (success=%s). "
                  "Environment id discovery failed.",
                  info_srv.c_str(), (res && res->success) ? "true" : "false");
      return {};
    }

    return res->id;
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger,
                "Failed to discover Tesseract environment id from '%s': %s. "
                "Environment id discovery failed.",
                info_srv.c_str(), e.what());
    return {};
  }
}

std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
makeMonitorInterface(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger,
                     const std::string& monitor_namespace,
                     std::chrono::milliseconds timeout,
                     const std::string& configured_env_id)
{
  if (!node)
  {
    RCLCPP_ERROR(logger, "makeMonitorInterface(): node is null");
    return nullptr;
  }

  const std::string discovered_env_id =
      discoverEnvironmentId(node, logger, monitor_namespace, timeout);

  if (!configured_env_id.empty() && !discovered_env_id.empty() &&
      configured_env_id != discovered_env_id)
  {
    RCLCPP_WARN(logger, "Configured env id '%s' does not match server id '%s'.",
                configured_env_id.c_str(), discovered_env_id.c_str());
  }

  const std::string env_id =
      discovered_env_id.empty() ? configured_env_id : discovered_env_id;

  if (env_id.empty())
  {
    RCLCPP_ERROR(logger, "Tesseract environment id is empty (discovery failed and no "
                         "configured id provided).");
    return nullptr;
  }

  auto mon = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitorInterface>(
      node, env_id);
  mon->addNamespace(monitor_namespace);

  const auto clamped_timeout =
      std::chrono::milliseconds{ std::max<int64_t>(0, timeout.count()) };
  if (!mon->waitForNamespace(monitor_namespace, clamped_timeout))
  {
    RCLCPP_ERROR(logger,
                 "Tesseract monitor not reachable under namespace '%s' after %ld ms",
                 monitor_namespace.c_str(), static_cast<long>(clamped_timeout.count()));
    return nullptr;
  }

  return mon;
}

std::shared_ptr<tesseract_monitoring::ROSEnvironmentMonitorInterface>
makeMonitorInterfaceFromParentNode(rclcpp::Node& parent_node,
                                   const rclcpp::Logger& logger,
                                   const std::string& monitor_namespace,
                                   std::chrono::milliseconds timeout,
                                   const std::string& configured_env_id)
{
  return makeMonitorInterface(makeMonitorClientNode(parent_node), logger,
                              monitor_namespace, timeout, configured_env_id);
}

}  // namespace rlc_utils::tesseract_utils
