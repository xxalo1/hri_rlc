#include "rlc_executive/servo/teleoperation_controller.hpp"

#include <memory>
#include <stdexcept>

#include "rlc_executive/servo/servo_joy_frontend.hpp"

namespace rlc_executive
{
namespace
{

std::unique_ptr<TeleoperationFrontend>
makeFrontend(rclcpp::Node& node, const TeleoperationConfig& cfg, TeleoperationSession& session)
{
  if (cfg.frontend == "joy")
  {
    return std::make_unique<ServoJoyFrontend>(node, cfg.joy, session);
  }

  throw std::invalid_argument("TeleoperationController: unsupported teleoperation frontend '" +
                              cfg.frontend + "'");
}

}  // namespace

TeleoperationController::TeleoperationController(rclcpp::Node& node,
                                                 const TeleoperationConfig& teleop_cfg)
  : cfg_(teleop_cfg),
    servo_client_(node, cfg_.servo),
    session_(node),
    frontend_(makeFrontend(node, cfg_, session_))
{
  frontend_->setEnabled(false);
}

ServoPauseResult
TeleoperationController::setActive(bool active, std::chrono::milliseconds timeout)
{
  if (!active)
  {
    frontend_->setEnabled(false);
    active_.store(false, std::memory_order_relaxed);
    session_.setRequest(DemoRequest::NONE);
    return servo_client_.setPaused(true, timeout);
  }

  const ServoPauseResult result = servo_client_.setPaused(false, timeout);
  if (!result.success)
  {
    frontend_->setEnabled(false);
    active_.store(false, std::memory_order_relaxed);
    return result;
  }

  session_.setRequest(DemoRequest::NONE);
  frontend_->setEnabled(true);
  active_.store(true, std::memory_order_relaxed);
  return result;
}

DemoRequest TeleoperationController::takeRequest()
{
  return session_.takeRequest();
}

DemoRequest TeleoperationController::peekRequest() const
{
  return session_.peekRequest();
}

std::optional<ServoStatusSnapshot> TeleoperationController::status() const
{
  return servo_client_.status();
}

}  // namespace rlc_executive
