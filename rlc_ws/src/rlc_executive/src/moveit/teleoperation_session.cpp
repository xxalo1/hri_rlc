#include "rlc_executive/moveit/teleoperation_session.hpp"

namespace rlc_executive
{

TeleoperationSession::TeleoperationSession(rclcpp::Node& node)
  : node_(&node)
{
}

TeleoperationSession::DemoRequest TeleoperationSession::takeRequest()
{
  std::scoped_lock lock(request_mtx_);

  const DemoRequest out = request_;
  request_ = DemoRequest::NONE;
  request_stamp_.reset();
  return out;
}

TeleoperationSession::DemoRequest TeleoperationSession::peekRequest() const
{
  std::scoped_lock lock(request_mtx_);
  return request_;
}

void TeleoperationSession::setRequest(DemoRequest request)
{
  std::scoped_lock lock(request_mtx_);
  request_ = request;

  if (request == DemoRequest::NONE)
  {
    request_stamp_.reset();
    return;
  }

  request_stamp_ = node_->now();
}

}  // namespace rlc_executive
