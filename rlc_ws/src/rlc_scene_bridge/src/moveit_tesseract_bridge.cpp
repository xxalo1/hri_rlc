#include "rlc_scene_bridge/scene_bridge.hpp"

#include <functional>
#include <stdexcept>
#include <utility>

#include <rmw/qos_profiles.h>

namespace rlc_scene_bridge
{
MoveItTesseractBridge::MoveItTesseractBridge(rclcpp::Node& node,
                                             MonitorInterfaceConstPtr monitor_interface,
                                             Options opt)
  : node_(node)
  , logger_(node_.get_logger().get_child("rlc_scene_bridge.scene_bridge"))
  , opt_(std::move(opt))
{
  auto monitor_interface = makeMonitorInterface(node_, opt.monitor_namespace);
  if (!monitor_interface)
  {
    RCLCPP_ERROR(getLogger(), "MoveItTesseractBridge() failed: monitor_interface is null");
    return false;
  }

  env_sync_ = std::make_unique<TesseractEnvSync>(
      std::move(monitor_interface), opt_.monitor_namespace, logger_, opt_.env_sync_opt);

  cbg_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS qos(rclcpp::KeepLast(opt_.scene_qos_depth));
  qos.reliable();
  qos.durability_volatile();

  rclcpp::SubscriptionOptions sub_opts;

  sub_opts.event_callbacks.incompatible_qos_callback =
      [this](rclcpp::QOSRequestedIncompatibleQoSInfo& info) {
        RCLCPP_ERROR(logger_,
                     "Incompatible QoS on monitored planning scene subscription. "
                     "Last policy kind: %d, total_count: %d",
                     info.last_policy_kind, info.total_count);
      };
  sub_opts.callback_group = cbg_;

  auto scene_cb = [this](PlanningSceneMsgConstPtr msg) { onSceneUpdate(std::move(msg)); };

  scene_sub_ = node_.create_subscription<PlanningSceneMsg>(opt_.scene_topic, qos,
                                                           scene_cb, sub_opts);

  get_scene_client_ = node_.create_client<GetPlanningScene>(
      opt_.get_scene_srv, rmw_qos_profile_services_default, cbg_);
}

MoveItTesseractBridge::~MoveItTesseractBridge()
{
  scene_sub_.reset();
  get_scene_client_.reset();
  env_sync_.reset();
}

void MoveItTesseractBridge::onSceneUpdate(PlanningSceneMsgConstPtr msg)
{
  if (!msg)
  {
    return;
  }

  const auto now_ns = node_.now().nanoseconds();
  const auto env_state = env_state_.load(std::memory_order_acquire);

  // Full scene.
  if (!msg->is_diff)
  {
    try
    {
      const bool ok = env_sync_->applyFullScene(*msg);
      if (!ok)
      {
        RCLCPP_WARN(logger_, "MoveItTesseractBridge: full scene apply rejected; "
                             "requesting baseline resync");
        env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
        requestSync();
        return;
      }

      env_state_.store(EnvState::SYNCHRONIZED, std::memory_order_release);
      last_update_ns_.store(node_.now().nanoseconds(), std::memory_order_release);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "MoveItTesseractBridge: exception while applying full PlanningScene; "
                   "requesting baseline resync: %s",
                   e.what());
      env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
      requestSync();
    }
    return;
  }

  // Diff scene.
  switch (env_state)
  {
    case EnvState::UNINITIALIZED:
    {
      requestSync();
      return;
    }
    case EnvState::SYNCHRONIZING:
    {
      return;
    }
    case EnvState::SYNCHRONIZED:
    {
      try
      {
        const bool ok = env_sync_->applyDiff(*msg);
        if (!ok)
        {
          RCLCPP_ERROR(logger_, "MoveItTesseractBridge: failed to apply PlanningScene "
                                "diff; requesting baseline resync");
          env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
          requestSync();
          return;
        }

        last_update_ns_.store(node_.now().nanoseconds(), std::memory_order_release);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(logger_,
                     "MoveItTesseractBridge: failed to apply PlanningScene diff; "
                     "requesting baseline resync: %s",
                     e.what());
        env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
        requestSync();
      }
      return;
    }
  }

  RCLCPP_ERROR(logger_, "MoveItTesseractBridge: unknown EnvState");
  env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
  requestSync();
}

void MoveItTesseractBridge::requestSync()
{
  std::scoped_lock<std::mutex> lk(sync_mtx_);

  const auto env_state = env_state_.load(std::memory_order_acquire);
  if (env_state == EnvState::SYNCHRONIZING)
    return;

  if (!get_scene_client_)
  {
    RCLCPP_ERROR(logger_, "requestSync(): GetPlanningScene client is null");
    return;
  }

  if (!get_scene_client_->service_is_ready())
  {
    if (!get_scene_client_->wait_for_service(opt_.srv_wait))
    {
      RCLCPP_WARN(logger_,
                  "requestSync(): service '%s' not available (wait=%ld ms). "
                  "Will retry on next diff.",
                  opt_.get_scene_srv.c_str(), static_cast<long>(opt_.srv_wait.count()));
      return;
    }
  }

  env_state_.store(EnvState::SYNCHRONIZING, std::memory_order_release);

  auto req = std::make_shared<GetPlanningScene::Request>();
  req->components.components = opt_.scene_components;

  auto req_cb = [this](rclcpp::Client<GetPlanningScene>::SharedFuture future) {
    onSyncResponse(std::move(future));
  };

  (void)get_scene_client_->async_send_request(req, req_cb);
}

void MoveItTesseractBridge::onSyncResponse(
    rclcpp::Client<GetPlanningScene>::SharedFuture future)
{
  std::shared_ptr<GetPlanningScene::Response> res;

  if (env_state_.load(std::memory_order_acquire) != EnvState::SYNCHRONIZING)
  {
    RCLCPP_WARN(logger_, "onSyncResponse(): received response while not synchronizing");
    return;
  }

  try
  {
    res = future.get();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "onSyncResponse(): GetPlanningScene future.get() failed: %s",
                 e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    return;
  }

  if (!res)
  {
    RCLCPP_ERROR(logger_, "onSyncResponse(): GetPlanningScene response is null");
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    return;
  }

  try
  {
    const bool ok = env_sync_->applyFullScene(res->scene);
    if (!ok)
    {
      RCLCPP_WARN(logger_, "MoveItTesseractBridge: full scene apply rejected; requesting "
                           "baseline resync");
      env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
      requestSync();
      return;
    }
    env_state_.store(EnvState::SYNCHRONIZED, std::memory_order_release);
    last_update_ns_.store(node_.now().nanoseconds(), std::memory_order_release);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_,
                 "onSyncResponse(): failed to apply full PlanningScene from service; "
                 "requesting baseline resync: %s",
                 e.what());
    env_state_.store(EnvState::UNINITIALIZED, std::memory_order_release);
    requestSync();
  }
}

bool MoveItTesseractBridge::isSynchronized() const noexcept
{
  return env_state_.load(std::memory_order_relaxed) == EnvState::SYNCHRONIZED;
}

rclcpp::Time MoveItTesseractBridge::lastUpdateStamp() const noexcept
{
  const int64_t ns = last_update_ns_.load(std::memory_order_relaxed);
  return rclcpp::Time(ns, RCL_ROS_TIME);
}

std::shared_ptr<tesseract_environment::Environment>
MoveItTesseractBridge::envSnapshot() const
{
  if (!env_sync_)
  {
    RCLCPP_ERROR_THROTTLE(
        logger_, *node_.get_clock(), 2000,
        "MoveItTesseractBridge::envSnapshot(): EnvSync is not constructed.");
    return nullptr;
  }

  if (!isSynchronized())
    return nullptr;

  auto env_uptr = env_sync_->snapshot();
  if (!env_uptr)
  {
    return nullptr;
  }
  return std::shared_ptr<tesseract_environment::Environment>(std::move(env_uptr));
}

}  // namespace rlc_scene_bridge
