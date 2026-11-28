// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/exceptions.hpp>

#include <tobas_constants/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>

namespace tobas
{
namespace
{
struct ActionBase
{
  using SharedPtr = std::shared_ptr<ActionBase>;

  std::string name;
  std::mutex mutex;
};

template <typename ActType>
struct Action : public ActionBase
{
  using SharedPtr = std::shared_ptr<Action<ActType>>;

  struct Goal
  {
    typename rclcpp_action::ClientGoalHandle<ActType>::SharedPtr client_gh;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>> server_gh;
    rclcpp::TimerBase::SharedPtr result_timer;
  };

  typename rclcpp_action::Server<ActType>::SharedPtr server;
  typename rclcpp_action::Client<ActType>::SharedPtr client;

  std::map<rclcpp_action::GoalUUID, Goal> goals;
};

class ActionInterfaceNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit ActionInterfaceNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr server_group_;
  std::vector<ActionBase::SharedPtr> actions_;

  template <typename ActType>
  void addAction(const std::string& _action_name);

  template <typename ActType>
  void actionFeedbackCallback(
    typename Action<ActType>::SharedPtr _action,
    const rclcpp_action::GoalUUID& _uuid,
    const typename ActType::Feedback::ConstSharedPtr& _fb);

  template <typename ActType>
  rclcpp_action::GoalResponse actionHandleGoal(
    typename Action<ActType>::SharedPtr _action,
    const rclcpp_action::GoalUUID& _uuid,
    const typename ActType::Goal::ConstSharedPtr& _goal);

  template <typename ActType>
  rclcpp_action::CancelResponse actionHandleCancel(
    typename Action<ActType>::SharedPtr _action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _gh);

  template <typename ActType>
  void actionHandleAccepted(
    typename Action<ActType>::SharedPtr _action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _gh);

  template <typename ActType>
  void actionResultCallback(
    typename Action<ActType>::SharedPtr _action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _gh,
    const typename rclcpp_action::ClientGoalHandle<ActType>::WrappedResult& _res);
};

ActionInterfaceNode::ActionInterfaceNode(const rclcpp::NodeOptions& _options)
  : super("action_interface", nodeOptions_Default(_options))
{
  // Goal/cancel callbacks wait for client responses, which run in the separate default group.
  server_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  addAction<tobas_mission_msgs::action::ExecuteMission>(action::kExecuteMission);
}

template <typename ActType>
void ActionInterfaceNode::addAction(const std::string& _action_name)
{
  using GoalPtr = typename ActType::Goal::ConstSharedPtr;
  using GoalHandlePtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>;

  const auto action = std::make_shared<Action<ActType>>();
  action->name = _action_name;

  const auto handle_goal = [this, action](const rclcpp_action::GoalUUID& _uuid, const GoalPtr& _goal)
  { return actionHandleGoal<ActType>(action, _uuid, _goal); };
  const auto handle_cancel = [this, action](const GoalHandlePtr& _gh)
  { return actionHandleCancel<ActType>(action, _gh); };
  const auto handle_accepted = [this, action](const GoalHandlePtr& _gh) { actionHandleAccepted<ActType>(action, _gh); };
  action->server = rclcpp_action::create_server<ActType>(
    this,
    addIfaceNS(_action_name),
    handle_goal,
    handle_cancel,
    handle_accepted,
    rcl_action_server_get_default_options(),
    server_group_);

  action->client = rclcpp_action::create_client<ActType>(this, _action_name);

  actions_.push_back(action);
}

template <typename ActType>
void ActionInterfaceNode::actionFeedbackCallback(
  typename Action<ActType>::SharedPtr _action,
  const rclcpp_action::GoalUUID& _uuid,
  const typename ActType::Feedback::ConstSharedPtr& _fb_in)
{
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>> server_gh;
  {
    const std::lock_guard lock(_action->mutex);
    if (const auto it = _action->goals.find(_uuid); it != _action->goals.end()) {
      server_gh = it->second.server_gh;
    }
  }

  if (!server_gh) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Waiting for '", _action->name, "' action server goal handle.");
    return;
  }

  const auto fb_out = std::make_shared<typename ActType::Feedback>(*_fb_in);  // Remove constness.
  server_gh->publish_feedback(fb_out);
}

template <typename ActType>
rclcpp_action::GoalResponse ActionInterfaceNode::actionHandleGoal(
  typename Action<ActType>::SharedPtr _action,
  const rclcpp_action::GoalUUID& _uuid,
  const typename ActType::Goal::ConstSharedPtr& _goal)
{
  using Client = rclcpp_action::Client<ActType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;

  if (!_action->client->action_server_is_ready()) {
    TOBAS_WARN("'", _action->name, "' action is not ready.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  typename Client::SendGoalOptions opts;
  opts.feedback_callback =
    [this, _action, _uuid](const typename GoalHandle::SharedPtr&, const typename ActType::Feedback::ConstSharedPtr& _fb)
  { actionFeedbackCallback<ActType>(_action, _uuid, _fb); };

  const auto future = _action->client->async_send_goal(*_goal, opts);
  future.wait();

  const auto client_gh = future.get();
  if (!client_gh) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const std::lock_guard lock(_action->mutex);
  _action->goals.emplace(_uuid, typename Action<ActType>::Goal{ client_gh, nullptr, nullptr });
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActType>
rclcpp_action::CancelResponse ActionInterfaceNode::actionHandleCancel(
  typename Action<ActType>::SharedPtr _action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _server_gh)
{
  typename rclcpp_action::ClientGoalHandle<ActType>::SharedPtr client_gh;
  {
    const std::lock_guard lock(_action->mutex);
    const auto it = _action->goals.find(_server_gh->get_goal_id());
    if (it == _action->goals.end()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    client_gh = it->second.client_gh;
  }

  try {
    const auto future = _action->client->async_cancel_goal(client_gh);
    future.wait();

    const auto res = future.get();
    const auto is_canceling = std::ranges::any_of(
      res->goals_canceling, [&client_gh](const auto& _goal) { return _goal.goal_id.uuid == client_gh->get_goal_id(); });
    if (res->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE && is_canceling) {
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
    // The downstream goal may finish before its result is forwarded in the server group.
  }

  return rclcpp_action::CancelResponse::REJECT;
}

template <typename ActType>
void ActionInterfaceNode::actionHandleAccepted(
  typename Action<ActType>::SharedPtr _action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _server_gh)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;

  typename GoalHandle::SharedPtr client_gh;
  {
    const std::lock_guard lock(_action->mutex);
    auto& goal = _action->goals.at(_server_gh->get_goal_id());
    goal.server_gh = _server_gh;
    client_gh = goal.client_gh;
  }

  // Return immediately so the server can accept cancellations and subsequent goals.
  const auto result_cb = [this, _action, _server_gh](const typename GoalHandle::WrappedResult& _res)
  {
    const std::lock_guard lock(_action->mutex);
    auto& goal = _action->goals.at(_server_gh->get_goal_id());
    // Complete in the server group after any cancel callback has returned and its state transition has finished.
    goal.result_timer = create_wall_timer(
      std::chrono::nanoseconds(0),
      [this, _action, _server_gh, _res]() { actionResultCallback<ActType>(_action, _server_gh, _res); },
      server_group_);
  };
  _action->client->async_get_result(client_gh, result_cb);
}

template <typename ActType>
void ActionInterfaceNode::actionResultCallback(
  typename Action<ActType>::SharedPtr _action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _server_gh,
  const typename rclcpp_action::ClientGoalHandle<ActType>::WrappedResult& _res)
{
  {
    const std::lock_guard lock(_action->mutex);
    const auto it = _action->goals.find(_server_gh->get_goal_id());
    if (it == _action->goals.end()) {
      return;
    }
    it->second.result_timer->cancel();
    _action->goals.erase(it);
  }

  switch (_res.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      _server_gh->succeed(_res.result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      _server_gh->canceled(_res.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      _server_gh->abort(_res.result);
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      TOBAS_ERROR("Invalid action result code: ", static_cast<int>(_res.code));
      _server_gh->abort(_res.result);
      break;
  }
}
}  // namespace
}  // namespace tobas

int main(int _argc, char* _argv[])
{
  rclcpp::init(_argc, _argv);
  const auto node = std::make_shared<tobas::ActionInterfaceNode>();
  rclcpp::executors::EventsCBGExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
}
