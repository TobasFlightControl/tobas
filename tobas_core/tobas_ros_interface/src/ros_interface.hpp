// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp_action/create_client.hpp>

#include <tobas_constants/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/util.hpp>

namespace tobas
{
struct TopicBase
{
  using SharedPtr = std::shared_ptr<TopicBase>;
};

template <typename MsgType>
struct Topic : public TopicBase
{
  using SharedPtr = std::shared_ptr<Topic<MsgType>>;

  typename rclcpp::Publisher<MsgType>::SharedPtr publisher;
  typename rclcpp::Subscription<MsgType>::SharedPtr subscriber;
};

struct ServiceBase
{
  using SharedPtr = std::shared_ptr<ServiceBase>;

  std::string name;
};

template <typename SrvType>
struct Service : public ServiceBase
{
  using SharedPtr = std::shared_ptr<Service<SrvType>>;

  typename rclcpp::Service<SrvType>::SharedPtr server;
  typename rclcpp::Client<SrvType>::SharedPtr client;
};

struct ActionBase
{
  using SharedPtr = std::shared_ptr<ActionBase>;

  std::string name;
};

template <typename ActType>
struct Action : public ActionBase
{
  using SharedPtr = std::shared_ptr<Action<ActType>>;

  typename rclcpp_action::Server<ActType>::SharedPtr server;
  typename rclcpp_action::Client<ActType>::SharedPtr client;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>> server_gh;
  typename rclcpp_action::ClientGoalHandle<ActType>::SharedPtr client_gh;
};

class RosInterfaceNode : public BaseNode
{
  using super = BaseNode;

public:
  explicit RosInterfaceNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  rclcpp::CallbackGroup::SharedPtr group_;

  std::vector<TopicBase::SharedPtr> topics_;
  std::vector<ServiceBase::SharedPtr> services_;
  std::vector<ActionBase::SharedPtr> actions_;

  void registerTopicsLogicToIface();
  void registerTopicsIfaceToLogic();
  void registerServices();
  void registerActions();

  template <typename MsgType>
  void
  addTopic(const std::string& _sub_topic, const std::string& _pub_topic, bool _latch, bool _reliable, size_t _queue_size);

  template <typename MsgType>
  void addTopicLogicToIface(
    const std::string& _sub_topic,
    const std::string& _pub_topic,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void addTopicIfaceToLogic(
    const std::string& _sub_topic,
    const std::string& _pub_topic,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType>
  void addService(const std::string& _service_name);

  template <typename ActType>
  void addAction(const std::string& _action_name);

  template <typename MsgType>
  void topicCallback(typename Topic<MsgType>::SharedPtr _topic, const typename MsgType::ConstSharedPtr& _msg);

  template <typename SrvType>
  void serviceCallback(
    typename Service<SrvType>::SharedPtr _service,
    const typename SrvType::Request::SharedPtr& _req,
    const typename SrvType::Response::SharedPtr& _res);

  template <typename ActType>
  void actionFeedbackCallback(
    typename Action<ActType>::SharedPtr _action,
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
};

template <typename MsgType>
void RosInterfaceNode::addTopic(
  const std::string& _sub_topic,
  const std::string& _pub_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  const auto topic = std::make_shared<Topic<MsgType>>();

  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);

  topic->publisher = create_publisher<MsgType>(_pub_topic, qos);

  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& _msg) { topicCallback<MsgType>(topic, _msg); };
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;
  topic->subscriber = create_subscription<MsgType>(_sub_topic, qos, cb, opts);

  topics_.push_back(topic);
}

template <typename MsgType>
void RosInterfaceNode::addTopicLogicToIface(
  const std::string& _sub_topic,
  const std::string& _pub_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  addTopic<MsgType>(_sub_topic, addIfaceNS(_pub_topic), _latch, _reliable, _queue_size);
}

template <typename MsgType>
void RosInterfaceNode::addTopicIfaceToLogic(
  const std::string& _sub_topic,
  const std::string& _pub_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  addTopic<MsgType>(addIfaceNS(_sub_topic), _pub_topic, _latch, _reliable, _queue_size);
}

template <typename SrvType>
void RosInterfaceNode::addService(const std::string& _service_name)
{
  const auto service = std::make_shared<Service<SrvType>>();
  service->name = _service_name;

  const auto cb =
    [this, service](const typename SrvType::Request::SharedPtr& _req, const typename SrvType::Response::SharedPtr& _res)
  { serviceCallback<SrvType>(service, _req, _res); };
  const auto qos = rclcpp::ServicesQoS();
  service->server = create_service<SrvType>(addIfaceNS(_service_name), cb, qos, group_);

  service->client = create_client<SrvType>(_service_name);

  services_.push_back(service);
}

template <typename ActType>
void RosInterfaceNode::addAction(const std::string& _action_name)
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
    group_);

  action->client = rclcpp_action::create_client<ActType>(this, _action_name, group_);

  actions_.push_back(action);
}

template <typename MsgType>
void RosInterfaceNode::topicCallback(
  typename Topic<MsgType>::SharedPtr _topic,
  const typename MsgType::ConstSharedPtr& _msg)
{
  // Subscribers are expected to be out of process, so publish without creating a unique_ptr or copying.
  _topic->publisher->publish(*_msg);
}

template <typename SrvType>
void RosInterfaceNode::serviceCallback(
  typename Service<SrvType>::SharedPtr _service,
  const typename SrvType::Request::SharedPtr& _req,
  const typename SrvType::Response::SharedPtr& _res)
{
  if (!_service->client->service_is_ready()) {
    TOBAS_ERROR("'", _service->name, "' service is not ready.");
    return;
  }

  auto future =
    _service->client->async_send_request(_req);  // `async_send_request` requires a non-const request pointer.
  future.wait();

  *_res = *future.get();  // `future` must be non-const here.
}

template <typename ActType>
void RosInterfaceNode::actionFeedbackCallback(
  typename Action<ActType>::SharedPtr _action,
  const typename ActType::Feedback::ConstSharedPtr& _fb_in)
{
  if (!_action->server_gh) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Waiting for '", _action->name, "' action server goal handle.");
    return;
  }

  const auto fb_out = std::make_shared<typename ActType::Feedback>(*_fb_in);  // Remove constness.
  _action->server_gh->publish_feedback(fb_out);
}

template <typename ActType>
rclcpp_action::GoalResponse RosInterfaceNode::actionHandleGoal(
  typename Action<ActType>::SharedPtr _action,
  const rclcpp_action::GoalUUID&,
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
    [this, _action](const typename GoalHandle::SharedPtr&, const typename ActType::Feedback::ConstSharedPtr& _fb)
  { actionFeedbackCallback<ActType>(_action, _fb); };

  const auto future = _action->client->async_send_goal(*_goal, opts);
  future.wait();

  _action->client_gh = future.get();
  if (_action->client_gh) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // -> `actionHandleAccepted` is called.
  }
  else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

template <typename ActType>
rclcpp_action::CancelResponse RosInterfaceNode::actionHandleCancel(
  typename Action<ActType>::SharedPtr _action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&)
{
  const auto future = _action->client->async_cancel_goal(_action->client_gh);
  future.wait();

  const auto res = future.get();
  if (res->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  else {
    return rclcpp_action::CancelResponse::REJECT;
  }
}

template <typename ActType>
void RosInterfaceNode::actionHandleAccepted(
  typename Action<ActType>::SharedPtr _action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& _server_gh)
{
  // Store `ServerGoalHandle` so feedback can be published from another method.
  _action->server_gh = _server_gh;

  const auto future = _action->client->async_get_result(_action->client_gh);
  future.wait();

  const auto res = future.get();
  switch (res.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      _server_gh->succeed(res.result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      _server_gh->canceled(res.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      _server_gh->abort(res.result);
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      TOBAS_ERROR("Invalid action result code: ", static_cast<int>(res.code));
      _server_gh->abort(res.result);
      break;
  }

  _action->server_gh.reset();
  _action->client_gh.reset();
}
}  // namespace tobas
