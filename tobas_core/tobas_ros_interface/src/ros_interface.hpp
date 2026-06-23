// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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
  using self = RosInterfaceNode;
  using super = BaseNode;

public:
  explicit RosInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
  addTopic(const std::string& sub_topic, const std::string& pub_topic, bool latch, bool reliable, size_t queue_size);

  template <typename MsgType>
  void addTopicLogicToIface(
    const std::string& sub_topic,
    const std::string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void addTopicIfaceToLogic(
    const std::string& sub_topic,
    const std::string& pub_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename SrvType>
  void addService(const std::string& service_name);

  template <typename ActType>
  void addAction(const std::string& action_name);

  template <typename MsgType>
  void topicCallback(typename Topic<MsgType>::SharedPtr topic, const typename MsgType::ConstSharedPtr& msg);

  template <typename SrvType>
  void serviceCallback(
    typename Service<SrvType>::SharedPtr service,
    const typename SrvType::Request::SharedPtr& req,
    const typename SrvType::Response::SharedPtr& res);

  template <typename ActType>
  void actionFeedbackCallback(
    typename Action<ActType>::SharedPtr action,
    const typename ActType::Feedback::ConstSharedPtr& fb);

  template <typename ActType>
  rclcpp_action::GoalResponse actionHandleGoal(
    typename Action<ActType>::SharedPtr action,
    const rclcpp_action::GoalUUID& uuid,
    const typename ActType::Goal::ConstSharedPtr& goal);

  template <typename ActType>
  rclcpp_action::CancelResponse actionHandleCancel(
    typename Action<ActType>::SharedPtr action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& gh);

  template <typename ActType>
  void actionHandleAccepted(
    typename Action<ActType>::SharedPtr action,
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& gh);
};

template <typename MsgType>
void RosInterfaceNode::addTopic(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto topic = std::make_shared<Topic<MsgType>>();

  const ros2::qos::QoS qos(latch, reliable, queue_size);

  topic->publisher = create_publisher<MsgType>(pub_topic, qos);

  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { topicCallback<MsgType>(topic, msg); };
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;
  topic->subscriber = create_subscription<MsgType>(sub_topic, qos, cb, opts);

  topics_.push_back(topic);
}

template <typename MsgType>
void RosInterfaceNode::addTopicLogicToIface(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(sub_topic, addIfaceNS(pub_topic), latch, reliable, queue_size);
}

template <typename MsgType>
void RosInterfaceNode::addTopicIfaceToLogic(
  const std::string& sub_topic,
  const std::string& pub_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  addTopic<MsgType>(addIfaceNS(sub_topic), pub_topic, latch, reliable, queue_size);
}

template <typename SrvType>
void RosInterfaceNode::addService(const std::string& service_name)
{
  const auto service = std::make_shared<Service<SrvType>>();
  service->name = service_name;

  const auto cb =
    [this, service](const typename SrvType::Request::SharedPtr& req, const typename SrvType::Response::SharedPtr& res)
  { serviceCallback<SrvType>(service, req, res); };
  const auto qos = rclcpp::ServicesQoS();
  service->server = create_service<SrvType>(addIfaceNS(service_name), cb, qos, group_);

  service->client = create_client<SrvType>(service_name);

  services_.push_back(service);
}

template <typename ActType>
void RosInterfaceNode::addAction(const std::string& action_name)
{
  using GoalPtr = typename ActType::Goal::ConstSharedPtr;
  using GoalHandlePtr = std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>;

  const auto action = std::make_shared<Action<ActType>>();
  action->name = action_name;

  const auto handle_goal = [this, action](const rclcpp_action::GoalUUID& uuid, const GoalPtr& goal)
  { return actionHandleGoal<ActType>(action, uuid, goal); };
  const auto handle_cancel = [this, action](const GoalHandlePtr& gh)
  { return actionHandleCancel<ActType>(action, gh); };
  const auto handle_accepted = [this, action](const GoalHandlePtr& gh) { actionHandleAccepted<ActType>(action, gh); };
  action->server = rclcpp_action::create_server<ActType>(
    this,
    addIfaceNS(action_name),
    handle_goal,
    handle_cancel,
    handle_accepted,
    rcl_action_server_get_default_options(),
    group_);

  action->client = rclcpp_action::create_client<ActType>(this, action_name, group_);

  actions_.push_back(action);
}

template <typename MsgType>
void RosInterfaceNode::topicCallback(
  typename Topic<MsgType>::SharedPtr topic,
  const typename MsgType::ConstSharedPtr& msg)
{
  // 購読者はプロセス外のみの想定なので unique_ptr は作らずコピー無しで発行する
  topic->publisher->publish(*msg);
}

template <typename SrvType>
void RosInterfaceNode::serviceCallback(
  typename Service<SrvType>::SharedPtr service,
  const typename SrvType::Request::SharedPtr& req,
  const typename SrvType::Response::SharedPtr& res)
{
  if (!service->client->service_is_ready()) {
    TOBAS_ERROR("\"", service->name, "\" service is not ready.");
    return;
  }

  auto future = service->client->async_send_request(req);  // req が ConstSharedPtr だとここでコケる
  future.wait();

  *res = *future.get();  // future が const だとここでコケる
}

template <typename ActType>
void RosInterfaceNode::actionFeedbackCallback(
  typename Action<ActType>::SharedPtr action,
  const typename ActType::Feedback::ConstSharedPtr& fb_in)
{
  if (!action->server_gh) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Waiting for \"", action->name, "\" action server goal handle.");
    return;
  }

  const auto fb_out = std::make_shared<typename ActType::Feedback>(*fb_in);  // const を外す
  action->server_gh->publish_feedback(fb_out);
}

template <typename ActType>
rclcpp_action::GoalResponse RosInterfaceNode::actionHandleGoal(
  typename Action<ActType>::SharedPtr action,
  const rclcpp_action::GoalUUID&,
  const typename ActType::Goal::ConstSharedPtr& goal)
{
  using Client = rclcpp_action::Client<ActType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActType>;

  if (!action->client->action_server_is_ready()) {
    TOBAS_WARN("\"", action->name, "\" action is not ready.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  typename Client::SendGoalOptions opts;
  opts.feedback_callback =
    [this, action](const typename GoalHandle::SharedPtr&, const typename ActType::Feedback::ConstSharedPtr& fb)
  { actionFeedbackCallback<ActType>(action, fb); };

  const auto future = action->client->async_send_goal(*goal, opts);
  future.wait();

  action->client_gh = future.get();
  if (action->client_gh) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // -> actionHandleAccepted が実行される
  }
  else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

template <typename ActType>
rclcpp_action::CancelResponse RosInterfaceNode::actionHandleCancel(
  typename Action<ActType>::SharedPtr action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>&)
{
  const auto future = action->client->async_cancel_goal(action->client_gh);
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
  typename Action<ActType>::SharedPtr action,
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActType>>& server_gh)
{
  // 別メソッドでフィードバックを発行するために ServerGoalHandle を保存
  action->server_gh = server_gh;

  const auto future = action->client->async_get_result(action->client_gh);
  future.wait();

  const auto res = future.get();
  switch (res.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      server_gh->succeed(res.result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      server_gh->canceled(res.result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      server_gh->abort(res.result);
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      TOBAS_ERROR("Invalid action result code: ", static_cast<int>(res.code));
      server_gh->abort(res.result);
      break;
  }

  action->server_gh.reset();
  action->client_gh.reset();
}
}  // namespace tobas
