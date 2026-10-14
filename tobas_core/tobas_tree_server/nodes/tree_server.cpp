// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_node/node.hpp>

#include <tobas_kdl_msgs_adapter/rigid_body_inertia.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/srv/attach_load.hpp>
#include <tobas_msgs/srv/detach_load.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class TreeServerNode : public BaseNode
{
  using self = TreeServerNode;
  using super = BaseNode;
  using AttachSrv = tobas_msgs::srv::AttachLoad;
  using DetachSrv = tobas_msgs::srv::DetachLoad;

public:
  explicit TreeServerNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  kdl::Tree tree_;
  kdl::TreeParser tree_parser_;
  std::unordered_set<std::string> load_ids_;

  ros2::PublisherPtr<kdl::Tree> tree_pub_;
  ros2::TimerPtr initial_publish_timer_;
  ros2::ServiceServerPtr<AttachSrv> attach_srv_;
  ros2::ServiceServerPtr<DetachSrv> detach_srv_;

  static std::string loadSegmentName(const std::string& _id);
  static kdl::RigidBodyInertia parseLoad(const AttachSrv::Request& _req);

  void publishTree();
  void initialPublishTimerCb();

  void attachCb(const AttachSrv::Request::ConstSharedPtr& _req, const AttachSrv::Response::SharedPtr& _res);
  void detachCb(const DetachSrv::Request::ConstSharedPtr& _req, const DetachSrv::Response::SharedPtr& _res);
};

TreeServerNode::TreeServerNode(const rclcpp::NodeOptions& _options)
  : super("tree_server", nodeOptions_Default(_options))
{
  // Load a KDL tree.
  const auto robot_description = getStringParam("robot_description");
  const auto tree = tree_parser_.parseFromText(robot_description);
  if (!tree) {
    TOBAS_ERROR("Failed to parse robot description: ", tree.error());
    return;
  }
  tree_ = std::move(*tree);

  // Validate the tree.
  if (const auto result = tree_.validate(); !result) {
    TOBAS_ERROR("KDL tree is invalid: ", result.error());
    return;
  }

  // Register ROS interfaces.
  tree_pub_ = createPublisher<kdl::Tree>(topic::kKdlTree, true, true);
  attach_srv_ = createService<AttachSrv>(service::kAttachLoad, &self::attachCb, this);
  detach_srv_ = createService<DetachSrv>(service::kDetachLoad, &self::detachCb, this);

  // Defer the initial publication until the executor starts processing callbacks.
  initial_publish_timer_ = createTimer(0s, &self::initialPublishTimerCb, this);
}

void TreeServerNode::initialPublishTimerCb()
{
  initial_publish_timer_->cancel();
  publishTree();
}

std::string TreeServerNode::loadSegmentName(const std::string& _id)
{
  return "attached_load/" + _id;
}

kdl::RigidBodyInertia TreeServerNode::parseLoad(const AttachSrv::Request& _req)
{
  kdl::Vector cog;
  tobas_kdl_msgs::VectorAdapter::convert_to_custom(_req.inertia.cog, cog);

  kdl::RotationalInertia i_cog;
  tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_custom(_req.inertia.i_cog, i_cog);

  return kdl::RigidBodyInertia(_req.inertia.mass, cog, i_cog);
}

void TreeServerNode::publishTree()
{
  auto tree_msg = std::make_unique<kdl::Tree>(tree_);
  tree_pub_->publish(std::move(tree_msg));
}

void TreeServerNode::attachCb(const AttachSrv::Request::ConstSharedPtr& _req, const AttachSrv::Response::SharedPtr& _res)
{
  _res->success = false;

  if (tree_.empty()) {
    _res->message = "Robot description has not been initialized.";
    return;
  }
  if (_req->load_id.empty()) {
    _res->message = "Load ID must not be empty.";
    return;
  }
  if (load_ids_.contains(_req->load_id)) {
    _res->message = "Load '" + _req->load_id + "' is already attached.";
    return;
  }

  const auto segment_name = loadSegmentName(_req->load_id);

  kdl::Joint joint;
  joint.name = segment_name + "/joint";
  joint.type = kdl::Joint::kFixed;

  const auto inertia = parseLoad(*_req);
  if (const auto result = inertia.validate(); !result) {
    _res->message = result.error();
    return;
  }

  const kdl::Segment segment(segment_name, joint, kdl::Frame::Identity(), inertia);
  if (!tree_.addSegment(segment, _req->parent_link)) {
    _res->message = "Failed to add load '" + _req->load_id + "'.";
    return;
  }

  load_ids_.insert(_req->load_id);
  publishTree();

  _res->success = true;
  _res->message.clear();
}

void TreeServerNode::detachCb(const DetachSrv::Request::ConstSharedPtr& _req, const DetachSrv::Response::SharedPtr& _res)
{
  _res->success = false;

  if (tree_.empty()) {
    _res->message = "Robot description has not been initialized.";
    return;
  }
  if (!load_ids_.contains(_req->load_id)) {
    _res->message = "Load '" + _req->load_id + "' is not attached.";
    return;
  }

  const auto segment_name = loadSegmentName(_req->load_id);
  if (!tree_.removeSegment(segment_name)) {
    _res->message = "Failed to remove load '" + _req->load_id + "'.";
    return;
  }

  load_ids_.erase(_req->load_id);
  publishTree();

  _res->success = true;
  _res->message.clear();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::TreeServerNode)
