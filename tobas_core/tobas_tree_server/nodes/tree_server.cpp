// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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

namespace tobas
{
class TreeServerNode : public BaseNode
{
  using self = TreeServerNode;
  using super = BaseNode;
  using AttachSrv = tobas_msgs::srv::AttachLoad;
  using DetachSrv = tobas_msgs::srv::DetachLoad;

public:
  explicit TreeServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  kdl::Tree tree_;
  kdl::TreeParser tree_parser_;
  std::unordered_set<std::string> load_ids_;

  ros2::PublisherPtr<kdl::Tree> tree_pub_;
  ros2::ServiceServerPtr<AttachSrv> attach_srv_;
  ros2::ServiceServerPtr<DetachSrv> detach_srv_;

  static std::string loadSegmentName(const std::string& id);
  static kdl::RigidBodyInertia parseLoad(const AttachSrv::Request& req);

  void publishTree();

  void attachCb(const AttachSrv::Request::ConstSharedPtr& req, const AttachSrv::Response::SharedPtr& res);
  void detachCb(const DetachSrv::Request::ConstSharedPtr& req, const DetachSrv::Response::SharedPtr& res);
};

TreeServerNode::TreeServerNode(const rclcpp::NodeOptions& options) : super("tree_server", nodeOptions_Default(options))
{
  const auto robot_description = getStringParam("robot_description");
  if (!tree_parser_.parseFromText(robot_description, tree_)) {
    TOBAS_ERROR("Failed to parse robot description: ", tree_parser_.errorMessage());
    return;
  }

  std::string error_msg;
  if (!tree_.isValid(error_msg)) {
    TOBAS_ERROR("KDL tree is invalid: ", error_msg);
    return;
  }

  tree_pub_ = createPublisher<kdl::Tree>(topic::kKdlTree, true, true);

  attach_srv_ = createService<AttachSrv>(service::kAttachLoad, &self::attachCb, this);
  detach_srv_ = createService<DetachSrv>(service::kDetachLoad, &self::detachCb, this);

  publishTree();
}

std::string TreeServerNode::loadSegmentName(const std::string& id)
{
  return "attached_load/" + id;
}

kdl::RigidBodyInertia TreeServerNode::parseLoad(const AttachSrv::Request& req)
{
  kdl::Vector cog;
  tobas_kdl_msgs::VectorAdapter::convert_to_custom(req.inertia.cog, cog);

  kdl::RotationalInertia i_cog;
  tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_custom(req.inertia.i_cog, i_cog);

  return kdl::RigidBodyInertia(req.inertia.mass, cog, i_cog);
}

void TreeServerNode::publishTree()
{
  auto tree_msg = std::make_unique<kdl::Tree>(tree_);
  tree_pub_->publish(std::move(tree_msg));
}

void TreeServerNode::attachCb(const AttachSrv::Request::ConstSharedPtr& req, const AttachSrv::Response::SharedPtr& res)
{
  res->success = false;

  if (tree_.empty()) {
    res->message = "Robot description has not been initialized.";
    return;
  }
  if (req->load_id.empty()) {
    res->message = "Load ID must not be empty.";
    return;
  }
  if (load_ids_.contains(req->load_id)) {
    res->message = "Load '" + req->load_id + "' is already attached.";
    return;
  }

  const auto segment_name = loadSegmentName(req->load_id);

  kdl::Joint joint;
  joint.name = segment_name + "/joint";
  joint.type = kdl::Joint::kFixed;

  const auto inertia = parseLoad(*req);
  if (!inertia.isValid(res->message)) {
    return;
  }

  const kdl::Segment segment(segment_name, joint, kdl::Frame::Identity(), inertia);
  if (!tree_.addSegment(segment, req->parent_link)) {
    res->message = "Failed to add load '" + req->load_id + "'.";
    return;
  }

  load_ids_.insert(req->load_id);
  publishTree();

  res->success = true;
  res->message.clear();
}

void TreeServerNode::detachCb(const DetachSrv::Request::ConstSharedPtr& req, const DetachSrv::Response::SharedPtr& res)
{
  res->success = false;

  if (tree_.empty()) {
    res->message = "Robot description has not been initialized.";
    return;
  }
  if (!load_ids_.contains(req->load_id)) {
    res->message = "Load '" + req->load_id + "' is not attached.";
    return;
  }

  const auto segment_name = loadSegmentName(req->load_id);
  if (!tree_.removeSegment(segment_name)) {
    res->message = "Failed to remove load '" + req->load_id + "'.";
    return;
  }

  load_ids_.erase(req->load_id);
  publishTree();

  res->success = true;
  res->message.clear();
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::TreeServerNode)
