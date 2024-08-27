#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
SetupAssistant::SetupAssistant(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
  : rviz_ros_node_(rviz_ros_node), node_(rviz_ros_node.lock()->get_raw_node()), tree_info_(tree_)
{
  auto rows = new QVBoxLayout();
  setLayout(rows);

  auto cols = qt::createFixedHeightQHBoxLayout(kHeaderHeight, rows);
  rows->addLayout(cols);

  frame_tree_ = new FrameTreeWidget(this);
  frame_tree_->setFixedWidth(kFrameTreeWidth);
  cols->addWidget(frame_tree_);

  rviz_ = new RvizWidget(this);
  rviz_->setMinimumWidth(kRvizMinWidth);
  cols->addWidget(rviz_);

  js_pub_ = new JointStatePublisherWidget(this);
  js_pub_->setFixedWidth(kJointStatePublisherWidth);
  cols->addWidget(js_pub_);

  // TODO: Tabs
}

const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& SetupAssistant::rvizRosNode()
{
  return rviz_ros_node_;
}

const rclcpp::Node::SharedPtr& SetupAssistant::node()
{
  return node_;
}

const kdl::Tree& SetupAssistant::tree()
{
  return tree_;
}

const TreeInformation& SetupAssistant::treeInfo()
{
  return tree_info_;
}

FrameTreeWidget* SetupAssistant::frameTree()
{
  return frame_tree_;
}

RvizWidget* SetupAssistant::rviz()
{
  return rviz_;
}

JointStatePublisherWidget* SetupAssistant::jointStatePublisher()
{
  return js_pub_;
}
}  // namespace setup_assistant
}  // namespace gui
