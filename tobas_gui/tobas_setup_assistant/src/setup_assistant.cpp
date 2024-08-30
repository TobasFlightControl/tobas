#include <format>
#include <hardware_interface/component_parser.hpp>

#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
SetupAssistant::SetupAssistant(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
  : rviz_ros_node_(rviz_ros_node), node_(rviz_ros_node.lock()->get_raw_node())
{
  auto rows = new QVBoxLayout(this);

  // Start
  start_ = new StartWidget(node_, robot_, settings_);
  rows->addWidget(start_);

  // Visualization
  auto cols = qt::createFixedHeightQHBoxLayout(kHeaderHeight, rows);
  rows->addLayout(cols);

  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  frame_tree_->setFixedWidth(kFrameTreeWidth);
  cols->addWidget(frame_tree_);

  rviz_ = new RvizWidget(rviz_ros_node, robot_);
  rviz_->setMinimumWidth(kRvizMinWidth);
  cols->addWidget(rviz_);

  js_pub_ = new JointStatePublisherWidget(node_, robot_);
  js_pub_->setFixedWidth(kJointStatePublisherWidth);
  cols->addWidget(js_pub_);

  // Settings
  settings_ = new SettingsWidget();
  rows->addWidget(settings_);
}
}  // namespace setup_assistant
}  // namespace gui
