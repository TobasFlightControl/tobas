#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
SetupAssistantWidget::SetupAssistantWidget(
  rclcpp::Node::SharedPtr node,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if)
  : node_(node), rviz_node_if_(rviz_node_if), robot_(node_), pkg_generator_(node_, robot_, settings_)
{
  const auto rows = new QVBoxLayout(this);

  // Start
  start_ = new StartWidget(node_, robot_, settings_);
  rows->addWidget(start_);

  // Visualization
  const auto cols = qt::createFixedHeightQHBoxLayout(kHeaderHeight, rows);
  rows->addLayout(cols);

  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  frame_tree_->setFixedWidth(kFrameTreeWidth);
  cols->addWidget(frame_tree_);

  rviz_ = new RvizWidget(rviz_node_if, robot_);
  rviz_->setMinimumWidth(kRvizMinWidth);
  cols->addWidget(rviz_);

  js_pub_ = new JointStatePublisherWidget(node_, robot_);
  js_pub_->setFixedWidth(kJointStatePublisherWidth);
  cols->addWidget(js_pub_);

  // Settings
  settings_ = new SettingsWidget(node_, robot_);
  rows->addWidget(settings_);

  // Connections
  connect(settings_->ros_package, &ROSPackageWidget::generateButtonClicked, this, &self::onGenerateButtonClicked);
}

void SetupAssistantWidget::onGenerateButtonClicked()
{
  pkg_generator_.generate();
}
}  // namespace setup_assistant
}  // namespace gui
