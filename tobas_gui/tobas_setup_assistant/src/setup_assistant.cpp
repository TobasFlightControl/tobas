#include <tobas_qt_tools/util.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace gui
{
namespace setup_assistant
{
SetupAssistantWidget::SetupAssistantWidget(
  rclcpp::Node::SharedPtr node,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if)
  : robot_(node)
{
  // 他のクラスにポインタを渡す際は必ずメモリ確保してから！
  // さもないと確保時にメモリ配置が変わってセグフォになる
  settings_ = new SettingsWidget(node, robot_);
  start_ = new StartWidget(node, robot_, settings_);
  rviz_ = new RvizWidget(rviz_node_if, robot_);
  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  jsp_ = new JointStatePublisherWidget(node, robot_);

  frame_tree_->setFixedWidth(kFrameTreeWidth);
  rviz_->setMinimumWidth(kRvizMinWidth);
  jsp_->setMinimumWidth(kJointStatePublisherMinWidth);

  pkg_generator_ = std::make_unique<PackageGenerator>(node, robot_, settings_);

  // Layout
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(start_);
  const auto cols = qt::createFixedHeightQHBoxLayout(kHeaderHeight, rows);
  rows->addLayout(cols);
  cols->addWidget(frame_tree_);
  cols->addWidget(rviz_);
  cols->addWidget(jsp_);
  rows->addWidget(settings_);

  // Connections
  connect(settings_->ros_package, &ROSPackageWidget::generateButtonClicked, this, &self::onGenerateButtonClicked);
}

void SetupAssistantWidget::onGenerateButtonClicked()
{
  pkg_generator_->generate();
}
}  // namespace setup_assistant
}  // namespace gui
