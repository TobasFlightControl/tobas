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
  // 他のクラスにポインタを渡す際は必ずメモリ確保してから！
  // さもないと確保時にメモリ配置が変わってセグフォになる危険性がある
  settings_ = new SettingsWidget(node_, robot_);
  start_ = new StartWidget(node_, robot_, settings_);
  rviz_ = new RvizWidget(rviz_node_if, robot_);
  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  js_pub_ = new JointStatePublisherWidget(node_, robot_);

  frame_tree_->setFixedWidth(kFrameTreeWidth);
  rviz_->setMinimumWidth(kRvizMinWidth);
  js_pub_->setFixedWidth(kJointStatePublisherWidth);

  // Layout
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(start_);
  const auto cols = qt::createFixedHeightQHBoxLayout(kHeaderHeight, rows);
  rows->addLayout(cols);
  cols->addWidget(frame_tree_);
  cols->addWidget(rviz_);
  cols->addWidget(js_pub_);
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
