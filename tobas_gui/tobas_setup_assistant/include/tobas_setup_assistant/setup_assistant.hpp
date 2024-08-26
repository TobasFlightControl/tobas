#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <tobas_qt_tools/widgets/widget.hpp>
#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./tree_information.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant : public qt::Widget
{
  Q_OBJECT

  using super = qt::Widget;

  static constexpr int kTabHeight = 30;  // 30以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;
  static constexpr int kSettingsMinHeight = 300;

public:
  explicit SetupAssistant();

  const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& rvizRosNode();
  const rclcpp::Node::SharedPtr& node();

  const TreeInformation& treeInfo();

  // TODO

private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::Node::SharedPtr node_;

  kdl::Tree tree_;
  TreeInformation tree_info_;
};
}  // namespace setup_assistant
}  // namespace gui
