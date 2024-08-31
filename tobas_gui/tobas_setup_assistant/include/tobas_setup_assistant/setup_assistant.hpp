#pragma once

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <tobas_qt_tools/widgets/widget.hpp>

#include "./common.hpp"
#include "./robot_info.hpp"
#include "./start/start.hpp"
#include "./frame_tree.hpp"
#include "./rviz.hpp"
#include "./joint_state_publisher.hpp"
#include "./settings.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant : public qt::Widget
{
  Q_OBJECT

  using self = SetupAssistant;
  using super = qt::Widget;

  static constexpr int kHeaderHeight = 350;
  static constexpr int kFrameTreeWidth = 200;
  static constexpr int kRvizMinWidth = 200;
  static constexpr int kJointStatePublisherWidth = 200;

public:
  explicit SetupAssistant(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  const rclcpp::Node::SharedPtr node_;

  RobotInfo robot_;

  StartWidget* start_;
  FrameTreeWidget* frame_tree_;
  RvizWidget* rviz_;
  JointStatePublisherWidget* js_pub_;
  SettingsWidget* settings_;
};
}  // namespace setup_assistant
}  // namespace gui
