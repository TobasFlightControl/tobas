#pragma once

#include "./common.hpp"
#include "./robot_info.hpp"
#include "./package_generator.hpp"
#include "./start/start.hpp"
#include "./frame_tree.hpp"
#include "./rviz.hpp"
#include "./joint_state_publisher.hpp"
#include "./settings.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistantWidget : public QWidget
{
  Q_OBJECT

  using self = SetupAssistantWidget;
  using super = QWidget;

  static constexpr int kHeaderHeight = 350;
  static constexpr int kFrameTreeWidth = 200;
  static constexpr int kRvizMinWidth = 200;
  static constexpr int kJointStatePublisherMinWidth = 300;

public:
  explicit SetupAssistantWidget(
    rclcpp::Node::SharedPtr node,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if);

private Q_SLOTS:
  void onGenerateButtonClicked();

private:
  RobotInfo robot_;
  std::unique_ptr<PackageGenerator> pkg_generator_;

  StartWidget* start_;
  FrameTreeWidget* frame_tree_;
  RvizWidget* rviz_;
  JointStatePublisherWidget* jsp_;
  SettingsWidget* settings_;
};
}  // namespace setup_assistant
}  // namespace gui
