#pragma once

#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "./common.hpp"
#include "./robot_info.hpp"
#include "./package_generator.hpp"
#include "./build_package_thread.hpp"
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

private:
  RobotInfo robot_;
  std::unique_ptr<PackageGenerator> pkg_generator_;

  ros2::SyncParamClient rsp_client_;

  qt::WaitSpinnerWidget spinner_;
  BuildPackageThread build_thread_;

  StartWidget* start_;
  FrameTreeWidget* frame_tree_;
  RvizWidget* rviz_;
  JointStatePublisherWidget* jsp_;
  SettingsWidget* settings_;

private Q_SLOTS:
  void onRobotLoaded();
  void onGenerateButtonClicked();
  void onBuildPackageFinished(bool success, const QString& output);
};
}  // namespace setup_assistant
}  // namespace gui
