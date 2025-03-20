#pragma once

#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "./constants.hpp"
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
namespace sa
{
class SetupAssistantWidget : public QWidget
{
  Q_OBJECT

  using self = SetupAssistantWidget;
  using super = QWidget;

public:
  explicit SetupAssistantWidget(rclcpp::Node::SharedPtr node);

  void reset();

private:
  RobotInfo robot_;
  Signals signals_;
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
}  // namespace sa
}  // namespace gui
