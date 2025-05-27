#pragma once

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>

#include "./build_package_thread.hpp"
#include "./constants.hpp"
#include "./frame_tree.hpp"
#include "./joint_state_publisher.hpp"
#include "./package_generator.hpp"
#include "./rotor_marker_publisher.hpp"
#include "./rviz.hpp"
#include "./settings.hpp"
#include "./start/start.hpp"

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
  RotorMarkerPublisher rotor_marker_publisher_;

  std::unique_ptr<PackageGenerator> pkg_generator_;
  ros2::SyncParamClient rsp_client_;
  qt::WaitSpinnerWidget spinner_;
  BuildPackageThread build_thread_;

  SettingsWidget* settings_;
  StartWidget* start_;
  RvizWidget* rviz_;
  FrameTreeWidget* frame_tree_;
  JointStatePublisherWidget* jsp_;

private Q_SLOTS:
  void onRobotLoaded();
  void onGenerateButtonClicked();
  void onBuildPackageFinished(bool success, const QString& output);
};
}  // namespace sa
}  // namespace gui
