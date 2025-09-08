#pragma once

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_gui_common/local_project_builder.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_gui_common/remote_project_builder.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include "./commanders/commanders.hpp"
#include "./dynamic_configuration/dynamic_configuration.hpp"
#include "./kill_gazebo.hpp"
#include "./simulation_settings/simulation_settings.hpp"

namespace gui
{
namespace sim
{
class SimulationWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

Q_SIGNALS:
  void started();
  void terminated();

public:
  explicit SimulationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void reset();
  bool updateProject(const std::filesystem::path& proj_path);

  bool isRunning() const;

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  const rclcpp::Node::SharedPtr node_;

  ssh::SSHClient ssh_client_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;
  cmn::ProjectPaths proj_paths_;
  cmn::LocalProjectBuilder local_proj_builder_;
  cmn::RemoteProjectBuilder remote_proj_builder_;

  uadf::Model uadf_;
  kdl::Tree tree_;
  tobas::Drone drone_;
  pid_t launch_pid_ = -1;

  qt::ToggleButton* start_stop_button_;

  SimulationSettingsWidget* sim_settings_;
  DynamicConfigWidget* dynamic_config_;
  CommandersWidget* commanders_;

  KillGazeboThread sitl_kill_gazebo_thread_;
  qt::WaitSpinnerWidget spinner_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  bool startSITL();
  void terminateSITL();

  bool startHITL();
  void terminateHITL();

  bool buildLocalPackage();
  bool launchGazebo(bool launch_core);

  bool startDynamicConfig();
  void resetDynamicConfig();

  bool startCommanders();
  void resetCommanders();

  static std::string boolToText(bool arg);

private Q_SLOTS:
  void onStartRequested();
  void onTerminateRequested();

  void onSitlKillGazeboThreadFinished(bool success, const QString& message);

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sim
}  // namespace gui
