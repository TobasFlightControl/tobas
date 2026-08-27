// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPointer>
#include <QProcess>
#include <QPushButton>
#include <QWidget>

#include <tobas_drone_core/drone.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include "./commanders/commanders.hpp"
#include "./dynamic_configuration/dynamic_configuration.hpp"
#include "./simulation_settings/simulation_settings.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class SimulationWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationWidget;
  using super = QWidget;

Q_SIGNALS:
  void started();
  void terminated();
  void telemetryLossExpected();

public:
  static constexpr int kDroneId = 0;

  explicit SimulationWidget(const rqt::RosQtBridge& bridge);

  void reset();
  void updateProject(const std::filesystem::path& proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

  bool isRunning() const;

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  rclcpp::Node::SharedPtr node_;

  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;
  cmn::ProjectPaths proj_paths_;

  uadf::Model uadf_;
  kdl::Tree tree_;
  Drone drone_;

  QPointer<QProcess> launch_proc_;
  qt::WaitSpinnerWidget spinner_;

  qt::ToggleButton* start_stop_button_;

  SimulationSettingsWidget* sim_settings_;
  DynamicConfigWidget* dynamic_config_;
  CommandersWidget* commanders_;

  bool project_loaded_ = false;

  std::map<QString, QString> makeGazeboLaunchArguments() const;
  void launchSimulation();
  void terminateLaunchProcess();
  void terminateSimulation();

private Q_SLOTS:
  void onStartRequested();
  void onTerminateRequested();
  void onLaunchProcessFinished(int code, QProcess::ExitStatus status);
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
