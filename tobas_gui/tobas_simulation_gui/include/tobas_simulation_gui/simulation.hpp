// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPointer>
#include <QProcess>
#include <QString>
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

namespace tobas::gui::sim
{
class SimulationWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationWidget;

Q_SIGNALS:
  void started();
  void terminated();
  void telemetryLossExpected();

public:
  static constexpr int kDroneId = 0;

  explicit SimulationWidget(const rqt::RosQtBridge& _bridge);
  ~SimulationWidget();

  void reset();
  void setProjectPath(const QString& _proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

  bool isRunning() const;

private:
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

  enum SimulationState
  {
    kIdle,
    kStarting,
    kRunning,
    kStopping,
  } state_ = kIdle;

  std::map<QString, QString> makeGazeboLaunchArguments() const;
  void launchSimulation();
  void terminateLaunchProcess();
  void terminateSimulation();
  void finalizeLaunchProcess(QProcess* _process, int _code, QProcess::ExitStatus _status);

private Q_SLOTS:
  void onStartRequested();
  void onTerminateRequested();
  void onLaunchProcessFinished(int _code, QProcess::ExitStatus _status);
  void onLaunchProcessErrorOccurred(QProcess::ProcessError _error);
};
}  // namespace tobas::gui::sim
