// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <tobas_actuator_test/actuator_test.hpp>
#include <tobas_control_system/control_system.hpp>
#include <tobas_flight_log_gui/flight_log.hpp>
#include <tobas_gui_common/network_config.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_gui_common/ssh_client.hpp>
#include <tobas_gui_common/ssh_config.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_parameter_tuning/parameter_tuning.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_sensor_calibration/sensor_calibration.hpp>
#include <tobas_simulation_gui/simulation.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include <tobas_msgs/msg/arming.hpp>

#include "./network_checker.hpp"
#include "./project_env_parser.hpp"
#include "./remote_connection.hpp"
#include "./restart_button.hpp"
#include "./shutdown_button.hpp"

namespace tobas
{
namespace gui
{
namespace gcs
{
class GroundControlStationWidget : public QWidget
{
  Q_OBJECT

  using self = GroundControlStationWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir";

  static constexpr int kPathMaxWidth = 400;
  static constexpr int kPowerButtonRadius = 40;

public:
  explicit GroundControlStationWidget(rclcpp::Node::SharedPtr node);

  void reset(bool include_simulation = true);
  void updateInternalDataStructures();

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  RosQtBridge bridge_;
  const NetworkChecker network_checker_;

  uadf::Model uadf_;
  kdl::Tree tree_;
  Drone drone_;

  ptree::PropertyClient property_client_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;
  cmn::ProjectPaths proj_paths_;
  cmn::Version proj_version_;
  cmn::SshConfig ssh_config_;
  cmn::NetworkConfig network_config_;
  cmn::SshClientWrapper ssh_client_;
  cmn::RemoteProjectBuilder remote_proj_builder_;
  ProjectEnvParser project_env_parser_;

  RemoteConnectionWidget* remote_conn_;

  QLineEdit* proj_path_;
  QPushButton* load_btn_;
  QPushButton* write_btn_;

  RestartButton* restart_btn_;
  ShutdownButton* shutdown_btn_;

  qt::WaitSpinnerWidget spinner_;

  sc::SensorCalibrationWidget* sensor_calib_;
  at::ActuatorTestWidget* actuator_test_;
  ctrl::ControlSystemWidget* control_system_;
  param::ParameterTuningWidget* param_tuning_;
  log::FlightLogWidget* flight_log_;
  sim::SimulationWidget* simulation_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  std::filesystem::path projectPath() const;

  std::expected<void, QString> restartInBackground();
  std::expected<void, QString> shutdownInBackground();

private Q_SLOTS:
  void onLoadButtonClicked();
  void onWriteButtonClicked();

  void onRestartButtonClicked(bool checked);
  void onShutdownButtonClicked(bool checked);

  void onSimRealStateChanged();
  void onRemoteConnectionDisconnected();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
