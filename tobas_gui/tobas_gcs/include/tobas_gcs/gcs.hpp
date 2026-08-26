// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QWidget>
#include <rclcpp/context.hpp>

#include <tobas_actuator_test/actuator_test.hpp>
#include <tobas_control_system/control_system.hpp>
#include <tobas_flight_log_gui/flight_log.hpp>
#include <tobas_gui_common/network_config.hpp>
#include <tobas_gui_common/ssh_client.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_parameter_tuning/parameter_tuning.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_sensor_calibration/sensor_calibration.hpp>
#include <tobas_simulation_gui/simulation.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include <tobas_msgs/msg/arming.hpp>

#include "./flight_controller_scanner.hpp"
#include "./project_env_parser.hpp"
#include "./remote_connection.hpp"

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

  static constexpr char kLastOpenedDirKey[] = "gcs/last_opened_dir";

public:
  explicit GroundControlStationWidget(int argc, char** argv);

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  RosQtBridge bridge_;

  uadf::Model uadf_;
  kdl::Tree tree_;
  Drone drone_;

  QSettings settings_store_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;
  cmn::Version proj_version_;
  cmn::NetworkConfig network_config_;
  ProjectEnvParser project_env_parser_;

  RemoteConnectionWidget* remote_conn_;

  QLineEdit* proj_path_;
  QPushButton* load_btn_;

  FlightControllerScanner* fc_scanner_;
  QComboBox* fc_selector_;
  QSpinBox* vehicle_id_;
  qt::ToggleButton* connect_btn_;
  QPushButton* write_btn_;
  QPushButton* restart_btn_;
  QPushButton* shutdown_btn_;

  qt::WaitSpinnerWidget spinner_;

  sc::SensorCalibrationWidget* sensor_calib_;
  at::ActuatorTestWidget* actuator_test_;
  ctrl::ControlSystemWidget* control_system_;
  param::ParameterTuningWidget* param_tuning_;
  log::FlightLogWidget* flight_log_;
  sim::SimulationWidget* simulation_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  bool telemetry_loss_expected_ = false;
  bool project_loaded_ = false;
  bool connection_ready_ = false;

  std::vector<std::string> ros_args_;
  rclcpp::Context::SharedPtr ros_context_;
  std::unique_ptr<ros2::AsyncNodeManager> ros_node_manager_;
  rclcpp::Node::SharedPtr ros_node_;

  std::optional<cmn::SshClientWrapper> ssh_client_;
  std::optional<cmn::RemoteProjectBuilder> remote_proj_builder_;

  void reset(bool include_simulation = true);
  void updateInternalDataStructures();
  void initializeRosConnection();
  void clearRosConnection();
  void connectToFlightController();
  void disconnectFromFlightController();
  bool waitForHeartbeat() const;

  void updateHeaderActionAvailability();
  void setFlightControllerPlaceholder(const QString& text);

  QString currentHost() const;
  int currentId() const;
  QString currentConnectionDescription() const;

  void expectTelemetryLoss();
  void clearExpectedTelemetryLoss();

  std::expected<void, QString> restartInBackground();
  std::expected<void, QString> shutdownInBackground();

private Q_SLOTS:
  void onLoadButtonClicked();
  void onConnectRequested();
  void onDisconnectRequested();
  void onWriteButtonClicked();

  void onFlightControllerScanFinished(const QVector<DiscoveredFlightController>& flight_controllers);
  void onFlightControllerScanFailed(const QString& message);

  void onRestartButtonClicked();
  void onShutdownButtonClicked();

  void onSimRealStateChanged();
  void onRemoteConnectionDisconnected();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
