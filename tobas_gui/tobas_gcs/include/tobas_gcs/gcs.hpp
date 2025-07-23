#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <tobas_gui_common/remote_project_builder.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include <tobas_control_system/control_system.hpp>
#include <tobas_flight_log_gui/flight_log.hpp>
#include <tobas_hardware_setup/hardware_setup.hpp>
#include <tobas_parameter_tuning_gui/parameter_tuning.hpp>
#include <tobas_simulation_gui/simulation.hpp>

#include <tobas_msgs/msg/arming.hpp>

#include "./config_env_parser.hpp"
#include "./restart_button.hpp"
#include "./restart_thread.hpp"
#include "./shutdown_button.hpp"
#include "./shutdown_thread.hpp"

namespace gui
{
namespace gcs
{
class GroundControlStationWidget : public QWidget
{
  Q_OBJECT

  using self = GroundControlStationWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/tobas_project";

  static constexpr int kPathMaxWidth = 400;
  static constexpr int kPowerButtonRadius = 40;

public:
  explicit GroundControlStationWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateInternalDataStructures();

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  const rclcpp::Node::SharedPtr node_;
  RosQtBridge bridge_;

  uadf::Model uadf_;
  kdl::Tree tree_;
  tobas::Drone drone_;

  ptree::PropertyClient property_client_;
  ssh::SSHClient ssh_client_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;
  common::RemoteProjectBuilder remote_proj_builder_;
  ConfigurationEnvParser config_env_parser_;

  QLineEdit* tbs_path_;
  QPushButton* load_btn_;
  QPushButton* write_btn_;

  RestartButton* restart_btn_;
  ShutdownButton* shutdown_btn_;

  RestartThread restart_thread_;
  ShutdownThread shutdown_thread_;

  qt::WaitSpinnerWidget spinner_;

  hw::HardwareSetupWidget* hardware_setup_;
  gcs::ControlSystemWidget* control_system_;
  param::ParameterTuningWidget* param_tuning_;
  log::FlightLogWidget* flight_log_;
  sim::SimulationWidget* simulation_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  std::filesystem::path tbsPath() const;

private Q_SLOTS:
  void onLoadButtonClicked();
  void onWriteButtonClicked();

  void onRestartButtonClicked(bool checked);
  void onShutdownButtonClicked(bool checked);
  void onRestartThreadFinished(bool success, const QString& message);
  void onShutdownThreadFinished(bool success, const QString& message);

  void onSimRealStateChanged();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace gcs
}  // namespace gui
