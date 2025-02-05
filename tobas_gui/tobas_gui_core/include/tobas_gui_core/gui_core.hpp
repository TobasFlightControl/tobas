#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_gui_common/remote_package_builder.hpp>
#include <tobas_msgs/msg/arming.hpp>

#include <tobas_homepage/homepage.hpp>
#include <tobas_setup_assistant/setup_assistant.hpp>
#include <tobas_hardware_setup/hardware_setup.hpp>
#include <tobas_control_system/control_system.hpp>
#include <tobas_parameter_tuning_gui/parameter_tuning.hpp>
#include <tobas_flight_log_gui/flight_log.hpp>
#include <tobas_simulation_gui/simulation.hpp>

#include "./urdf_builder.hpp"
#include "./restart_button.hpp"
#include "./shutdown_button.hpp"

namespace gui
{
namespace core
{
class GUICoreWidget : public QWidget
{
  Q_OBJECT

  using self = GUICoreWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir/tobas_configuration_package";

  static constexpr int kPathMaxWidth = 400;
  static constexpr int kPowerButtonRadius = 40;

public:
  explicit GUICoreWidget(rclcpp::Node::SharedPtr node);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;

  kdl::Tree tree_;
  tobas::Drone drone_;

  ptree::PropertyClient property_client_;
  ssh::SSHClient ssh_client_;
  common::RemotePackageBuilder package_builder_;

  QLineEdit* tbs_path_;
  QPushButton* browse_btn_;
  QPushButton* load_btn_;
  QPushButton* write_btn_;

  RestartButton* restart_btn_;
  ShutdownButton* shutdown_btn_;

  homepage::HomepageWidget* homepage_;
  URDFBuilder* urdf_builder_;
  setup_assistant::SetupAssistantWidget* setup_assistant_;
  hardware_setup::HardwareSetupWidget* hardware_setup_;
  control_system::ControlSystemWidget* control_system_;
  param_tuning::ParameterTuningWidget* param_tuning_;
  log::FlightLogWidget* flight_log_;
  sim::SimulationWidget* simulation_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

  std::filesystem::path tbsPath() const;

private Q_SLOTS:
  void onBrowseButtonClicked();
  void onLoadButtonClicked();
  void onWriteButtonClicked();
  void onRestartButtonClicked();
  void onShutdownButtonClicked();
};
}  // namespace core
}  // namespace gui
