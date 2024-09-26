#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_property_client/property_client.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_gui_common/remote_package_builder.hpp>

#include <tobas_homepage/homepage.hpp>
#include <tobas_setup_assistant/setup_assistant.hpp>
#include <tobas_hardware_setup/hardware_setup.hpp>
#include <tobas_mission_planner/mission_planner.hpp>
#include <tobas_parameter_tuning_gui/parameter_tuning.hpp>

#include "./urdf_builder.hpp"

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

  static constexpr int kPathWidth = 300;
  static constexpr int kButtonWidth = 50;

public:
  explicit GUICoreWidget(rclcpp::Node::SharedPtr node);

private:
  const rclcpp::Node::SharedPtr node_;

  tobas::Drone drone_;

  ptree::PropertyClient property_cli_;
  ssh::SSHClient ssh_cli_;
  common::RemotePackageBuilder package_builder_;

  QLineEdit* tbs_path_;
  QPushButton* load_button_;
  QPushButton* send_button_;

  QPushButton* shutdown_button_;

  homepage::HomepageWidget* homepage_;
  URDFBuilder* urdf_builder_;
  setup_assistant::SetupAssistantWidget* setup_assistant_;
  hardware_setup::HardwareSetupWidget* hardware_setup_;
  mission_planner::MissionPlannerWidget* mission_planner_;
  param_tuning::ParameterTuningWidget* param_tuning_;

  void updateInternalDataStructures();
  std::filesystem::path tbsPath();

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSendButtonClicked();
  void onShutdownButtonClicked();
};
}  // namespace core
}  // namespace gui
