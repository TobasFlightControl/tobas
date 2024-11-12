#pragma once

#include <QPushButton>
#include <std_msgs/msg/bool.hpp>

#include <tobas_linux/command_executor.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_gui_common/local_package_builder.hpp>
#include <tobas_gui_common/remote_package_builder.hpp>

#include "./static_configuration/static_configuration.hpp"
#include "./dynamic_configuration/dynamic_configuration.hpp"

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

public:
  explicit SimulationWidget(rclcpp::Node::SharedPtr node);
  ~SimulationWidget();

  bool updateTBSPath(const std::filesystem::path& tbs_path);

private:
  const rclcpp::Node::SharedPtr node_;
  linux::CommandExecutor cmd_executor_;
  ssh::SSHClient ssh_client_;
  common::LocalPackageBuilder local_pkg_builder_;
  common::RemotePackageBuilder remote_pkg_builder_;

  std::filesystem::path tbs_path_;
  tobas::Drone drone_;
  pid_t launch_pid_ = -1;

  QPushButton* start_button_;
  QPushButton* terminate_button_;

  StaticConfigWidget* static_config_;
  DynamicConfigWidget* dynamic_config_;

  std_msgs::msg::Bool::ConstSharedPtr arming_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);

  bool reset();
  bool killGazeboLaunch();

  bool startSITL();
  bool terminateSITL();

  bool startHITL();
  bool terminateHITL();

  bool buildLocalPackage();
  bool launchGazebo(bool launch_core);
  bool initializeDynamicConfig();

  static std::string boolToText(bool arg);

private Q_SLOTS:
  void onStartButtonClicked();
  void onTerminateButtonClicked();
};
}  // namespace sim
}  // namespace gui
