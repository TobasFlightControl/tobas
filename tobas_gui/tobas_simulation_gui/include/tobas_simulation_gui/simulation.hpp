#pragma once

#include <QPushButton>

#include <tobas_linux/command_executor.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_gui_common/local_package_builder.hpp>

#include "./wind_parameters.hpp"

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

  void reset();
  void killGazebo();
  bool updateTBSPath(const std::filesystem::path& tbs_path);

private:
  const rclcpp::Node::SharedPtr node_;
  linux::CommandExecutor cmd_executor_;
  common::LocalPackageBuilder pkg_builder_;
  std::filesystem::path tbs_path_;
  tobas::Drone drone_;
  pid_t gazebo_pid_ = -1;

  QPushButton* start_button_;
  QPushButton* terminate_button_;

  WindParamsWidget* wind_params_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onTerminateButtonClicked();
};
}  // namespace sim
}  // namespace gui
