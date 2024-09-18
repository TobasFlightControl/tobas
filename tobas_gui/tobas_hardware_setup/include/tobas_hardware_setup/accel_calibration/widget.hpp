#pragma once

#include <QPushButton>

#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace hardware_setup
{
class AccelCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = AccelCalibrationWidget;
  using super = BaseHardwareSetupWidget;

public:
  explicit AccelCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

private:
  const rclcpp::Node::SharedPtr node_;

  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  AccelCalibrationThread thread_;

  tobas::Drone::ConstSharedPtr drone_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCalibrationFinished(bool success, const QString& output);
};
}  // namespace hardware_setup
}  // namespace gui
