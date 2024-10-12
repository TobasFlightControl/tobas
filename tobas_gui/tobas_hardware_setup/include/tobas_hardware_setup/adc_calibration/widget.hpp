#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace hardware_setup
{
class ADCCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = ADCCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kBoxWidth = 100;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit ADCCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  qt::DoubleSpinBox* voltage_;
  QPushButton* start_button_;
  QLineEdit* adc_coef_;

  qt::WaitSpinnerWidget spinner_;
  ADCCalibrationThread thread_;

  std_msgs::msg::Bool::ConstSharedPtr arming_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCalibrationFinished(bool success, const QString& output);
};
}  // namespace hardware_setup
}  // namespace gui
