#pragma once

#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>

#include "../base.hpp"

namespace gui
{
namespace hardware_setup
{
class RCInputCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = RCInputCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kPwmMin = 900;
  static constexpr int kPwmMax = 2100;
  static constexpr int kMinSignalRange = 300;
  static constexpr int kRangeSideShort = 30;
  static constexpr int kRangeSideLong = 300;

  static constexpr char kModeText[] = "Program               Stabilize               Acrobat";
  static constexpr char kOnOffText[] = "ON                                                       OFF";

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit RCInputCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;
  qt::HPositionBarWidget* mode_range_;
  qt::HPositionBarWidget* estop_range_;
  qt::HPositionBarWidget* gpsw_range_;

  ros2::SubscriberPtr<tobas_hal_msgs::msg::Sbus> sbus_sub_;

  void reset();

  void sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();
};
}  // namespace hardware_setup
}  // namespace gui
