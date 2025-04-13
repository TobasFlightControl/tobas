#pragma once

#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/rate_manager.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>

#include "../base.hpp"

namespace gui
{
namespace hw
{
class RCInputCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = RCInputCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  // SBUSのスロットル範囲は172-1811が基本
  static constexpr int kMinPeriod = 0;
  static constexpr int kMaxPeriod = 2000;

  static constexpr int kMinSignalRange = 300;
  static constexpr int kRangeSideShort = 50;

  static constexpr char kOnText[] = "ON";
  static constexpr char kOffText[] = "OFF";

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

  static constexpr double kTopicRate = 30.;  // [Hz]

public:
  explicit RCInputCalibrationWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  ros2::RateManager rate_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;
  qt::HPositionBarWidget* enable_range_;
  qt::HPositionBarWidget* kill_range_;
  qt::HPositionBarWidget* mode_range_;
  qt::HPositionBarWidget* sub_mode_range_;

  std::array<QLabel*, tobas::kMaxNumOfGpsw> gpsw_labels_;
  std::array<qt::HPositionBarWidget*, tobas::kMaxNumOfGpsw> gpsw_ranges_;

  ros2::SubscriberPtr<tobas_msgs::msg::Sbus> sbus_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;

  size_t numOfGpswChannels() const;

  bool saveParamsGCS();
  bool saveParamsFC();

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();
};
}  // namespace hw
}  // namespace gui
