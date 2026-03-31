#pragma once

#include <QPushButton>

#include <tobas_constants/rc_input.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class RCInputCalibrationWidget : public BaseWidget
{
  Q_OBJECT

  using self = RCInputCalibrationWidget;
  using super = BaseWidget;

  // SBUSのスロットル範囲は172-1811が基本
  static constexpr int kMinPeriod = 0;
  static constexpr int kMaxPeriod = 2000;

  // S.BUSの各チャネルの値の範囲は最低でも1000，最大で2000以下であるため，
  // 1000を閾値にしておけば3段階スイッチの2段階までしか動かさないヒューマンエラーを防げる．
  static constexpr int kMinSignalRange = 1000;

  static constexpr char kOnText[] = "ON";
  static constexpr char kOffText[] = "OFF";

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kRangeSideShort = 50;

public:
  explicit RCInputCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge, const Drone& drone);

  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const Drone& drone_;

  bool running_;
  tobas_msgs::msg::Sbus::ConstSharedPtr sbus_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;
  qt::HPositionBarWidget* mode_range_;
  qt::HPositionBarWidget* sub_mode_range_;
  qt::HPositionBarWidget* enable_range_;
  qt::HPositionBarWidget* kill_range_;

  std::array<QLabel*, kMaxNumOfGpsw> gpsw_labels_;
  std::array<qt::HPositionBarWidget*, kMaxNumOfGpsw> gpsw_ranges_;

  size_t numOfGpswChannels() const;

  bool saveParamsToGcs();
  bool saveParamsToFc();

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
