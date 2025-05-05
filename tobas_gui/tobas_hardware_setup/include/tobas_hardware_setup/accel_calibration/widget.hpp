#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/arming.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace hw
{
class AccelCalibrationWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = AccelCalibrationWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit AccelCalibrationWidget(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void setNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  AccelCalibrationThread thread_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onCalibrationFinished(bool success, const QString& output);
};
}  // namespace hw
}  // namespace gui
