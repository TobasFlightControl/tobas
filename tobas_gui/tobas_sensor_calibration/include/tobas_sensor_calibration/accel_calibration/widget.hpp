#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/register.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace sc
{
class AccelCalibrationWidget : public BaseWidget
{
  Q_OBJECT

  using self = AccelCalibrationWidget;
  using super = BaseWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit AccelCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void setNamespace(const std::string& ns);

private:
  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  AccelCalibrationThread thread_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onCalibrationFinished(bool success, const QString& output);

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
