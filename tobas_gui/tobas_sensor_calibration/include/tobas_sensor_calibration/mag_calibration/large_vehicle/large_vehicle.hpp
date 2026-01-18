#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace gui
{
namespace sc
{
class LargeVehicleMagCalibWidget : public BaseMagCalibWidget
{
  Q_OBJECT

  using self = LargeVehicleMagCalibWidget;
  using super = BaseMagCalibWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit LargeVehicleMagCalibWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void reset() override;
  void setNamespace(const std::string& ns) override;

private:
  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  LargeVehicleMagCalibThread thread_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
