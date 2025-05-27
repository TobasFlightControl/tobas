#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./commands_publisher.hpp"

namespace gui
{
namespace hw
{
class JointTestWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = JointTestWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit JointTestWidget(
    rclcpp::Node::SharedPtr node,
    const RosQtBridge& bridge,
    const kdl::Tree& tree,
    const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const tobas::Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  JointCommandsPublisherWidget* commands_publisher_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace hw
}  // namespace gui
