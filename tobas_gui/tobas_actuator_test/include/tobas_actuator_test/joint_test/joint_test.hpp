#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./commands_publisher.hpp"

namespace tobas
{
namespace gui
{
namespace at
{
class JointTestWidget : public BaseWidget
{
  Q_OBJECT

  using self = JointTestWidget;
  using super = BaseWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit JointTestWidget(
    rclcpp::Node::SharedPtr node,
    const RosQtBridge& bridge,
    const kdl::Tree& tree,
    const Drone& drone);

  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();

  int numRegisteredChannels() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* zero_button_;
  QPushButton* home_button_;

  JointCommandsPublisherWidget* commands_publisher_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onZeroButtonClicked();
  void onHomeButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace at
}  // namespace gui
}  // namespace tobas
