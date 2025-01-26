#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs_adapter/gps.hpp>

#include "./status.hpp"

namespace gui
{
namespace control_system
{
class OtherStatusViewerWidget : public QWidget
{
  Q_OBJECT

  using self = OtherStatusViewerWidget;
  using super = QWidget;

public:
  explicit OtherStatusViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* arming_status_;
  StatusWidget* gps_status_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gps> gps_sub_;

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps);
};
}  // namespace control_system
}  // namespace gui
