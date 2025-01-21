#pragma once

#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace control_system
{
class PostArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PostArmCheckViewerWidget;
  using super = QWidget;

public:
  explicit PostArmCheckViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* gyro_noise_status_;
  StatusWidget* accel_noise_status_;
  StatusWidget* mag_alignment_status_;
  StatusWidget* latency_status_;

  std_msgs::msg::Bool::ConstSharedPtr arming_;

  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PostArmCheck> prearm_check_sub_;

  void reset();

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void postArmCheckCb(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& postarm_check);
};
}  // namespace control_system
}  // namespace gui
