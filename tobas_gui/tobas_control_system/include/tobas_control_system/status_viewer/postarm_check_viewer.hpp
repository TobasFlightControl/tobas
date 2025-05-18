#pragma once

#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class PostArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PostArmCheckViewerWidget;
  using super = QWidget;

Q_SIGNALS:
  void armingReceived(bool arming);
  void postArmCheckReceived(
    bool gyro_noise_too_large,
    bool accel_noise_too_large,
    bool mag_offset_too_large,
    bool mag_misalignment,
    bool latency_too_large);

public:
  explicit PostArmCheckViewerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* gyro_noise_status_;
  StatusWidget* accel_noise_status_;
  StatusWidget* mag_offset_status_;
  StatusWidget* mag_alignment_status_;
  StatusWidget* latency_status_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PostArmCheck> postarm_check_sub_;

  void armingCbRos(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void postArmCheckCbRos(const tobas_msgs::msg::PostArmCheck::ConstSharedPtr& postarm_check);

private Q_SLOTS:
  void armingCbQt(bool arming);
  void postArmCheckCbQt(
    bool gyro_noise_too_large,
    bool accel_noise_too_large,
    bool mag_offset_too_large,
    bool mag_misalignment,
    bool latency_too_large);
};
}  // namespace gcs
}  // namespace gui
