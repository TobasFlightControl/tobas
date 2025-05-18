#pragma once

#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class PreArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PreArmCheckViewerWidget;
  using super = QWidget;

Q_SIGNALS:
  void armingReceived(bool arming);
  void preArmCheckReceived(
    uint8_t node_connection_unstable,
    uint8_t battery_voltage_too_low,
    uint8_t cpu_temperature_too_high,
    uint8_t rotor_communication_error,
    uint8_t attitude_too_steep,
    uint8_t position_unstable,
    uint8_t position_inaccurate,
    uint8_t velocity_inaccurate,
    uint8_t attitude_inaccurate,
    uint8_t heading_inaccurate,
    bool ok);

public:
  explicit PreArmCheckViewerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* node_connection_status_;
  StatusWidget* battery_voltage_status_;
  StatusWidget* cpu_temp_status_;
  StatusWidget* rotor_comm_status_;
  StatusWidget* attitude_level_status_;
  StatusWidget* pos_stability_status_;
  StatusWidget* pos_accuracy_status_;
  StatusWidget* vel_accuracy_status_;
  StatusWidget* atti_accuracy_status_;
  StatusWidget* head_accuracy_status_;
  StatusWidget* ready_status_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  void armingCbRos(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void preArmCheckCbRos(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

private Q_SLOTS:
  void armingCbQt(bool arming);
  void preArmCheckCbQt(
    uint8_t node_connection_unstable,
    uint8_t battery_voltage_too_low,
    uint8_t cpu_temperature_too_high,
    uint8_t rotor_communication_error,
    uint8_t attitude_too_steep,
    uint8_t position_unstable,
    uint8_t position_inaccurate,
    uint8_t velocity_inaccurate,
    uint8_t attitude_inaccurate,
    uint8_t heading_inaccurate,
    bool ok);
};
}  // namespace gcs
}  // namespace gui
