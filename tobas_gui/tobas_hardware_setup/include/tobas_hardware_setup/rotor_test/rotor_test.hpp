#pragma once

#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>

#include "../base.hpp"
#include "./rotor_widget.hpp"

namespace gui
{
namespace hardware_setup
{
class RotorTestWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = RotorTestWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kChannelSize = 8;  // TODO: ハードウェアの最大チャンネル数に合わせる
  static constexpr auto kPublishPeriod = std::chrono::milliseconds(10);
  static constexpr auto kWaitForService = std::chrono::seconds(3);

public:
  explicit RotorTestWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* save_button_;

  std::array<RotorWidget*, kChannelSize> rotors_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  bool is_running_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> cur_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm>::SharedPtr set_arm_sc_;
  ros2::SyncServiceClient<tobas_msgs::srv::GetRotorControlGains>::SharedPtr get_gains_sc_;
  ros2::SyncServiceClient<tobas_msgs::srv::SetRotorControlGains>::SharedPtr set_gains_sc_;
  ros2::SyncServiceClient<std_srvs::srv::Trigger>::SharedPtr save_gains_sc_;

  ros2::TimerPtr publish_timer_;

  void reset();
  void publishTargetSppeds();
  bool loadCurrentGains();
  bool armRotors(bool arming);

  void currentStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& cur_states);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onSaveButtonClicked();

  void onTargetRPMChanged(int rpm, size_t ch);
  void onGainChanged(int gain, size_t ch);
};
}  // namespace hardware_setup
}  // namespace gui
