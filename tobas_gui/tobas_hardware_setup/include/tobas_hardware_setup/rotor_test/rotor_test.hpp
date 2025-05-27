#pragma once

#include <QPushButton>
#include <QTimer>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/srv/get_rotor_control_gains.hpp>
#include <tobas_msgs/srv/set_rotor_control_gains.hpp>

#include "../base.hpp"
#include "./rotor_widget.hpp"

namespace gui
{
namespace hw
{
class RotorTestWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = RotorTestWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kChannelSize = 8;    // TODO: ハードウェアの最大チャンネル数に合わせる
  static constexpr int kUpdatePeriod = 10;  // [ms]
  static constexpr auto kWaitForService = std::chrono::seconds(3);

public:
  explicit RotorTestWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge, const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const RosQtBridge& bridge_;
  const tobas::Drone& drone_;
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* save_button_;

  std::array<RotorWidget*, kChannelSize> rotor_widgets_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_pub_;

  ros2::SyncServiceClient<tobas_msgs::srv::GetRotorControlGains>::SharedPtr get_gains_sc_;
  ros2::SyncServiceClient<tobas_msgs::srv::SetRotorControlGains>::SharedPtr set_gains_sc_;
  ros2::SyncServiceClient<std_srvs::srv::Trigger>::SharedPtr save_gains_sc_;

  QMetaObject::Connection rotor_states_conn_;

  QTimer update_timer_;

  void publishTargetSppeds();
  bool loadCurrentGains();

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onSaveButtonClicked();

  void onTargetRPMChanged(int rpm, size_t ch);
  void onGainChanged(int gain, size_t ch);

  void onUpdateTimerTimeout();

  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& cur_states);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace hw
}  // namespace gui
