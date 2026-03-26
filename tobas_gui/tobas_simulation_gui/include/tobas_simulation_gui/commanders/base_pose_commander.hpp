#pragma once

#include <array>

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

namespace gui
{
namespace sim
{
class BasePoseCommanderWidget : public QWidget
{
  Q_OBJECT

  using self = BasePoseCommanderWidget;
  using super = QWidget;

  static constexpr double kHomeAltitude = 3.;  // [m]

public:
  explicit BasePoseCommanderWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge, const tobas::Drone& drone);

  void updateInternalDataStructures();

  bool start();
  void reset();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  qt::ToggleButton* arming_button_;

  std::array<qt::DoubleSliderDisplay*, 3> cmd_xyz_;  // [m]
  std::array<qt::IntSliderDisplay*, 3> cmd_rpy_;     // [deg]

  QPushButton* home_button_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_msgs::RCInput::ConstSharedPtr rcin_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAcc> pva_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> pvay_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> pvapy_pub_;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm>::SharedPtr set_arm_sc_;

  bool isRunning() const;

  void publishCurrentCommand();
  bool armRotors(bool arming);

private Q_SLOTS:
  void onArmRequested();
  void onDisarmRequested();

  void onValueChanged();

  void onHomeButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace sim
}  // namespace gui
