#pragma once

#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_command_msgs_adapter/pose_twist_accel.hpp>

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

  static constexpr int kButtonWidth = 80;
  static constexpr int kButtonHeight = 50;

  static constexpr double kHomeAltitude = 3.;  // [m]

public:
  explicit BasePoseCommanderWidget(rclcpp::Node::SharedPtr node);

  bool start(const std::string& ns);
  void terminate();

private:
  const rclcpp::Node::SharedPtr node_;

  qt::ToggleButton* arming_button_;
  QPushButton* home_button_;

  qt::DoubleSliderDisplay* cmd_x_;
  qt::DoubleSliderDisplay* cmd_y_;
  qt::DoubleSliderDisplay* cmd_z_;
  qt::DoubleSliderDisplay* cmd_roll_;
  qt::DoubleSliderDisplay* cmd_pitch_;
  qt::DoubleSliderDisplay* cmd_yaw_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> pvay_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PoseTwistAccel> pta_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm>::SharedPtr set_arm_sc_;

  void reset();
  void publishCurrentCommand();
  bool armRotors(bool arming);

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

private Q_SLOTS:
  void onValueChanged();

  void onArmButtonClicked();
  void onDisarmButtonClicked();
  void onHomeButtonClicked();
};
}  // namespace sim
}  // namespace gui
