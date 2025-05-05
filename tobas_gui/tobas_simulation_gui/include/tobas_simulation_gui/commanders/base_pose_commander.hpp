#pragma once

#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
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

  static constexpr int kArmingButtonWidth = 100;
  static constexpr int kArmingButtonHeight = 40;
  static constexpr int kCommandButtonHeight = 40;

  static constexpr double kHomeAltitude = 3.;  // [m]

public:
  explicit BasePoseCommanderWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

  bool start();
  void reset();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  qt::ToggleButton* arming_button_;

  qt::DoubleSliderDisplay* cmd_x_;
  qt::DoubleSliderDisplay* cmd_y_;
  qt::DoubleSliderDisplay* cmd_z_;
  qt::DoubleSliderDisplay* cmd_roll_;
  qt::DoubleSliderDisplay* cmd_pitch_;
  qt::DoubleSliderDisplay* cmd_yaw_;

  QPushButton* home_button_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> pos_vel_yaw_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm>::SharedPtr set_arm_sc_;

  void publishCurrentCommand();
  bool armRotors(bool arming);

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

private Q_SLOTS:
  void onArmRequested();
  void onDisarmRequested();

  void onValueChanged();

  void onHomeButtonClicked();
};
}  // namespace sim
}  // namespace gui
