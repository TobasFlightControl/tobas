#pragma once

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/range.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RCInput.h>

namespace tobas_rc_teleop
{
class RcinToVelocityYaw : public tobas::BaseNode
{
  static constexpr double kDefaultMaxHorizontalVelocity = 3.;
  static constexpr double kDefaultMaxVerticalVelocity = 3.;
  static constexpr double kDefaultMaxYawrate = dh_std::deg2rad(180.);

  using super = tobas::BaseNode;

public:
  explicit RcinToVelocityYaw(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  enum Stage
  {
    CHECK_PREREQUISITES,
    FIRST_RCIN,
    TOGGLE_OFF,
    TOGGLE_ON,
  };

  Stage stage_ = CHECK_PREREQUISITES;
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::VelocityYaw vel_yaw_;
  ros::Time t_last_rcin_;

  // ROS parameters
  double max_hor_vel_;  // [m/s]
  double max_ver_vel_;  // [m/s]
  double max_yawrate_;  // [rad/s]
  double dead_zone_rate_;

  // Constant values
  dh_std::Range<double> dead_zone_;

  // PubSub
  ros::Publisher vel_yaw_pub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber rcin_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void rcInputCb(const tobas_msgs::RCInputConstPtr& rcin);
};
}  // namespace tobas_rc_teleop
