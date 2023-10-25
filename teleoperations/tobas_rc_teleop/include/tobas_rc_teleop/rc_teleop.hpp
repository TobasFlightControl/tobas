#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RCInput.h>

#include "./pos_vel_acc_yaw.hpp"
#include "./position_yaw.hpp"
#include "./velocity_yaw.hpp"
#include "./rpy_thrust.hpp"

namespace tobas_rc_teleop
{
class RCTeleop : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit RCTeleop(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    FIRST_RCIN,
    ESTOP_ON,
    RUNNING,
  };

  enum command_t
  {
    NONE,
    POS_VEL_ACC_YAW,
    POSITION_YAW,
    VELOCITY_YAW,
    ACCELERATION_YAW,
    RPY_THRUST,
    SPEED_ROLL_DPITCH,
  };

  stage_t stage_ = CHECK_PREREQUISITES;
  command_t last_cmd_type_ = NONE;
  tobas_msgs::PoseTwistConstPtr pt_;
  tobas_msgs::BatteryConstPtr battery_;

  // rosparams
  double dead_zone_rate_;
  std::vector<std::string> mode_names_;

  // Constant values
  dh_std::Range<double> dead_zone_;
  std::vector<command_t> mode2cmd_;

  // Controllers
  PosVelAccYawController pvay_ctrl_;
  PositionYawController pos_yaw_ctrl_;
  VelocityYawController vel_yaw_ctrl_;
  RollPitchYawThrustController rpy_thrust_ctrl_;

  // PubSub
  ros::Publisher event_pub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber rcin_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void rcInputCb(const tobas_msgs::RCInputConstPtr& rcin);
};
}  // namespace tobas_rc_teleop
