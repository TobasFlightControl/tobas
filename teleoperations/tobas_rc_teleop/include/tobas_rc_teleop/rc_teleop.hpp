#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RCInput.h>

#include "./velocity_yaw.hpp"
#include "./roll_pitch_yawrate_thrust.hpp"

namespace tobas_rc_teleop
{
class RCTeleop : public tobas::BaseNode
{
  // Constants
  static constexpr double kErrorPeriod = 3.;  // [s]

  // Default parameters
  static constexpr double kDefaultDeadZoneRate = 0.1;

  using super = tobas::BaseNode;

public:
  explicit RCTeleop(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

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
    POSITION_YAW,
    VELOCITY_YAW,
    ACCELERATION_YAW,
    ROLL_PITCH_YAW_THRUST,
    ROLL_PITCH_YAWRATE_THRUST,
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
  VelocityYawController velocity_yaw_ctrl_;
  RollPitchYawrateThrustController roll_pitch_yawrate_thrust_ctrl_;

  // PubSub
  ros::Publisher vel_yaw_pub_;
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
