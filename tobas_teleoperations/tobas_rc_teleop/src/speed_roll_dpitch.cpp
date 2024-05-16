#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
SpeedRollDeltaPitchController::SpeedRollDeltaPitchController(const tobas::Drone& drone)
  : super(drone)
{
}

void SpeedRollDeltaPitchController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  cmd_pub_ = nh.advertise<tobas_msgs::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic, 1);
}

void SpeedRollDeltaPitchController::reset(const tobas_msgs::Odometry&)
{
}

void SpeedRollDeltaPitchController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry&,
  const double&)
{
  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::SpeedRollDeltaPitch>();
  cmd->speed = remap(rcin.throttle, min_speed_, max_speed_);  // TODO: 機体の制限速度を考慮
  cmd->roll = remapDead(rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(rcin.pitch, -max_dpitch_, max_dpitch_);

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void SpeedRollDeltaPitchController::getRosParams(ros::NodeHandle& pnh)
{
  tobas_ros::getParam(
    pnh, "speed_roll_dpitch/min_speed", min_speed_, kDefaultMinSpeed, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "speed_roll_dpitch/max_speed", max_speed_, kDefaultMaxSpeed, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "speed_roll_dpitch/max_roll", max_roll_, kDefaultMaxRoll, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "speed_roll_dpitch/max_dpitch", max_dpitch_, kDefaultMaxDeltaPitch, tobas_ros::POSITIVE);

  ROS_CHECK(pnh, min_speed_ < max_speed_, "Maximum speed must be greater than minimum speed.");
}
}  // namespace tobas_rc_teleop
