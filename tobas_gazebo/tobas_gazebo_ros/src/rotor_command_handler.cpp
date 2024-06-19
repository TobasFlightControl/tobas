#include <std_msgs/Bool.h>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/Throttles.h>

#include "../include/tobas_gazebo_ros/rotor_command_handler.hpp"
#include "../include/tobas_gazebo_ros/constants.hpp"

using namespace std;

namespace tobas_gazebo_ros
{
RotorCommandHandler::RotorCommandHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  drone_.loadFromParam(nh_);

  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
  arming_pub_ = nh_.advertise<std_msgs::Bool>(tobas::kArmingTopic, 1, true);

  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this);
  tar_speeds_sub_ = nh_.subscribe(tobas::kRotorSpeedsCmdTopic, 1, &self::targetRotorSpeedsCb, this);

  get_arm_ss_ = nh_.advertiseService(tobas::kGetArmSrv, &self::getArmCb, this);
  set_arm_ss_ = nh_.advertiseService(tobas::kSetArmSrv, &self::setArmCb, this);
  pre_arm_check_sc_ = nh_.serviceClient<std_srvs::Trigger>(tobas::kPreArmCheckSrv);

  publishArming();
}

void RotorCommandHandler::publishArming()
{
  const auto arming_msg = boost::make_shared<std_msgs::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_.publish(arming_msg);
}

void RotorCommandHandler::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RotorCommandHandler::targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  if (!is_armed_)
    return;

  if (battery_ == nullptr)
  {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod, "The rotors cannot be rotated because battery state has not been received "
                    "yet.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_.numRotors())
  {
    TOBAS_ERROR("Size mismatch: ", data_size, " != ", drone_.numRotors());
    return;
  }

  // Create throttle message
  const auto throttles = boost::make_shared<tobas_msgs::Throttles>();
  throttles->header = tar_speeds->header;
  throttles->data.resize(data_size);

  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    // Check the validity of the target rotation speed
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_->voltage);
    auto tar_speed = tar_speeds->speeds[rotor_idx];
    if (tar_speed < 0.)
    {
      TOBAS_WARN("Negative rotation speed is commanded on CH", rotor_idx, ": ", tar_speed, " < 0 [rad/s]");
      tar_speed = 0.;
    }
    else if (tar_speed > max_speed + tobas::kRotSpeedMargin)
    {
      TOBAS_WARN("Target rotation speed of CH", rotor_idx, " is too high: ", tar_speed, " > ", max_speed, " [rad/s]");
      tar_speed = max_speed;
    }

    // Fill throttle
    throttles->data[rotor_idx] = drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
  }

  // Publish throttle message
  throttles_pub_.publish(throttles);
}

bool RotorCommandHandler::getArmCb(tobas_msgs::GetArmRequest&, tobas_msgs::GetArmResponse& res)
{
  res.arming = is_armed_;
  return true;
}

bool RotorCommandHandler::setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res)
{
  if (!is_armed_ && req.arming)
  {
    // Pre-arm check
    if (!req.ignore_pre_arm_check)
    {
      if (!pre_arm_check_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
      {
        res.success = false;
        res.message = "Failed to connect to pre-arm check service server.";
        return true;
      }
      if (!pre_arm_check_sc_.call(pre_arm_check_msg_) || !pre_arm_check_msg_.response.success)
      {
        res.success = false;
        res.message = pre_arm_check_msg_.response.message;
        return true;
      }
    }

    // Arming
    ros::Duration(tobas::kDisarmDuration).sleep();  // 実機に近づけるためArmに要する時間だけスリープ
    is_armed_ = req.arming;
  }
  else if (is_armed_ && !req.arming)
  {
    // Disarming
    is_armed_ = req.arming;
  }

  publishArming();

  res.success = true;
  return true;
}
}  // namespace tobas_gazebo_ros
