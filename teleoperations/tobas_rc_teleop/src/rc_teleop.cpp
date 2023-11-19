#include <dh_std_tools/math.hpp>
#include <dh_std_tools/string.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/PoseTwistAccelCommand.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

#include "../include/tobas_rc_teleop/rc_teleop.hpp"
#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/velocity_yaw.hpp"
#include "../include/tobas_rc_teleop/rpy_thrust.hpp"
#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"

using namespace std;
using namespace ros::message_traits;
using namespace dh_std;

namespace tobas_rc_teleop
{
RCTeleop::RCTeleop(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();

  dead_zone_.lower = -dead_zone_rate_ / 2;
  dead_zone_.upper = dead_zone_rate_ / 2;

  for (const auto& mode_name : mode_names_)
  {
    if (mode_name == split(DataType<tobas_msgs::PosVelAccYaw>::value(), '/').back())
    {
      controllers_.push_back(make_unique<PosVelAccYawController>());
    }
    else if (mode_name == split(DataType<tobas_msgs::PositionYaw>::value(), '/').back())
    {
      controllers_.push_back(make_unique<PositionYawController>());
    }
    else if (mode_name == split(DataType<tobas_msgs::VelocityYaw>::value(), '/').back())
    {
      controllers_.push_back(make_unique<VelocityYawController>());
    }
    else if (mode_name == split(DataType<tobas_msgs::RollPitchYawThrust>::value(), '/').back())
    {
      controllers_.push_back(make_unique<RollPitchYawThrustController>());
    }
    else if (mode_name == split(DataType<tobas_msgs::PoseTwistAccelCommand>::value(), '/').back())
    {
      controllers_.push_back(make_unique<PoseTwistAccelController>());
    }
    else if (mode_name == split(DataType<tobas_msgs::SpeedRollDeltaPitch>::value(), '/').back())
    {
      rosError(name_, "Not implemented yet.");  // TODO
    }
    else
    {
      throw runtime_error("Invalid flight mode: " + mode_name);
    }
  }

  for (const auto& ctrl : controllers_)
    ctrl->initialize(nh_, pnh_);

  registerPublishers();
  registerSubscribers();
}

void RCTeleop::getRosParams()
{
  dh_ros::getParam(
    pnh_, "dead_zone_rate", dead_zone_rate_, kDefaultDeadZoneRate, dh_ros::NON_NEGATIVE);
  if (dead_zone_rate_ >= 1)
    ROS_THROW_NAMED(name_, "'dead_zone_rate' must be lower than 1.");

  dh_ros::getParam(pnh_, "mode_names", mode_names_);
}

void RCTeleop::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>(tobas::kEventTopic, 1);
}

void RCTeleop::registerSubscribers()
{
  super::registerSubscribers();

  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
  rcin_sub_ = nh_.subscribe(tobas::kRcInputTopic, 1, &self::rcInputCb, this, tcpNoDelay());
}

void RCTeleop::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void RCTeleop::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;
}

void RCTeleop::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RCTeleop::rcInputCb(const tobas_msgs::RCInputConstPtr& rcin)
{
  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (odom_ != nullptr && battery_ != nullptr)
      {
        stage_ = FIRST_RCIN;
      }
      break;
    }

    case FIRST_RCIN:
    {
      if (!rcin->e_stop)
      {
        rosInfoThrottle(
          kInfoPeriod, name_, "Please start with the transmitter's E-Stop toggle ON.");
      }
      else
      {
        rosInfo(name_, "RC transmitter is ready. Set E-Stop toggle OFF to start control.");
        stage_ = ESTOP_ON;
      }
      break;
    }

    case ESTOP_ON:
    {
      if (!rcin->e_stop)
      {
        stage_ = RUNNING;
      }
      break;
    }

    case RUNNING:
    {
      if (rcin->e_stop)
      {
        rosWarn(name_, "Emergency stop requested. Shutting down the system.");
        const auto event = boost::make_shared<tobas_msgs::Event>();
        event->data = tobas_msgs::Event::STOP;
        event_pub_.publish(event);
      }

      const auto cur_mode = static_cast<int>(rcin->mode);

      if (cur_mode >= controllers_.size())
      {
        rosErrorThrottle(
          kErrorPeriod, name_,
          "You tried to set flight mode " << cur_mode << ", which is out of range.");
        return;
      }

      if (cur_mode != last_mode_)
      {
        controllers_[cur_mode]->reset(*odom_);
        rosInfo(name_, "Command type changed from " << last_mode_ << " to " << cur_mode << ".");
        last_mode_ = cur_mode;
        break;
      }

      controllers_[cur_mode]->update(*rcin, *odom_, battery_->voltage, dead_zone_);

      break;
    }

    default:
    {
      ROS_THROW_NAMED(name_, "Invalid state: " << static_cast<int>(stage_));
    }
  }
}
}  // namespace tobas_rc_teleop
