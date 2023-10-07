#include <dh_std_tools/math.hpp>
#include <dh_std_tools/string.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/AccelerationYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

#include "../include/tobas_rc_teleop/rc_teleop.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace ros::message_traits;
using namespace dh_std;

namespace tobas_rc_teleop
{
RCTeleop::RCTeleop(ros::NodeHandle nh, ros::NodeHandle pnh, string name) : super(nh, pnh, name)
{
  getRosParams();

  dead_zone_.lower = -dead_zone_rate_ / 2;
  dead_zone_.upper = dead_zone_rate_ / 2;

  for (const auto& mode_name : mode_names_)
  {
    if (mode_name == split(DataType<tobas_msgs::PosVelAccYaw>::value(), '/').back())
    {
      mode2cmd_.push_back(POS_VEL_ACC_YAW);
      pvay_ctrl_.initialize(nh, pnh);
    }
    else if (mode_name == split(DataType<tobas_msgs::PositionYaw>::value(), '/').back())
    {
      mode2cmd_.push_back(POSITION_YAW);
      pos_yaw_ctrl_.initialize(nh, pnh);
    }
    else if (mode_name == split(DataType<tobas_msgs::VelocityYaw>::value(), '/').back())
    {
      mode2cmd_.push_back(VELOCITY_YAW);
      vel_yaw_ctrl_.initialize(nh, pnh);
    }
    else if (mode_name == split(DataType<tobas_msgs::AccelerationYaw>::value(), '/').back())
    {
      mode2cmd_.push_back(ACCELERATION_YAW);
      rosError(name_, "Not implemented yet.");  // TODO
    }
    else if (mode_name == split(DataType<tobas_msgs::RollPitchYawThrust>::value(), '/').back())
    {
      mode2cmd_.push_back(RPY_THRUST);
      rpy_thrust_ctrl_.initialize(nh, pnh);
    }
    else if (mode_name == split(DataType<tobas_msgs::SpeedRollDeltaPitch>::value(), '/').back())
    {
      mode2cmd_.push_back(SPEED_ROLL_DPITCH);
      rosError(name_, "Not implemented yet.");  // TODO
    }
    else
    {
      throw runtime_error("Invalid flight mode: " + mode_name);
    }
  }

  registerPublishers();
  registerSubscribers();
}

void RCTeleop::getRosParams()
{
  dh_ros::getParam(
    pnh_, "dead_zone_rate", dead_zone_rate_, kDefaultDeadZoneRate, dh_ros::NON_NEGATIVE);
  if (dead_zone_rate_ >= 1.)
  {
    rosthrow(name_, "'dead_zone_rate' must be lower than 1.");
  }

  dh_ros::getParam(pnh_, "mode_names", mode_names_);
}

void RCTeleop::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>(tobas::kEventTopic, 1);
}

void RCTeleop::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &RCTeleop::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &RCTeleop::poseTwistCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &RCTeleop::batteryCb, this, tcpNoDelay());
  rcin_sub_ = nh_.subscribe(tobas::kRcInputTopic, 1, &RCTeleop::rcInputCb, this, tcpNoDelay());
}

void RCTeleop::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void RCTeleop::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;
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
      if (pt_ != nullptr && battery_ != nullptr)
      {
        stage_ = FIRST_RCIN;
      }
      break;
    }

    case FIRST_RCIN:
    {
      if (!rcin->e_stop)
      {
        rosErrorThrottle(
          kErrorPeriod, name_, "Please start with the transmitter's E-Stop toggle ON.");
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
        auto event = boost::make_shared<tobas_msgs::Event>();
        event->data = tobas_msgs::Event::SHUTDOWN;
        event_pub_.publish(event);
        nh_.shutdown();
      }

      const auto& cmd_type = mode2cmd_[rcin->mode];

      if (cmd_type != last_cmd_type_)
      {
        switch (cmd_type)
        {
          case POS_VEL_ACC_YAW:
            pvay_ctrl_.reset(*pt_);
            break;
          case POSITION_YAW:
            pos_yaw_ctrl_.reset(*pt_);
            break;
          case VELOCITY_YAW:
            vel_yaw_ctrl_.reset(*pt_);
            break;
          case ACCELERATION_YAW:
            rosErrorThrottle(kErrorPeriod, name_, "Not implemented yet.");  // TODO
            break;
          case RPY_THRUST:
            rpy_thrust_ctrl_.reset(*pt_);
            break;
          case SPEED_ROLL_DPITCH:
            rosErrorThrottle(kErrorPeriod, name_, "Not implemented yet.");  // TODO
            break;
          default:
            throw runtime_error("Invalid command type: " + to_string(cmd_type));
        }

        rosInfo(name_, "Command type changed from " << last_cmd_type_ << " to " << cmd_type << ".");
        last_cmd_type_ = cmd_type;
        break;
      }

      switch (cmd_type)
      {
        case POS_VEL_ACC_YAW:
          pvay_ctrl_.update(*rcin, dead_zone_);
          break;
        case POSITION_YAW:
          pos_yaw_ctrl_.update(*rcin, dead_zone_);
          break;
        case VELOCITY_YAW:
          vel_yaw_ctrl_.update(*rcin, dead_zone_);
          break;
        case ACCELERATION_YAW:
          rosErrorThrottle(kErrorPeriod, name_, "Not implemented yet.");  // TODO
          break;
        case RPY_THRUST:
          rpy_thrust_ctrl_.update(*rcin, battery_->voltage, dead_zone_);
          break;
        case SPEED_ROLL_DPITCH:
          rosErrorThrottle(kErrorPeriod, name_, "Not implemented yet.");  // TODO
          break;
        default:
          throw runtime_error("Invalid command type: " + to_string(cmd_type));
      }

      break;
    }

    default:
    {
      rosthrow(name_, "Invalid state: " << static_cast<int>(stage_));
    }
  }
}
}  // namespace tobas_rc_teleop
