#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"
#include "../../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
SpeedRollDeltaPitchPublisher::SpeedRollDeltaPitchPublisher()
  : super(),
    trim_(drone_),
    keyboard_(getKeyboardControls()),
    is_initialized_(false),
    pressure_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &SpeedRollDeltaPitchPublisher::checkTopicsTimerCb,
      this,
      false),
    instruction_timer_(
      nh_,
      kInstructionTimerPeriod,
      &SpeedRollDeltaPitchPublisher::instructionTimerCb,
      this,
      false)
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S    : Increase/Decrease speed\n"
                 "A/D    : Turn left/right\n"
                 "Up/Down: Nose up/down\n"
                 "Ctrl-C : Quit\n";

  getRosParams();
  drone_.loadFromParam(ns_);

  trim_.updateInternalDataStructures();
  q_0_.resize(drone_.tree().getNrOfJoints());

  const auto repeat_interval = keyboard_->repeat_interval * 1e-3;  // ms -> s
  rosInfo("Keyboard repeat interval is " << keyboard_->repeat_interval << " [ms].");

  delta_speed_ = max_linacc_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  registerPublishers();
  registerSubscribers();
}

void SpeedRollDeltaPitchPublisher::run()
{
  check_topics_timer_.start();
  instruction_timer_.start();
  rosInfo(instruction_);

  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    if (!is_initialized_)
    {
      if (isReady())
      {
        initialize();
        is_initialized_ = true;
      }
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    trim_.update(cmd_.speed, air_density_, q_0_);

    const auto c = key_reader_.readKey();
    switch (c)
    {
      case kKeyCode_W:
      {
        rosInfoThrottle(kInfoPeriod, "Increase speed");
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed + delta_speed_);
        break;
      }
      case kKeyCode_S:
      {
        rosInfoThrottle(kInfoPeriod, "Decrease speed");
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed - delta_speed_);
        break;
      }
      case kKeyCode_A:
      {
        rosInfoThrottle(kInfoPeriod, "Turn left");
        cmd_.roll = dh_std::clamp(cmd_.roll - delta_rot_, -max_roll_, max_roll_);
        break;
      }
      case kKeyCode_D:
      {
        rosInfoThrottle(kInfoPeriod, "Turn right");
        cmd_.roll = dh_std::clamp(cmd_.roll + delta_rot_, -max_roll_, max_roll_);
        break;
      }
      case kKeyCode_Up:
      {
        rosInfoThrottle(kInfoPeriod, "Nose up");
        cmd_.delta_pitch =
          dh_std::clamp(cmd_.delta_pitch - delta_rot_, -max_delta_pitch_, max_delta_pitch_);
        break;
      }
      case kKeyCode_Down:
      {
        rosInfoThrottle(kInfoPeriod, "Nose down");
        cmd_.delta_pitch =
          dh_std::clamp(cmd_.delta_pitch + delta_rot_, -max_delta_pitch_, max_delta_pitch_);

        break;
      }
    }

    cmd_pub_.publish(cmd_);

    ros::spinOnce();
    rate.sleep();
  }
}

void SpeedRollDeltaPitchPublisher::getRosParams()
{
  dh_ros::getParam(
    "~max_linear_acceleration", max_linacc_, kDefaultMaxLinearAcceleration, dh_ros::POSITIVE);
  dh_ros::getParam(
    "~max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity, dh_ros::POSITIVE);
  dh_ros::getParam("~maximum_roll", max_roll_, kDefaultMaximumRoll, dh_ros::POSITIVE);
  dh_ros::getParam(
    "~maximum_delta_pitch", max_delta_pitch_, kDefaultMaximumDeltaPitch, dh_ros::POSITIVE);
}

void SpeedRollDeltaPitchPublisher::registerPublishers()
{
  cmd_pub_ = nh_.advertise<tobas_msgs::SpeedRollDeltaPitch>("command/speed_roll_delta_pitch", 1);
}

void SpeedRollDeltaPitchPublisher::registerSubscribers()
{
  air_pressure_sub_ =
    nh_.subscribe("air_pressure", 1, &SpeedRollDeltaPitchPublisher::airPressureCb, this);
}

bool SpeedRollDeltaPitchPublisher::isReady()
{
  return pressure_received_;
}

void SpeedRollDeltaPitchPublisher::initialize()
{
  // cmd_.speed = trim_.speedLimit(air_density_).lower + 0.1;
  cmd_.speed = trim_.takeOffSpeed(air_density_);
}

void SpeedRollDeltaPitchPublisher::airPressureCb(const sensor_msgs::FluidPressure& msg)
{
  if (!pressure_received_)
  {
    pressure_received_ = true;
  }

  air_density_ = dh_std::pressureToDensity(msg.fluid_pressure);
}

void SpeedRollDeltaPitchPublisher::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pressure_received_)
  {
    rosWarn("Air pressure is not received yet.");
  }
}

void SpeedRollDeltaPitchPublisher::instructionTimerCb(const ros::TimerEvent&)
{
  rosInfo(instruction_);
}
}  // namespace tobas_keyboard_teleop
