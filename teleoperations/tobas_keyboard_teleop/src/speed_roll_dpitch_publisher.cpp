#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_keyboard_teleop/speed_roll_dpitch_publisher.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
SpeedRollDeltaPitchPublisher::SpeedRollDeltaPitchPublisher(
  ros::NodeHandle nh,
  ros::NodeHandle pnh,
  string name)
  : super(nh, pnh, name),
    trim_(drone_),
    keyboard_(getKeyboardControls()),
    check_topics_timer_(
      nh_,
      tobas::kCheckTopicsTimerPeriod,
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
                 "W/S       : Increase/Decrease speed\n"
                 "Up/Down   : Nose up/down\n"
                 "Left/Right: Turn left/right\n"
                 "Ctrl-C    : Quit\n";

  getRosParams();
  drone_.loadFromParam(nh_);

  trim_.updateInternalDataStructures();
  q_0_.resize(drone_.tree().getNrOfJoints());

  const auto repeat_interval = keyboard_->repeat_interval * 1e-3;  // ms -> s
  rosInfo(name_, "Keyboard repeat interval is " << keyboard_->repeat_interval << " [ms].");

  delta_speed_ = max_linacc_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  registerPublishers();
  registerSubscribers();
}

void SpeedRollDeltaPitchPublisher::run()
{
  check_topics_timer_.start();

  dh_ros::Rate rate(kUpdateRate);
  while (nh_.ok())
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

    // コマンドを更新
    const auto c = key_reader_.readKey();
    switch (c)
    {
      case kKeyCode_W:
      {
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed + delta_speed_);
        rosInfoThrottle(kInfoPeriod, name_, "Increase speed");
        break;
      }
      case kKeyCode_S:
      {
        cmd_.speed = trim_.speedLimit(air_density_).clamp(cmd_.speed - delta_speed_);
        rosInfoThrottle(kInfoPeriod, name_, "Decrease speed");
        break;
      }
      case kKeyCode_Up:
      {
        cmd_.delta_pitch =
          dh_std::clamp(cmd_.delta_pitch - delta_rot_, -max_delta_pitch_, max_delta_pitch_);
        rosInfoThrottle(kInfoPeriod, name_, "Nose up");
        break;
      }
      case kKeyCode_Down:
      {
        cmd_.delta_pitch =
          dh_std::clamp(cmd_.delta_pitch + delta_rot_, -max_delta_pitch_, max_delta_pitch_);
        rosInfoThrottle(kInfoPeriod, name_, "Nose down");
        break;
      }
      case kKeyCode_Left:
      {
        cmd_.roll = dh_std::clamp(cmd_.roll - delta_rot_, -max_roll_, max_roll_);
        rosInfoThrottle(kInfoPeriod, name_, "Turn left");
        break;
      }
      case kKeyCode_Right:
      {
        cmd_.roll = dh_std::clamp(cmd_.roll + delta_rot_, -max_roll_, max_roll_);
        rosInfoThrottle(kInfoPeriod, name_, "Turn right");
        break;
      }
    }

    // コマンドを発行
    const auto cmd_ptr = boost::make_shared<tobas_msgs::SpeedRollDeltaPitch>(cmd_);
    cmd_pub_.publish(cmd_ptr);

    ros::spinOnce();
    rate.sleep();
  }
}

void SpeedRollDeltaPitchPublisher::getRosParams()
{
  dh_ros::getParam(
    pnh_, "max_linear_acceleration", max_linacc_, kDefaultMaxLinearAcceleration, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "max_angular_velocity", max_angvel_, kDefaultMaxAngularVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "maximum_roll", max_roll_, kDefaultMaximumRoll, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "maximum_delta_pitch", max_delta_pitch_, kDefaultMaximumDeltaPitch, dh_ros::POSITIVE);
}

void SpeedRollDeltaPitchPublisher::registerPublishers()
{
  cmd_pub_ = nh_.advertise<tobas_msgs::SpeedRollDeltaPitch>("command/speed_roll_delta_pitch", 1);
}

void SpeedRollDeltaPitchPublisher::registerSubscribers()
{
  event_sub_ =
    nh_.subscribe("event", 1, &SpeedRollDeltaPitchPublisher::eventCb, this, tcpNoDelay());
  air_pressure_sub_ = nh_.subscribe(
    "air_pressure", 1, &SpeedRollDeltaPitchPublisher::airPressureCb, this, tcpNoDelay());
}

bool SpeedRollDeltaPitchPublisher::isReady()
{
  return pressure_received_;
}

void SpeedRollDeltaPitchPublisher::initialize()
{
  // cmd_.speed = trim_.speedLimit(air_density_).lower + 0.1;
  cmd_.speed = trim_.takeOffSpeed(air_density_);

  // インストラクションを開始
  instruction_timer_.start();
  rosInfo(name_, instruction_);
}

void SpeedRollDeltaPitchPublisher::eventCb(const tobas_msgs::EventConstPtr& event)
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

void SpeedRollDeltaPitchPublisher::airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg)
{
  air_density_ = dh_std::pressureToDensity(msg->fluid_pressure);

  if (!pressure_received_)
  {
    pressure_received_ = true;
  }
}

void SpeedRollDeltaPitchPublisher::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pressure_received_)
  {
    rosWarn(name_, "Air pressure is not received yet.");
  }
}

void SpeedRollDeltaPitchPublisher::instructionTimerCb(const ros::TimerEvent&)
{
  rosInfo(name_, instruction_);
}
}  // namespace tobas_keyboard_teleop
