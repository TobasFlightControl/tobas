#include <dh_std_tools/math.hpp>
#include <dh_std_tools/unix.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_real/common.hpp>

#include "../../include/tobas_motor_test/motors_handler.hpp"

using namespace std;
using namespace tobas_real;

namespace tobas_motor_test
{
MotorsHandler::MotorsHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  if (!dh_std::isSuperUser())
    ROS_THROW_NAMED(name_, "Please execute with root privileges.");

  getRosParams();

  // Setup PWM driver
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    setupRCOutput(pwm_, channel);

  // Send disarm command
  rosInfo(name_, "Sending disarm command for " << kDisarmDuration << " seconds.");
  sendDisarm();
  rosInfo(name_, "Disarming finished. The motors are ready to rotate.");

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &MotorsHandler::mainTimerCb, this);
}

void MotorsHandler::getRosParams()
{
}

void MotorsHandler::registerPublishers()
{
}

void MotorsHandler::registerSubscribers()
{
  super::registerSubscribers();

  throttles_sub_ =
    nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
}

void MotorsHandler::sendDisarm()
{
  const ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kDisarmDuration)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.setDutyCycle(channel, kPwmDisarm))
        rosError(name_, "Failed to set PWM duty cycle on PIN " << pinFromChannel(channel) << ".");
    }

    ros::Duration(kDisarmInterval).sleep();
  }
}

void MotorsHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

void MotorsHandler::throttlesCb(const tobas_msgs::ThrottlesConstPtr& throttles)
{
  throttles_ = throttles;
}

void MotorsHandler::mainTimerCb(const ros::TimerEvent&)
{
  uint32_t data_size = throttles_->data.size();

  // Check array size
  if (data_size > kServoRailSize)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The size of throttle data is "
        << data_size << ", which is larger than the size of servo rail " << kServoRailSize
        << ". The excess will be ignored.");
    data_size = kServoRailSize;
  }

  // 与えられたスロットルを指令
  for (uint32_t channel = 0; channel < min(data_size, kServoRailSize); ++channel)
  {
    const auto throttle = clamp(throttles_->data[channel], 0., 1.);
    const auto period = dh_std::remap<double>(throttle, 0., 1., kPwmMin, kPwmMax);
    if (!pwm_.setDutyCycle(channel, period))
      rosError(name_, "Failed to set PWM duty cycle on PIN " << pinFromChannel(channel) << ".");
  }

  // ESCの自動停止を防ぐため，足りない分は全て最小値を指令
  for (uint32_t channel = data_size; channel < kServoRailSize; ++channel)
  {
    if (!pwm_.setDutyCycle(channel, kPwmMin))
      rosError(name_, "Failed to set PWM duty cycle on PIN " << pinFromChannel(channel) << ".");
  }
}
}  // namespace tobas_motor_test
