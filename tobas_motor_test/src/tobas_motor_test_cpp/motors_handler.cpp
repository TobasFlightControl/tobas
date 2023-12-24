#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/unix.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_real/common.hpp>

#include "../../include/tobas_motor_test/motors_handler.hpp"

#define PWM_FAIL_ERROR(channel)                                                                    \
  {                                                                                                \
    ROS_ERROR_STREAM("Failed to set PWM duty cycle on PIN " << pinFromChannel(channel) << ".");    \
  }

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
  if (!tobas_std::isSuperUser())
    ROS_THROW_NAMED(name_, "Please execute with root privileges.");

  getRosParams();

  // Setup PWM driver
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
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
    for (size_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.setDutyCycle(channel, kPwmDisarm))
        PWM_FAIL_ERROR(channel);
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
  // コマンドが来るまでは，ESCの自動停止を防ぐために最小値を指令して終了
  if (throttles_ == nullptr)
  {
    rosInfoThrottle(kInfoPeriod, name_, "Waiting for " << tobas::kThrottlesCmdTopic);

    for (size_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.setDutyCycle(channel, kPwmMin))
        PWM_FAIL_ERROR(channel);
    }

    return;
  }

  rosInfoOnce(name_, "First throttle command is received.");

  // データサイズを決定
  size_t data_size = throttles_->data.size();
  if (data_size > kServoRailSize)
  {
    rosWarnThrottle(
      kInfoPeriod, name_,
      "The size of throttle data is "
        << data_size << ", which is larger than the size of servo rail " << kServoRailSize
        << ". The excess will be ignored.");
    data_size = kServoRailSize;
  }

  // 与えられたスロットルを指令
  for (size_t channel = 0; channel < data_size; ++channel)
  {
    const auto throttle = clamp(throttles_->data[channel], 0., 1.);
    const auto period = tobas_std::remap<double>(throttle, 0., 1., kPwmMin, kPwmMax);
    if (!pwm_.setDutyCycle(channel, period))
      PWM_FAIL_ERROR(channel);
  }

  // 足りない分は全て最小値を指令
  for (size_t channel = data_size; channel < kServoRailSize; ++channel)
  {
    if (!pwm_.setDutyCycle(channel, kPwmMin))
      PWM_FAIL_ERROR(channel);
  }
}
}  // namespace tobas_motor_test
