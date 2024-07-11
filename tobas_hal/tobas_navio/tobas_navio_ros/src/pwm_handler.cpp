#include <tobas_math/core.hpp>

#include "../include/tobas_navio_ros/pwm_handler.hpp"

using namespace std;

namespace tobas_navio_ros
{
PwmHandler::PwmHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  // パルスが出力され始めたらexportを受け付けなくなるため，最初に全部やってしまう
  // 周波数は固定
  for (size_t channel = 0; channel < navio::PWM::kChannelCount; ++channel)
  {
    if (!pwm_.initialize(channel))
      TOBAS_EXIT("Failed to initialize PWM CH", channel, ".");
    if (!pwm_.setFrequency(channel, kPwmFrequency))
      TOBAS_EXIT("Failed to set frequency of PWM CH", channel, ".");
  }

  throttles_sub_ = nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCb, this, tcpNoDelay());
  enable_pwm_srv_ = nh_.advertiseService(tobas::kEnablePwmSrv, &self::enablePwmCb, this);
}

PwmHandler::~PwmHandler()
{
  for (size_t channel = 0; channel < navio::PWM::kChannelCount; ++channel)
  {
    // PWMが有効化されていたら無効化する
    // unexportは不安定なので行わない
    if (is_enabled_.at(channel))
    {
      if (!pwm_.disable(channel))
        TOBAS_ERROR("Failed to disable PWM CH", channel, ".");
    }
  }
}

void PwmHandler::throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles)
{
  // PWMのデューティサイクルを更新
  for (const auto& throttle : throttles->throttles)
  {
    if (throttle.channel >= navio::PWM::kChannelCount)
    {
      TOBAS_ERROR("PWM CH", throttle.channel, " does not exist.");
      continue;
    }

    if (!is_enabled_.at(throttle.channel))
    {
      TOBAS_ERROR("PWM CH", throttle.channel, " is disabled.");
      continue;
    }

    const auto period =
      math::remap<double>(throttle.throttle, tobas::kMinThrottle, tobas::kMaxThrottle, tobas::kPwmMin, tobas::kPwmMax);
    if (!pwm_.setDutyCycle(throttle.channel, period))
      TOBAS_FATAL("Failed to set PWM duty cycle on CH", throttle.channel, ".");
  }
}

bool PwmHandler::enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res)
{
  if (req.channel >= navio::PWM::kChannelCount)
  {
    TOBAS_ERROR("PWM channel out of range.");
    res.success = false;
    return true;
  }

  if (req.enable)
  {
    if (!pwm_.enable(req.channel))
    {
      TOBAS_ERROR("Failed to enable PWM CH", req.channel, ".");
      res.success = false;
      return true;
    }
    is_enabled_.at(req.channel) = true;
  }
  else
  {
    if (!pwm_.disable(req.channel))
    {
      TOBAS_ERROR("Failed to disable PWM CH", req.channel, ".");
      res.success = false;
      return true;
    }
    is_enabled_.at(req.channel) = false;
  }

  res.success = true;
  return true;
}
}  // namespace tobas_navio_ros
