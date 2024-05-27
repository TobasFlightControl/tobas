#include <tobas_ros_tools/rosparam.hpp>

#include "../include/tobas_navio_ros/pwm_handler.hpp"

using namespace std;

namespace tobas_navio_ros
{
PwmHandler::PwmHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  // パルスが出力され始めたらexportを受け付けなくなるため，最初に全部やってしまう
  // 周波数は固定
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!pwm_.initialize(channel))
      TOBAS_EXIT("Failed to initialize PWM CH", channel, ".");
    if (!pwm_.setFrequency(channel, kPwmFrequency))
      TOBAS_EXIT("Failed to set frequency of PWM CH", channel, ".");
  }

  pwms_sub_ = nh_.subscribe(tobas::kPwmCmdTopic, 1, &self::pwmsCb, this, tcpNoDelay());
  enable_pwm_srv_ = nh_.advertiseService(tobas::kEnablePwmSrv, &self::enablePwmCb, this);
}

PwmHandler::~PwmHandler()
{
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
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

void PwmHandler::pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms)
{
  // PWMのデューティサイクルを更新
  for (const auto& pwm : pwms->pwm)
  {
    if (pwm.channel >= kServoRailSize)
    {
      TOBAS_ERROR("PWM CH", pwm.channel, " does not exist.");
      continue;
    }

    if (!is_enabled_.at(pwm.channel))
    {
      TOBAS_ERROR("PWM CH", pwm.channel, " is disabled.");
      continue;
    }

    if (!pwm_.setDutyCycle(pwm.channel, pwm.period))
      TOBAS_FATAL("Failed to set PWM duty cycle on CH", pwm.channel, ".");
  }
}

bool PwmHandler::enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res)
{
  res.success = false;

  if (req.channel >= kServoRailSize)
  {
    TOBAS_ERROR("PWM channel out of range.");
    return true;
  }

  if (req.enable)
  {
    if (!pwm_.enable(req.channel))
    {
      TOBAS_ERROR("Failed to enable PWM CH", req.channel, ".");
      return true;
    }
    is_enabled_.at(req.channel) = true;
  }
  else
  {
    if (!pwm_.disable(req.channel))
    {
      TOBAS_ERROR("Failed to disable PWM CH", req.channel, ".");
      return true;
    }
    is_enabled_.at(req.channel) = false;
  }

  res.success = true;
  return true;
}
}  // namespace tobas_navio_ros
