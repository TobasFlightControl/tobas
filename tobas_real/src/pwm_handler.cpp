#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_real/pwm_handler.hpp"

using namespace std;

namespace tobas_real
{
PwmHandler::PwmHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  // パルスが出力され始めたらexportを受け付けなくなるため，最初に全部やってしまう
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
    pwm_.initialize(channel);

  getRosParams();
  registerPublishers();
  registerSubscribers();
  registerServiceServers();
}

PwmHandler::~PwmHandler()
{
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
  {
    // PWMが有効化されていたら無効化する
    // unexportは不安定なので行わない
    if (pwm_states_.at(channel).is_enabled)
    {
      if (!pwm_.disable(channel))
        rosError(name_, "Failed to disable PWM CH" << channel << ".");
    }
  }
}

void PwmHandler::getRosParams()
{
}

void PwmHandler::registerPublishers()
{
}

void PwmHandler::registerSubscribers()
{
  pwms_sub_ = nh_.subscribe(tobas::kPwmCmdTopic, 1, &self::pwmsCb, this, tcpNoDelay());
}

void PwmHandler::registerServiceServers()
{
  setup_pwm_srv_ = nh_.advertiseService(tobas::kSetupPwmSrv, &self::setupPwmCb, this);
  enable_pwm_srv_ = nh_.advertiseService(tobas::kEnablePwmSrv, &self::enablePwmCb, this);
}

void PwmHandler::pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms)
{
  // PWMのデューティサイクルを更新
  for (const auto& pwm : pwms->pwm)
  {
    if (pwm.channel >= kServoRailSize)
    {
      rosError(name_, "PWM CH" << pwm.channel << " does not exist.");
      continue;
    }

    if (!pwm_states_.at(pwm.channel).is_enabled)
    {
      rosError(name_, "PWM CH" << pwm.channel << " is disabled.");
      continue;
    }

    if (!pwm_.setDutyCycle(pwm.channel, pwm.period))
      rosFatal(name_, "Failed to set PWM duty cycle on CH" << pwm.channel << ".");
  }
}

bool PwmHandler::setupPwmCb(tobas_msgs::SetupPwmRequest& req, tobas_msgs::SetupPwmResponse& res)
{
  res.success = false;

  if (req.channel >= kServoRailSize)
  {
    rosError(name_, "PWM channel out of range.");
    return true;
  }

  // Set frequency
  if (!pwm_.setFrequency(req.channel, req.frequency))
  {
    rosError(name_, "Failed to set frequency of PWM CH" << req.channel << ".");
    return true;
  }

  // Enable
  if (!pwm_.enable(req.channel))
  {
    rosError(name_, "Failed to enable PWM CH" << req.channel << ".");
    return true;
  }

  pwm_states_.at(req.channel).is_initialized = true;
  res.success = true;
  return true;
}

bool PwmHandler::enablePwmCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res)
{
  res.success = false;

  if (req.channel >= kServoRailSize)
  {
    rosError(name_, "PWM channel out of range.");
    return true;
  }

  if (!pwm_states_.at(req.channel).is_initialized)
  {
    rosError(name_, "PWM CH" << req.channel << " is not initialized.");
    return true;
  }

  if (req.enable)
  {
    if (!pwm_.enable(req.channel))
    {
      rosError(name_, "Failed to enable PWM CH" << req.channel << ".");
      return true;
    }
    pwm_states_.at(req.channel).is_enabled = true;
  }
  else
  {
    if (!pwm_.disable(req.channel))
    {
      rosError(name_, "Failed to disable PWM CH" << req.channel << ".");
      return true;
    }
    pwm_states_.at(req.channel).is_enabled = false;
  }

  res.success = true;
  return true;
}
}  // namespace tobas_real
