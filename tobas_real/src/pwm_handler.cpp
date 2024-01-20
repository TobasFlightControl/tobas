#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_real/pwm_handler.hpp"

using namespace std;

namespace tobas_real
{
PwmHandler::PwmHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
  registerServiceServers();
}

PwmHandler::~PwmHandler()
{
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
  {
    const auto& pwm_state = pwm_states_.at(channel);

    if (pwm_state.enabled)
      pwm_.disable(channel);

    if (pwm_state.exported)
      pwm_.remove(channel);
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
  initialize_srv_ = nh_.advertiseService(tobas::kInitializePwmSrv, &self::initializeCb, this);
  enable_srv_ = nh_.advertiseService(tobas::kEnablePwmSrv, &self::enableCb, this);
  set_freq_srv_ = nh_.advertiseService(tobas::kSetPwmFreqSrv, &self::setFreqCb, this);
}

void PwmHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

    auto& pwm_state = pwm_states_.at(pwm.channel);

    if (!pwm_state.exported)
    {
      rosError(name_, "PWM CH" << pwm.channel << " is not exported.");
      continue;
    }

    if (!pwm_state.enabled)
    {
      rosError(name_, "PWM CH" << pwm.channel << " is not enabled.");
      continue;
    }

    pwm_state.period = pwm.period;
  }

  // 有効化されている全てのPWMを発行
  for (size_t channel = 0; channel < kServoRailSize; ++channel)
  {
    const auto& pwm_state = pwm_states_.at(channel);

    if (!pwm_state.exported || !pwm_state.enabled)
      continue;

    if (!pwm_.setDutyCycle(channel, pwm_state.period))
      rosFatal(name_, "Failed to set PWM duty cycle on CH" << channel << ".");
  }
}

bool PwmHandler::initializeCb(
  tobas_msgs::InitializePwmRequest& req,
  tobas_msgs::InitializePwmResponse& res)
{
  res.success = pwm_.initialize(req.channel);
  return true;
}

bool PwmHandler::enableCb(tobas_msgs::EnablePwmRequest& req, tobas_msgs::EnablePwmResponse& res)
{
  if (req.enable)
    res.success = pwm_.enable(req.channel);
  else
    res.success = pwm_.disable(req.channel);
  return true;
}

bool PwmHandler::setFreqCb(
  tobas_msgs::SetPwmFrequencyRequest& req,
  tobas_msgs::SetPwmFrequencyResponse& res)
{
  res.success = pwm_.setFrequency(req.channel, req.frequency);
  return true;
}
}  // namespace tobas_real
