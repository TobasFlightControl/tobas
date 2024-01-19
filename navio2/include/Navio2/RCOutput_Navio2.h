#pragma once

#include "./PWM.h"

class RCOutput_Navio2
{
public:
  inline explicit RCOutput_Navio2();

  inline bool initialize(const size_t& channel);
  inline bool enable(const size_t& channel);
  inline bool disable(const size_t& channel);
  inline bool setFrequency(const size_t& channel, const size_t& frequency);
  inline bool setDutyCycle(const size_t& channel, const double& period_us);

private:
  PWM pwm_;
};

inline RCOutput_Navio2::RCOutput_Navio2()
{
}

inline bool RCOutput_Navio2::initialize(const size_t& channel)
{
  return pwm_.init(channel);
}

inline bool RCOutput_Navio2::enable(const size_t& channel)
{
  return pwm_.enable(channel);
}

inline bool RCOutput_Navio2::disable(const size_t& channel)
{
  return pwm_.disable(channel);
}

inline bool RCOutput_Navio2::setFrequency(const size_t& channel, const size_t& frequency)
{
  return pwm_.setFrequency(channel, frequency);
}

inline bool RCOutput_Navio2::setDutyCycle(const size_t& channel, const double& period_us)
{
  return pwm_.setDutyCycle(channel, period_us / 1000);
}
