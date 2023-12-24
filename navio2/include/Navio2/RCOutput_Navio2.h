#pragma once

#include "../Common/RCOutput.h"
#include "./PWM.h"

class RCOutput_Navio2 : public RCOutput
{
public:
  inline explicit RCOutput_Navio2();

  inline bool initialize(const size_t& channel) override;
  inline bool enable(const size_t& channel) override;
  inline bool setFrequency(const size_t& channel, const size_t& frequency) override;
  inline bool setDutyCycle(const size_t& channel, const double& period_us) override;

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

inline bool RCOutput_Navio2::setFrequency(const size_t& channel, const size_t& frequency)
{
  return pwm_.setPeriod(channel, frequency);
}

inline bool RCOutput_Navio2::setDutyCycle(const size_t& channel, const double& period_us)
{
  return pwm_.setDutyCycle(channel, period_us / 1000);
}
