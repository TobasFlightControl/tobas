#pragma once

#include <cinttypes>

class PWM
{
public:
  explicit PWM();

  bool initialize(const size_t& channel);
  bool enable(const size_t& channel);
  bool disable(const size_t& channel);
  bool setFrequency(const size_t& channel, const size_t& freq);
  bool setDutyCycle(const size_t& channel, const double& period_us);
};
