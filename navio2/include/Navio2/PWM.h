#pragma once

#include <cinttypes>

class PWM
{
public:
  explicit PWM();

  bool init(const size_t& channel);
  bool enable(const size_t& channel);
  bool setPeriod(const size_t& channel, const size_t& freq);
  bool setDutyCycle(const size_t& channel, const double& period_ms);
};
