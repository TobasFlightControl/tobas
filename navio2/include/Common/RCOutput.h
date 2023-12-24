#pragma once

#include <cstddef>

class RCOutput
{
public:
  virtual bool initialize(const size_t& channel) = 0;
  virtual bool enable(const size_t& channel) = 0;
  virtual bool setFrequency(const size_t& channel, const size_t& frequency) = 0;
  virtual bool setDutyCycle(const size_t& channel, const double& period_us) = 0;
};
