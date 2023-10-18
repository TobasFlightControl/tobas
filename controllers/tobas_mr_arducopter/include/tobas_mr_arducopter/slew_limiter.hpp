#pragma once

#include <stdint.h>

#include "./low_pass_filter.hpp"

namespace tobas_mr_arducopter
{
/**
 * @brief slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum slew rate
 */
class SlewLimiter
{
  // number of positive and negative consecutive slew rate exceedance events recorded where a value
  // of 2 corresponds to a complete cycle
  static constexpr uint32_t kSlewLimiterNumEvents = 2;

public:
  explicit SlewLimiter(const double& slew_rate_max, const double& slew_rate_tau);

  /* apply filter to sample, returning multiplier between 0 and 1 to keep output within slew rate */
  double modifier(double sample, double dt, uint32_t now_ms);

  /* get last oscillation slew rate */
  inline double getSlewRate() const;

private:
  const double& slew_rate_max_;
  const double& slew_rate_tau_;
  LowPassFilter slew_filter_;
  double output_skew_rate_;
  double modifier_slew_rate_;
  double last_sample_;
  double max_pos_slew_rate_;
  double max_neg_slew_rate_;
  uint32_t max_pos_slew_event_ms_;
  uint32_t max_neg_slew_event_ms_;
  uint8_t pos_event_index_;
  uint8_t neg_event_index_;
  uint32_t pos_event_ms_[kSlewLimiterNumEvents];
  uint32_t neg_event_ms_[kSlewLimiterNumEvents];
  bool pos_event_stored_;
  bool neg_event_stored_;
};

inline double SlewLimiter::getSlewRate() const
{
  return output_skew_rate_;
}
}  // namespace tobas_mr_arducopter
