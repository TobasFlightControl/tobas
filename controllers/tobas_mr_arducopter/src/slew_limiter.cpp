#include <cmath>

#include "../include/tobas_mr_arducopter/slew_limiter.hpp"

// time in msec required for a half cycle of the slowest oscillation frequency expected
#define WINDOW_MS 300
#define MODIFIER_GAIN 1.5  // ratio of modifier reduction to slew rate exceedance ratio
#define DERIVATIVE_CUTOFF_FREQ 25.

using namespace std;

namespace tobas_mr_arducopter
{
SlewLimiter::SlewLimiter(const double& _slew_rate_max, const double& _slew_rate_tau)
  : slew_rate_max_(_slew_rate_max), slew_rate_tau_(_slew_rate_tau)
{
  slew_filter_.setCutoffFrequency(DERIVATIVE_CUTOFF_FREQ);
  slew_filter_.reset(0);
}

double SlewLimiter::modifier(double sample, double dt, uint32_t now_ms)
{
  if (dt <= 0)
  {
    return 1;
  }

  // Calculate a low pass filtered slew rate
  const double slew_rate = slew_filter_.apply((sample - last_sample_) / dt, dt);
  last_sample_ = sample;

  // Apply a filter to decay maximum seen slew rate once the value had left the window period
  const double decay_alpha = min(dt, slew_rate_tau_) / slew_rate_tau_;

  // Apply a filter to increases in slew rate only to reduce the effect of gusts and large
  // controller setpoint changes
  const double attack_alpha = min(2. * decay_alpha, 1.);

  // Decay the peak positive and negative slew rate if they are outside the window
  // Never drop PID gains below 10% of configured value
  if (slew_rate > max_pos_slew_rate_)
  {
    max_pos_slew_rate_ = slew_rate;
    max_pos_slew_event_ms_ = now_ms;
  }
  else if (now_ms - max_pos_slew_event_ms_ > WINDOW_MS)
  {
    max_pos_slew_rate_ *= (1 - decay_alpha);
  }

  if (-slew_rate > max_neg_slew_rate_)
  {
    max_neg_slew_rate_ = -slew_rate;
    max_neg_slew_event_ms_ = now_ms;
  }
  else if (now_ms - max_neg_slew_event_ms_ > WINDOW_MS)
  {
    max_neg_slew_rate_ *= (1 - decay_alpha);
  }

  const double raw_slew_rate = 0.5 * (max_pos_slew_rate_ + max_neg_slew_rate_);
  output_skew_rate_ = (1. - attack_alpha) * output_skew_rate_ + attack_alpha * raw_slew_rate;
  output_skew_rate_ = min(output_skew_rate_, raw_slew_rate);

  if (slew_rate_max_ <= 0)
  {
    return 1;
  }

  // Constrain slew rate used for calculation
  const double limited_raw_slew_rate =
    0.5
    * (min(max_pos_slew_rate_, 10 * slew_rate_max_) + min(max_neg_slew_rate_, 10 * slew_rate_max_));

  // Store a series of positive slew rate exceedance events
  if (!pos_event_stored_ && slew_rate > slew_rate_max_)
  {
    if (pos_event_index_ >= kSlewLimiterNumEvents)
    {
      pos_event_index_ = 0;
    }
    pos_event_ms_[pos_event_index_] = now_ms;
    pos_event_index_++;
    pos_event_stored_ = true;
    neg_event_stored_ = false;
  }

  // Store a series of negative slew rate exceedance events
  if (!neg_event_stored_ && -slew_rate > slew_rate_max_)
  {
    if (neg_event_index_ >= kSlewLimiterNumEvents)
    {
      neg_event_index_ = 0;
    }
    neg_event_ms_[neg_event_index_] = now_ms;
    neg_event_index_++;
    neg_event_stored_ = true;
    pos_event_stored_ = false;
  }

  // Find the oldest event time
  uint32_t oldest_ms = now_ms;
  for (uint8_t index = 0; index < kSlewLimiterNumEvents; index++)
  {
    oldest_ms = min(oldest_ms, pos_event_ms_[index]);
    oldest_ms = min(oldest_ms, neg_event_ms_[index]);
  }

  // Apply a further reduction when the oldest exceedance event falls outside the window required
  // for the specified number of exceedance events. This prevents spikes due to control mode
  // changed, etc causing unwanted gain reduction and is only applied to the slew rate used for gain
  // reduction
  double modifier_input = limited_raw_slew_rate;
  if (now_ms - oldest_ms > (kSlewLimiterNumEvents + 1) * WINDOW_MS)
  {
    const double oldest_time_from_window =
      1e-3 * (double)(now_ms - oldest_ms - (kSlewLimiterNumEvents + 1) * WINDOW_MS);
    modifier_input *= expf(-oldest_time_from_window / slew_rate_tau_);
  }

  modifier_slew_rate_ = (1. - attack_alpha) * modifier_slew_rate_ + attack_alpha * modifier_input;
  modifier_slew_rate_ = min(modifier_slew_rate_, modifier_input);

  // Calculate the gain adjustment
  double mod = 1.;
  if (modifier_slew_rate_ > slew_rate_max_)
  {
    mod =
      slew_rate_max_ / (slew_rate_max_ + MODIFIER_GAIN * (modifier_slew_rate_ - slew_rate_max_));
  }

  return mod;
}
}  // namespace tobas_mr_arducopter
