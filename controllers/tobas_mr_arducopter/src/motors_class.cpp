#include "../include/tobas_mr_arducopter/motors_class.hpp"

#define AP_MOTORS_SLEW_FILTER_CUTOFF 50.

namespace tobas_mr_arducopter
{
// Constructor
Motors::Motors()
  : _throttle_filter(),
    _throttle_slew_filter(),
    _spool_desired(DesiredSpoolState::SHUT_DOWN),
    _spool_state(SpoolState::SHUT_DOWN),
    _air_density_ratio(1.)
{
  // setup throttle filtering
  _throttle_filter.setCutoffFrequency(0.);
  _throttle_filter.reset(0.);

  _throttle_slew_filter.setCutoffFrequency(AP_MOTORS_SLEW_FILTER_CUTOFF);
  _throttle_slew_filter.reset(0.);

  // init limit flags
  limit.roll = true;
  limit.pitch = true;
  limit.yaw = true;
  limit.throttle_lower = true;
  limit.throttle_upper = true;
  _thrust_boost = false;
  _thrust_balanced = true;
}

uint8_t Motors::get_lost_motor() const
{
  return 0;
}

void Motors::set_desired_spool_state(DesiredSpoolState spool)
{
  if (_armed || (spool == DesiredSpoolState::SHUT_DOWN))
  {
    _spool_desired = spool;
  }
}

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void Motors::set_radio_passthrough(
  double roll_input,
  double pitch_input,
  double throttle_input,
  double yaw_input)
{
  _roll_radio_passthrough = roll_input;
  _pitch_radio_passthrough = pitch_input;
  _throttle_radio_passthrough = throttle_input;
  _yaw_radio_passthrough = yaw_input;
}

double Motors::get_roll_factor(uint8_t i)
{
  return 0.;
}

double Motors::get_pitch_factor(uint8_t i)
{
  return 0.;
}

bool Motors::is_motor_enabled(uint8_t i)
{
  return false;
}

bool Motors::init_targets_on_arming() const
{
  return true;
}

void Motors::set_limit_flag_pitch_roll_yaw(bool flag)
{
  limit.roll = flag;
  limit.pitch = flag;
  limit.yaw = flag;
}
}  // namespace tobas_mr_arducopter
