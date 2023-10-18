#pragma once

#include <algorithm>

#include "./low_pass_filter.hpp"

namespace tobas_mr_arducopter
{
class Motors
{
public:
  explicit Motors();

  // check initialisation succeeded
  virtual bool arming_checks(size_t buflen, char* buffer) const;
  // for 6DoF vehicles, sets the roll and pitch offset, this rotates the thrust vector in body frame
  virtual void set_roll_pitch(double roll_deg, double pitch_deg){};
  virtual double get_throttle_hover() const = 0;

  inline bool initialised_ok() const;
  inline void set_initialised_ok(bool val);

  // set motor interlock status
  inline void set_interlock(bool set);
  // get motor interlock status.  true means motors run, false motors don't run
  inline bool get_interlock() const;

  // get/set spoolup block
  inline bool get_spoolup_block() const;
  inline void set_spoolup_block(bool set);

  // set_roll, set_pitch, set_yaw, set_throttle
  inline void set_roll(double roll_in);
  inline void set_roll_ff(double roll_in);
  inline void set_pitch(double pitch_in);
  inline void set_pitch_ff(double pitch_in);
  inline void set_yaw(double yaw_in);
  inline void set_yaw_ff(double yaw_in);
  inline void set_throttle(double throttle_in);
  inline void set_throttle_avg_max(double throttle_avg_max);
  inline void set_throttle_filter_cutoff(double filt_hz);
  inline void set_slew_filter_cutoff(double filt_hz);
  inline void set_forward(double forward_in);
  inline void set_lateral(double lateral_in);

  // accessors for roll, pitch, yaw and throttle inputs to motors
  inline double get_roll() const;
  inline double get_roll_ff() const;
  inline double get_pitch() const;
  inline double get_pitch_ff() const;
  inline double get_yaw() const;
  inline double get_yaw_ff() const;
  inline double get_throttle_out() const;
  inline double get_throttle() const;
  inline double get_throttle_bidirectional() const;
  inline double get_throttle_slew_rate() const;
  inline double get_forward() const;
  inline double get_lateral() const;

  // motor failure handling
  inline void set_thrust_boost(bool enable);
  inline bool get_thrust_boost() const;

  virtual uint8_t get_lost_motor() const;

  // desired spool states
  enum class DesiredSpoolState : uint8_t
  {
    SHUT_DOWN = 0,           // all motors should move to stop
    GROUND_IDLE = 1,         // all motors should move to ground idle
    THROTTLE_UNLIMITED = 2,  // motors should move to being a state where throttle is unconstrained
                             // (e.g. by start up procedure)
  };

  void set_desired_spool_state(DesiredSpoolState spool);
  inline DesiredSpoolState get_desired_spool_state(void) const;

  // spool states
  enum class SpoolState : uint8_t
  {
    SHUT_DOWN = 0,           // all motors stop
    GROUND_IDLE = 1,         // all motors at ground idle
    SPOOLING_UP = 2,         // increasing maximum throttle while stabilizing
    THROTTLE_UNLIMITED = 3,  // throttle is no longer constrained by start up procedure
    SPOOLING_DOWN = 4,       // decreasing maximum throttle while stabilizing
  };

  // get_spool_state - get current spool state
  inline SpoolState get_spool_state(void) const;

  // set_density_ratio - sets air density as a proportion of sea level density
  inline void set_air_density_ratio(double ratio);

  // set_dt / get_dt - dt is the time since the last time the motor mixers were updated
  //   _dt should be set based on the time of the last IMU read used by these controllers
  //   the motor mixers should run on each loop to ensure normal operation
  inline void set_dt(double dt);
  inline double get_dt() const;

  // structure for holding motor limit flags
  struct AP_Motors_limit
  {
    bool roll;            // we have reached roll or pitch limit
    bool pitch;           // we have reached roll or pitch limit
    bool yaw;             // we have reached yaw limit
    bool throttle_lower;  // we have reached throttle's lower limit
    bool throttle_upper;  // we have reached throttle's upper limit
  } limit;

  // set limit flag for pitch, roll and yaw
  void set_limit_flag_pitch_roll_yaw(bool flag);

  // output - sends commands to the motors
  virtual void output() = 0;

  // output_min - sends minimum values out to the motors
  virtual void output_min() = 0;

  // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being
  // used)
  //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
  virtual uint32_t get_motor_mask() = 0;

  // pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
  void set_radio_passthrough(
    double roll_input,
    double pitch_input,
    double throttle_input,
    double yaw_input);

  // return the roll factor of any motor, this is used for tilt rotors and tail sitters
  // using copter motors for forward flight
  virtual double get_roll_factor(uint8_t i);
  // return the pitch factor of any motor
  virtual double get_pitch_factor(uint8_t i);
  // return whether a motor is enabled or not
  virtual bool is_motor_enabled(uint8_t i);
  // This function required for tradheli. Tradheli initializes targets when going from unarmed to
  // armed state. This function is overriden in motors_heli class.   Always true for multicopters.
  virtual bool init_targets_on_arming() const;

  // write log, to be called at 10hz
  virtual void Log_Write(){};

protected:
  // update the throttle input filter
  virtual void update_throttle_filter() = 0;

  // internal variables
  double _dt;            // time difference (in seconds) since the last loop time
  double _roll_in;       // desired roll control from attitude controllers, -1 ~ +1
  double _roll_in_ff;    // desired roll feed forward control from attitude controllers, -1 ~ +1
  double _pitch_in;      // desired pitch control from attitude controller, -1 ~ +1
  double _pitch_in_ff;   // desired pitch feed forward control from attitude controller, -1 ~ +1
  double _yaw_in;        // desired yaw control from attitude controller, -1 ~ +1
  double _yaw_in_ff;     // desired yaw feed forward control from attitude controller, -1 ~ +1
  double _throttle_in;   // last throttle input from set_throttle caller
  double _throttle_out;  // throttle after mixing is complete
  double _throttle_slew_rate;           // throttle slew rate from input
  double _forward_in;                   // last forward input from set_forward caller
  double _lateral_in;                   // last lateral input from set_lateral caller
  double _throttle_avg_max;             // last throttle input from set_throttle_avg_max
  LowPassFilter _throttle_filter;       // pilot throttle input filter
  LowPassFilter _throttle_slew_filter;  // filter for the output of the throttle slew
  DesiredSpoolState _spool_desired;     // desired spool state
  SpoolState _spool_state;              // current spool mode

  // air pressure compensation variables
  double _air_density_ratio;  // air density / sea level density - decreases in altitude
  // mask of what channels need fast output
  uint32_t _motor_fast_mask;
  // mask of what channels need to use SERVOn_MIN/MAX for output mapping
  uint32_t _motor_pwm_range_mask;

  // roll input from pilot in -1 ~ +1 range.  used for setup and
  // providing servo feedback while landed
  double _roll_radio_passthrough;
  // pitch input from pilot in -1 ~ +1 range.  used for setup and
  // providing servo feedback while landed
  double _pitch_radio_passthrough;
  // throttle/collective input from pilot in 0 ~ 1 range.  used
  // for setup and providing servo feedback while landed
  double _throttle_radio_passthrough;
  // yaw input from pilot in -1 ~ +1 range.  used for setup and
  // providing servo feedback while landed
  double _yaw_radio_passthrough;

  // motor failure handling
  bool _thrust_boost;     // true if thrust boost is enabled to handle motor failure
  bool _thrust_balanced;  // true when output thrust is well balanced
  // choice between highest and second highest motor output for output
  // mixing (0 ~ 1). Zero is normal operation
  double _thrust_boost_ratio;

  // output_test_seq - spin a motor at the pwm value specified
  //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
  //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
  virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) = 0;

private:
  bool _armed;           // 0 if disarmed, 1 if armed
  bool _interlock;       // 1 if the motor interlock is enabled, 0 if disabled
  bool _initialised_ok;  // 1 if initialisation was successful
  bool _spoolup_block;   // true if spoolup is blocked
};

inline bool Motors::initialised_ok() const
{
  return _initialised_ok;
}

inline void Motors::set_initialised_ok(bool val)
{
  _initialised_ok = val;
}

inline void Motors::set_interlock(bool set)
{
  _interlock = set;
}

inline bool Motors::get_interlock() const
{
  return _interlock;
}

inline bool Motors::get_spoolup_block() const
{
  return _spoolup_block;
}

inline void Motors::set_spoolup_block(bool set)
{
  _spoolup_block = set;
}

inline void Motors::set_roll(double roll_in)
{
  _roll_in = roll_in;
}

inline void Motors::set_roll_ff(double roll_in)
{
  _roll_in_ff = roll_in;
}

inline void Motors::set_pitch(double pitch_in)
{
  _pitch_in = pitch_in;
}

inline void Motors::set_pitch_ff(double pitch_in)
{
  _pitch_in_ff = pitch_in;
}

inline void Motors::set_yaw(double yaw_in)
{
  _yaw_in = yaw_in;
}

inline void Motors::set_yaw_ff(double yaw_in)
{
  _yaw_in_ff = yaw_in;
}

inline void Motors::set_throttle(double throttle_in)
{
  _throttle_in = throttle_in;
}

inline void Motors::set_throttle_avg_max(double throttle_avg_max)
{
  _throttle_avg_max = std::clamp(throttle_avg_max, 0., 1.);
}

inline void Motors::set_throttle_filter_cutoff(double filt_hz)
{
  _throttle_filter.setCutoffFrequency(filt_hz);
}

inline void Motors::set_slew_filter_cutoff(double filt_hz)
{
  _throttle_slew_filter.setCutoffFrequency(filt_hz);
}

inline void Motors::set_forward(double forward_in)
{
  _forward_in = forward_in;
}

inline void Motors::set_lateral(double lateral_in)
{
  _lateral_in = lateral_in;
}

inline double Motors::get_roll() const
{
  return _roll_in;
}

inline double Motors::get_roll_ff() const
{
  return _roll_in_ff;
}

inline double Motors::get_pitch() const
{
  return _pitch_in;
}

inline double Motors::get_pitch_ff() const
{
  return _pitch_in_ff;
}

inline double Motors::get_yaw() const
{
  return _yaw_in;
}

inline double Motors::get_yaw_ff() const
{
  return _yaw_in_ff;
}

inline double Motors::get_throttle_out() const
{
  return _throttle_out;
}

inline double Motors::get_throttle() const
{
  return std::clamp(_throttle_filter.get(), 0., 1.);
}

inline double Motors::get_throttle_bidirectional() const
{
  return std::clamp(2 * (_throttle_filter.get() - 0.), -1., 1.);
}

inline double Motors::get_throttle_slew_rate() const
{
  return _throttle_slew_rate;
}

inline double Motors::get_forward() const
{
  return _forward_in;
}

inline double Motors::get_lateral() const
{
  return _lateral_in;
}

inline void Motors::set_thrust_boost(bool enable)
{
  _thrust_boost = enable;
}

inline bool Motors::get_thrust_boost() const
{
  return _thrust_boost;
}

inline Motors::DesiredSpoolState Motors::get_desired_spool_state(void) const
{
  return _spool_desired;
}

inline Motors::SpoolState Motors::get_spool_state(void) const
{
  return _spool_state;
}

inline void Motors::set_air_density_ratio(double ratio)
{
  _air_density_ratio = ratio;
}

inline void Motors::set_dt(double dt)
{
  _dt = dt;
}

inline double Motors::get_dt() const
{
  return _dt;
}
}  // namespace tobas_mr_arducopter
