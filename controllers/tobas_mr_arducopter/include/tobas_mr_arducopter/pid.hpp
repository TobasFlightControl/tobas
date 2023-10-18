#pragma once

#include <cmath>
#include <stdlib.h>

#include "./pid_info.hpp"
#include "./slew_limiter.hpp"
#include "./math.hpp"

#define AC_PID_TFILT_HZ_DEFAULT 0.   // default input filter frequency
#define AC_PID_EFILT_HZ_DEFAULT 0.   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT 20.  // default input filter frequency
#define AC_PID_RESET_TC 0.16         // Time constant for integrator reset decay to zero

namespace tobas_mr_arducopter
{
class PID
{
public:
  explicit PID(
    double initial_p,
    double initial_i,
    double initial_d,
    double initial_ff,
    double initial_imax,
    double initial_filt_T_hz,
    double initial_filt_E_hz,
    double initial_filt_D_hz,
    double initial_srmax = 0.,
    double initial_srtau = 1.);

  // updateAll - set target and measured inputs to PID controller and calculate outputs
  // target and error are filtered
  // the derivative is then calculated and filtered
  // the integral is then updated based on the setting of the limit flag
  double updateAll(
    double target,
    double measurement,
    double dt,
    uint32_t now_ms,
    bool limit = false,
    double boost = 1.);

  // updateError - set error input to PID controller and calculate outputs
  // target is set to zero and error is set and filtered
  // the derivative then is calculated and filtered
  // the integral is then updated based on the setting of the limit flag
  // Target and Measured must be set manually for logging purposes.
  // TODO: remove function when it is no longer used.
  double updateError(double error, double dt, uint32_t now_ms, bool limit = false);

  // update_i - update the integral
  // if the limit flag is set the integral is only allowed to shrink
  void update_i(double dt, bool limit);

  // get_pid - get results from pid controller
  inline double get_p() const;
  inline double get_i() const;
  inline double get_d() const;
  inline double get_ff();

  // reset_I - reset the integrator
  inline void reset_I();

  // resetFilter - input filter will be reset to the next value provided to set_input()
  inline void resetFilter();

  /// operator function call for easy initialisation
  void operator()(
    double p_val,
    double i_val,
    double d_val,
    double ff_val,
    double imax_val,
    double input_filt_T_hz,
    double input_filt_E_hz,
    double input_filt_D_hz);

  // get accessors
  inline double& kP();
  inline double& kI();
  inline double& kD();
  inline double& kIMAX();
  inline double& ff();
  inline double& filt_T_hz();
  inline double& filt_E_hz();
  inline double& filt_D_hz();
  inline double& slew_limit();
  inline double imax() const;

  // get_filt_T_alpha - get the target filter alpha
  inline double get_filt_T_alpha(double dt) const;
  // get_filt_E_alpha - get the error filter alpha
  inline double get_filt_E_alpha(double dt) const;
  // get_filt_D_alpha - get the derivative filter alpha
  inline double get_filt_D_alpha(double dt) const;

  // set accessors
  inline void kP(const double v);
  inline void kI(const double v);
  inline void kD(const double v);
  inline void ff(const double v);
  inline void imax(const double v);
  // filt_T_hz - set target filter hz
  inline void filt_T_hz(const double v);
  // filt_E_hz - set error filter hz
  inline void filt_E_hz(const double v);
  // filt_D_hz - set derivative filter hz
  inline void filt_D_hz(const double v);
  // slew_limit - set slew limit
  inline void slew_limit(const double v);

  // set the desired and actual rates (for logging purposes)
  inline void setTargetRate(double target);
  inline void setActualRate(double actual);

  // integrator setting functions
  inline void setIntegrator(double target, double measurement, double i);
  inline void setIntegrator(double error, double i);
  inline void setIntegrator(double i);
  inline void relaxIntegrator(double integrator, double dt, double time_constant);

  // set slew limiter scale factor
  inline void setSlewLimitScale(int8_t scale);
  // return current slew rate of slew limiter. Will return 0 if SMAX is zero
  inline double getSlewRate(void) const;
  inline const PidInfo& getPidInfo(void) const;

private:
  // parameters
  double kp_;
  double ki_;
  double kd_;
  double kff_;
  double kimax_;
  double filt_T_hz_;  // PID target filter frequency in Hz
  double filt_E_hz_;  // PID error filter frequency in Hz
  double filt_D_hz_;  // PID derivative filter frequency in Hz
  double slew_rate_max_;

  // the time constant tau is not currently configurable, but is set
  // as an double to make it easy to make it configurable for a
  // single user of PID by adding the parameter in the param
  // table of the parent class. It is made public for this reason
  double slew_rate_tau_;

  SlewLimiter slew_limiter_{ slew_rate_max_, slew_rate_tau_ };

  // flags
  struct ac_pid_flags
  {
    bool _reset_filter : 1;  // true when input filter should be reset during next call to set_input
  } flags_;

  // internal variables
  double integrator_;  // integrator value
  double target_;      // target value to enable filtering
  double error_;       // error value to enable filtering
  double derivative_;  // derivative value to enable filtering
  int8_t slew_limit_scale_;

  PidInfo pid_info_;

  const double default_kp_;
  const double default_ki_;
  const double default_kd_;
  const double default_kff_;
  const double default_kimax_;
  const double default_filt_T_hz_;
  const double default_filt_E_hz_;
  const double default_filt_D_hz_;
  const double default_slew_rate_max_;
};

inline double PID::get_p() const
{
  return error_ * kp_;
}

inline double PID::get_i() const
{
  return integrator_;
}

inline double PID::get_d() const
{
  return kd_ * derivative_;
}

inline double PID::get_ff()
{
  pid_info_.FF = target_ * kff_;
  return target_ * kff_;
}

inline void PID::reset_I()
{
  integrator_ = 0.0;
}

inline void PID::resetFilter()
{
  flags_._reset_filter = true;
}

inline double& PID::kP()
{
  return kp_;
}

inline double& PID::kI()
{
  return ki_;
}

inline double& PID::kD()
{
  return kd_;
}

inline double& PID::kIMAX()
{
  return kimax_;
}

inline double& PID::ff()
{
  return kff_;
}

inline double& PID::filt_T_hz()
{
  return filt_T_hz_;
}

inline double& PID::filt_E_hz()
{
  return filt_E_hz_;
}

inline double& PID::filt_D_hz()
{
  return filt_D_hz_;
}

inline double& PID::slew_limit()
{
  return slew_rate_max_;
}

inline double PID::imax() const
{
  return kimax_;
}

inline double PID::get_filt_T_alpha(double dt) const
{
  return calcLowPassAlphaDt(dt, filt_T_hz_);
}

inline double PID::get_filt_E_alpha(double dt) const
{
  return calcLowPassAlphaDt(dt, filt_E_hz_);
}

inline double PID::get_filt_D_alpha(double dt) const
{
  return calcLowPassAlphaDt(dt, filt_D_hz_);
}

inline void PID::kP(const double v)
{
  kp_ = v;
}

inline void PID::kI(const double v)
{
  ki_ = v;
}

inline void PID::kD(const double v)
{
  kd_ = v;
}

inline void PID::ff(const double v)
{
  kff_ = v;
}

inline void PID::imax(const double v)
{
  kimax_ = std::fabs(v);
}

inline void PID::filt_T_hz(double hz)
{
  filt_T_hz_ = std::fabs(hz);
}

inline void PID::filt_E_hz(double hz)
{
  filt_E_hz_ = std::fabs(hz);
}

inline void PID::filt_D_hz(double hz)
{
  filt_D_hz_ = std::fabs(hz);
}

inline void PID::slew_limit(double smax)
{
  slew_rate_max_ = std::fabs(smax);
}

inline void PID::setTargetRate(double target)
{
  pid_info_.target = target;
}

inline void PID::setActualRate(double actual)
{
  pid_info_.actual = actual;
}

inline void PID::setIntegrator(double target, double measurement, double integrator)
{
  setIntegrator(target - measurement, integrator);
}

inline void PID::setIntegrator(double error, double integrator)
{
  integrator_ = std::clamp(integrator - error * kp_, -kimax_, kimax_);
}

inline void PID::setIntegrator(double integrator)
{
  integrator_ = std::clamp(integrator, -kimax_, kimax_);
}

inline void PID::relaxIntegrator(double integrator, double dt, double time_constant)
{
  integrator = std::clamp(integrator, -kimax_, kimax_);
  if (dt > 0)
  {
    integrator_ = integrator_ + (integrator - integrator_) * (dt / (dt + time_constant));
  }
}

inline void PID::setSlewLimitScale(int8_t scale)
{
  slew_limit_scale_ = scale;
}

inline double PID::getSlewRate(void) const
{
  return slew_limiter_.getSlewRate();
}

inline const PidInfo& PID::getPidInfo(void) const
{
  return pid_info_;
}
}  // namespace tobas_mr_arducopter
