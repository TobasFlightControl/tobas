#include <algorithm>
#include <string.h>

#include "../include/tobas_mr_arducopter/pid.hpp"

using namespace std;

namespace tobas_mr_arducopter
{
PID::PID(
  double initial_p,
  double initial_i,
  double initial_d,
  double initial_ff,
  double initial_imax,
  double initial_filt_T_hz,
  double initial_filt_E_hz,
  double initial_filt_D_hz,
  double initial_srmax,
  double initial_srtau)
  : default_kp_(initial_p),
    default_ki_(initial_i),
    default_kd_(initial_d),
    default_kff_(initial_ff),
    default_kimax_(initial_imax),
    default_filt_T_hz_(initial_filt_T_hz),
    default_filt_E_hz_(initial_filt_E_hz),
    default_filt_D_hz_(initial_filt_D_hz),
    default_slew_rate_max_(initial_srmax)
{
  // this param is not in the table, so its default is no loaded in the call above
  slew_rate_tau_ = initial_srtau;

  // reset input filter to first value received
  flags_._reset_filter = true;

  memset(&pid_info_, 0, sizeof(pid_info_));

  // slew limit scaler allows for plane to use degrees/sec slew
  // limit
  slew_limit_scale_ = 1;
}

double PID::updateAll(
  double target,
  double measurement,
  double dt,
  uint32_t now_ms,
  bool limit,
  double boost)
{
  // don't process inf or NaN
  if (!isfinite(target) || !isfinite(measurement))
  {
    return 0.;
  }

  // reset input filter to value received
  if (flags_._reset_filter)
  {
    flags_._reset_filter = false;
    target_ = target;
    error_ = target_ - measurement;
    derivative_ = 0.;
  }
  else
  {
    double error_last = error_;
    target_ += get_filt_T_alpha(dt) * (target - target_);
    error_ += get_filt_E_alpha(dt) * ((target_ - measurement) - error_);

    // calculate and filter derivative
    if (dt > 0)
    {
      double derivative = (error_ - error_last) / dt;
      derivative_ += get_filt_D_alpha(dt) * (derivative - derivative_);
    }
  }

  // update I term
  update_i(dt, limit);

  double P_out = (error_ * kp_);
  double D_out = (derivative_ * kd_);

  // calculate slew limit modifier for P+D
  pid_info_.Dmod =
    slew_limiter_.modifier((pid_info_.P + pid_info_.D) * slew_limit_scale_, dt, now_ms);
  pid_info_.slew_rate = slew_limiter_.getSlewRate();

  P_out *= pid_info_.Dmod;
  D_out *= pid_info_.Dmod;

  // boost output if required
  P_out *= boost;
  D_out *= boost;

  pid_info_.target = target_;
  pid_info_.actual = measurement;
  pid_info_.error = error_;
  pid_info_.P = P_out;
  pid_info_.D = D_out;

  return P_out + integrator_ + D_out;
}

double PID::updateError(double error, double dt, uint32_t now_ms, bool limit)
{
  // don't process inf or NaN
  if (!isfinite(error))
  {
    return 0.;
  }

  target_ = 0.;

  // reset input filter to value received
  if (flags_._reset_filter)
  {
    flags_._reset_filter = false;
    error_ = error;
    derivative_ = 0.;
  }
  else
  {
    double error_last = error_;
    error_ += get_filt_E_alpha(dt) * (error - error_);

    // calculate and filter derivative
    if (dt > 0)
    {
      double derivative = (error_ - error_last) / dt;
      derivative_ += get_filt_D_alpha(dt) * (derivative - derivative_);
    }
  }

  // update I term
  update_i(dt, limit);

  double P_out = (error_ * kp_);
  double D_out = (derivative_ * kd_);

  // calculate slew limit modifier for P+D
  pid_info_.Dmod =
    slew_limiter_.modifier((pid_info_.P + pid_info_.D) * slew_limit_scale_, dt, now_ms);
  pid_info_.slew_rate = slew_limiter_.getSlewRate();

  P_out *= pid_info_.Dmod;
  D_out *= pid_info_.Dmod;

  pid_info_.target = 0.;
  pid_info_.actual = 0.;
  pid_info_.error = error_;
  pid_info_.P = P_out;
  pid_info_.D = D_out;

  return P_out + integrator_ + D_out;
}

void PID::update_i(double dt, bool limit)
{
  if (ki_ != 0 && dt > 0)
  {
    // Ensure that integrator can only be reduced if the output is saturated
    if (!limit || ((integrator_ > 0 && error_) || (integrator_ < 0 && error_ > 0)))
    {
      integrator_ += ((double)error_ * ki_) * dt;
      integrator_ = clamp(integrator_, -kimax_, kimax_);
    }
  }
  else
  {
    integrator_ = 0.;
  }
  pid_info_.I = integrator_;
  pid_info_.limit = limit;
}

void PID::operator()(
  double p_val,
  double i_val,
  double d_val,
  double ff_val,
  double imax_val,
  double input_filt_T_hz,
  double input_filt_E_hz,
  double input_filt_D_hz)
{
  kp_ = p_val;
  ki_ = i_val;
  kd_ = d_val;
  kff_ = ff_val;
  kimax_ = fabs(imax_val);
  filt_T_hz_ = input_filt_T_hz;
  filt_E_hz_ = input_filt_E_hz;
  filt_D_hz_ = input_filt_D_hz;
}
}  // namespace tobas_mr_arducopter
