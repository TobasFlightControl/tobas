#include "../include/tobas_mr_arducopter/attitude_control.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

using namespace std;

#define ATTITUDE_CONTROL_INPUT_TC_DEFAULT 0.15  // Medium
// Min lean angle so that vehicle can maintain limited control
#define ATTITUDE_CONTROL_ANGLE_LIMIT_MIN 10.0

namespace tobas_mr_arducopter
{
AttitudeControl::AttitudeControl(Motors& motors)
  : _p_angle_roll(ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_pitch(ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_yaw(ATTITUDE_CONTROL_ANGLE_P),
    _angle_boost(0),
    _use_sqrt_controller(true),
    _throttle_rpy_mix_desired(ATTITUDE_CONTROL_THR_MIX_DEFAULT),
    _throttle_rpy_mix(ATTITUDE_CONTROL_THR_MIX_DEFAULT),
    _motors(motors)
{
}

double AttitudeControl::get_slew_yaw_max_degs() const
{
  if (_ang_vel_yaw_max <= 0)
  {
    return _slew_yaw * 0.01;
  }
  return min(_ang_vel_yaw_max, _slew_yaw * 0.01);
}

void AttitudeControl::relax_attitude_controllers()
{
  // Initialize the attitude variables to the current attitude
  _ahrs.get_quat_body_to_ned(_attitude_target);
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
  _attitude_ang_error.initialise();

  // Initialize the angular rate variables to the current rate
  _ang_vel_target = _ahrs.get_gyro();
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
  _ang_vel_body = _ahrs.get_gyro();

  // Initialize remaining variables
  _thrust_error_angle = 0.;

  // Reset the PID filters
  get_rate_roll_pid().resetFilter();
  get_rate_pitch_pid().resetFilter();
  get_rate_yaw_pid().resetFilter();

  // Reset the I terms
  reset_rate_controller_I_terms();
}

void AttitudeControl::relax_attitude_controllers(bool exclude_pitch)
{
  relax_attitude_controllers();
}

void AttitudeControl::reset_rate_controller_I_terms()
{
  get_rate_roll_pid().reset_I();
  get_rate_pitch_pid().reset_I();
  get_rate_yaw_pid().reset_I();
}

void AttitudeControl::reset_rate_controller_I_terms_smoothly()
{
  get_rate_roll_pid().relaxIntegrator(0.0, _dt, ATTITUDE_RATE_RELAX_TC);
  get_rate_pitch_pid().relaxIntegrator(0.0, _dt, ATTITUDE_RATE_RELAX_TC);
  get_rate_yaw_pid().relaxIntegrator(0.0, _dt, ATTITUDE_RATE_RELAX_TC);
}

void AttitudeControl::input_quaternion(QuaternionD& attitude_desired_quat, Vector3d ang_vel_target)
{
  QuaternionD attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat;
  Vector3d attitude_error_angle;
  attitude_error_quat.to_axis_angle(attitude_error_angle);

  // Limit the angular velocity
  ang_vel_limit(
    ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max),
    radians(_ang_vel_yaw_max));

  if (_rate_bf_ff_enabled)
  {
    // When acceleration limiting and feedforward are enabled, the sqrt controller is used to
    // compute an euler angular velocity that will cause the euler angle to smoothly stop at the
    // input angle with limited deceleration and an exponential decay specified by _input_tc at the
    // end.
    _ang_vel_target.x = input_shaping_angle(
      wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x,
      ang_vel_target.x, radians(_ang_vel_roll_max), _dt);
    _ang_vel_target.y = input_shaping_angle(
      wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y,
      ang_vel_target.y, radians(_ang_vel_pitch_max), _dt);
    _ang_vel_target.z = input_shaping_angle(
      wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z,
      ang_vel_target.z, radians(_ang_vel_yaw_max), _dt);
  }
  else
  {
    _attitude_target = attitude_desired_quat;
    _ang_vel_target = ang_vel_target;
  }

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // Convert body-frame angular velocity into euler angle derivative of desired attitude
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

  // rotate target and normalize
  QuaternionD attitude_desired_update;
  attitude_desired_update.from_axis_angle(ang_vel_target * _dt);
  attitude_desired_quat = attitude_desired_quat * attitude_desired_update;
  attitude_desired_quat.normalize();

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(
  double euler_roll_angle_cd,
  double euler_pitch_angle_cd,
  double euler_yaw_rate_cds)
{
  // Convert from centidegrees on public interface to radians
  double euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
  double euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
  double euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
  euler_roll_angle += get_roll_trim_rad();

  if (_rate_bf_ff_enabled)
  {
    // translate the roll pitch and yaw acceleration limits to the euler axis
    const Vector3d euler_accel = euler_accel_limit(
      _euler_angle_target, Vector3d{ get_accel_roll_max_radss(), get_accel_pitch_max_radss(),
                                     get_accel_yaw_max_radss() });

    // When acceleration limiting and feedforward are enabled, the sqrt controller is used to
    // compute an euler angular velocity that will cause the euler angle to smoothly stop at the
    // input angle with limited deceleration and an exponential decay specified by smoothing_gain at
    // the end.
    _euler_rate_target.x = input_shaping_angle(
      wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x,
      _euler_rate_target.x, _dt);
    _euler_rate_target.y = input_shaping_angle(
      wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y,
      _euler_rate_target.y, _dt);

    // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular
    // acceleration about the yaw axis, slewing the output rate towards the input rate.
    _euler_rate_target.z =
      input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector
    // for feedforward
    euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
    // Limit the angular velocity
    ang_vel_limit(
      _ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max),
      radians(_ang_vel_yaw_max));
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
  }
  else
  {
    // When feedforward is not enabled, the target euler angle is input into the target and the
    // feedforward rate is zeroed.
    _euler_angle_target.x = euler_roll_angle;
    _euler_angle_target.y = euler_pitch_angle;
    _euler_angle_target.z += euler_yaw_rate * _dt;
    // Compute quaternion target attitude
    _attitude_target.from_euler(
      _euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();
  }

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_euler_angle_roll_pitch_yaw(
  double euler_roll_angle_cd,
  double euler_pitch_angle_cd,
  double euler_yaw_angle_cd,
  bool slew_yaw)
{
  // Convert from centidegrees on public interface to radians
  double euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
  double euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
  double euler_yaw_angle = radians(euler_yaw_angle_cd * 0.01f);

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
  euler_roll_angle += get_roll_trim_rad();

  const double slew_yaw_max_rads = get_slew_yaw_max_rads();
  if (_rate_bf_ff_enabled)
  {
    // translate the roll pitch and yaw acceleration limits to the euler axis
    const Vector3d euler_accel = euler_accel_limit(
      _euler_angle_target, Vector3d{ get_accel_roll_max_radss(), get_accel_pitch_max_radss(),
                                     get_accel_yaw_max_radss() });

    // When acceleration limiting and feedforward are enabled, the sqrt controller is used to
    // compute an euler angular velocity that will cause the euler angle to smoothly stop at the
    // input angle with limited deceleration and an exponential decay specified by _input_tc at the
    // end.
    _euler_rate_target.x = input_shaping_angle(
      wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x,
      _euler_rate_target.x, _dt);
    _euler_rate_target.y = input_shaping_angle(
      wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y,
      _euler_rate_target.y, _dt);
    _euler_rate_target.z = input_shaping_angle(
      wrap_PI(euler_yaw_angle - _euler_angle_target.z), _input_tc, euler_accel.z,
      _euler_rate_target.z, _dt);
    if (slew_yaw)
    {
      _euler_rate_target.z = clamp(_euler_rate_target.z, -slew_yaw_max_rads, slew_yaw_max_rads);
    }

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector
    // for feedforward
    euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
    // Limit the angular velocity
    ang_vel_limit(
      _ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max),
      radians(_ang_vel_yaw_max));
    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
  }
  else
  {
    // When feedforward is not enabled, the target euler angle is input into the target and the
    // feedforward rate is zeroed.
    _euler_angle_target.x = euler_roll_angle;
    _euler_angle_target.y = euler_pitch_angle;
    if (slew_yaw)
    {
      // Compute constrained angle error
      double angle_error = clamp(
        wrap_PI(euler_yaw_angle - _euler_angle_target.z), -slew_yaw_max_rads * _dt,
        slew_yaw_max_rads * _dt);
      // Update attitude target from constrained angle error
      _euler_angle_target.z = wrap_PI(angle_error + _euler_angle_target.z);
    }
    else
    {
      _euler_angle_target.z = euler_yaw_angle;
    }
    // Compute quaternion target attitude
    _attitude_target.from_euler(
      _euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();
  }

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_euler_rate_roll_pitch_yaw(
  double euler_roll_rate_cds,
  double euler_pitch_rate_cds,
  double euler_yaw_rate_cds)
{
  // Convert from centidegrees on public interface to radians
  double euler_roll_rate = radians(euler_roll_rate_cds * 0.01f);
  double euler_pitch_rate = radians(euler_pitch_rate_cds * 0.01f);
  double euler_yaw_rate = radians(euler_yaw_rate_cds * 0.01f);

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  if (_rate_bf_ff_enabled)
  {
    // translate the roll pitch and yaw acceleration limits to the euler axis
    const Vector3d euler_accel = euler_accel_limit(
      _euler_angle_target, Vector3d{ get_accel_roll_max_radss(), get_accel_pitch_max_radss(),
                                     get_accel_yaw_max_radss() });

    // When acceleration limiting is enabled, the input shaper constrains angular acceleration,
    // slewing the output rate towards the input rate.
    _euler_rate_target.x =
      input_shaping_ang_vel(_euler_rate_target.x, euler_roll_rate, euler_accel.x, _dt, _rate_rp_tc);
    _euler_rate_target.y = input_shaping_ang_vel(
      _euler_rate_target.y, euler_pitch_rate, euler_accel.y, _dt, _rate_rp_tc);
    _euler_rate_target.z =
      input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector
    // for feedforward
    euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
  }
  else
  {
    // When feedforward is not enabled, the target euler angle is input into the target and the
    // feedforward rate is zeroed. Pitch angle is restricted to +- 85.0 degrees to avoid gimbal lock
    // discontinuities.
    _euler_angle_target.x = wrap_PI(_euler_angle_target.x + euler_roll_rate * _dt);
    _euler_angle_target.y =
      clamp(_euler_angle_target.y + euler_pitch_rate * _dt, radians(-85.), radians(85.));
    _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + euler_yaw_rate * _dt);

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();

    // Compute quaternion target attitude
    _attitude_target.from_euler(
      _euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
  }

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_rate_bf_roll_pitch_yaw(
  double roll_rate_bf_cds,
  double pitch_rate_bf_cds,
  double yaw_rate_bf_cds)
{
  // Convert from centidegrees on public interface to radians
  double roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
  double pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
  double yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  if (_rate_bf_ff_enabled)
  {
    // Compute acceleration-limited body frame rates
    // When acceleration limiting is enabled, the input shaper constrains angular acceleration about
    // the axis, slewing the output rate towards the input rate.
    _ang_vel_target.x = input_shaping_ang_vel(
      _ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.y = input_shaping_ang_vel(
      _ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.z = input_shaping_ang_vel(
      _ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
  }
  else
  {
    // When feedforward is not enabled, the quaternion is calculated and is input into the target
    // and the feedforward rate is zeroed.
    QuaternionD attitude_target_update;
    attitude_target_update.from_axis_angle(
      Vector3d{ roll_rate_rads * _dt, pitch_rate_rads * _dt, yaw_rate_rads * _dt });
    _attitude_target = _attitude_target * attitude_target_update;
    _attitude_target.normalize();

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();
  }

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_rate_bf_roll_pitch_yaw_2(
  double roll_rate_bf_cds,
  double pitch_rate_bf_cds,
  double yaw_rate_bf_cds)
{
  // Convert from centidegrees on public interface to radians
  double roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
  double pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
  double yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

  // Compute acceleration-limited body frame rates
  // When acceleration limiting is enabled, the input shaper constrains angular acceleration about
  // the axis, slewing the output rate towards the input rate.
  _ang_vel_target.x = input_shaping_ang_vel(
    _ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
  _ang_vel_target.y = input_shaping_ang_vel(
    _ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
  _ang_vel_target.z = input_shaping_ang_vel(
    _ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

  // Update the unused targets attitude based on current attitude to condition mode change
  _ahrs.get_quat_body_to_ned(_attitude_target);
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
  // Convert body-frame angular velocity into euler angle derivative of desired attitude
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
  _ang_vel_body = _ang_vel_target;
}

void AttitudeControl::input_rate_bf_roll_pitch_yaw_3(
  double roll_rate_bf_cds,
  double pitch_rate_bf_cds,
  double yaw_rate_bf_cds)
{
  // Convert from centidegrees on public interface to radians
  double roll_rate_rads = radians(roll_rate_bf_cds * 0.01f);
  double pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
  double yaw_rate_rads = radians(yaw_rate_bf_cds * 0.01f);

  // Update attitude error
  Vector3d attitude_error;
  _attitude_ang_error.to_axis_angle(attitude_error);

  QuaternionD attitude_ang_error_update_quat;
  // limit the integrated error angle
  double err_mag = attitude_error.length();
  if (err_mag > ATTITUDE_THRUST_ERROR_ANGLE)
  {
    attitude_error *= ATTITUDE_THRUST_ERROR_ANGLE / err_mag;
    _attitude_ang_error.from_axis_angle(attitude_error);
  }

  Vector3d gyro_latest = _ahrs.get_gyro_latest();
  attitude_ang_error_update_quat.from_axis_angle(
    Vector3d{ (_ang_vel_target.x - gyro_latest.x) * _dt, (_ang_vel_target.y - gyro_latest.y) * _dt,
              (_ang_vel_target.z - gyro_latest.z) * _dt });
  _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;

  // Compute acceleration-limited body frame rates
  // When acceleration limiting is enabled, the input shaper constrains angular acceleration about
  // the axis, slewing the output rate towards the input rate.
  _ang_vel_target.x = input_shaping_ang_vel(
    _ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
  _ang_vel_target.y = input_shaping_ang_vel(
    _ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
  _ang_vel_target.z = input_shaping_ang_vel(
    _ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

  // Retrieve quaternion body attitude
  QuaternionD attitude_body;
  _ahrs.get_quat_body_to_ned(attitude_body);

  // Update the unused targets attitude based on current attitude to condition mode change
  _attitude_target = attitude_body * _attitude_ang_error;

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // Convert body-frame angular velocity into euler angle derivative of desired attitude
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

  // Compute the angular velocity target from the integrated rate error
  _attitude_ang_error.to_axis_angle(attitude_error);
  _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);
  _ang_vel_body += _ang_vel_target;

  // ensure Quaternions stay normalized
  _attitude_ang_error.normalize();
}

void AttitudeControl::input_angle_step_bf_roll_pitch_yaw(
  double roll_angle_step_bf_cd,
  double pitch_angle_step_bf_cd,
  double yaw_angle_step_bf_cd)
{
  // Convert from centidegrees on public interface to radians
  double roll_step_rads = radians(roll_angle_step_bf_cd * 0.01f);
  double pitch_step_rads = radians(pitch_angle_step_bf_cd * 0.01f);
  double yaw_step_rads = radians(yaw_angle_step_bf_cd * 0.01f);

  // rotate attitude target by desired step
  QuaternionD attitude_target_update;
  attitude_target_update.from_axis_angle(
    Vector3d{ roll_step_rads, pitch_step_rads, yaw_step_rads });
  _attitude_target = _attitude_target * attitude_target_update;
  _attitude_target.normalize();

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // Set rate feedforward requests to zero
  _euler_rate_target.zero();
  _ang_vel_target.zero();

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_thrust_vector_rate_heading(
  const Vector3d& thrust_vector,
  double heading_rate_cds,
  bool slew_yaw)
{
  // Convert from centidegrees on public interface to radians
  double heading_rate = radians(heading_rate_cds * 0.01f);
  if (slew_yaw)
  {
    // a zero _angle_vel_yaw_max means that setting is disabled
    const double slew_yaw_max_rads = get_slew_yaw_max_rads();
    heading_rate = clamp(heading_rate, -slew_yaw_max_rads, slew_yaw_max_rads);
  }

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // convert thrust vector to a quaternion attitude
  QuaternionD thrust_vec_quat = attitude_from_thrust_vector(thrust_vector, 0.);

  // calculate the angle error in x and y.
  double thrust_vector_diff_angle;
  QuaternionD thrust_vec_correction_quat;
  Vector3d attitude_error;
  double returned_thrust_vector_angle;
  thrust_vector_rotation_angles(
    thrust_vec_quat, _attitude_target, thrust_vec_correction_quat, attitude_error,
    returned_thrust_vector_angle, thrust_vector_diff_angle);

  if (_rate_bf_ff_enabled)
  {
    // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular
    // acceleration about the yaw axis, slewing the output rate towards the input rate.
    _ang_vel_target.x = input_shaping_angle(
      attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
    _ang_vel_target.y = input_shaping_angle(
      attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);

    // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular
    // acceleration about the yaw axis, slewing the output rate towards the input rate.
    _ang_vel_target.z = input_shaping_ang_vel(
      _ang_vel_target.z, heading_rate, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // Limit the angular velocity
    ang_vel_limit(
      _ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max),
      radians(_ang_vel_yaw_max));
  }
  else
  {
    QuaternionD yaw_quat;
    yaw_quat.from_axis_angle(Vector3d{ 0., 0., heading_rate * _dt });
    _attitude_target = _attitude_target * thrust_vec_correction_quat * yaw_quat;

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();
  }

  // Convert body-frame angular velocity into euler angle derivative of desired attitude
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_thrust_vector_heading(
  const Vector3d& thrust_vector,
  double heading_angle_cd,
  double heading_rate_cds)
{
  // a zero _angle_vel_yaw_max means that setting is disabled
  const double slew_yaw_max_rads = get_slew_yaw_max_rads();

  // Convert from centidegrees on public interface to radians
  double heading_rate =
    clamp(radians(heading_rate_cds * 0.01f), -slew_yaw_max_rads, slew_yaw_max_rads);
  double heading_angle = radians(heading_angle_cd * 0.01f);

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

  // convert thrust vector and heading to a quaternion attitude
  const QuaternionD desired_attitude_quat =
    attitude_from_thrust_vector(thrust_vector, heading_angle);

  if (_rate_bf_ff_enabled)
  {
    // calculate the angle error in x and y.
    Vector3d attitude_error;
    double thrust_vector_diff_angle;
    QuaternionD thrust_vec_correction_quat;
    double returned_thrust_vector_angle;
    thrust_vector_rotation_angles(
      desired_attitude_quat, _attitude_target, thrust_vec_correction_quat, attitude_error,
      returned_thrust_vector_angle, thrust_vector_diff_angle);

    // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular
    // acceleration about the yaw axis, slewing the output rate towards the input rate.
    _ang_vel_target.x = input_shaping_angle(
      attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
    _ang_vel_target.y = input_shaping_angle(
      attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);
    _ang_vel_target.z = input_shaping_angle(
      attitude_error.z, _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, heading_rate,
      slew_yaw_max_rads, _dt);

    // Limit the angular velocity
    ang_vel_limit(
      _ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), slew_yaw_max_rads);
  }
  else
  {
    // set persisted quaternion target attitude
    _attitude_target = desired_attitude_quat;

    // Set rate feedforward requests to zero
    _euler_rate_target.zero();
    _ang_vel_target.zero();
  }

  // Convert body-frame angular velocity into euler angle derivative of desired attitude
  ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

  // Call quaternion attitude controller
  attitude_controller_run_quat();
}

void AttitudeControl::input_thrust_vector_heading(
  const Vector3d& thrust_vector,
  HeadingCommand heading)
{
  switch (heading.heading_mode)
  {
    case HeadingMode::Rate_Only:
      input_thrust_vector_rate_heading(thrust_vector, heading.yaw_rate_cds);
      break;
    case HeadingMode::Angle_Only:
      input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, 0.0);
      break;
    case HeadingMode::Angle_And_Rate:
      input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, heading.yaw_rate_cds);
      break;
  }
}

QuaternionD
AttitudeControl::attitude_from_thrust_vector(Vector3d thrust_vector, double heading_angle) const
{
  const Vector3d thrust_vector_up{ 0., 0., -1. };

  if (thrust_vector.length_squared() == 0)
  {
    thrust_vector = thrust_vector_up;
  }
  else
  {
    thrust_vector.normalize();
  }

  // the cross product of the desired and target thrust vector defines the rotation vector
  Vector3d thrust_vec_cross = thrust_vector_up % thrust_vector;

  // the dot product is used to calculate the angle between the target and desired thrust vectors
  const double thrust_vector_angle = acosf(clamp(thrust_vector_up * thrust_vector, -1., 1.));

  // Normalize the thrust rotation vector
  const double thrust_vector_length = thrust_vec_cross.length();
  if (thrust_vector_length == 0 || thrust_vector_angle == 0)
  {
    thrust_vec_cross = thrust_vector_up;
  }
  else
  {
    thrust_vec_cross /= thrust_vector_length;
  }

  QuaternionD thrust_vec_quat;
  thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle);
  QuaternionD yaw_quat;
  yaw_quat.from_axis_angle(Vector3d{ 0., 0., 1. }, heading_angle);
  return thrust_vec_quat * yaw_quat;
}

void AttitudeControl::attitude_controller_run_quat()
{
  // This represents a quaternion rotation in NED frame to the body
  QuaternionD attitude_body;
  _ahrs.get_quat_body_to_ned(attitude_body);

  // This vector represents the angular error to rotate the thrust vector using x and y and heading
  // using z
  Vector3d attitude_error;
  thrust_heading_rotation_angles(
    _attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

  // Compute the angular velocity corrections in the body frame from the attitude error
  _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);

  // ensure angular velocity does not go over configured limits
  ang_vel_limit(
    _ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max),
    radians(_ang_vel_yaw_max));

  // rotation from the target frame to the body frame
  QuaternionD rotation_target_to_body = attitude_body.inverse() * _attitude_target;

  // target angle velocity vector in the body frame
  Vector3d ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target;

  // Correct the thrust vector and smoothly add feedforward and yaw input
  _feedforward_scalar = 1.;
  if (_thrust_error_angle > ATTITUDE_THRUST_ERROR_ANGLE * 2.)
  {
    _ang_vel_body.z = _ahrs.get_gyro().z;
  }
  else if (_thrust_error_angle > ATTITUDE_THRUST_ERROR_ANGLE)
  {
    _feedforward_scalar =
      (1. - (_thrust_error_angle - ATTITUDE_THRUST_ERROR_ANGLE) / ATTITUDE_THRUST_ERROR_ANGLE);
    _ang_vel_body.x += ang_vel_body_feedforward.x * _feedforward_scalar;
    _ang_vel_body.y += ang_vel_body_feedforward.y * _feedforward_scalar;
    _ang_vel_body.z += ang_vel_body_feedforward.z;
    _ang_vel_body.z =
      _ahrs.get_gyro().z * (1.0 - _feedforward_scalar) + _ang_vel_body.z * _feedforward_scalar;
  }
  else
  {
    _ang_vel_body += ang_vel_body_feedforward;
  }

  if (_rate_bf_ff_enabled)
  {
    // rotate target and normalize
    QuaternionD attitude_target_update;
    attitude_target_update.from_axis_angle(
      Vector3d{ _ang_vel_target.x * _dt, _ang_vel_target.y * _dt, _ang_vel_target.z * _dt });
    _attitude_target = _attitude_target * attitude_target_update;
  }

  // ensure QuaternionD stay normalised
  _attitude_target.normalize();

  // Record error to handle EKF resets
  _attitude_ang_error = attitude_body.inverse() * _attitude_target;
}

void AttitudeControl::thrust_heading_rotation_angles(
  QuaternionD& attitude_target,
  const QuaternionD& attitude_body,
  Vector3d& attitude_error,
  double& thrust_angle,
  double& thrust_error_angle) const
{
  QuaternionD thrust_vector_correction;
  thrust_vector_rotation_angles(
    attitude_target, attitude_body, thrust_vector_correction, attitude_error, thrust_angle,
    thrust_error_angle);

  // Todo: Limit roll an pitch error based on output saturation and maximum error.

  // Limit Yaw Error based on maximum acceleration - Update to include output saturation and maximum
  // error. Currently the limit is based on the maximum acceleration using the linear part of the
  // SQRT controller. This should be updated to be based on an angle limit, saturation, or unlimited
  // based on user defined parameters.
  QuaternionD yaw_vec_correction_quat;
  if (
    _p_angle_yaw.kP() != 0
    && fabs(attitude_error.z) > ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP())
  {
    attitude_error.z = clamp(
      wrap_PI(attitude_error.z), -ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP(),
      ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / _p_angle_yaw.kP());
    yaw_vec_correction_quat.from_axis_angle(Vector3d{ 0., 0., attitude_error.z });
    attitude_target = attitude_body * thrust_vector_correction * yaw_vec_correction_quat;
  }
}

void AttitudeControl::thrust_vector_rotation_angles(
  const QuaternionD& attitude_target,
  const QuaternionD& attitude_body,
  QuaternionD& thrust_vector_correction,
  Vector3d& attitude_error,
  double& thrust_angle,
  double& thrust_error_angle) const
{
  // The direction of thrust is [0,0,-1] is any body-fixed frame, inc. body frame and target frame.
  const Vector3d thrust_vector_up{ 0., 0., -1. };

  // attitude_target and attitute_body are passive rotations from target / body frames to the NED
  // frame

  // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the target thrust vector in the
  // inertial frame
  Vector3d att_target_thrust_vec = attitude_target * thrust_vector_up;  // target thrust vector

  // Rotating [0,0,-1] by attitude_target expresses (gets a view of) the current thrust vector in
  // the inertial frame
  Vector3d att_body_thrust_vec = attitude_body * thrust_vector_up;  // current thrust vector

  // the dot product is used to calculate the current lean angle for use of external functions
  thrust_angle = acosf(clamp(thrust_vector_up * att_body_thrust_vec, -1., 1.));

  // the cross product of the desired and target thrust vector defines the rotation vector
  Vector3d thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;

  // the dot product is used to calculate the angle between the target and desired thrust vectors
  thrust_error_angle = acosf(clamp(att_body_thrust_vec * att_target_thrust_vec, -1., 1.));

  // Normalize the thrust rotation vector
  double thrust_vector_length = thrust_vec_cross.length();
  if (thrust_vector_length == 0 || thrust_error_angle == 0)
  {
    thrust_vec_cross = thrust_vector_up;
  }
  else
  {
    thrust_vec_cross /= thrust_vector_length;
  }

  // thrust_vector_correction is defined relative to the body frame but its axis `thrust_vec_cross`
  // was computed in the inertial frame. First rotate it by the inverse of attitude_body to express
  // it back in the body frame
  thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
  thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle);

  // calculate the angle error in x and y.
  Vector3d rotation;
  thrust_vector_correction.to_axis_angle(rotation);
  attitude_error.x = rotation.x;
  attitude_error.y = rotation.y;

  // calculate the remaining rotation required after thrust vector is rotated transformed to the
  // body frame heading_vector_correction
  QuaternionD heading_vec_correction_quat =
    thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

  // calculate the angle error in z (x and y should be zero here).
  heading_vec_correction_quat.to_axis_angle(rotation);
  attitude_error.z = rotation.z;
}

double AttitudeControl::input_shaping_angle(
  double error_angle,
  double input_tc,
  double accel_max,
  double target_ang_vel,
  double desired_ang_vel,
  double max_ang_vel,
  double dt)
{
  // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
  desired_ang_vel += sqrt_controller(error_angle, 1. / max(input_tc, 0.01), accel_max, dt);
  if (max_ang_vel > 0)
  {
    desired_ang_vel = clamp(desired_ang_vel, -max_ang_vel, max_ang_vel);
  }

  // Acceleration is limited directly to smooth the beginning of the curve.
  return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt, 0.);
}

double AttitudeControl::input_shaping_ang_vel(
  double target_ang_vel,
  double desired_ang_vel,
  double accel_max,
  double dt,
  double input_tc)
{
  if (input_tc > 0)
  {
    // Calculate the acceleration to smoothly achieve rate. Jerk is not limited.
    double error_rate = desired_ang_vel - target_ang_vel;
    double desired_ang_accel = sqrt_controller(error_rate, 1. / max(input_tc, 0.01), 0., dt);
    desired_ang_vel = target_ang_vel + desired_ang_accel * dt;
  }
  // Acceleration is limited directly to smooth the beginning of the curve.
  if (is_positive(accel_max))
  {
    double delta_ang_vel = accel_max * dt;
    return clamp(desired_ang_vel, target_ang_vel - delta_ang_vel, target_ang_vel + delta_ang_vel);
  }
  else
  {
    return desired_ang_vel;
  }
}

void AttitudeControl::input_shaping_rate_predictor(
  const Vector2d& error_angle,
  Vector2d& target_ang_vel,
  double dt) const
{
  if (_rate_bf_ff_enabled)
  {
    // translate the roll pitch and yaw acceleration limits to the euler axis
    target_ang_vel.x = input_shaping_angle(
      wrap_PI(error_angle.x), _input_tc, get_accel_roll_max_radss(), target_ang_vel.x, dt);
    target_ang_vel.y = input_shaping_angle(
      wrap_PI(error_angle.y), _input_tc, get_accel_pitch_max_radss(), target_ang_vel.y, dt);
  }
  else
  {
    const double angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
    const double angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
    target_ang_vel.x = angleP_roll * wrap_PI(error_angle.x);
    target_ang_vel.y = angleP_pitch * wrap_PI(error_angle.y);
  }
  // Limit the angular velocity correction
  Vector3d ang_vel(target_ang_vel.x, target_ang_vel.y, 0.);
  ang_vel_limit(ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), 0.);

  target_ang_vel.x = ang_vel.x;
  target_ang_vel.y = ang_vel.y;
}

void AttitudeControl::ang_vel_limit(
  Vector3d& euler_rad,
  double ang_vel_roll_max,
  double ang_vel_pitch_max,
  double ang_vel_yaw_max) const
{
  if (ang_vel_roll_max == 0 || ang_vel_pitch_max == 0)
  {
    if (!ang_vel_roll_max == 0)
    {
      euler_rad.x = clamp(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
    }
    if (ang_vel_pitch_max != 0)
    {
      euler_rad.y = clamp(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
    }
  }
  else
  {
    Vector2d thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max, euler_rad.y / ang_vel_pitch_max);
    double thrust_vector_length = thrust_vector_ang_vel.length();
    if (thrust_vector_length > 1.)
    {
      euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
      euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
    }
  }
  if (ang_vel_yaw_max != 0)
  {
    euler_rad.z = clamp(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
  }
}

Vector3d AttitudeControl::euler_accel_limit(const Vector3d& euler_rad, const Vector3d& euler_accel)
{
  double sin_phi = clamp(fabs(sin(euler_rad.x)), 0.1, 1.);
  double cos_phi = clamp(fabs(cos(euler_rad.x)), 0.1, 1.);
  double sin_theta = clamp(fabs(sin(euler_rad.y)), 0.1, 1.);

  Vector3d rot_accel;
  if (
    euler_accel.x == 0 || euler_accel.y == 0 || euler_accel.z == 0 || euler_accel.x < 0
    || euler_accel.y < 0 || euler_accel.z < 0)
  {
    rot_accel.x = euler_accel.x;
    rot_accel.y = euler_accel.y;
    rot_accel.z = euler_accel.z;
  }
  else
  {
    rot_accel.x = euler_accel.x;
    rot_accel.y = min(euler_accel.y / cos_phi, euler_accel.z / sin_phi);
    rot_accel.z =
      min(min(euler_accel.x / sin_theta, euler_accel.y / sin_phi), euler_accel.z / cos_phi);
  }
  return rot_accel;
}

void AttitudeControl::reset_target_and_rate(bool reset_rate)
{
  // move attitude target to current attitude
  _ahrs.get_quat_body_to_ned(_attitude_target);

  if (reset_rate)
  {
    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector
    // for feedforward
    _ang_vel_target.zero();
    _euler_angle_target.zero();
  }
}

void AttitudeControl::reset_yaw_target_and_rate(bool reset_rate)
{
  // move attitude target to current heading
  double yaw_shift = _ahrs.yaw - _euler_angle_target.z;
  QuaternionD _attitude_target_update;
  _attitude_target_update.from_axis_angle(Vector3d{ 0., 0., yaw_shift });
  _attitude_target = _attitude_target_update * _attitude_target;

  if (reset_rate)
  {
    // set yaw rate to zero
    _euler_rate_target.z = 0.;

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector
    // for feedforward
    euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
  }
}

void AttitudeControl::inertial_frame_reset()
{
  // Retrieve quaternion body attitude
  QuaternionD attitude_body;
  _ahrs.get_quat_body_to_ned(attitude_body);

  // Recalculate the target quaternion
  _attitude_target = attitude_body * _attitude_ang_error;

  // calculate the attitude target euler angles
  _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
}

void AttitudeControl::euler_rate_to_ang_vel(
  const Vector3d& euler_rad,
  const Vector3d& euler_rate_rads,
  Vector3d& ang_vel_rads)
{
  double sin_theta = sin(euler_rad.y);
  double cos_theta = cos(euler_rad.y);
  double sin_phi = sin(euler_rad.x);
  double cos_phi = cos(euler_rad.x);

  ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
  ang_vel_rads.y = cos_phi * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
  ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

// Convert an angular velocity vector to a 321-intrinsic euler angle derivative
// Returns false if the vehicle is pitched 90 degrees up or down
bool AttitudeControl::ang_vel_to_euler_rate(
  const Vector3d& euler_rad,
  const Vector3d& ang_vel_rads,
  Vector3d& euler_rate_rads)
{
  double sin_theta = sin(euler_rad.y);
  double cos_theta = cos(euler_rad.y);
  double sin_phi = sin(euler_rad.x);
  double cos_phi = cos(euler_rad.x);

  // When the vehicle pitches all the way up or all the way down, the euler angles become
  // discontinuous. In this case, we just return false.
  if (cos_theta == 0)
  {
    return false;
  }

  euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y
                      + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z;
  euler_rate_rads.y = cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
  euler_rate_rads.z =
    (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
  return true;
}

Vector3d
AttitudeControl::update_ang_vel_target_from_att_error(const Vector3d& attitude_error_rot_vec_rad)
{
  Vector3d rate_target_ang_vel;

  // Compute the roll angular velocity demand from the roll angle error
  const double angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
  if (_use_sqrt_controller && get_accel_roll_max_radss() != 0)
  {
    rate_target_ang_vel.x = sqrt_controller(
      attitude_error_rot_vec_rad.x, angleP_roll,
      clamp(
        get_accel_roll_max_radss() / 2., ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS,
        ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS),
      _dt);
  }
  else
  {
    rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
  }

  // Compute the pitch angular velocity demand from the pitch angle error
  const double angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
  if (_use_sqrt_controller && get_accel_pitch_max_radss() != 0)
  {
    rate_target_ang_vel.y = sqrt_controller(
      attitude_error_rot_vec_rad.y, angleP_pitch,
      clamp(
        get_accel_pitch_max_radss() / 2., ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS,
        ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS),
      _dt);
  }
  else
  {
    rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
  }

  // Compute the yaw angular velocity demand from the yaw angle error
  const double angleP_yaw = _p_angle_yaw.kP() * _angle_P_scale.z;
  if (_use_sqrt_controller && get_accel_yaw_max_radss() != 0)
  {
    rate_target_ang_vel.z = sqrt_controller(
      attitude_error_rot_vec_rad.z, angleP_yaw,
      clamp(
        get_accel_yaw_max_radss() / 2., ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS,
        ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS),
      _dt);
  }
  else
  {
    rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
  }

  // reset angle P scaling, saving used value for logging
  _angle_P_scale_used = _angle_P_scale;
  _angle_P_scale = VECTOR_111;

  return rate_target_ang_vel;
}

double AttitudeControl::get_althold_lean_angle_max_cd() const
{
  // convert to centi-degrees for public interface
  return max(ToDeg(_althold_lean_angle_max), ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.;
}

void AttitudeControl::get_rpy_srate(double& roll_srate, double& pitch_srate, double& yaw_srate)
{
  roll_srate = get_rate_roll_pid().getPidInfo().slew_rate;
  pitch_srate = get_rate_pitch_pid().getPidInfo().slew_rate;
  yaw_srate = get_rate_yaw_pid().getPidInfo().slew_rate;
}
}  // namespace tobas_mr_arducopter
