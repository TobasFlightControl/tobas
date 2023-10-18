#pragma once

#include "./motors_class.hpp"
#include "./p.hpp"
#include "./pid.hpp"
#include "./vector3.hpp"
#include "./quaternion.hpp"

namespace tobas_mr_arducopter
{
class AttitudeControl
{
  static constexpr Vector3d VECTOR_111{ 1., 1., 1. };

public:
  explicit AttitudeControl(Motors& motors);

  // set_dt / get_dt - dt is the time since the last time the attitude controllers were updated
  // _dt should be set based on the time of the last IMU read used by these controllers
  // the attitude controller should run updates for active controllers on each loop to ensure normal
  // operation
  inline void set_dt(double dt);
  inline double get_dt() const;

  // pid accessors
  inline P& get_angle_roll_p();
  inline P& get_angle_pitch_p();
  inline P& get_angle_yaw_p();
  virtual PID& get_rate_roll_pid() = 0;
  virtual PID& get_rate_pitch_pid() = 0;
  virtual PID& get_rate_yaw_pid() = 0;

  // get the roll acceleration limit in centidegrees/s/s or radians/s/s
  inline double get_accel_roll_max_cdss() const;
  inline double get_accel_roll_max_radss() const;

  // Sets the roll acceleration limit in centidegrees/s/s
  inline void set_accel_roll_max_cdss(double accel_roll_max);

  // get the pitch acceleration limit in centidegrees/s/s or radians/s/s
  inline double get_accel_pitch_max_cdss() const;
  inline double get_accel_pitch_max_radss() const;

  // Sets the pitch acceleration limit in centidegrees/s/s
  inline void set_accel_pitch_max_cdss(double accel_pitch_max);

  // get the yaw acceleration limit in centidegrees/s/s or radians/s/s
  inline double get_accel_yaw_max_cdss() const;
  inline double get_accel_yaw_max_radss() const;

  // Sets the yaw acceleration limit in centidegrees/s/s
  inline void set_accel_yaw_max_cdss(double accel_yaw_max);

  // get the roll angular velocity limit in radians/s
  inline double get_ang_vel_roll_max_rads() const;

  // get the pitch angular velocity limit in radians/s
  inline double get_ang_vel_pitch_max_rads() const;

  // get the yaw angular velocity limit in radians/s
  inline double get_ang_vel_yaw_max_rads() const;

  // get the slew yaw rate limit in deg/s
  double get_slew_yaw_max_degs() const;

  // get the rate control input smoothing time constant
  inline double get_input_tc() const;

  // set the rate control input smoothing time constant
  inline void set_input_tc(double input_tc);

  // Ensure attitude controller have zero errors to relax rate controller output
  void relax_attitude_controllers();

  // Used by child class AC_AttitudeControl_TS to change behaviour for tailsitter quadplanes
  virtual void relax_attitude_controllers(bool exclude_pitch);

  // reset rate controller I terms
  void reset_rate_controller_I_terms();

  // reset rate controller I terms smoothly to zero in 0.5 seconds
  void reset_rate_controller_I_terms_smoothly();

  // Sets attitude target to vehicle attitude and sets all rates to zero
  // If reset_rate is false rates are not reset to allow the rate controllers to run
  void reset_target_and_rate(bool reset_rate = true);

  // Sets yaw target to vehicle heading and sets yaw rate to zero
  // If reset_rate is false rates are not reset to allow the rate controllers to run
  void reset_yaw_target_and_rate(bool reset_rate = true);

  // handle reset of attitude from EKF since the last iteration
  void inertial_frame_reset();

  // Command a QuaternionD attitude with feedforward and smoothing
  // attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular
  // velocity
  virtual void input_quaternion(QuaternionD& attitude_desired_quat, Vector3d ang_vel_target);

  // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward
  // and smoothing
  virtual void input_euler_angle_roll_pitch_euler_rate_yaw(
    double euler_roll_angle_cd,
    double euler_pitch_angle_cd,
    double euler_yaw_rate_cds);
  // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
  virtual void input_euler_angle_roll_pitch_yaw(
    double euler_roll_angle_cd,
    double euler_pitch_angle_cd,
    double euler_yaw_angle_cd,
    bool slew_yaw);

  // Command euler yaw rate and pitch angle with roll angle specified in body frame
  // (implemented only in AC_AttitudeControl_TS for tailsitter quadplanes)
  virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(
    bool plane_controls,
    double euler_roll_angle_cd,
    double euler_pitch_angle_cd,
    double euler_yaw_rate_cds)
  {
  }

  // Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
  virtual void input_euler_rate_roll_pitch_yaw(
    double euler_roll_rate_cds,
    double euler_pitch_rate_cds,
    double euler_yaw_rate_cds);

  // Command an angular velocity with angular velocity feedforward and smoothing
  virtual void input_rate_bf_roll_pitch_yaw(
    double roll_rate_bf_cds,
    double pitch_rate_bf_cds,
    double yaw_rate_bf_cds);

  // Command an angular velocity with angular velocity feedforward and smoothing
  virtual void input_rate_bf_roll_pitch_yaw_2(
    double roll_rate_bf_cds,
    double pitch_rate_bf_cds,
    double yaw_rate_bf_cds);

  // Command an angular velocity with angular velocity smoothing using rate loops only with
  // integrated rate error stabilization
  virtual void input_rate_bf_roll_pitch_yaw_3(
    double roll_rate_bf_cds,
    double pitch_rate_bf_cds,
    double yaw_rate_bf_cds);

  // Command an angular step (i.e change) in body frame angle
  virtual void input_angle_step_bf_roll_pitch_yaw(
    double roll_angle_step_bf_cd,
    double pitch_angle_step_bf_cd,
    double yaw_angle_step_bf_cd);

  // Command a thrust vector in the earth frame and a heading angle and/or rate
  virtual void input_thrust_vector_rate_heading(
    const Vector3d& thrust_vector,
    double heading_rate_cds,
    bool slew_yaw = true);
  virtual void input_thrust_vector_heading(
    const Vector3d& thrust_vector,
    double heading_angle_cd,
    double heading_rate_cds);
  inline void input_thrust_vector_heading(const Vector3d& thrust_vector, double heading_cd);

  // Converts thrust vector and heading angle to quaternion rotation in the earth frame
  QuaternionD attitude_from_thrust_vector(Vector3d thrust_vector, double heading_angle) const;

  // Run angular velocity controller and send outputs to the motors
  virtual void rate_controller_run() = 0;

  // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
  void euler_rate_to_ang_vel(
    const Vector3d& euler_rad,
    const Vector3d& euler_rate_rads,
    Vector3d& ang_vel_rads);

  // Convert an angular velocity vector to a 321-intrinsic euler angle derivative
  // Returns false if the vehicle is pitched 90 degrees up or down
  bool ang_vel_to_euler_rate(
    const Vector3d& euler_rad,
    const Vector3d& ang_vel_rads,
    Vector3d& euler_rate_rads);

  // Specifies whether the attitude controller should use the square root controller in the attitude
  // correction. This is used during Autotune to ensure the P term is tuned without being influenced
  // by the acceleration limit of the square root controller.
  inline void use_sqrt_controller(bool use_sqrt_cont);

  // Return 321-intrinsic euler angles in centidegrees representing the rotation from NED earth
  // frame to the attitude controller's target attitude.
  // **NOTE** Using vector3f*deg(100) is more efficient than deg(vector3f)*100 or deg(vector3d*100)
  // because it gives the same result with the fewest multiplications. Even though it may look like
  // a bug, it is intentional. See issue 4895.
  inline Vector3d get_att_target_euler_cd() const;
  inline const Vector3d& get_att_target_euler_rad() const;

  // Return the body-to-NED target attitude used by the quadplane-specific attitude control input
  // methods
  inline QuaternionD get_attitude_target_quat() const;
  // Return the angular velocity of the target (setpoint) [rad/s] in the target attitude frame
  inline const Vector3d& get_attitude_target_ang_vel() const;
  // Return the angle between the target thrust vector and the current thrust vector.
  inline double get_att_error_angle_deg() const;

  // Set x-axis angular velocity in centidegrees/s
  inline void rate_bf_roll_target(double rate_cds);
  // Set y-axis angular velocity in centidegrees/s
  inline void rate_bf_pitch_target(double rate_cds);
  // Set z-axis angular velocity in centidegrees/s
  inline void rate_bf_yaw_target(double rate_cds);
  // Set x-axis system identification angular velocity in degrees/s
  inline void rate_bf_roll_sysid(double rate);
  // Set y-axis system identification angular velocity in degrees/s
  inline void rate_bf_pitch_sysid(double rate);
  // Set z-axis system identification angular velocity in degrees/s
  inline void rate_bf_yaw_sysid(double rate);
  // Set x-axis system identification actuator
  inline void actuator_roll_sysid(double command);
  // Set y-axis system identification actuator
  inline void actuator_pitch_sysid(double command);
  // Set z-axis system identification actuator
  inline void actuator_yaw_sysid(double command);

  // Return roll rate step size in radians/s that results in maximum output after 4 time steps
  double max_rate_step_bf_roll();
  // Return pitch rate step size in radians/s that results in maximum output after 4 time steps
  double max_rate_step_bf_pitch();
  // Return yaw rate step size in radians/s that results in maximum output after 4 time steps
  double max_rate_step_bf_yaw();

  // Return angular velocity in radians used in the angular velocity controller
  inline Vector3d rate_bf_targets() const;
  // return the angular velocity of the target (setpoint) attitude rad/s
  inline const Vector3d& get_rate_ef_targets() const;

  // Enable or disable body-frame feed forward
  inline void bf_feedforward(bool enable_or_disable);
  // Return body-frame feed forward setting
  inline bool get_bf_feedforward();

  // Update Alt_Hold angle maximum
  virtual void update_althold_lean_angle_max(double throttle_in) = 0;

  // Set output throttle
  virtual void set_throttle_out(double throttle_in, bool apply_angle_boost, double filt_cutoff) = 0;

  // get throttle passed into attitude controller (i.e. throttle_in provided to set_throttle_out)
  inline double get_throttle_in() const;

  // Return throttle increase applied for tilt compensation
  inline double angle_boost() const;

  // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
  virtual double get_althold_lean_angle_max_cd() const;

  // Return tilt angle in degrees
  inline double lean_angle_deg() const;

  // calculates the velocity correction from an angle error. The angular velocity has acceleration
  // and deceleration limits including basic jerk limiting using smoothing_gain
  static double input_shaping_angle(
    double error_angle,
    double input_tc,
    double accel_max,
    double target_ang_vel,
    double desired_ang_vel,
    double max_ang_vel,
    double dt);
  static inline double input_shaping_angle(
    double error_angle,
    double input_tc,
    double accel_max,
    double target_ang_vel,
    double dt);

  // Shapes the velocity request based on a rate time constant. The angular acceleration and
  // deceleration is limited.
  static double input_shaping_ang_vel(
    double target_ang_vel,
    double desired_ang_vel,
    double accel_max,
    double dt,
    double input_tc);

  // calculates the expected angular velocity correction from an angle error based on the
  // AttitudeControl settings. This function can be used to predict the delay associated with
  // angle requests.
  void input_shaping_rate_predictor(
    const Vector2d& error_angle,
    Vector2d& target_ang_vel,
    double dt) const;

  // translates body frame acceleration limits to the euler axis
  void ang_vel_limit(
    Vector3d& euler_rad,
    double ang_vel_roll_max,
    double ang_vel_pitch_max,
    double ang_vel_yaw_max) const;

  // translates body frame acceleration limits to the euler axis
  Vector3d euler_accel_limit(const Vector3d& euler_rad, const Vector3d& euler_accel);

  // Calculates the body frame angular velocities to follow the target attitude
  void attitude_controller_run_quat();

  // thrust_heading_rotation_angles - calculates two ordered rotations to move the attitude_body
  // quaternion to the attitude_target quaternion. The maximum error in the yaw axis is limited
  // based on the angle yaw P value and acceleration.
  void thrust_heading_rotation_angles(
    QuaternionD& attitude_target,
    const QuaternionD& attitude_body,
    Vector3d& attitude_error,
    double& thrust_angle,
    double& thrust_error_angle) const;

  // thrust_vector_rotation_angles - calculates two ordered rotations to move the attitude_body
  // quaternion to the attitude_target quaternion. The first rotation corrects the thrust vector and
  // the second rotation corrects the heading vector.
  void thrust_vector_rotation_angles(
    const QuaternionD& attitude_target,
    const QuaternionD& attitude_body,
    QuaternionD& thrust_vector_correction,
    Vector3d& attitude_error,
    double& thrust_angle,
    double& thrust_error_angle) const;

  // sanity check parameters.  should be called once before take-off
  virtual void parameter_sanity_check()
  {
  }

  // return true if the rpy mix is at lowest value
  virtual bool is_throttle_mix_min() const
  {
    return true;
  }

  // control rpy throttle mix
  virtual void set_throttle_mix_min()
  {
  }
  virtual void set_throttle_mix_man()
  {
  }
  virtual void set_throttle_mix_max(double ratio)
  {
  }
  virtual void set_throttle_mix_value(double value)
  {
  }
  virtual double get_throttle_mix(void) const
  {
    return 0;
  }

  // enable use of flybass passthrough on heli
  virtual void use_flybar_passthrough(bool passthrough, bool tail_passthrough)
  {
  }

  // use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage on heli
  virtual void use_leaky_i(bool leaky_i)
  {
  }

  // set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according
  // to vehicle condition.
  virtual void set_hover_roll_trim_scalar(double scalar)
  {
  }

  // Return angle in centidegrees to be added to roll angle for hover collective learn. Used by heli
  // to counteract tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
  virtual double get_roll_trim_cd()
  {
    return 0;
  }

  // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame
  // rate target for yaw
  virtual void passthrough_bf_roll_pitch_rate_yaw(
    double roll_passthrough,
    double pitch_passthrough,
    double yaw_rate_bf_cds){};

  // enable inverted flight on backends that support it
  virtual void set_inverted_flight(bool inverted)
  {
  }

  // get the slew rate value for roll, pitch and yaw, for oscillation detection in lua scripts
  void get_rpy_srate(double& roll_srate, double& pitch_srate, double& yaw_srate);

  // Sets the roll and pitch rate shaping time constant
  inline void set_roll_pitch_rate_tc(double input_tc);
  // Sets the yaw rate shaping time constant
  inline void set_yaw_rate_tc(double input_tc);

  // setup a one loop angle P scale multiplier. This replaces any previous scale applied
  // so should only be used when only one source of scaling is needed
  inline void set_angle_P_scale(const Vector3d& angle_P_scale);

  // setup a one loop angle P scale multiplier, multiplying by any
  // previously applied scale from this loop. This allows for more
  // than one type of scale factor to be applied for different
  // purposes
  inline void set_angle_P_scale_mult(const Vector3d& angle_P_scale);

  // get the value of the angle P scale that was used in the last loop, for logging
  inline const Vector3d& get_angle_P_scale_logging(void) const;

  // setup a one loop PD scale multiplier, multiplying by any
  // previously applied scale from this loop. This allows for more
  // than one type of scale factor to be applied for different
  // purposes
  inline void set_PD_scale_mult(const Vector3d& pd_scale);

  // get the value of the PD scale that was used in the last loop, for logging
  inline const Vector3d& get_PD_scale_logging(void) const;

protected:
  // Update rate_target_ang_vel using attitude_error_rot_vec_rad
  Vector3d update_ang_vel_target_from_att_error(const Vector3d& attitude_error_rot_vec_rad);

  // Return angle in radians to be added to roll angle. Used by heli to counteract
  // tail rotor thrust in hover. Overloaded by AC_Attitude_Heli to return angle.
  virtual double get_roll_trim_rad()
  {
    return 0;
  }

  // Return the yaw slew rate limit in radians/s
  double get_slew_yaw_max_rads() const
  {
    return radians(get_slew_yaw_max_degs());
  }

  // Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
  double _slew_yaw;

  // Maximum angular velocity (in degrees/second) for earth-frame roll, pitch and yaw axis
  double _ang_vel_roll_max;
  double _ang_vel_pitch_max;
  double _ang_vel_yaw_max;

  // Maximum rotation acceleration for earth-frame roll axis
  double _accel_roll_max;

  // Maximum rotation acceleration for earth-frame pitch axis
  double _accel_pitch_max;

  // Maximum rotation acceleration for earth-frame yaw axis
  double _accel_yaw_max;

  // Enable/Disable body frame rate feed forward
  char _rate_bf_ff_enabled;

  // Enable/Disable angle boost
  char _angle_boost_enabled;

  // angle controller P objects
  P _p_angle_roll;
  P _p_angle_pitch;
  P _p_angle_yaw;

  // Angle limit time constant (to maintain altitude)
  double _angle_limit_tc;

  // rate controller input smoothing time constant
  double _input_tc;

  // Intersampling period in seconds
  double _dt;

  // This represents a 321-intrinsic rotation in NED frame to the target (setpoint)
  // attitude used in the attitude controller, in radians.
  Vector3d _euler_angle_target;

  // This represents the angular velocity of the target (setpoint) attitude used in
  // the attitude controller as 321-intrinsic euler angle derivatives, in radians per
  // second.
  Vector3d _euler_rate_target;

  // This represents a quaternion rotation in NED frame to the target (setpoint)
  // attitude used in the attitude controller.
  QuaternionD _attitude_target;

  // This represents the angular velocity of the target (setpoint) attitude used in
  // the attitude controller as an angular velocity vector, in radians per second in
  // the target attitude frame.
  Vector3d _ang_vel_target;

  // This represents the angular velocity in radians per second in the body frame, used in the
  // angular velocity controller.
  Vector3d _ang_vel_body;

  // This is the angular velocity in radians per second in the body frame, added to the output
  // angular attitude controller by the System Identification Mode. It is reset to zero immediately
  // after it is used.
  Vector3d _sysid_ang_vel_body;

  // This is the unitless value added to the output of the PID by the System Identification Mode.
  // It is reset to zero immediately after it is used.
  Vector3d _actuator_sysid;

  // This represents a quaternion attitude error in the body frame, used for inertial frame reset
  // handling.
  QuaternionD _attitude_ang_error;

  // The angle between the target thrust vector and the current thrust vector.
  double _thrust_angle;

  // The angle between the target thrust vector and the current thrust vector.
  double _thrust_error_angle;

  // throttle provided as input to attitude controller.  This does not include angle boost.
  double _throttle_in = 0.;

  // This represents the throttle increase applied for tilt compensation.
  // Used only for logging.
  double _angle_boost;

  // Specifies whether the attitude controller should use the square root controller in the attitude
  // correction. This is used during Autotune to ensure the P term is tuned without being influenced
  // by the acceleration limit of the square root controller.
  bool _use_sqrt_controller;

  // Filtered Alt_Hold lean angle max - used to limit lean angle when throttle is saturated using
  // Alt_Hold
  double _althold_lean_angle_max = 0.;

  // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2
  // seconds
  double _throttle_rpy_mix_desired;

  // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
  double _throttle_rpy_mix;

  // Yaw feed forward percent to allow zero yaw actuator output during extreme roll and pitch
  // corrections
  double _feedforward_scalar = 1.;

  // rate controller input smoothing time constant
  double _rate_rp_tc;
  double _rate_y_tc;

  // angle P scaling vector for roll, pitch, yaw
  Vector3d _angle_P_scale{ 1, 1, 1 };

  // angle scale used for last loop, used for logging
  Vector3d _angle_P_scale_used;

  // PD scaling vector for roll, pitch, yaw
  Vector3d _pd_scale{ 1, 1, 1 };

  // PD scale used for last loop, used for logging
  Vector3d _pd_scale_used;

  Motors& _motors;

  /*
    state of control monitoring
  */
  struct
  {
    double rms_roll_P;
    double rms_roll_D;
    double rms_pitch_P;
    double rms_pitch_D;
    double rms_yaw;
  } _control_monitor;

  // update state in ControlMonitor
  void control_monitor_filter_pid(double value, double& rms_P);
  void control_monitor_update(void);

  // true in inverted flight mode
  bool _inverted_flight;

public:
  // structure for angle and/or rate target
  enum class HeadingMode
  {
    Angle_Only,
    Angle_And_Rate,
    Rate_Only
  };

  struct HeadingCommand
  {
    double yaw_angle_cd;
    double yaw_rate_cds;
    HeadingMode heading_mode;
  };

  void input_thrust_vector_heading(const Vector3d& thrust_vector, HeadingCommand heading);
};

inline void AttitudeControl::set_dt(double dt)
{
  _dt = dt;
}

inline double AttitudeControl::get_dt() const
{
  return _dt;
}

inline P& AttitudeControl::get_angle_roll_p()
{
  return _p_angle_roll;
}

inline P& AttitudeControl::get_angle_pitch_p()
{
  return _p_angle_pitch;
}

inline P& AttitudeControl::get_angle_yaw_p()
{
  return _p_angle_yaw;
}

inline double AttitudeControl::get_accel_roll_max_cdss() const
{
  return _accel_roll_max;
}

inline double AttitudeControl::get_accel_roll_max_radss() const
{
  return radians(_accel_roll_max * 0.01f);
}

inline void AttitudeControl::set_accel_roll_max_cdss(double accel_roll_max)
{
  _accel_roll_max = accel_roll_max;
}

inline double AttitudeControl::get_accel_pitch_max_cdss() const
{
  return _accel_pitch_max;
}

inline double AttitudeControl::get_accel_pitch_max_radss() const
{
  return radians(_accel_pitch_max * 0.01);
}

inline void AttitudeControl::set_accel_pitch_max_cdss(double accel_pitch_max)
{
  _accel_pitch_max = accel_pitch_max;
}

inline double AttitudeControl::get_accel_yaw_max_cdss() const
{
  return _accel_yaw_max;
}

inline double AttitudeControl::get_accel_yaw_max_radss() const
{
  return radians(_accel_yaw_max * 0.01);
}

inline void AttitudeControl::set_accel_yaw_max_cdss(double accel_yaw_max)
{
  _accel_yaw_max = accel_yaw_max;
}

inline double AttitudeControl::get_ang_vel_roll_max_rads() const
{
  return radians(_ang_vel_roll_max);
}

inline double AttitudeControl::get_ang_vel_pitch_max_rads() const
{
  return radians(_ang_vel_pitch_max);
}

inline double AttitudeControl::get_ang_vel_yaw_max_rads() const
{
  return radians(_ang_vel_yaw_max);
}

inline double AttitudeControl::get_input_tc() const
{
  return _input_tc;
}

inline void AttitudeControl::set_input_tc(double input_tc)
{
  _input_tc = std::clamp(input_tc, 0., 1.);
}

inline void
AttitudeControl::input_thrust_vector_heading(const Vector3d& thrust_vector, double heading_cd)
{
  input_thrust_vector_heading(thrust_vector, heading_cd, 0.);
}

inline void AttitudeControl::use_sqrt_controller(bool use_sqrt_cont)
{
  _use_sqrt_controller = use_sqrt_cont;
}

inline Vector3d AttitudeControl::get_att_target_euler_cd() const
{
  return _euler_angle_target * degrees(100.);
}

inline const Vector3d& AttitudeControl::get_att_target_euler_rad() const
{
  return _euler_angle_target;
}

inline QuaternionD AttitudeControl::get_attitude_target_quat() const
{
  return _attitude_target;
}

inline const Vector3d& AttitudeControl::get_attitude_target_ang_vel() const
{
  return _ang_vel_target;
}

inline double AttitudeControl::get_att_error_angle_deg() const
{
  return degrees(_thrust_error_angle);
}

inline void AttitudeControl::rate_bf_roll_target(double rate_cds)
{
  _ang_vel_body.x = radians(rate_cds * 0.01);
}

inline void AttitudeControl::rate_bf_pitch_target(double rate_cds)
{
  _ang_vel_body.y = radians(rate_cds * 0.01);
}

inline void AttitudeControl::rate_bf_yaw_target(double rate_cds)
{
  _ang_vel_body.z = radians(rate_cds * 0.01);
}

inline void AttitudeControl::rate_bf_roll_sysid(double rate)
{
  _sysid_ang_vel_body.x = rate;
}

inline void AttitudeControl::rate_bf_pitch_sysid(double rate)
{
  _sysid_ang_vel_body.y = rate;
}

inline void AttitudeControl::rate_bf_yaw_sysid(double rate)
{
  _sysid_ang_vel_body.z = rate;
}

inline void AttitudeControl::actuator_roll_sysid(double command)
{
  _actuator_sysid.x = command;
}

inline void AttitudeControl::actuator_pitch_sysid(double command)
{
  _actuator_sysid.y = command;
}

inline void AttitudeControl::actuator_yaw_sysid(double command)
{
  _actuator_sysid.z = command;
}

inline Vector3d AttitudeControl::rate_bf_targets() const
{
  return _ang_vel_body + _sysid_ang_vel_body;
}

inline const Vector3d& AttitudeControl::get_rate_ef_targets() const
{
  return _euler_rate_target;
}

inline void AttitudeControl::bf_feedforward(bool enable_or_disable)
{
  _rate_bf_ff_enabled = enable_or_disable;
}

inline bool AttitudeControl::get_bf_feedforward()
{
  return _rate_bf_ff_enabled;
}

inline double AttitudeControl::get_throttle_in() const
{
  return _throttle_in;
}

inline double AttitudeControl::angle_boost() const
{
  return _angle_boost;
}

inline double AttitudeControl::lean_angle_deg() const
{
  return degrees(_thrust_angle);
}

inline double AttitudeControl::input_shaping_angle(
  double error_angle,
  double input_tc,
  double accel_max,
  double target_ang_vel,
  double dt)
{
  return input_shaping_angle(error_angle, input_tc, accel_max, target_ang_vel, 0., 0., dt);
}

inline void AttitudeControl::set_roll_pitch_rate_tc(double input_tc)
{
  _rate_rp_tc = input_tc;
}

inline void AttitudeControl::set_yaw_rate_tc(double input_tc)
{
  _rate_y_tc = input_tc;
}

inline void AttitudeControl::set_angle_P_scale(const Vector3d& angle_P_scale)
{
  _angle_P_scale = angle_P_scale;
}

inline void AttitudeControl::set_angle_P_scale_mult(const Vector3d& angle_P_scale)
{
  _angle_P_scale *= angle_P_scale;
}

inline const Vector3d& AttitudeControl::get_angle_P_scale_logging(void) const
{
  return _angle_P_scale_used;
}

inline void AttitudeControl::set_PD_scale_mult(const Vector3d& pd_scale)
{
  _pd_scale *= pd_scale;
}

inline const Vector3d& AttitudeControl::get_PD_scale_logging(void) const
{
  return _pd_scale_used;
}
}  // namespace tobas_mr_arducopter
