#include <algorithm>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_gazebo_plugins/RotorDebug.h>

#include "./rotor_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/time.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboRotorPlugin::GazeboRotorPlugin() : super()
{
}

void GazeboRotorPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  // Get SDF parameters
  getSdfParams(sdf);

  // Add model error to the model constants
  addModelError();

  // Store the pointer to the model
  model_ = model;

  // Get the pointer to the joint
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified joint \"" << joint_name_ << "\".");

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  parent_link_ = link_->GetParentJointsLinks()[0];

  // Initialize the first order filter
  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, 0.);

  // Register publishers and subscribers to the ROS master
  registerPubSub();

  // Listen to the update event
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboRotorPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "jointName", joint_name_);
  getSdfParam(sdf, "motorNumber", motor_number_, NON_NEGATIVE);

  if (sdf->HasElement("turningDirection"))
  {
    const auto turning_direction = sdf->GetElement("turningDirection")->Get<string>();
    if (turning_direction == "cw")
      direction_ = -1;
    else if (turning_direction == "ccw")
      direction_ = 1;
    else
      gzthrow(kPluginName << ": Please only use 'cw' or 'ccw' as turningDirection.");
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a turning direction ('cw' or 'ccw').");
  }

  getSdfParam(sdf, "rotSpeedCoefficients", rot_speed_coefs_);
  if (rot_speed_coefs_.X() <= 0)
    gzthrow(kPluginName << ": The first term of 'rotationSpeedCoefficients' must be positive.");
  if (rot_speed_coefs_.Y() < 0)
    gzthrow(
      kPluginName << ": The second term of 'rotationSpeedCoefficients' must be non-negative.");

  getSdfParam(sdf, "motorConstant", motor_const_, NON_NEGATIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, NON_NEGATIVE);
  getSdfParam(sdf, "rotorDragCoefficient", rotor_drag_coef_, NON_NEGATIVE);

  getSdfParam(
    sdf, "maxModelErrorRate", max_model_error_rate_, kDefaultMaxModelErrorRate, NON_NEGATIVE);

  getSdfParam(sdf, "timeConstantUp", time_const_up_, POSITIVE);
  getSdfParam(sdf, "timeConstantDown", time_const_down_, POSITIVE);
  if (time_const_up_ > kTimeConstWarnThreshold)
    gzwarn << kPluginName << ": The value provided for 'timeConstantUp' appears to be too large: "
           << time_const_up_ << "[s]. Please check settings and datasheet." << endl;
  if (time_const_down_ > kTimeConstWarnThreshold)
    gzwarn << kPluginName << ": The value provided for 'timeConstantDown' appears to be too large: "
           << time_const_down_ << "[s]. Please check settings and datasheet." << endl;

  getSdfParam(
    sdf, "checkDelayThreshold", check_delay_threshold_, kDefaultCheckDelayThreshold, false);
  getSdfParam(
    sdf, "autoResetTimeThreshold", auto_reset_time_thr_, kDefaultAutoStopTimeThreshold, false);
}

void GazeboRotorPlugin::onUpdate(const common::UpdateInfo& info)
{
  const auto& cur_time = info.simTime;

  if (!is_initialized_)
  {
    if (isReady())
      is_initialized_ = true;

    // Check topics
    // ros::Timer cannot be used for shared library.
    if (cur_time > kCheckTopicsTimeThreshold)
    {
      if (!battery_received_)
        gzerr << kPluginName << ": " << ns_ << "/" << tobas::kBatteryGtTopic
              << " is not received yet." << endl;
      if (!wind_received_)
        gzerr << kPluginName << ": " << ns_ << "/" << tobas::kWindGtTopic << " is not received yet."
              << endl;
    }
    return;
  }

  // Check elapsed time after last command
  const auto time_after_last_cmd = cur_time - last_cmd_time_;
  if (is_activated_ && time_after_last_cmd > auto_reset_time_thr_)
  {
    cmd_rot_speed_ = minRotSpeed();
    is_activated_ = false;
    gzmsg << kPluginName << ": Motor " << motor_number_ << " is automatically stopped because "
          << auto_reset_time_thr_ << " seconds have elapsed since the last command." << endl;
  }

  // Get rotation speed
  const auto rot_speed_sim = joint_->GetVelocity(0);
  const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;

  // Compute time after previous simulation time
  const auto dt = (cur_time - prev_sim_time_).Double();
  prev_sim_time_ = cur_time;

  // Check aliasing
  if (abs(rot_speed_sim) * dt > M_PI)
  {
    GZ_WARN_THROTTLE(
      kWarnPeriod, kPluginName << ": Aliasing on motor [" << motor_number_
                               << "] might occur. Lower simulation time step.");
  }

  // Update simulation state
  applyForceAndTorque(rot_speed_real, info.simTime);
  updateRotationSpeed(dt);
}

void GazeboRotorPlugin::registerPubSub()
{
  debug_pub_ = nh_.advertise<tobas_gazebo_plugins::RotorDebug>(
    "/" + ns_ + "/" + kDebugTopicPrefix + "_" + to_string(motor_number_), 1);

  rotor_speeds_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + tobas::kRotorSpeedsCmdTopic, 1, &self::rotorSpeedsCmdCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
  throttles_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + tobas::kThrottlesCmdTopic, 1, &self::throttlesCmdCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
  battery_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + tobas::kBatteryGtTopic, 1, &self::batteryCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
  wind_sub_ = nh_.subscribe(
    "/" + ns_ + "/" + tobas::kWindGtTopic, 1, &self::windSpeedCb, this,
    ros::TransportHints().reliable().tcpNoDelay());
}

bool GazeboRotorPlugin::isReady()
{
  return battery_received_ && wind_received_;
}

void GazeboRotorPlugin::addModelError()
{
  // 一様乱数を作成
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  UniformDistribution uniform(-1, 1);

  // 回転数-電圧の関係式
  // 1次の係数はKv値から概ね正確な値が分かるため，2次の係数にのみ誤差を加える．
  rot_speed_coefs_.Y() *= (1 + max_model_error_rate_ * uniform(rnd_gen));

  // 空力定数
  motor_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  moment_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  rotor_drag_coef_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
}

void GazeboRotorPlugin::applyForceAndTorque(const double& rot_speed, const common::Time cur_time)
{
  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // Get joint axes
  const auto global_axis = joint_->GlobalAxis(0);
  const auto local_axis = joint_->LocalAxis(0);

  // (1) first term: Thrust Force
  const auto rot_speed_sgn = dh_std::sign(rot_speed);
  const auto thrust = direction_ * rot_speed_sgn * motor_const_ * dh_std::sqr(rot_speed);
  const auto thrust_W = thrust * global_axis;
  link_->AddForce(thrust_W);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVel() - wind_vel_W_;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-abs(rot_speed) * rotor_drag_coef_) * linvel_perp_W;
  link_->AddForce(h_force_W);

  // (2) first term: Rotor drag torque
  const auto pose_diff = link_->WorldCoGPose() - parent_link_->WorldCoGPose();
  const auto drag_torque_child = (-direction_ * thrust * moment_const_) * local_axis;
  const auto drag_torque_parent = pose_diff.Rot().RotateVector(drag_torque_child);
  parent_link_->AddRelativeTorque(drag_torque_parent);

  // Publish debug message
  const auto debug_msg = boost::make_shared<tobas_gazebo_plugins::RotorDebug>();
  timeGazeboToRos(cur_time, debug_msg->header.stamp);
  debug_msg->rotation_speed = joint_->GetVelocity(0) * kRotorSpeedSlowdownSim;
  vectorGazeboToKDL(thrust_W, debug_msg->thrust_force);
  vectorGazeboToKDL(h_force_W, debug_msg->horizontal_force);
  vectorGazeboToKDL(drag_torque_parent, debug_msg->drag_torque);
  debug_pub_.publish(debug_msg);
}

void GazeboRotorPlugin::updateRotationSpeed(const double& dt)
{
  assert(dt > 0.);

  // Check rotor speed limit and get set value
  auto set_rot_speed = cmd_rot_speed_;
  const auto max_rot_speed = maxRotSpeed();
  const auto min_rot_speed = minRotSpeed();
  if (cmd_rot_speed_ < min_rot_speed)
  {
    // エラーを出すのは指令値が負のときのみ．[0, min_rot_speed]の時は修正するだけにする．
    if (cmd_rot_speed_ < 0.)
    {
      gzerr << kPluginName << ": Negative rotor speed is commanded on index " << motor_number_
            << ": " << cmd_rot_speed_ << " < 0 [rad/s]" << endl;
    }
    set_rot_speed = min_rot_speed;
  }
  else if (cmd_rot_speed_ > max_rot_speed + kRotorSpeedCheckMargin)
  {
    gzerr << kPluginName << ": Commanded rotor speed on index " << motor_number_
          << " is too high: " << cmd_rot_speed_ << " > " << max_rot_speed << " [rad/s]" << endl;
    set_rot_speed = max_rot_speed;
  }

  // Apply the filter on the rotation speed
  const auto ref_rot_speed = rotor_speed_filter_.update(set_rot_speed, dt);
  joint_->SetVelocity(0, direction_ * ref_rot_speed / kRotorSpeedSlowdownSim);
}

void GazeboRotorPlugin::processCommandCommon(const size_t& data_size, const ros::Time& stamp)
{
  // Check index
  if (motor_number_ >= static_cast<int>(data_size))
  {
    gzerr << kPluginName << ": You tried to access index " << motor_number_
          << " of the rotor command message array which is of size " << data_size << endl;
    return;
  }

  // Check delay
  const auto delay = (prev_sim_time_ - stamp).toSec();
  // cout << "delay: " << delay << " [s]" << endl;
  if (delay > check_delay_threshold_)
  {
    GZ_WARN_THROTTLE(
      kWarnPeriod, kPluginName << ": The delay from sensors to the motor command " << delay
                               << "[s] is over " << check_delay_threshold_ << "[s].");
  }
  else if (delay < 0)
  {
    GZ_ERROR_THROTTLE(
      kErrorPeriod, kPluginName << ": Timestamp of the motor command precedes the current time.");
  }

  // Update last commanded time
  last_cmd_time_ = prev_sim_time_;

  // Now the motor is activated
  is_activated_ = true;
}

double GazeboRotorPlugin::rotSpeedFromVoltage(const double& voltage)
{
  const auto& a = rot_speed_coefs_.X();
  const auto& b = rot_speed_coefs_.Y();
  return b > 0 ? (sqrt(dh_std::sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

double GazeboRotorPlugin::maxRotSpeed()
{
  return rotSpeedFromVoltage(battery_->voltage);
}

double GazeboRotorPlugin::minRotSpeed()
{
  return maxRotSpeed() * tobas::kMotorSpinArm;
}

void GazeboRotorPlugin::rotorSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  if (!battery_received_)
    return;

  processCommandCommon(rotor_speeds->speeds.size(), rotor_speeds->header.stamp);
  cmd_rot_speed_ = rotor_speeds->speeds[motor_number_];
}

void GazeboRotorPlugin::throttlesCmdCb(const tobas_msgs::ThrottlesConstPtr& throttles)
{
  if (!battery_received_)
    return;

  processCommandCommon(throttles->data.size(), throttles->header.stamp);
  const auto input_voltage = battery_->voltage * std::clamp(throttles->data[motor_number_], 0., 1.);
  cmd_rot_speed_ = rotSpeedFromVoltage(input_voltage);
}

void GazeboRotorPlugin::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;

  // 最初のバッテリー電圧取得時に最小回転数を目標回転数に設定する
  if (!battery_received_)
  {
    battery_received_ = true;
    cmd_rot_speed_ = minRotSpeed();
  }
}

void GazeboRotorPlugin::windSpeedCb(const tobas_msgs::WindConstPtr& wind)
{
  vectorKDLToGazebo(wind->vel, wind_vel_W_);

  if (!wind_received_)
  {
    wind_received_ = true;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRotorPlugin);
}  // namespace gazebo
