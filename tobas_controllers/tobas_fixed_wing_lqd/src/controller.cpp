#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/frames.hpp>
#include <tobas_ros2_tools/rosparam.hpp>
#include <tobas_tools/conversions/coordinates.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/RotorSpeeds.h>

#include "../include/tobas_fixed_wing_lqd/controller.hpp"
#include "../include/tobas_fixed_wing_lqd/constants.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_fixed_wing_lqd
{
Controller::Controller(, const string& name)
  : super(node, pnh, name), x_rotors_(drone_, tobas::X_POSITIVE), eom_(drone_), server_(pnh_)
{
  drone_.loadFromParam(node_);

  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  if (x_rotors_.count() == 0)
    TOBAS_EXIT("The number of propellers is zero.");

  q_0_.resize(tree_.getNrOfJoints());

  setScales();
  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);
  lqd_.last_input = VectorXd::Zero(eom_.inputSize());

  // Register publishers
  rot_speeds_pub_ = node_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  deflections_pub_ = node_.advertise<tobas_msgs::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic, 1);
  feedback_pub_ = node_.advertise<tobas_msgs::FixedWingControllerFeedback>("fixed_wing_controller_feedback", 1);

  // Register subscribers
  air_pressure_sub_ = node_.subscribe(tobas::kAirPressureTopic, 1, &self::airPressureCb, this, tcpNoDelay());
  battery_sub_ = node_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  odom_sub_ = node_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  arming_sub_ = node_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this, tcpNoDelay());
  cmd_sub_ = node_.subscribe(tobas::kSpeedRollDpitchCmdTopic, 1, &self::commandCb, this, tcpNoDelay());

  // Dynamic Reconfigure
  server_.setCallback(std::bind(&self::dynamicReconfigureCb, this, _1, _2));
}

bool Controller::isReadyToControl()
{
  if (air_pressure_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kAirPressureTopic);
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kBatteryLpfTopic);
    return false;
  }

  if (odom_nwu_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kOdometryTopic);
    return false;
  }

  if (odom_nwu_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kArmingTopic);
    return false;
  }

  if (!arming_->data)
    return false;

  return true;
}

void Controller::setScales()
{
  // 状態変数のスケール
  lqd_.state_scale.resize(eom_.kStateSize);
  lqd_.state_scale(eom_.kStateIdx_u) = eom_.trimCondition().takeOffSpeed(kStandardAirDensity);
  lqd_.state_scale(eom_.kStateIdx_alpha) = drone_.fixed_wing.vehicle.alpha_limit.range();
  lqd_.state_scale(eom_.kStateIdx_beta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_phi) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_theta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_p) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_q) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_r) = M_PI;

  // 制御入力のスケール
  lqd_.input_scale.resize(eom_.inputSize());
  const auto thrust_scale = tobas::getMass() * tobas::kGravity / x_rotors_.count();
  lqd_.input_scale.block(0, 0, x_rotors_.count(), 1).fill(thrust_scale);
  for (size_t i = 0; i < drone_.fixed_wing.control_surfaces.size(); ++i)
    lqd_.input_scale(x_rotors_.count() + i) = drone_.fixed_wing.control_surfaces.at(i).angle_limit.range();
}

void Controller::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();
  odom_ned_.frame.M.getRPY(cur_roll_, cur_pitch_, cur_yaw_);

  // TODO: 横系のトリムも考慮
  lqd_.current_state(eom_.kStateIdx_u) = odom_ned_.twist.vel.x() - trim.u();
  lqd_.current_state(eom_.kStateIdx_alpha) = tobas::angleOfAttack(odom_ned_.twist.vel) - trim.alpha();
  lqd_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(odom_ned_.twist.vel);
  lqd_.current_state(eom_.kStateIdx_phi) = cur_roll_;
  lqd_.current_state(eom_.kStateIdx_theta) = cur_pitch_ - trim.theta();
  lqd_.current_state(eom_.kStateIdx_p) = odom_ned_.twist.rot.x();
  lqd_.current_state(eom_.kStateIdx_q) = odom_ned_.twist.rot.y();
  lqd_.current_state(eom_.kStateIdx_r) = odom_ned_.twist.rot.z();
}

void Controller::updateSetStateVector()
{
  const auto& trim = eom_.trimCondition();

  // 失速しないように速度制限をした上で目標推力を計算
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  const auto tar_speed = trim.speedLimit(rho).clamp(cmd_ned_.speed);
  const auto tar_u = tar_speed * cos(eom_.trimCondition().alpha());

  lqd_.target_state(eom_.kStateIdx_u) = tar_u - trim.u();
  lqd_.target_state(eom_.kStateIdx_alpha) = 0.;
  lqd_.target_state(eom_.kStateIdx_beta) = 0.;
  lqd_.target_state(eom_.kStateIdx_phi) = cmd_ned_.roll;
  lqd_.target_state(eom_.kStateIdx_theta) = cmd_ned_.delta_pitch;
  lqd_.target_state(eom_.kStateIdx_p) = 0.;
  lqd_.target_state(eom_.kStateIdx_q) = 0.;
  lqd_.target_state(eom_.kStateIdx_r) = 0.;
}

void Controller::publishRotSpeeds(const VectorXd& thrust)
{
  const auto rot_speeds = make_unique<tobas_msgs::RotorSpeeds>();
  rot_speeds->header.stamp = odom_ned_.header.stamp;

  rot_speeds->speeds.resize(drone_.rotors.size(), 0.);
  for (size_t i = 0; i < static_cast<size_t>(thrust.rows()); ++i)
    rot_speeds->speeds[x_rotors_.rotorIdx(i)] = x_rotors_.rotSpeedFromThrust(i, max(0., thrust(i)));

  rot_speeds_pub_.publish(rot_speeds);
}

void Controller::publishDeflections(const VectorXd& deflections)
{
  const auto deflections_msg = make_unique<tobas_msgs::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = odom_ned_.header.stamp;
  deflections_msg->deflections = eigen_tools::toStdVector(deflections);
  deflections_pub_.publish(deflections_msg);
}

void Controller::publishFeedback(const VectorXd& du)
{
  const auto& trim = eom_.trimCondition();
  const auto feedback = make_unique<tobas_msgs::FixedWingControllerFeedback>();

  feedback->trim_thrusts.resize(drone_.rotors.size());
  feedback->delta_thrusts.resize(drone_.rotors.size());
  feedback->trim_deflections.resize(drone_.fixed_wing.control_surfaces.size());
  feedback->delta_deflections.resize(drone_.fixed_wing.control_surfaces.size());

  feedback->trim_u = trim.u();
  feedback->trim_alpha = trim.alpha();

  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    feedback->trim_thrusts[x_rotors_.rotorIdx(i)] = eom_.trimInput()(i);
    feedback->delta_thrusts[x_rotors_.rotorIdx(i)] = du(i);
  }

  for (size_t i = 0; i < drone_.fixed_wing.control_surfaces.size(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback->trim_deflections[i] = eom_.trimInput()[u_idx];
    feedback->delta_deflections[i] = du(u_idx);
  }

  feedback_pub_.publish(feedback);
}

void Controller::airPressureCb(const sensor_msgs::msg::FluidPressureConstPtr& msg)
{
  air_pressure_ = msg;
}

void Controller::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void Controller::odomCb(const tobas_msgs::OdometryConstPtr& odom_nwu)
{
  if (odom_nwu_ == nullptr)
  {
    odom_nwu_ = odom_nwu;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom_nwu->header.stamp - odom_nwu_->header.stamp).seconds();
  odom_nwu_ = odom_nwu;

  if (!isReadyToControl())
    return;

  // コマンドが来ていなければスキップ
  if (cmd_nwu_ == nullptr)
    return;

  // NWU -> NED
  tf::odometryNwuToNed(*odom_nwu_, odom_ned_);
  tf::speedRollDeltaPitchNwuToNed(*cmd_nwu_, cmd_ned_);

  // 現在の速度を使って状態方程式を更新
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  switch (eom_.update(odom_ned_.twist.vel.norm(), rho, battery_->voltage, q_0_))
  {
    case tobas::SolverI::E_NO_ERROR:
      break;
    case tobas::SolverI::E_WARN:
      TOBAS_WARN(eom_.errorMessage());
      break;
    case tobas::SolverI::E_ERROR:
      TOBAS_ERROR(eom_.errorMessage());
      return;
    default:
      TOBAS_WARN("Unknown error code from MicroDisturbanceEoM.");
      break;
  }

  lqd_.dynamics.A = eom_.A();
  lqd_.dynamics.B = eom_.B();

  updateCurrentStateVector();
  updateSetStateVector();

  // 最適制御入力を求める
  const VectorXd du = lqd_.solve(dt);
  const VectorXd u = eom_.trimInput() + du;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << lqd_ << endl;

  const VectorXd thrust = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.fixed_wing.control_surfaces.size(), 1);

  // Publish
  publishRotSpeeds(thrust);
  publishDeflections(deflections);
  publishFeedback(du);
}

void Controller::armingCb(const std_msgs::BoolConstPtr& arming)
{
  arming_ = arming;

  if (!arming->data)
  {
    cmd_nwu_ = nullptr;
    lqd_.last_input.setZero();
    TOBAS_INFO("Controller is reset.");
  }
}

void Controller::commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  // TODO: コマンドレベルの処理

  cmd_nwu_ = cmd_nwu;
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  if (cfg.forward_speed_weight <= 0)
  {
    TOBAS_ERROR("Forward speed weight must be positive.");
    return;
  }
  if (cfg.alpha_weight <= 0)
  {
    TOBAS_ERROR("Alpha weight must be positive.");
    return;
  }
  if (cfg.beta_weight <= 0)
  {
    TOBAS_ERROR("Beta weight must be positive.");
    return;
  }
  if (cfg.attitude_weight <= 0)
  {
    TOBAS_ERROR("Attitude weight must be positive.");
    return;
  }
  if (cfg.angular_velocity_weight <= 0)
  {
    TOBAS_ERROR("Angular velocity weight must be positive.");
    return;
  }

  // 状態変数の重み
  lqd_.state_weight(eom_.kStateIdx_u) = cfg.forward_speed_weight;
  lqd_.state_weight(eom_.kStateIdx_alpha) = cfg.alpha_weight;
  lqd_.state_weight(eom_.kStateIdx_beta) = cfg.beta_weight;
  lqd_.state_weight(eom_.kStateIdx_phi) = cfg.attitude_weight;
  lqd_.state_weight(eom_.kStateIdx_theta) = cfg.attitude_weight;
  lqd_.state_weight(eom_.kStateIdx_p) = cfg.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_q) = cfg.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_r) = cfg.angular_velocity_weight;

  // 制御入力の重み
  const auto thrust_weight = exp10(cfg.thrust_weight_log10);
  const auto deflection_weight = exp10(cfg.deflection_weight_log10);
  lqd_.input_weight.head(x_rotors_.count()).fill(thrust_weight);
  lqd_.input_weight.tail(drone_.fixed_wing.control_surfaces.size()).fill(deflection_weight);

  // 制御入力の変化率の重み
  const auto thrust_rate_weight = exp10(cfg.thrust_rate_weight_log10);
  const auto deflection_rate_weight = exp10(cfg.deflection_rate_weight_log10);
  lqd_.input_rate_weight.head(x_rotors_.count()).fill(thrust_rate_weight);
  lqd_.input_rate_weight.tail(drone_.fixed_wing.control_surfaces.size()).fill(deflection_rate_weight);

  TOBAS_INFO("Dynamic parameters are updated.");
}
}  // namespace tobas_fixed_wing_lqd
