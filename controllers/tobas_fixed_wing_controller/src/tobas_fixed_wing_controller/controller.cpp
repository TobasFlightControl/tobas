#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/util.hpp>

#include <tobas_tools/utils.hpp>
#include <tobas_tools/conversions/coordinates.hpp>

#include "../../include/tobas_fixed_wing_controller/controller.hpp"
#include "../../include/tobas_fixed_wing_controller/constants.hpp"

#define WEIGHT_SCALER 1e+6  // TODO: QPの数値エラーを防ぐために重みにかける定数を自動調整

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace dh_std;

namespace tobas_fixed_wing_controller
{
Controller::Controller() : super()
{
  getRosParams();
  drone_.loadFromParam(ns_);

  hor_prop_idxes_ = drone_.rotorConfigIdxInAxis(Axis::X_POSITIVE);
  num_hor_props_ = hor_prop_idxes_.size();
  num_cs_ = drone_.fixedWingConfig().control_surfaces.size();
  u_dim_ = num_hor_props_ + num_cs_;

  is_initialized_ = false;
  rotor_speeds_msg_.speeds.resize(drone_.numRotors());
  deflections_msg_.deflections.resize(num_cs_);

  cont_.reset(new FixedWingMicroDisturbanceDynamics(drone_));
  c2d_.reset(new ctrl::C2D_RK4(cont_->stateSize, u_dim_));

  mpc_.decay_time_consts.resize(ctrlSize);
  setCz();
  mpc_.input_rate_weight.resize(u_dim_);
  mpc_.input_weight.resize(u_dim_);
  mpc_.control_weight.resize(ctrlSize);
  setInputRateConstraint();
  mpc_.control_constraint.resize(ctrlSize, 0);
  mpc_.current_state.resize(cont_->stateSize);
  mpc_.set_state.resize(ctrlSize);
  mpc_.last_input = VectorXd::Zero(u_dim_);

  dynamicReconfigureCb(init_dynamic_config_, 0);

  registerPublishers();
  registerSubscribers();
  createTimers();
}

void Controller::getRosParams()
{
  dh_ros::getParam(ctrlName + "/prediction_horizon", init_dynamic_config_.prediction_horizon);
  dh_ros::getParam(ctrlName + "/prediction_steps", init_dynamic_config_.prediction_steps);
  dh_ros::getParam(ctrlName + "/beta_decay", init_dynamic_config_.beta_decay);
  dh_ros::getParam(ctrlName + "/rotation_decay", init_dynamic_config_.rotation_decay);
  dh_ros::getParam(ctrlName + "/beta_weight", init_dynamic_config_.beta_weight);
  dh_ros::getParam(ctrlName + "/rotation_weight", init_dynamic_config_.rotation_weight);
  dh_ros::getParam(
    ctrlName + "/thrust_force_weight_exp", init_dynamic_config_.thrust_force_weight_exp);
  dh_ros::getParam(
    ctrlName + "/thrust_force_rate_weight_exp", init_dynamic_config_.thrust_force_rate_weight_exp);
  dh_ros::getParam(ctrlName + "/deflection_weight_exp", init_dynamic_config_.deflection_weight_exp);
  dh_ros::getParam(
    ctrlName + "/deflection_rate_weight_exp", init_dynamic_config_.deflection_rate_weight_exp);
}

void Controller::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1, false);
  deflections_pub_ =
    nh_.advertise<tobas_msgs::ControlSurfaceDeflections>("command/deflections", 1, false);
}

void Controller::registerSubscribers()
{
  base_state_sub_ = nh_.subscribe("base_state", 1, &Controller::baseStateCb, this);
  cmd_sub_ = nh_.subscribe("command/speed_roll_delta_pitch", 1, &Controller::commandCb, this);
}

void Controller::createTimers()
{
  check_topics_timer_ =
    nh_.createTimer(ros::Duration(checkTopicsTimerPeriod), &Controller::checkTopicsTimerCb, this);
}

void Controller::initialize()
{
}

void Controller::runOnce()
{
  updateCurrentStateVector();

  // MPCを解いて最適制御入力を求める
  VectorXd du = mpc_.solveMPC();
  VectorXd u = cont_->trimInput() + du;

  // 各ロータの回転数を発行
  VectorXd thrust = u.block(0, 0, num_hor_props_, 1);
  updateRotorSpeeds(thrust);
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  // 各操舵面の偏角を発行
  VectorXd deflections = u.block(num_hor_props_, 0, num_cs_, 1);
  updateDeflections(deflections);
  deflections_pub_.publish(deflections_msg_);
}

void Controller::setCz()
{
  mpc_.Cz = MatrixXd::Zero(ctrlSize, cont_->stateSize);

  mpc_.Cz(ctrlIdx_beta, cont_->stateIdx_beta) = 1;
  mpc_.Cz(ctrlIdx_phi, cont_->stateIdx_phi) = 1;
  mpc_.Cz(ctrlIdx_theta, cont_->stateIdx_theta) = 1;
}

void Controller::setInputConstraint()
{
  // TODO
}

void Controller::setInputRateConstraint()
{
  VectorXd lb = VectorXd::Constant(u_dim_, numeric_limits<double>::lowest());
  VectorXd ub = VectorXd::Constant(u_dim_, numeric_limits<double>::max());

  for (int i = 0; i < num_cs_; ++i)
  {
    const auto& max_angle_rate = drone_.fixedWingConfig().control_surfaces[i].max_angle_rate;
    lb(num_hor_props_ + i) = -max_angle_rate;
    ub(num_hor_props_ + i) = +max_angle_rate;
  }

  mpc_.input_rate_constraint = ctrl::matIneqFromRange(lb, ub);
}

void Controller::updateWeight_Q(double beta_weight, double rot_weight)
{
  constexpr double beta_scale = M_PI;
  constexpr double rot_scale = M_PI;

  mpc_.control_weight(ctrlIdx_beta) = beta_weight / sqr(beta_scale) * WEIGHT_SCALER;
  mpc_.control_weight(ctrlIdx_phi) = mpc_.control_weight(ctrlIdx_theta) =
    rot_weight / sqr(rot_scale) * WEIGHT_SCALER;
}

void Controller::updateWeight_S(int thrust_weight_exp, int deflection_weight_exp)
{
  for (int i = 0; i < num_hor_props_; ++i)
  {
    double thrust_scale = drone_.maxThrust(hor_prop_idxes_[i]);
    mpc_.input_weight(i) = pow(10, thrust_weight_exp) / sqr(thrust_scale) * WEIGHT_SCALER;
  }

  constexpr double deflection_scale = M_PI;
  double deflection_weight = pow(10, deflection_weight_exp) / sqr(deflection_scale) * WEIGHT_SCALER;
  mpc_.input_weight.block(num_hor_props_, 0, num_cs_, 1) =
    VectorXd::Constant(num_cs_, deflection_weight);
}

void Controller::updateWeight_R(
  int thrust_rate_weight_exp,
  int deflection_rate_weight_exp,
  double dt)
{
  for (int i = 0; i < num_hor_props_; ++i)
  {
    double thrust_rate_scale = drone_.maxThrust(hor_prop_idxes_[i]) * dt;
    mpc_.input_rate_weight(i) =
      pow(10, thrust_rate_weight_exp) / sqr(thrust_rate_scale) * WEIGHT_SCALER;
  }

  double deflection_rate_scale = M_PI * dt;
  double deflection_rate_weight =
    pow(10, deflection_rate_weight_exp) / sqr(deflection_rate_scale) * WEIGHT_SCALER;
  mpc_.input_rate_weight.block(num_hor_props_, 0, num_cs_, 1) =
    VectorXd::Constant(num_cs_, deflection_rate_weight);
}

void Controller::updateCurrentStateVector()
{
  const Vector linvel_B = cur_bs_.pose.euler * cur_bs_.twist.vel;

  mpc_.current_state(cont_->stateIdx_u) = linvel_B.x() - cont_->trimState_u();
  mpc_.current_state(cont_->stateIdx_alpha) = angleOfAttack(linvel_B) - cont_->trimState_alpha();
  mpc_.current_state(cont_->stateIdx_beta) = angleOfSideSlip(linvel_B) - cont_->trimState_beta();
  mpc_.current_state(cont_->stateIdx_phi) = cur_bs_.pose.euler.roll - cont_->trimState_phi();
  mpc_.current_state(cont_->stateIdx_theta) = cur_bs_.pose.euler.pitch - cont_->trimState_theta();
  mpc_.current_state(cont_->stateIdx_p) = cur_bs_.twist.rot.x() - cont_->trimState_p();
  mpc_.current_state(cont_->stateIdx_q) = cur_bs_.twist.rot.y() - cont_->trimState_q();
  mpc_.current_state(cont_->stateIdx_r) = cur_bs_.twist.rot.z() - cont_->trimState_r();
}

void Controller::updateSetStateVector(double tar_roll, double tar_delta_pitch)
{
  mpc_.set_state(ctrlIdx_beta) = 0.;  // 横滑り角の目標値は常に0を設定
  mpc_.set_state(ctrlIdx_phi) = tar_roll;
  mpc_.set_state(ctrlIdx_theta) = tar_delta_pitch;
}

void Controller::updateRotorSpeeds(const VectorXd& thrust)
{
  ROS_ASSERT(thrust.rows() == num_hor_props_);

  for (int i = 0; i < thrust.rows(); ++i)
  {
    if (thrust(i) < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: " + to_string(thrust(i)) + " [N]");
      // TODO: 防御モードに移行
    }

    const auto& idx = hor_prop_idxes_[i];
    rotor_speeds_msg_.speeds[idx] =
      sqrt(max(thrust(i), 0.) / drone_.rotorConfigs()[idx].motor_constant);
  }
}

void Controller::updateDeflections(const VectorXd& deflections)
{
  ROS_ASSERT(deflections.rows() == num_cs_);

  deflections_msg_.deflections = eigen_tools::toStdVector(deflections);
}

void Controller::baseStateCb(const StateMsg& bs_nwu)
{
  // 初期化処理
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    initialize();
    is_initialized_ = true;
    dh_ros::rosInfo("Controller is ready.");
    return;
  }

  // コールバックの時点で全てNED座標系に変換しておく
  tf::baseStateNwuToNed(bs_nwu, cur_bs_);

  // メイン処理
  runOnce();
}

void Controller::commandCb(const CmdMsg& cmd_nwu)
{
  if (cmd_nwu.speed < 0.)
  {
    dh_ros::rosError("Negative speed is commanded.");
    return;
  }

  cont_->update(cmd_nwu.speed);
  setInputConstraint();
  updateSetStateVector(cmd_nwu.roll, -cmd_nwu.delta_pitch);  // NWU->NEDに変換して渡す
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent& event)
{
  dh_ros::rosWarn("Base state is not received yet.");
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  ROS_ASSERT(cfg.prediction_horizon > 0.);
  ROS_ASSERT(cfg.prediction_steps > 0);
  ROS_ASSERT(cfg.beta_decay >= 0.);
  ROS_ASSERT(cfg.rotation_decay >= 0.);
  ROS_ASSERT(cfg.beta_weight > 0.);
  ROS_ASSERT(cfg.rotation_weight > 0.);

  mpc_.time_step = cfg.prediction_horizon / cfg.prediction_steps;
  mpc_.prediction_steps = mpc_.input_steps = cfg.prediction_steps;
  mpc_.decay_time_consts[ctrlIdx_beta] = cfg.rotation_decay;
  mpc_.decay_time_consts[ctrlIdx_phi] = mpc_.decay_time_consts[ctrlIdx_theta] = cfg.rotation_decay;

  mpc_.discrete_dynamics.resize(
    cfg.prediction_steps, ctrl::LinearDynamics(cont_->stateSize, u_dim_));

  updateWeight_Q(cfg.beta_weight, cfg.rotation_weight);
  updateWeight_S(cfg.thrust_force_weight_exp, cfg.deflection_weight_exp);
  updateWeight_R(cfg.thrust_force_rate_weight_exp, cfg.deflection_rate_weight_exp, mpc_.time_step);
}
}  // namespace tobas_fixed_wing_controller
