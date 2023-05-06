#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <tobas_tools/utils.hpp>

#include "../../include/tobas_fixed_wing_controller/controller.hpp"
#include "../../include/tobas_fixed_wing_controller/constants.hpp"

#define WEIGHT_SCALER 1e+6  // TODO: QPの数値エラーを防ぐために重みにかける定数を自動調整

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_fixed_wing_controller
{
Controller::Controller() : super()
{
  getRosParams();
  drone_.loadFromParam(ns_);

  rotor_idx_in_use_ = drone_.rotorConfigIdxInAxis(Axis::X_POSITIVE);
  num_hor_props_ = rotor_idx_in_use_.size();
  num_cs_ = drone_.fixedWingConfig().control_surfaces.size();
  u_dim_ = num_hor_props_ + num_cs_;

  is_initialized_ = false;
  rotor_speeds_msg_.speeds.resize(drone_.numRotors());
  deflections_msg_.deflections.resize(num_cs_);

  c2d_.reset(new ctrl::C2D_RK4(dStateSize, u_dim_));

  mpc_.decay_time_consts.resize(ctrlSize);
  setCz();
  mpc_.input_rate_weight.resize(u_dim_);
  mpc_.input_weight.resize(u_dim_);
  mpc_.control_weight.resize(ctrlSize);
  mpc_.control_constraint.resize(ctrlSize, 0);
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

void Controller::initialize(const StateMsg& bs)
{
}

void Controller::runOnce(const StateMsg& bs)
{
  updateCurrentStateVector(bs);

  VectorXd du = mpc_.solveMPC();
  VectorXd u = u_0_ + du;

  VectorXd thrust = u.block(0, 0, num_hor_props_, 1);
  updateRotorSpeeds(thrust);
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  VectorXd deflections = u.block(num_hor_props_, 0, num_cs_, 1);
  updateDeflections(deflections);
  deflections_pub_.publish(deflections_msg_);
}

void Controller::setCz()
{
  mpc_.Cz = MatrixXd::Zero(ctrlSize, dStateSize);

  mpc_.Cz(ctrlIdx_beta, dStateIdx_beta) = 1;
  mpc_.Cz(ctrlIdx_phi, dStateIdx_phi) = 1;
  mpc_.Cz(ctrlIdx_theta, dStateIdx_theta) = 1;
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
    double thrust_scale = drone_.maxThrust(rotor_idx_in_use_[i]);
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
    double thrust_rate_scale = drone_.maxThrust(rotor_idx_in_use_[i]) * dt;
    mpc_.input_rate_weight(i) =
      pow(10, thrust_rate_weight_exp) / sqr(thrust_rate_scale) * WEIGHT_SCALER;
  }

  double deflection_rate_scale = M_PI * dt;
  double deflection_rate_weight =
    pow(10, deflection_rate_weight_exp) / sqr(deflection_rate_scale) * WEIGHT_SCALER;
  mpc_.input_rate_weight.block(num_hor_props_, 0, num_cs_, 1) =
    VectorXd::Constant(num_cs_, deflection_rate_weight);
}

void Controller::updateTrimDynamics(double tar_V)
{
  // TODO
}

void Controller::updateCurrentStateVector(const StateMsg& bs)
{
  // TODO
}

void Controller::updateSetStateVector(double tar_roll, double tar_pitch)
{
  // TODO
}

void Controller::updateRotorSpeeds(const Eigen::VectorXd& thrust)
{
  // TODO
}

void Controller::updateDeflections(const Eigen::VectorXd& deflections)
{
  // TODO
}

void Controller::baseStateCb(const StateMsg& bs)
{
  // 初期化処理
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    initialize(bs);
    is_initialized_ = true;
    dh_ros::rosInfo("Controller is ready.");
    return;
  }

  // メイン処理
  runOnce(bs);
}

void Controller::commandCb(const CmdMsg& cmd)
{
  // TODO
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

  mpc_.discrete_dynamics.resize(cfg.prediction_steps, ctrl::LinearDynamics(dStateSize, u_dim_));

  updateWeight_Q(cfg.beta_weight, cfg.rotation_weight);
  updateWeight_S(cfg.thrust_force_weight_exp, cfg.deflection_weight_exp);
  updateWeight_R(cfg.thrust_force_rate_weight_exp, cfg.deflection_rate_weight_exp, mpc_.time_step);
}
}  // namespace tobas_fixed_wing_controller
