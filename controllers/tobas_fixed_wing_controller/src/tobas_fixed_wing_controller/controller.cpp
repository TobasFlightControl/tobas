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

  is_initialized_ = false;
  rotor_speeds_msg_.speeds.resize(drone_.numRotors());
  deflections_msg_.deflections.resize(cont_->controlSurfacesSize());

  cont_.reset(new FixedWingMicroDisturbanceDynamics(drone_));
  c2d_.reset(new ctrl::C2D_RK4(cont_->kStateSize, cont_->inputSize()));

  mpc_.decay_time_consts.resize(kCtrlSize);
  setCz();
  setScales();
  mpc_.input_rate_weight.resize(cont_->inputSize());
  mpc_.input_weight.resize(cont_->inputSize());
  mpc_.control_weight.resize(kCtrlSize);
  setInputRateConstraint();
  mpc_.control_constraint.resize(kCtrlSize, 0);
  mpc_.current_state.resize(cont_->kStateSize);
  mpc_.set_state.resize(kCtrlSize);
  mpc_.last_input = VectorXd::Zero(cont_->inputSize());

  dynamicReconfigureCb(init_dynamic_config_, 0);

  registerPublishers();
  registerSubscribers();
  createTimers();
}

void Controller::getRosParams()
{
  dh_ros::getParam(kCtrlName + "/prediction_horizon", init_dynamic_config_.prediction_horizon);
  dh_ros::getParam(kCtrlName + "/prediction_steps", init_dynamic_config_.prediction_steps);
  dh_ros::getParam(kCtrlName + "/beta_decay", init_dynamic_config_.beta_decay);
  dh_ros::getParam(kCtrlName + "/rotation_decay", init_dynamic_config_.rotation_decay);
  dh_ros::getParam(kCtrlName + "/beta_weight", init_dynamic_config_.beta_weight);
  dh_ros::getParam(kCtrlName + "/rotation_weight", init_dynamic_config_.rotation_weight);
  dh_ros::getParam(
    kCtrlName + "/thrust_force_weight_exp", init_dynamic_config_.thrust_force_weight_exp);
  dh_ros::getParam(
    kCtrlName + "/thrust_force_rate_weight_exp", init_dynamic_config_.thrust_force_rate_weight_exp);
  dh_ros::getParam(
    kCtrlName + "/deflection_weight_exp", init_dynamic_config_.deflection_weight_exp);
  dh_ros::getParam(
    kCtrlName + "/deflection_rate_weight_exp", init_dynamic_config_.deflection_rate_weight_exp);
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
    nh_.createTimer(ros::Duration(kCheckTopicsTimerPeriod), &Controller::checkTopicsTimerCb, this);
}

void Controller::initialize()
{
}

void Controller::runOnce()
{
  updateCurrentStateVector();

  // MPCを解いて最適制御入力を求める
  const VectorXd du = mpc_.solveMPC();
  const VectorXd u = cont_->trimInput() + du;

  // 各ロータの回転数を発行
  const VectorXd thrust = u.block(0, 0, cont_->horizontalPropsSize(), 1);
  updateRotorSpeeds(thrust);
  rotor_speeds_pub_.publish(rotor_speeds_msg_);

  // 各操舵面の偏角を発行
  const VectorXd deflections =
    u.block(cont_->horizontalPropsSize(), 0, cont_->controlSurfacesSize(), 1);
  updateDeflections(deflections);
  deflections_pub_.publish(deflections_msg_);
}

void Controller::setCz()
{
  mpc_.Cz = MatrixXd::Zero(kCtrlSize, cont_->kStateSize);

  mpc_.Cz(kCtrlIdx_beta, cont_->kStateIdx_beta) = 1;
  mpc_.Cz(kCtrlIdx_phi, cont_->kStateIdx_phi) = 1;
  mpc_.Cz(kCtrlIdx_theta, cont_->kStateIdx_theta) = 1;
}

void Controller::setScales()
{
  // 制御変数のスケール
  mpc_.control_scale.resize(kCtrlSize);
  mpc_.control_scale(kCtrlIdx_beta) = M_PI;
  mpc_.control_scale(kCtrlIdx_phi) = mpc_.control_scale(kCtrlIdx_theta) = M_PI;

  // 制御入力のスケール
  mpc_.input_scale.resize(cont_->inputSize());
  for (int i = 0; i < cont_->horizontalPropsSize(); ++i)
  {
    const auto& rotor_idx = cont_->horizontalPropIndex(i);
    mpc_.input_scale(i) = drone_.maxThrust(rotor_idx);
  }
  mpc_.input_weight.block(cont_->horizontalPropsSize(), 0, cont_->controlSurfacesSize(), 1) =
    VectorXd::Constant(cont_->controlSurfacesSize(), M_PI);
}

void Controller::setInputConstraint()
{
  const auto lb = cont_->minDeltaInput();
  const auto ub = cont_->maxDeltaInput();
  mpc_.input_constraint = ctrl::matIneqFromRange(lb, ub);
}

void Controller::setInputRateConstraint()
{
  VectorXd lb = VectorXd::Constant(cont_->inputSize(), numeric_limits<double>::lowest());
  VectorXd ub = VectorXd::Constant(cont_->inputSize(), numeric_limits<double>::max());

  for (int i = 0; i < cont_->controlSurfacesSize();
       ++i)  // FIXME: 遅延が大きいなら舵角の変化率の制約は消してもいいかも
  {
    const auto& max_angle_rate = drone_.fixedWingConfig().control_surfaces[i].max_angle_rate;
    lb(cont_->horizontalPropsSize() + i) = -max_angle_rate;
    ub(cont_->horizontalPropsSize() + i) = +max_angle_rate;
  }

  mpc_.input_rate_constraint = ctrl::matIneqFromRange(lb, ub);
}

void Controller::updateCurrentStateVector()
{
  const Vector linvel_B = cur_bs_.pose.euler * cur_bs_.twist.vel;

  mpc_.current_state(cont_->kStateIdx_u) = linvel_B.x() - cont_->trimState_u();
  mpc_.current_state(cont_->kStateIdx_alpha) = angleOfAttack(linvel_B) - cont_->trimState_alpha();
  mpc_.current_state(cont_->kStateIdx_beta) = angleOfSideSlip(linvel_B) - cont_->trimState_beta();
  mpc_.current_state(cont_->kStateIdx_phi) = cur_bs_.pose.euler.roll - cont_->trimState_phi();
  mpc_.current_state(cont_->kStateIdx_theta) = cur_bs_.pose.euler.pitch - cont_->trimState_theta();
  mpc_.current_state(cont_->kStateIdx_p) = cur_bs_.twist.rot.x() - cont_->trimState_p();
  mpc_.current_state(cont_->kStateIdx_q) = cur_bs_.twist.rot.y() - cont_->trimState_q();
  mpc_.current_state(cont_->kStateIdx_r) = cur_bs_.twist.rot.z() - cont_->trimState_r();
}

void Controller::updateSetStateVector(double tar_roll, double tar_delta_pitch)
{
  mpc_.set_state(kCtrlIdx_beta) = 0.;  // 横滑り角の目標値は常に0を設定
  mpc_.set_state(kCtrlIdx_phi) = tar_roll;
  mpc_.set_state(kCtrlIdx_theta) = tar_delta_pitch;
}

void Controller::updateRotorSpeeds(const VectorXd& thrust)
{
  ROS_ASSERT(thrust.rows() == cont_->horizontalPropsSize());

  for (int i = 0; i < thrust.rows(); ++i)
  {
    if (thrust(i) < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: " + to_string(thrust(i)) + " [N]");
      // TODO: 防御モードに移行
    }

    const auto& rotor_idx = cont_->horizontalPropIndex(i);
    rotor_speeds_msg_.speeds[rotor_idx] =
      sqrt(max(thrust(i), 0.) / drone_.rotorConfig(rotor_idx).motor_constant);
  }
}

void Controller::updateDeflections(const VectorXd& deflections)
{
  ROS_ASSERT(deflections.rows() == cont_->controlSurfacesSize());

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

  cont_->update(cmd_nwu.speed, cur_bs_.pose.pos.z());
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
  mpc_.decay_time_consts(kCtrlIdx_beta) = cfg.rotation_decay;
  mpc_.decay_time_consts(kCtrlIdx_phi) = mpc_.decay_time_consts(kCtrlIdx_theta) =
    cfg.rotation_decay;

  mpc_.discrete_dynamics.resize(
    cfg.prediction_steps, ctrl::LinearDynamics(cont_->kStateSize, cont_->inputSize()));

  // 制御変数の重み
  mpc_.control_weight(kCtrlIdx_beta) = cfg.beta_weight;
  mpc_.control_weight(kCtrlIdx_phi) = mpc_.control_weight(kCtrlIdx_theta) = cfg.rotation_weight;

  // 制御入力の重み
  mpc_.input_weight.block(0, 0, cont_->horizontalPropsSize(), 1) =
    VectorXd::Constant(cont_->horizontalPropsSize(), pow(10, cfg.thrust_force_weight_exp));
  mpc_.input_weight.block(cont_->horizontalPropsSize(), 0, cont_->controlSurfacesSize(), 1) =
    VectorXd::Constant(cont_->controlSurfacesSize(), pow(10, cfg.deflection_weight_exp));

  // 制御入力の変化率の重み
  mpc_.input_rate_weight.block(0, 0, cont_->horizontalPropsSize(), 1) =
    VectorXd::Constant(cont_->horizontalPropsSize(), pow(10, cfg.thrust_force_rate_weight_exp));
  mpc_.input_rate_weight.block(cont_->horizontalPropsSize(), 0, cont_->controlSurfacesSize(), 1) =
    VectorXd::Constant(cont_->controlSurfacesSize(), pow(10, cfg.deflection_rate_weight_exp));
}
}  // namespace tobas_fixed_wing_controller
