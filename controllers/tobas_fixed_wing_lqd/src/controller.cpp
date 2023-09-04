#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/standard_atmosphere.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_fixed_wing_lqd/controller.hpp"
#include "../include/tobas_fixed_wing_lqd/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_fixed_wing_lqd
{
Controller::Controller(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    x_rotors_(drone_, tobas::Axis::X_POSITIVE),
    eom_(drone_),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &Controller::checkTopicsTimerCb, this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();
  drone_.loadFromParam(nh_);

  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  if (x_rotors_.count() == 0)
  {
    rosthrow(name_, "The number of propellers is zero.");
  }

  q_0_.resize(drone_.tree().getNrOfJoints());

  setScales();
  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);

  configure(cfg_);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::getRosParams()
{
  dh_ros::getParam(nh_, kCtrlName + "/forward_speed_weight", cfg_.forward_speed_weight);
  dh_ros::getParam(nh_, kCtrlName + "/alpha_weight", cfg_.alpha_weight);
  dh_ros::getParam(nh_, kCtrlName + "/beta_weight", cfg_.beta_weight);
  dh_ros::getParam(nh_, kCtrlName + "/attitude_weight", cfg_.attitude_weight);
  dh_ros::getParam(nh_, kCtrlName + "/angular_velocity_weight", cfg_.angular_velocity_weight);

  dh_ros::getParam(nh_, kCtrlName + "/thrust_weight_exp", cfg_.thrust_weight_exp);
  dh_ros::getParam(nh_, kCtrlName + "/thrust_rate_weight_exp", cfg_.thrust_rate_weight_exp);
  dh_ros::getParam(nh_, kCtrlName + "/deflection_weight_exp", cfg_.deflection_weight_exp);
  dh_ros::getParam(nh_, kCtrlName + "/deflection_rate_weight_exp", cfg_.deflection_rate_weight_exp);
}

void Controller::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
  deflections_pub_ = nh_.advertise<tobas_msgs::ControlSurfaceDeflections>("command/deflections", 1);
  feedback_pub_ =
    nh_.advertise<tobas_msgs::FixedWingControllerFeedback>("fixed_wing_controller_feedback", 1);
}

void Controller::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &Controller::eventCb, this, tcpNoDelay());
  air_pressure_sub_ =
    nh_.subscribe("air_pressure", 1, &Controller::airPressureCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe("battery", 1, &Controller::batteryCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe("pose_twist", 1, &Controller::poseTwistCb, this, tcpNoDelay());
  cmd_sub_ =
    nh_.subscribe("command/speed_roll_delta_pitch", 1, &Controller::commandCb, this, tcpNoDelay());
}

bool Controller::isReady()
{
  return pressure_received_ && battery_received_ && pt_received_;
}

void Controller::publishTakeoffCommand()
{
  // 各ロータの回転数を発行
  const auto rotor_speeds_msg = boost::make_shared<tobas_msgs::RotorSpeeds>();
  rotor_speeds_msg->header.stamp = pt_ned_.header.stamp;
  rotor_speeds_msg->speeds.resize(drone_.numRotors(), 0.);
  for (uint32_t i = 0; i < x_rotors_.count(); ++i)
  {
    rotor_speeds_msg->speeds[x_rotors_.rotorIdx(i)] =
      x_rotors_.rotSpeedFromVoltage(i, battery_->voltage);
  }
  rotor_speeds_pub_.publish(rotor_speeds_msg);

  // 各操舵面の偏角を発行
  const auto deflections_msg = boost::make_shared<tobas_msgs::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = pt_ned_.header.stamp;
  deflections_msg->deflections.resize(drone_.numControlSurfaces(), 0.);
  deflections_msg->deflections[eom_.elevatorIndex()] = eom_.trimCondition().elevator();
  deflections_pub_.publish(deflections_msg);
}

void Controller::initialize()
{
  // 最新の制御時刻
  t_last_loop_ = pt_ned_.header.stamp;

  // 制御入力の初期値
  lqd_.last_input = VectorXd::Zero(eom_.inputSize());

  // コマンドの初期値
  const auto& trim = eom_.trimCondition();
  cmd_ned_.speed = trim.takeOffSpeed(air_density_);
  cmd_ned_.roll = 0.;
  cmd_ned_.delta_pitch = kInitialDeltaPitch;
}

void Controller::runOnce()
{
  // 時刻を更新
  const auto& cur_time = pt_ned_.header.stamp;
  const auto dt = (cur_time - t_last_loop_).toSec();
  t_last_loop_ = cur_time;

  // 現在の速度を使って状態方程式を更新
  if (eom_.update(pt_ned_.twist.vel.Norm(), air_density_, battery_->voltage, q_0_) < 0)
  {
    rosError(name_, eom_.errorMessage());
  }

  lqd_.dynamics.A = eom_.A();
  lqd_.dynamics.B = eom_.B();

  updateCurrentStateVector();
  updateSetStateVector(cmd_ned_.roll, cmd_ned_.delta_pitch);

  // 最適制御入力を求める
  const VectorXd du = lqd_.solveLQD(dt);
  const VectorXd u = eom_.trimInput() + du;

  // For debug
  // cout << "A_cont:" << endl << eom_.A() << endl;
  // cout << "B_cont:" << endl << eom_.B() << endl;
  // cout << lqd_ << endl;

  const VectorXd thrust = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1);

  // Publish
  publishRotorSpeeds(thrust);
  publishDeflections(deflections);
  publishFeedback(du);
}

void Controller::setScales()
{
  // 状態変数のスケール
  lqd_.state_scale.resize(eom_.kStateSize);
  lqd_.state_scale(eom_.kStateIdx_u) = eom_.trimCondition().takeOffSpeed(kStandardAirDensity);
  lqd_.state_scale(eom_.kStateIdx_alpha) = drone_.vehicle().alpha_limit.range();
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
  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    lqd_.input_scale(x_rotors_.count() + i) = drone_.controlSurface(i).angle_limit.range();
  }
}

void Controller::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();

  // TODO: 横系のトリムも考慮
  lqd_.current_state(eom_.kStateIdx_u) = pt_ned_.twist.vel.x() - trim.u();
  lqd_.current_state(eom_.kStateIdx_alpha) = tobas::angleOfAttack(pt_ned_.twist.vel) - trim.alpha();
  lqd_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(pt_ned_.twist.vel);
  lqd_.current_state(eom_.kStateIdx_phi) = pt_ned_.pose.euler.roll;
  lqd_.current_state(eom_.kStateIdx_theta) = pt_ned_.pose.euler.pitch - trim.theta();
  lqd_.current_state(eom_.kStateIdx_p) = pt_ned_.twist.rot.x();
  lqd_.current_state(eom_.kStateIdx_q) = pt_ned_.twist.rot.y();
  lqd_.current_state(eom_.kStateIdx_r) = pt_ned_.twist.rot.z();
}

void Controller::updateSetStateVector(double tar_roll, double tar_delta_pitch)
{
  const auto& trim = eom_.trimCondition();
  const auto tar_u = cmd_ned_.speed * cos(eom_.trimCondition().alpha());

  lqd_.target_state(eom_.kStateIdx_u) = tar_u - trim.u();
  lqd_.target_state(eom_.kStateIdx_alpha) = 0.;
  lqd_.target_state(eom_.kStateIdx_beta) = 0.;
  lqd_.target_state(eom_.kStateIdx_phi) = tar_roll;
  lqd_.target_state(eom_.kStateIdx_theta) = tar_delta_pitch;
  lqd_.target_state(eom_.kStateIdx_p) = 0.;
  lqd_.target_state(eom_.kStateIdx_q) = 0.;
  lqd_.target_state(eom_.kStateIdx_r) = 0.;
}

void Controller::publishRotorSpeeds(const Eigen::VectorXd& thrust)
{
  const auto rotor_speeds_msg = boost::make_shared<tobas_msgs::RotorSpeeds>();
  rotor_speeds_msg->header.stamp = pt_ned_.header.stamp;

  rotor_speeds_msg->speeds.resize(drone_.numRotors(), 0.);
  for (uint32_t i = 0; i < thrust.rows(); ++i)
  {
    if (thrust(i) < -1.)
    {
      rosFatal(name_, "Negative thrust force: " << thrust(i) << " [N]");
      // TODO: 防御モードに移行
    }

    rotor_speeds_msg->speeds[x_rotors_.rotorIdx(i)] =
      x_rotors_.rotSpeedFromThrust(i, max(0., thrust(i)));
  }

  rotor_speeds_pub_.publish(rotor_speeds_msg);
}

void Controller::publishDeflections(const Eigen::VectorXd& deflections)
{
  const auto deflections_msg = boost::make_shared<tobas_msgs::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = pt_ned_.header.stamp;
  deflections_msg->deflections = eigen_tools::toStdVector(deflections);
  deflections_pub_.publish(deflections_msg);
}

void Controller::publishFeedback(const Eigen::VectorXd& du)
{
  const auto& trim = eom_.trimCondition();
  const auto feedback = boost::make_shared<tobas_msgs::FixedWingControllerFeedback>();

  feedback->trim_thrusts.resize(drone_.numRotors());
  feedback->delta_thrusts.resize(drone_.numRotors());
  feedback->trim_deflections.resize(drone_.numControlSurfaces());
  feedback->delta_deflections.resize(drone_.numControlSurfaces());

  feedback->trim_u = trim.u();
  feedback->trim_alpha = trim.alpha();

  for (uint32_t i = 0; i < x_rotors_.count(); ++i)
  {
    feedback->trim_thrusts[x_rotors_.rotorIdx(i)] = eom_.trimInput()(i);
    feedback->delta_thrusts[x_rotors_.rotorIdx(i)] = du(i);
  }

  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback->trim_deflections[i] = eom_.trimInput()[u_idx];
    feedback->delta_deflections[i] = du(u_idx);
  }

  feedback_pub_.publish(feedback);
}

void Controller::configure(const ConfigType& cfg)
{
  ROS_ASSERT(cfg.forward_speed_weight > 0.);
  ROS_ASSERT(cfg.alpha_weight > 0.);
  ROS_ASSERT(cfg.beta_weight > 0.);
  ROS_ASSERT(cfg.attitude_weight > 0.);
  ROS_ASSERT(cfg.angular_velocity_weight > 0.);

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
  const auto thrust_weight = pow(10, cfg.thrust_weight_exp);
  const auto deflection_weight = pow(10, cfg.deflection_weight_exp);
  lqd_.input_weight.topRows(x_rotors_.count()).fill(thrust_weight);
  lqd_.input_weight.bottomRows(drone_.numControlSurfaces()).fill(deflection_weight);

  // 制御入力の変化率の重み
  const auto thrust_rate_weight = pow(10, cfg.thrust_rate_weight_exp);
  const auto deflection_rate_weight = pow(10, cfg.deflection_rate_weight_exp);
  lqd_.input_rate_weight.topRows(x_rotors_.count()).fill(thrust_rate_weight);
  lqd_.input_rate_weight.bottomRows(drone_.numControlSurfaces()).fill(deflection_rate_weight);
}

void Controller::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void Controller::airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg)
{
  air_density_ = dh_std::pressureToDensity(msg->fluid_pressure);

  if (!pressure_received_)
  {
    pressure_received_ = true;
  }
}

void Controller::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;

  if (!battery_received_)
  {
    battery_received_ = true;
  }
}

void Controller::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt_nwu)
{
  // コールバックの時点で全てNED座標系に変換しておく
  tf::baseStateNwuToNed(*pt_nwu, pt_ned_);

  if (!pt_received_)
  {
    pt_received_ = true;
  }

  switch (state_)
  {
    case START:
    {
      if (isReady())
      {
        rosInfo(name_, "Controller is ready.");
        check_topics_timer_.stop();
        state_ = TAKEOFF;
      }
      break;
    }
    case TAKEOFF:
    {
      const auto cur_V = pt_ned_.twist.vel.Norm();
      const auto min_V = eom_.trimCondition().minimumSpeed(air_density_);
      const auto eom_error = eom_.update(max(cur_V, min_V), air_density_, battery_->voltage, q_0_);
      if (eom_error < 0)
      {
        rosError(name_, eom_.errorMessage());
      }

      publishTakeoffCommand();

      // 最低速度を上回ったら制御開始
      const auto cur_speed = pt_nwu->twist.vel.Norm();
      if (cur_speed > eom_.trimCondition().minimumSpeed(air_density_))
      {
        initialize();
        state_ = FLIGHT;
        rosInfo(name_, "The aircraft takes off and begins flight control.");
      }
      break;
    }
    case FLIGHT:
    {
      runOnce();
      break;
    }
    case LANDING:
    {
      // TODO
      break;
    }
  }
}

void Controller::commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu)
{
  if (!(state_ == FLIGHT))
  {
    rosError(name_, "Not in flight state.");
    return;
  }

  if (!eom_.trimCondition().speedLimit(air_density_).inRange(cmd_nwu->speed))
  {
    rosError(name_, "Invalid speed is commanded.");
    return;
  }

  tf::speedRollDeltaPitchNwuToNed(*cmd_nwu, cmd_ned_);
}

void Controller::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!pressure_received_)
  {
    rosWarn(name_, "Air pressure is not received yet.");
  }

  if (!battery_received_)
  {
    rosWarn(name_, "Battery state is not received yet.");
  }

  if (!pt_received_)
  {
    rosWarn(name_, "Pose & Twist is not received yet.");
  }
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  configure(cfg);
}
}  // namespace tobas_fixed_wing_lqd
