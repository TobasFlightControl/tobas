#include <tobas_std_tools/vector.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_mr_pid/ControllerFeedback.h>

#include "../include/tobas_mr_pid/controller_ros.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_mr_pid
{
ControllerRos::ControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    js_converter_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    acc_ctrl_(drone_),
    mixer_(drone_),
    check_topics_timer_(nh_, tobas::kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  z_rotors_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  acc_ctrl_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::getRosParams()
{
  tobas_ros::getParam(
    pnh_, "do_thrust_correction", do_thrust_correction_, kDefaultDoThrustCorrection);
}

void ControllerRos::registerPublishers()
{
  rot_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_mr_pid::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  if (drone_.isTransformable())
    joint_state_sub_ =
      nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  if (do_thrust_correction_)
    thrust_corr_factor_sub_ = nh_.subscribe(
      tobas::kThrustCorrectionFactorTopic, 1, &self::thrustCorrectionFactorCb, this, tcpNoDelay());

  pvay_sub_ =
    nh_.subscribe(tobas::kPosVelAccYawCmdTopic, 1, &self::posVelAccYawCb, this, tcpNoDelay());
  rpyt_sub_ = nh_.subscribe(tobas::kRpyThrustCmdTopic, 1, &self::rpyThrustCb, this, tcpNoDelay());
}

bool ControllerRos::isReady() const
{
  if (odom_ == nullptr)
    return false;

  if (battery_ == nullptr)
    return false;

  if (drone_.isTransformable() && js_ == nullptr)
    return false;

  if (do_thrust_correction_ && thrust_corr_factor_ == nullptr)
    return false;

  return true;
}

void ControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  odom_ = odom;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      t_last_loop_ = odom->header.stamp;
      is_initialized_ = true;
      TOBAS_GOOD("Controller is ready.");
    }
    return;
  }

  // 時刻を更新
  const auto dt = (odom->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = odom->header.stamp;

  // Create a feedback message
  auto feedback = boost::make_shared<tobas_mr_pid::ControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
      tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();

    // 世界座標系から見た現在の速度を計算
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    const Vector tar_acc_fb(pos_ctrl_.update(
      odom->frame.p.data, cur_vel_W.data, tar_pvay_->pos.data, tar_pvay_->vel.data, dt));
    const auto tar_acc = tar_pvay_->acc + tar_acc_fb;

    // 推力和と目標姿勢を計算
    acc_ctrl_.update(
      odom->frame.M, tar_acc, tar_rpyt_->thrust, tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は加速度指令をそのまま流す
    tar_rpyt_->level = tar_pvay_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_->yaw;

    // Fill feedback
    feedback->target_position = tar_pvay_->pos;
    feedback->target_velocity_global = tar_pvay_->vel;
    feedback->target_velocity_local = odom->frame.M.inverse(tar_pvay_->vel);
    feedback->target_acceleration_global = tar_acc;
    feedback->target_acceleration_local = odom->frame.M.inverse(tar_acc);
    feedback->position_integral_error.data = pos_ctrl_.integralError();
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
      rosError(name_, "Joint state converter failed: " << js_converter_.errorMessage());

    // 目標角加速度を計算
    const auto tar_dgyro =
      ori_ctrl_.update(Euler(odom->frame.M), odom->twist.rot, tar_rpyt_->rpy, Vector::Zero(), dt);

    // プロペラの推力を計算
    // TODO: H-momentを考慮
    const VectorXd thrusts = mixer_.solve(
      dt, battery_->voltage, js_converter_.getPositionsKDL(), odom->twist.rot.data,
      Vector3d::Zero(), tar_dgyro.data, tar_rpyt_->thrust);

    // 目標回転数を発行
    const auto tar_rot_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
    tar_rot_speeds->header.stamp = odom->header.stamp;
    tar_rot_speeds->speeds.resize(drone_.numRotors(), 0.);
    for (size_t i = 0; i < static_cast<size_t>(thrusts.rows()); ++i)
    {
      auto thrust = max(0., thrusts(i));
      if (do_thrust_correction_ && thrust_corr_factor_ != nullptr)  // 推力補正
        thrust *= thrust_corr_factor_->data;
      tar_rot_speeds->speeds[z_rotors_.rotorIdx(i)] = z_rotors_.rotSpeedFromThrust(i, thrust);
    }
    rot_speeds_pub_.publish(tar_rot_speeds);

    // フィードバックを発行
    feedback->target_orientation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;
    feedback_pub_.publish(feedback);
  }
}

void ControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    rosError(name_, "The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerRos::thrustCorrectionFactorCb(const std_msgs::Float64ConstPtr& msg)
{
  thrust_corr_factor_ = msg;
}

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay)
{
  if (!is_initialized_)
    return;

  // コマンドレベルの処理
  if (!cmd_level_handler_.update(pvay->level.data, ros::Time::now()))
    return;

  // コマンドを更新
  tar_pvay_ = boost::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::WORLD, odom_->frame.M, *tar_pvay_))
  {
    rosError(name_, "Failed to change command frame. Probably the frame id is invalid.");
    tar_pvay_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt)
{
  if (!is_initialized_)
    return;

  if (!cmd_level_handler_.update(rpyt->level.data, ros::Time::now()))
    return;

  // 外側の制御を止める
  tar_pvay_ = nullptr;

  // コマンドを更新
  tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

void ControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (battery_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kBatteryLpfTopic);

  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (drone_.isTransformable() && js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic);

  if (do_thrust_correction_ && thrust_corr_factor_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kThrustCorrectionFactorTopic);
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  // 位置制御
  pos_cfg_.hor_natural_freq = cfg.horizontal_natural_frequency;
  pos_cfg_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  pos_cfg_.hor_ki = cfg.horizontal_i_gain;
  pos_cfg_.ver_natural_freq = cfg.vertical_natural_frequency;
  pos_cfg_.ver_damp_ratio = cfg.vertical_damping_ratio;
  pos_cfg_.ver_ki = cfg.vertical_i_gain;
  pos_cfg_.max_hor_acc = cfg.max_horizontal_accel;
  pos_cfg_.max_ver_acc = cfg.max_vertical_accel;
  pos_ctrl_.configure(pos_cfg_);

  // 非線形変換
  acc_cfg_.max_attitude = cfg.max_attitude;
  acc_cfg_.h_force_comp_rate = 0;  // TODO
  acc_ctrl_.configure(acc_cfg_);

  // 姿勢制御器
  ori_cfg_.atti_natural_freq = cfg.attitude_natural_frequency;
  ori_cfg_.atti_damp_ratio = cfg.attitude_damping_ratio;
  ori_cfg_.atti_ki = cfg.attitude_i_gain;
  ori_cfg_.head_natural_freq = cfg.heading_natural_frequency;
  ori_cfg_.head_damp_ratio = cfg.heading_damping_ratio;
  ori_cfg_.head_ki = cfg.heading_i_gain;
  ori_ctrl_.configure(ori_cfg_);

  // TODO: Mixerの設定

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_pid
