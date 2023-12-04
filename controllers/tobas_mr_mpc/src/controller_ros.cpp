#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_msgs/Throttles.h>
#include <tobas_mr_mpc/ControllerFeedback.h>

#include "../include/tobas_mr_mpc/controller_ros.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_mr_mpc
{
ControllerRos::ControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    js_converter_(drone_),
    acc_ctrl_(drone_),
    ori_ctrl_(drone_),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  z_rotors_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  acc_ctrl_.updateInternalDataStructures();
  ori_ctrl_.updateInternalDataStructures();

  q_.resize(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::getRosParams()
{
}

void ControllerRos::registerPublishers()
{
  throttles_pub_ = nh_.advertise<tobas_msgs::Throttles>(tobas::kThrottlesCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_mr_mpc::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  super::registerSubscribers();

  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
  wind_sub_ = nh_.subscribe(tobas::kWindTopic, 1, &self::windCb, this, tcpNoDelay());
  rotor_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsTopic, 1, &self::rotorSpeedsCb, this, tcpNoDelay());
  if (drone_.isTransformable())
  {
    joint_state_sub_ =
      nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  }

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

  if (wind_ == nullptr)
    return false;

  if (rotor_speeds_ == nullptr)
    return false;

  if (drone_.isTransformable() && js_ == nullptr)
    return false;

  return true;
}

void ControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      check_topics_timer_.stop();
      break;
    default:
      break;
  }
}

void ControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  // 状態を更新
  odom_ = odom;

  // 初期化
  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      t_last_loop_ = odom->header.stamp;
      is_initialized_ = true;
      DH_GOOD("Controller is ready.");
    }
    return;
  }

  // 時刻を更新
  const auto dt = (odom->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = odom->header.stamp;

  // Create a feedback message
  const auto feedback = boost::make_shared<tobas_mr_mpc::ControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
    {
      tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
    }

    // 世界座標系から見た現在の速度，加速度を計算
    const Vector cur_vel_W = odom->pose.euler * odom->twist.vel;
    const Vector cur_acc_W = odom->pose.euler * odom->accel.linear;

    // 目標加速度を計算
    pos_ctrl_.update(
      odom->pose.pos, cur_vel_W, cur_acc_W, tar_pvay_->pos, tar_pvay_->vel, dt, tar_acc_fb_);
    const auto tar_acc = tar_pvay_->acc + tar_acc_fb_;

    // 推力和と目標姿勢を計算
    acc_ctrl_.update(
      odom->pose.euler, odom->twist.vel, wind_->vel, rotor_speeds_->speeds, tar_acc,
      tar_rpyt_->thrust, tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は位置指令をそのまま流す
    tar_rpyt_->level = tar_pvay_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_->yaw;

    // Fill feedback
    feedback->target_position = tar_pvay_->pos;
    feedback->target_velocity_global = tar_pvay_->vel;
    feedback->target_velocity_local = odom->pose.euler.inverse(tar_pvay_->vel);
    feedback->target_acceleration_global = tar_acc;
    feedback->target_acceleration_local = odom->pose.euler.inverse(tar_acc);
    feedback->position_integral_error = Vector(pos_ctrl_.positionIntegralError());
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    // 処理の遅延を防ぐため，JointStateのコールバックではなくここで行う
    if (drone_.isTransformable() && !js_converter_.convert(*js_, q_))
      rosError(name_, "Failed to parse JointState.");

    // プロペラ推力を計算
    Eigen::VectorXd thrusts;
    try
    {
      // stopwatch_.start();
      thrusts = ori_ctrl_.solve(
        dt, odom->pose.euler, odom->twist, wind_->vel, q_, battery_->voltage, rotor_speeds_->speeds,
        tar_rpyt_->thrust, tar_rpyt_->rpy);
      // stopwatch_.stop();
    }
    catch (const exception& e)
    {
      // MPCがコケたり．前回の推力をそのまま継続して印加する．
      rosFatal(name_, e.what());
    }

    // スロットルを発行
    const auto throttles = boost::make_shared<tobas_msgs::Throttles>();
    throttles->header.stamp = odom->header.stamp;
    throttles->data.resize(drone_.numRotors(), tobas::kArmThrottle);
    for (int i = 0; i < thrusts.rows(); ++i)
    {
      const auto thrust = max(0., thrusts(i));
      const auto& rotor_idx = z_rotors_.rotorIdx(i);
      throttles->data[rotor_idx] = z_rotors_.throttleFromThrust(i, thrust, battery_->voltage);
    }
    throttles_pub_.publish(throttles);

    // フィードバックを発行
    feedback->target_orientation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;
    feedback->target_thrusts = eigen_tools::toStdVector(thrusts);
    feedback->mpc_thrusts = eigen_tools::toStdVector(ori_ctrl_.mpcThrusts());
    feedback_pub_.publish(feedback);
  }
}

void ControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::windCb(const tobas_msgs::WindConstPtr& wind)
{
  wind_ = wind;
}

void ControllerRos::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
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

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay)
{
  if (odom_ == nullptr)
    return;

  // コマンドレベルの処理
  if (!updateCommandLevel(cmd_level_, pvay->level.data))
    return;

  // コマンドを更新
  tar_pvay_ = boost::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::GLOBAL, odom_->pose.euler, *tar_pvay_))
  {
    rosError(name_, "Failed to change command frame. Probably the frame id is invalid.");
    tar_pvay_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust)
{
  if (odom_ == nullptr)
    return;

  if (!updateCommandLevel(cmd_level_, rpy_thrust->level.data))
    return;

  // 外側の制御を止める
  tar_pvay_ = nullptr;

  // コマンドを更新
  tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpy_thrust);
}

void ControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (odom_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kOdometryTopic);

  if (battery_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kBatteryTopic);

  if (wind_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kWindTopic);

  if (rotor_speeds_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kRotorSpeedsTopic);

  if (drone_.isTransformable() && js_ == nullptr)
    rosInfo(name_, "Waiting for " << ns() << tobas::kJointStatesTopic);
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  pos_cfg_.acc_delay_time_const = cfg.attitude_decay;  // 加速度の遅延 = 姿勢の遅延
  pos_cfg_.hor_pos_weight = cfg.horizontal_position_weight;
  pos_cfg_.ver_pos_weight = cfg.vertical_position_weight;
  pos_cfg_.hor_vel_weight = cfg.horizontal_velocity_weight;
  pos_cfg_.ver_vel_weight = cfg.vertical_velocity_weight;
  pos_cfg_.hor_acc_weight = cfg.horizontal_accel_weight;
  pos_cfg_.ver_acc_weight = cfg.vertical_accel_weight;
  pos_cfg_.hor_posint_weight = cfg.horizontal_position_integral_weight;
  pos_cfg_.ver_posint_weight = cfg.vertical_position_integral_weight;
  pos_cfg_.jerk_weight_log10 = cfg.jerk_weight_log10;
  pos_cfg_.max_hor_posint_error = cfg.max_horizontal_position_integral_error;
  pos_cfg_.max_ver_posint_error = cfg.max_vertical_position_integral_error;
  pos_cfg_.max_hor_acc = cfg.max_horizontal_accel;
  pos_cfg_.max_ver_acc = cfg.max_vertical_accel;
  pos_ctrl_.configure(pos_cfg_);

  acc_cfg_.max_attitude = cfg.max_attitude;
  // acc_cfg_.h_force_comp_rate = cfg.horizontal_force_compensation_rate;
  acc_cfg_.h_force_comp_rate = 0;  // FIXME: 並進EoMで風外乱を考慮するとオーバーシュートが過大
  acc_ctrl_.configure(acc_cfg_);

  ori_cfg_.h_force_comp_rate = cfg.horizontal_force_compensation_rate;
  ori_cfg_.kp = cfg.orientation_kp;
  ori_cfg_.kd = cfg.orientation_kd;
  ori_cfg_.pred_horizon = cfg.prediction_horizon;
  ori_cfg_.pred_steps = cfg.prediction_steps;
  ori_cfg_.attitude_decay = cfg.attitude_decay;
  ori_cfg_.heading_decay = cfg.heading_decay;
  ori_cfg_.angvel_decay = cfg.angular_velocity_decay;
  ori_cfg_.attitude_weight = cfg.attitude_weight;
  ori_cfg_.heading_weight = cfg.heading_weight;
  ori_cfg_.angvel_weight = cfg.angular_velocity_weight;
  ori_cfg_.thrust_rate_weight_log10 = cfg.thrust_rate_weight_log10;
  ori_ctrl_.configure(ori_cfg_);

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_mpc
