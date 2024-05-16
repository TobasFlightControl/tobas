#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_np_pid/ControllerFeedback.h>

#include "../include/tobas_np_pid/controller_ros.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_np_pid
{
ControllerRos::ControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), js_converter_(drone_.tree()), mixer_(drone_), server_(pnh_)
{
  drone_.loadFromParam(nh_);

  js_converter_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::registerPublishers()
{
  rot_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_np_pid::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  if (drone_.isTransformable())
    js_sub_ = nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());

  arming_sub_ = nh_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this, tcpNoDelay());
  cmd_sub_ = nh_.subscribe(tobas::kPoseTwistAccelCmdTopic, 1, &self::commandCb, this, tcpNoDelay());
}

bool ControllerRos::isReadyToControl()
{
  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kOdometryTopic);
    return false;
  }

  if (odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kBatteryLpfTopic);
    return false;
  }

  if (drone_.isTransformable() && js_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kJointStatesTopic);
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

void ControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (odom_ == nullptr)
  {
    odom_ = odom;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom->header.stamp - odom_->header.stamp).toSec();
  odom_ = odom;

  if (!isReadyToControl())
    return;

  // コマンドが来ていなければスキップ
  if (cmd_ == nullptr)
    return;

  // 可動関節の角度を更新
  if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());

  // 位置制御器
  const Vector cur_vel_W = odom->frame.M * odom->twist.vel;  // 世界座標系から見た現在の速度
  const Vector tar_acc_fb(
    pos_pid_.update(odom->frame.p.data, cur_vel_W.data, cmd_->pos.data, cmd_->vel.data, dt));
  const Vector tar_acc_W = cmd_->acc + tar_acc_fb;

  // 姿勢制御器
  const Vector tar_dgyro_fb =
    ori_pid_.update(Euler(odom->frame.M), odom->twist.rot, cmd_->rpy, cmd_->gyro, dt);
  const Vector tar_dgyro_B = cmd_->dgyro + tar_dgyro_fb;

  // ミキサーで6軸加速度をプロペラの推力に変換
  const VectorXd thrusts = mixer_.solve(
    battery_->voltage, js_converter_.getPositionsKDL(), odom->frame.M, odom->twist.rot, tar_acc_W,
    tar_dgyro_B);

  // 目標回転数を発行
  const auto tar_rot_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  tar_rot_speeds->header.stamp = odom->header.stamp;
  tar_rot_speeds->speeds.resize(drone_.numRotors(), 0.);
  for (size_t rotor_idx = 0; rotor_idx < static_cast<size_t>(thrusts.rows()); ++rotor_idx)
  {
    const auto thrust = max(0., thrusts(rotor_idx));
    tar_rot_speeds->speeds[rotor_idx] = drone_.rotSpeedFromThrust(rotor_idx, thrust);
  }
  rot_speeds_pub_.publish(tar_rot_speeds);

  // フィードバックを発行
  // 目標位置速度はコマンドそのままだが，発行されていない間も安定して描画するためにメッセージに含めている
  const auto feedback = boost::make_shared<tobas_np_pid::ControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;
  feedback->target_position = cmd_->pos;
  feedback->target_orientation = cmd_->rpy;
  feedback->target_twist_local.vel = odom->frame.M.inverse(cmd_->vel);
  feedback->target_twist_local.rot = cmd_->gyro;
  feedback->target_twist_global.vel = cmd_->vel;
  feedback->target_twist_global.rot = odom->frame.M * cmd_->gyro;
  feedback->target_accel_local.linear = odom->frame.M.inverse(tar_acc_W);
  feedback->target_accel_local.angular = tar_dgyro_B;
  feedback->target_accel_global.linear = tar_acc_W;
  feedback->target_accel_global.angular = odom->frame.M * tar_dgyro_B;
  feedback->position_integral_error.data = pos_pid_.integralError();
  feedback->orientation_integral_error = Euler(ori_pid_.integralError());
  feedback_pub_.publish(feedback);
}

void ControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::jointStateCb(const sensor_msgs::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    TOBAS_ERROR("The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerRos::armingCb(const std_msgs::BoolConstPtr& arming)
{
  arming_ = arming;

  // Disarm時にコマンドをリセットする．でないと再度アームした時に前回のコマンドでモータが回り始めてしまう．
  if (!arming->data)
  {
    cmd_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }
}

void ControllerRos::commandCb(const tobas_msgs::PoseTwistAccelCommandConstPtr& cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(
      tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(cmd->level.data, ros::Time::now()))
  {
    TOBAS_WARN_THROTTLE(
      tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // コマンドを更新
  cmd_ = boost::make_shared<tobas_msgs::PoseTwistAccelCommand>(*cmd);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::WORLD, odom_->frame.M, *cmd_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame id is invalid.");
    cmd_ = nullptr;
    return;
  }
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
  pos_pid_.configure(pos_cfg_);

  // 姿勢制御器
  ori_cfg_.atti_natural_freq = cfg.attitude_natural_frequency;
  ori_cfg_.atti_damp_ratio = cfg.attitude_damping_ratio;
  ori_cfg_.atti_ki = cfg.attitude_i_gain;
  ori_cfg_.head_natural_freq = cfg.heading_natural_frequency;
  ori_cfg_.head_damp_ratio = cfg.heading_damping_ratio;
  ori_cfg_.head_ki = cfg.heading_i_gain;
  ori_pid_.configure(ori_cfg_);

  // ミキサー
  mixer_cfg_.linear_weight = cfg.mixer_linear_weight;
  mixer_cfg_.angular_weight = cfg.mixer_angular_weight;
  mixer_cfg_.thrust_weight_log10 = cfg.mixer_thrust_weight_log10;
  mixer_.configure(mixer_cfg_);

  TOBAS_INFO("Dynamic parameters are updated.");
}
}  // namespace tobas_np_pid
