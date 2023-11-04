#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_np_pid/ControllerFeedback.h>

#include "../include/tobas_np_pid/controller_ros.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_np_pid
{
ControllerRos::ControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    jnt_name_parser_(drone_.tree()),
    mixer_(drone_),
    check_topics_timer_(nh_, tobas::kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  jnt_name_parser_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

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
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_np_pid::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  super::registerSubscribers();

  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
  if (drone_.isTransformable())
  {
    joint_state_sub_ =
      nh_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  }

  cmd_sub_ = nh_.subscribe(tobas::kPoseTwistAccelCmdTopic, 1, &self::commandCb, this, tcpNoDelay());
}

bool ControllerRos::isReady() const
{
  if (pt_ == nullptr)
    return false;

  if (battery_ == nullptr)
    return false;

  if (drone_.isTransformable() && js_ == nullptr)
    return false;

  return true;
}

void ControllerRos::updateJointArray()
{
  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    try
    {
      const auto& kdl_idx = jnt_name_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      const auto msg_idx = dh_std::findIndex(js_->name, jnt_name);  // msg内でのインデックス
      q_(kdl_idx) = js_->position[msg_idx];
    }
    catch (const exception& e)
    {
      rosError(name_, e.what());
    }
  }
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

void ControllerRos::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      t_last_loop_ = pt->header.stamp;
      is_initialized_ = true;
      rosInfo(name_, "Controller is ready.");
    }
    return;
  }

  // 時刻を更新
  const auto dt = (pt->header.stamp - t_last_loop_).toSec();
  t_last_loop_ = pt->header.stamp;

  // コマンドが来ていなければスキップ
  if (cmd_ == nullptr)
    return;

  // 可動関節の角度を更新
  if (drone_.isTransformable())
    updateJointArray();

  // 位置制御器
  const Vector cur_vel_W = pt->pose.euler * pt->twist.vel;  // 世界座標系から見た現在の速度
  const Vector tar_acc_fb(
    pos_pid_.update(pt->pose.pos.data, cur_vel_W.data, cmd_->pos.data, cmd_->vel.data, dt));
  const Vector tar_acc_W = cmd_->acc + tar_acc_fb;

  // 姿勢制御器
  const Vector tar_dgyro_fb =
    ori_pid_.update(pt->pose.euler, pt->twist.rot, cmd_->rpy, cmd_->gyro, dt);
  const Vector tar_dgyro_B = cmd_->dgyro + tar_dgyro_fb;

  // プロペラの推力を計算
  const VectorXd thrusts =
    mixer_.solve(battery_->voltage, q_, pt->pose.euler, pt->twist.rot, tar_acc_W, tar_dgyro_B);

  // 回転数を発行
  const auto rotor_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  rotor_speeds->header.stamp = pt->header.stamp;
  rotor_speeds->speeds.resize(drone_.numRotors(), 0.);
  for (uint32_t i = 0; i < thrusts.rows(); ++i)
    rotor_speeds->speeds[i] = drone_.rotSpeedFromThrust(i, max(0., thrusts(i)));
  rotor_speeds_pub_.publish(rotor_speeds);

  // フィードバックを発行
  auto feedback = boost::make_shared<tobas_np_pid::ControllerFeedback>();
  feedback->header.stamp = pt->header.stamp;
  feedback->target_position = cmd_->pos;
  feedback->target_velocity_global = cmd_->vel;
  feedback->target_velocity_local = pt->pose.euler.inverse(cmd_->vel);
  feedback->target_acceleration_global = tar_acc_W;
  feedback->target_acceleration_local = pt->pose.euler.inverse(tar_acc_W);
  feedback->target_orientation = cmd_->rpy;
  feedback->position_integral_error.data = pos_pid_.integralError();
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
    rosError(name_, "The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerRos::commandCb(const tobas_msgs::PoseTwistAccelCommandConstPtr& cmd)
{
  if (pt_ == nullptr)
    return;

  // コマンドレベルの処理
  if (!updateCommandLevel(cmd_level_, cmd->level.data))
    return;

  // コマンドを更新
  cmd_ = cmd;
}

void ControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (battery_ == nullptr)
    rosWarn(name_, nh_.getNamespace() << "/" << tobas::kBatteryTopic << " is not received yet.");

  if (pt_ == nullptr)
    rosWarn(name_, nh_.getNamespace() << "/" << tobas::kPoseTwistTopic << " is not received yet.");

  if (drone_.isTransformable() && js_ == nullptr)
    rosWarn(
      name_, nh_.getNamespace() << "/" << tobas::kJointStatesTopic << " is not received yet.");
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  pos_cfg_.hor_kp = cfg.horizontal_p_gain;
  pos_cfg_.hor_ki = cfg.horizontal_i_gain;
  pos_cfg_.hor_kd = cfg.horizontal_d_gain;
  pos_cfg_.ver_kp = cfg.vertical_p_gain;
  pos_cfg_.ver_ki = cfg.vertical_i_gain;
  pos_cfg_.ver_kd = cfg.vertical_d_gain;
  pos_cfg_.max_hor_acc = cfg.max_horizontal_accel;
  pos_cfg_.max_ver_acc = cfg.max_vertical_accel;
  pos_cfg_.max_hor_acc_int = cfg.max_horizontal_accel_I;
  pos_cfg_.max_ver_acc_int = cfg.max_vertical_accel_I;
  pos_pid_.configure(pos_cfg_);

  ori_cfg_.atti_kp = cfg.attitude_p_gain;
  ori_cfg_.atti_ki = cfg.attitude_i_gain;
  ori_cfg_.atti_kd = cfg.attitude_d_gain;
  ori_cfg_.head_kp = cfg.heading_p_gain;
  ori_cfg_.head_ki = cfg.heading_i_gain;
  ori_cfg_.head_kd = cfg.heading_d_gain;
  ori_cfg_.max_atti_acc_int = cfg.max_attitude_accel_I;
  ori_cfg_.max_head_acc_int = cfg.max_heading_accel_I;
  ori_pid_.configure(ori_cfg_);

  mixer_cfg_.linear_weight = cfg.mixer_linear_weight;
  mixer_cfg_.angular_weight = cfg.mixer_angular_weight;
  mixer_cfg_.thrust_weight_log10 = cfg.mixer_thrust_weight_log10;
  mixer_.configure(mixer_cfg_);

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_np_pid
