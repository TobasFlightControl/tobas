#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/geometry.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_mr_pid/ControllerFeedback.h>

#include "../include/tobas_mr_pid/controller_ros.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_mr_pid
{
ControllerRos::ControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    jnt_name_parser_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    acc_controller_(drone_),
    mixer_(drone_),
    check_topics_timer_(nh_, tobas::kCheckTopicsTimerPeriod, &self::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  jnt_name_parser_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();
  acc_controller_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  is_transformable_ = drone_.postureDefiningJoints().size() > 0;
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
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorCmdTopic, 1);
  feedback_pub_ =
    nh_.advertise<tobas_mr_pid::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &self::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
  if (is_transformable_)
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
  if (pt_ == nullptr)
    return false;

  if (battery_ == nullptr)
    return false;

  if (is_transformable_ && js_ == nullptr)
    return false;

  return true;
}

bool ControllerRos::isCommandLevelOk(const tobas_msgs::CommandLevel& level)
{
  if (level.data < cmd_level_)
  {
    rosErrorThrottle(
      tobas::kCommandLevelErrorPeriod, name_,
      "The command is ignored because its level " << static_cast<int>(level.data)
                                                  << "is lower than the current command level "
                                                  << static_cast<int>(cmd_level_) << ".");
    return false;
  }

  if (level.data > cmd_level_)
  {
    rosInfo(
      name_, "The command level is raised from " << static_cast<int>(cmd_level_) << " to "
                                                 << static_cast<int>(level.data) << ".");
    cmd_level_ = level.data;
  }

  return true;
}

void ControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
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

  // Create a feedback message
  auto feedback = boost::make_shared<tobas_mr_pid::ControllerFeedback>();
  feedback->header.stamp = pt->header.stamp;

  // Translation Controller
  if (tar_pvay_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
    {
      tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
    }

    // 世界座標系から見た現在の速度を計算
    const Vector cur_vel_W = pt->pose.euler * pt->twist.vel;

    // 目標加速度を計算
    trans_controller_.update(
      pt->pose.pos, cur_vel_W, tar_pvay_->pos, tar_pvay_->vel, tar_acc_fb_, dt);
    const auto tar_acc = tar_pvay_->acc + tar_acc_fb_;

    // 推力和と目標姿勢を計算
    acc_controller_.update(
      pt->pose.euler, tar_acc, tar_rpyt_->thrust, tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は加速度指令をそのまま流す
    tar_rpyt_->level = tar_pvay_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_->yaw;

    // Fill feedback
    feedback->target_position = tar_pvay_->pos;
    feedback->target_velocity_global = tar_pvay_->vel;
    feedback->target_velocity_local = pt->pose.euler.Inverse(tar_pvay_->vel);
    feedback->target_acceleration_global = tar_acc;
    feedback->target_acceleration_local = pt->pose.euler * tar_acc;
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    // 処理の遅延を防ぐため，JointStateのコールバックではなくここで行う
    if (is_transformable_)
    {
      for (const auto& jnt_name : drone_.postureDefiningJoints())
      {
        try
        {
          const auto msg_idx = dh_std::findIndex(js_->name, jnt_name);  // msg内でのインデックス
          const auto& jnt_pos = js_->position[msg_idx];
          const auto& kdl_idx = jnt_name_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
          q_(kdl_idx) = jnt_pos;
        }
        catch (const exception& e)
        {
          rosError(name_, e.what());
        }
      }
    }

    // 目標オイラー角加速度を計算
    const auto& rpy = pt->pose.euler;
    const Vector3d rpyd =
      eigen_tools::eulerrateFromAngvelLocal(pt->twist.rot.data, rpy.roll, rpy.pitch);
    const Vector3d tar_rpydd = rot_controller_.update(
      rpy.toVector().data, rpyd, tar_rpyt_->rpy.toVector().data, Vector3d::Zero(), dt);

    // オイラー角加速度から角加速度を計算
    const Vector3d tar_dgyro =
      eigen_tools::angaccFromEuleraccLocal(rpy.roll, rpy.pitch, rpyd, tar_rpydd);

    // プロペラの推力を計算
    const VectorXd thrusts =
      mixer_.solve(battery_->voltage, q_, pt->twist.rot.data, tar_dgyro, tar_rpyt_->thrust);

    // モータ速度メッセージを作成
    const auto rotor_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
    rotor_speeds->header.stamp = pt->header.stamp;
    rotor_speeds->speeds.resize(drone_.numRotors(), 0.);
    for (uint32_t i = 0; i < thrusts.rows(); ++i)
    {
      rotor_speeds->speeds[z_rotors_.rotorIdx(i)] =
        z_rotors_.rotSpeedFromThrust(i, max(0., thrusts(i)));
    }

    // モータ速度を発行
    rotor_speeds_pub_.publish(rotor_speeds);

    // Fill feedback
    feedback->target_rotation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;

    // Publish feedback
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

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay)
{
  if (pt_ == nullptr)
    return;

  // コマンドレベルの処理
  if (!isCommandLevelOk(pvay->level))
    return;

  // コマンドを更新
  tar_pvay_ = boost::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::GLOBAL, pt_->pose.euler, *tar_pvay_))
  {
    rosError(name_, "Failed to change command frame. Probably the frame id is invalid.");
    tar_pvay_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt)
{
  if (pt_ == nullptr)
    return;

  if (!isCommandLevelOk(rpyt->level))
    return;

  // 外側の制御を止める
  tar_pvay_ = nullptr;

  // コマンドを更新
  tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

void ControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (battery_ == nullptr)
    rosWarn(name_, "Battery state is not received yet.");

  if (pt_ == nullptr)
    rosWarn(name_, "Pose & Twist is not received yet.");

  if (is_transformable_ && js_ == nullptr)
    rosWarn(name_, "Joint states are not received yet.");
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  trans_config_.hor_kp = cfg.horizontal_p_gain;
  trans_config_.hor_ki = cfg.horizontal_i_gain;
  trans_config_.hor_kd = cfg.horizontal_d_gain;
  trans_config_.ver_kp = cfg.vertical_p_gain;
  trans_config_.ver_ki = cfg.vertical_i_gain;
  trans_config_.ver_kd = cfg.vertical_d_gain;
  trans_config_.max_hor_vel = cfg.max_horizontal_velocity;
  trans_config_.max_ver_vel = cfg.max_vertical_velocity;
  trans_controller_.configure(trans_config_);

  acc_config_.max_hor_acc = cfg.max_horizontal_accel;
  acc_config_.max_ver_acc = cfg.max_vertical_accel;
  acc_config_.max_attitude = cfg.max_attitude;
  acc_config_.h_force_coef = 0;  // TODO
  acc_controller_.configure(acc_config_);

  rot_config_.atti_kp = cfg.attitude_p_gain;
  rot_config_.atti_ki = cfg.attitude_i_gain;
  rot_config_.atti_kd = cfg.attitude_d_gain;
  rot_config_.head_kp = cfg.heading_p_gain;
  rot_config_.head_ki = cfg.heading_i_gain;
  rot_config_.head_kd = cfg.heading_d_gain;
  rot_config_.max_attitude = cfg.max_attitude;
  rot_config_.max_heading_error = cfg.max_heading_error;
  rot_controller_.configure(rot_config_);

  // TODO: Mixerの設定

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_pid
