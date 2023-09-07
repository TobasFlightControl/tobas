#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_multirotor_controller/controller_ros.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
ControllerRos::ControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    jnt_name_parser_(drone_.tree()),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    rot_controller_(drone_),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &ControllerRos::checkTopicsTimerCb, this),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  jnt_name_parser_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();
  rot_controller_.updateInternalDataStructures();

  is_transformable_ = drone_.postureDefiningJoints().size() > 0;
  q_.resize(drone_.tree().getNrOfJoints());

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&ControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ControllerRos::getRosParams()
{
}

void ControllerRos::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>("command/motor_speed", 1);
}

void ControllerRos::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &ControllerRos::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe("pose_twist", 1, &ControllerRos::poseTwistCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe("battery", 1, &ControllerRos::batteryCb, this, tcpNoDelay());
  if (is_transformable_)
  {
    joint_state_sub_ =
      nh_.subscribe("joint_states", 1, &ControllerRos::jointStateCb, this, tcpNoDelay());
  }

  pos_yaw_sub_ =
    nh_.subscribe("command/position_yaw", 1, &ControllerRos::posYawCb, this, tcpNoDelay());
  vel_yaw_sub_ =
    nh_.subscribe("command/velocity_yaw", 1, &ControllerRos::velYawCb, this, tcpNoDelay());
  acc_yaw_sub_ =
    nh_.subscribe("command/acceleration_yaw", 1, &ControllerRos::accYawCb, this, tcpNoDelay());
  rpy_thrust_sub_ =
    nh_.subscribe("command/rpy_thrust", 1, &ControllerRos::rpyThrustCb, this, tcpNoDelay());
  rpyd_thrust_sub_ = nh_.subscribe(
    "command/roll_pitch_yawrate_thrust", 1, &ControllerRos::rpydThrustCb, this, tcpNoDelay());
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
      kErrorPeriod, name_,
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
      is_initialized_ = true;
      rosInfo(name_, "Controller is ready.");
    }
    return;
  }

  // Position Controller
  if (tar_pos_yaw_ != nullptr)
  {
    if (tar_vel_yaw_ == nullptr)
    {
      tar_vel_yaw_ = boost::make_shared<tobas_msgs::VelocityYaw>();
      tar_vel_yaw_->frame_id.data = tobas_msgs::FrameId::GLOBAL;
    }

    // 目標速度を計算
    pos_controller_.update(pt->pose.pos, tar_pos_yaw_->pos, tar_vel_yaw_->vel);

    // コマンドレベルとヨー角は位置指令をそのまま流す
    tar_vel_yaw_->level = tar_pos_yaw_->level;
    tar_vel_yaw_->yaw = tar_pos_yaw_->yaw;
  }

  // Velocity Controller
  if (tar_vel_yaw_ != nullptr)
  {
    if (tar_acc_yaw_ == nullptr)
    {
      tar_acc_yaw_ = boost::make_shared<tobas_msgs::AccelerationYaw>();
      tar_acc_yaw_->frame_id.data = tobas_msgs::FrameId::GLOBAL;
    }

    // 世界座標系から見た現在の速度を計算
    const Vector cur_vel_W = pt->pose.euler * pt->twist.vel;

    // 目標加速度を計算
    vel_controller_.update(cur_vel_W, tar_vel_yaw_->vel, tar_acc_yaw_->acc);

    // コマンドレベルとヨー角は速度指令をそのまま流す
    tar_acc_yaw_->level = tar_vel_yaw_->level;
    tar_acc_yaw_->yaw = tar_vel_yaw_->yaw;
  }

  // Acceleration Controller
  if (tar_acc_yaw_ != nullptr)
  {
    if (tar_rpy_thrust_ == nullptr)
    {
      tar_rpy_thrust_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
    }

    // 推力和と目標姿勢を計算
    acc_controller_.update(
      tar_acc_yaw_->acc, pt->pose.euler.yaw, tar_rpy_thrust_->thrust, tar_rpy_thrust_->rpy.roll,
      tar_rpy_thrust_->rpy.pitch);

    // コマンドレベルとヨー角は加速度指令をそのまま流す
    tar_rpy_thrust_->level = tar_acc_yaw_->level;
    tar_rpy_thrust_->rpy.yaw = tar_acc_yaw_->yaw;
  }

  // Rotation Controller
  if (tar_rpy_thrust_ != nullptr)
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

    // プロペラ推力を計算
    try
    {
      // stopwatch_.start();
      rot_controller_.update(
        pt->pose.euler, pt->twist.rot, q_, battery_->voltage, tar_rpy_thrust_->thrust,
        tar_rpy_thrust_->rpy, u_opt_);
      // stopwatch_.stop();
    }
    catch (const exception& e)  // MPCがコケたり
    {
      rosError(name_, e.what());
      return;
    }

    // モータ速度メッセージを作成
    const auto rotor_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
    rotor_speeds->header.stamp = pt->header.stamp;
    rotor_speeds->speeds.resize(drone_.numRotors(), 0.);
    for (uint32_t i = 0; i < u_opt_.rows(); ++i)
    {
      if (u_opt_(i) < -1.)
      {
        rosFatal(name_, "Negative thrust force: " << u_opt_(i) << " [N]");
        // TODO: 防御モードに移行
      }
      rotor_speeds->speeds[z_rotors_.rotorIdx(i)] =
        z_rotors_.rotSpeedFromThrust(i, max(0., u_opt_(i)));
    }

    // モータ速度を発行
    rotor_speeds_pub_.publish(rotor_speeds);
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

void ControllerRos::posYawCb(const tobas_msgs::PositionYawConstPtr& pos_yaw)
{
  if (pt_ == nullptr)
  {
    return;
  }

  if (!isCommandLevelOk(pos_yaw->level))
  {
    return;
  }

  tar_pos_yaw_ = boost::make_shared<tobas_msgs::PositionYaw>(*pos_yaw);
}

void ControllerRos::velYawCb(const tobas_msgs::VelocityYawConstPtr& vel_yaw)
{
  if (pt_ == nullptr)
  {
    return;
  }

  // コマンドレベルの処理
  if (!isCommandLevelOk(vel_yaw->level))
  {
    return;
  }

  // メモリ確保
  if (tar_vel_yaw_ == nullptr)
  {
    tar_vel_yaw_ = boost::make_shared<tobas_msgs::VelocityYaw>();
    tar_vel_yaw_->frame_id.data = tobas_msgs::FrameId::GLOBAL;
  }

  // 外側の制御を止める
  tar_pos_yaw_ = nullptr;

  // 目標速度を更新
  switch (vel_yaw->frame_id.data)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tar_vel_yaw_->vel = vel_yaw->vel;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      tar_vel_yaw_->vel = pt_->pose.euler * vel_yaw->vel;
      break;
    }
    default:
    {
      rosError(name_, "Invalid FrameId: " << static_cast<int>(vel_yaw->frame_id.data));
      return;
    }
  }

  // 目標ヨー角を更新 (そのまま流すだけ)
  tar_vel_yaw_->yaw = vel_yaw->yaw;
}

void ControllerRos::accYawCb(const tobas_msgs::AccelerationYawConstPtr& acc_yaw)
{
  if (pt_ == nullptr)
  {
    return;
  }

  // コマンドレベルの処理
  if (!isCommandLevelOk(acc_yaw->level))
  {
    return;
  }

  // メモリ確保
  if (tar_acc_yaw_ == nullptr)
  {
    tar_acc_yaw_ = boost::make_shared<tobas_msgs::AccelerationYaw>();
    tar_acc_yaw_->frame_id.data = tobas_msgs::FrameId::GLOBAL;
  }

  // 外側の制御を止める
  tar_pos_yaw_ = nullptr;
  tar_vel_yaw_ = nullptr;

  // 目標速度を更新
  switch (acc_yaw->frame_id.data)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tar_acc_yaw_->acc = acc_yaw->acc;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      tar_acc_yaw_->acc = pt_->pose.euler * acc_yaw->acc;
      break;
    }
    default:
    {
      rosError(name_, "Invalid FrameId: " << static_cast<int>(acc_yaw->frame_id.data));
      return;
    }
  }

  // 目標ヨー角を更新 (そのまま流すだけ)
  tar_acc_yaw_->yaw = acc_yaw->yaw;
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust)
{
  if (pt_ == nullptr)
  {
    return;
  }

  if (!isCommandLevelOk(rpy_thrust->level))
  {
    return;
  }

  // 外側の制御を止める
  tar_pos_yaw_ = nullptr;
  tar_vel_yaw_ = nullptr;
  tar_acc_yaw_ = nullptr;

  // コマンドを更新
  tar_rpy_thrust_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpy_thrust);
}

void ControllerRos::rpydThrustCb(const tobas_msgs::RollPitchYawrateThrustConstPtr& rpyd_thrust)
{
  if (pt_ == nullptr)
  {
    return;
  }

  if (!isCommandLevelOk(rpyd_thrust->level))
  {
    return;
  }

  // メモリ確保
  if (tar_rpy_thrust_ == nullptr)
  {
    tar_rpy_thrust_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
  }

  // 外側の制御を止める
  tar_pos_yaw_ = nullptr;
  tar_vel_yaw_ = nullptr;
  tar_acc_yaw_ = nullptr;

  // ロール角，ピッチ角，推力和の目標値を更新
  tar_rpy_thrust_->rpy.roll = rpyd_thrust->roll;
  tar_rpy_thrust_->rpy.pitch = rpyd_thrust->pitch;
  tar_rpy_thrust_->thrust = rpyd_thrust->thrust;

  // ヨー角の目標値を更新
  if (rpyd_thrust_received_)
  {
    const auto cur_time = ros::Time::now();
    const auto dt = (cur_time - t_last_rpyd_thrust_).toSec();
    t_last_rpyd_thrust_ = cur_time;
    if (dt > kRollPitchYawrateThrustTimeout)
    {
      rosInfo(
        name_, "The time gap from the previous command is over "
                 << kRollPitchYawrateThrustTimeout << " seconds. The command is reset.");
      rpyd_thrust_received_ = false;
      return;
    }
    tar_rpy_thrust_->rpy.yaw += rpyd_thrust->yawrate * dt;  // 変化量を足す
  }
  else
  {
    t_last_rpyd_thrust_ = ros::Time::now();
    tar_rpy_thrust_->rpy.yaw = pt_->pose.euler.yaw;  // 最初は現在のヨー角を指令
    rpyd_thrust_received_ = true;
  }
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
  pos_params_.hor_natural_freq = cfg.horizontal_natural_frequency;
  pos_params_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  pos_params_.ver_natural_freq = cfg.vertical_natural_frequency;
  pos_params_.ver_damp_ratio = cfg.vertical_damping_ratio;
  pos_controller_.configure(pos_params_);

  vel_params_.hor_natural_freq = cfg.horizontal_natural_frequency;
  vel_params_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  vel_params_.ver_natural_freq = cfg.vertical_natural_frequency;
  vel_params_.ver_damp_ratio = cfg.vertical_damping_ratio;
  vel_params_.max_hor_vel = cfg.max_horizontal_velocity;
  vel_params_.max_ver_vel = cfg.max_vertical_velocity;
  vel_controller_.configure(vel_params_);

  acc_params_.max_hor_acc = cfg.max_horizontal_accel;
  acc_params_.max_ver_acc = cfg.max_vertical_accel;
  acc_controller_.configure(acc_params_);

  rot_params_.pred_horizon = cfg.prediction_horizon;
  rot_params_.pred_steps = cfg.prediction_steps;
  rot_params_.attitude_decay = cfg.attitude_decay;
  rot_params_.heading_decay = cfg.heading_decay;
  rot_params_.angvel_decay = cfg.angular_velocity_decay;
  rot_params_.attitude_weight = cfg.attitude_weight;
  rot_params_.heading_weight = cfg.heading_weight;
  rot_params_.angvel_weight = cfg.angular_velocity_weight;
  rot_params_.thrust_weight_exp = cfg.thrust_weight_exp;
  rot_params_.thrust_rate_weight_exp = cfg.thrust_rate_weight_exp;
  rot_controller_.configure(rot_params_);

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
