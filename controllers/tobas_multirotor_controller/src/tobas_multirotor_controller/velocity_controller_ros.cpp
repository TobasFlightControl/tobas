#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <dh_std_tools/vector.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/operators.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_tools/conversions/eigen_msg.hpp>
#include <tobas_msgs/FrameId.h>

#include "../../include/tobas_multirotor_controller/velocity_controller_ros.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
VelocityControllerRos::VelocityControllerRos()
  : jnt_name_parser_(tree_),
    is_initialized_(false),
    bs_received_(false),
    js_received_(false),
    cmd_received_(false)
{
  getRosParams();

  is_transformable_ = required_joints_.size() > 0;

  // Treeを取得
  if (!kdl_parser::treeFromString(description_, tree_))
  {
    dh_ros::RuntimeError("Failed to get KDL tree.");
  }

  jnt_name_parser_.updateInternalDataStructures();

  // 各コントローラを初期化
  vel_controller_.reset(new VelocityController(dynamic_params_vel_));
  acc_controller_.reset(
    new AccelerationController(tree_, gravity_, battery_voltage_, rotor_configs_));
  rot_controller_.reset(
    new RotationController(tree_, gravity_, battery_voltage_, rotor_configs_, dynamic_params_rot_));

  q_.resize(tree_.getNrOfJoints());
  rotor_speeds_.speeds.resize(num_rotors_);

  registerPublishers();
  registerSubscribers();
  createTimers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&VelocityControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void VelocityControllerRos::getRosParams()
{
  dh_ros::getParam("/drone_name", drone_name_);
  dh_ros::getParam("/robot_description", description_);
  dh_ros::getParam("/num_rotors", num_rotors_);
  dh_ros::getParam<vector<string>>("/required_joint_names", required_joints_);
  dh_ros::getParam("/gravity", gravity_);
  dh_ros::getParam("/battery_voltage", battery_voltage_);
  getRotorConfigs(rotor_configs_);

  // velocity controller
  dh_ros::getParam(ctrlPrefix + "/natural_frequency", dynamic_params_vel_.natural_freq);
  dh_ros::getParam(ctrlPrefix + "/damping_ratio", dynamic_params_vel_.damp_ratio);

  // rotation_controller
  dh_ros::getParam(ctrlPrefix + "/prediction_horizon", dynamic_params_rot_.pred_horizon);
  dh_ros::getParam(ctrlPrefix + "/prediction_steps", dynamic_params_rot_.pred_steps);
  dh_ros::getParam(ctrlPrefix + "/rotation_decay", dynamic_params_rot_.rot_decay);
  dh_ros::getParam(ctrlPrefix + "/angular_velocity_decay", dynamic_params_rot_.angvel_decay);
  dh_ros::getParam(ctrlPrefix + "/rotation_weight", dynamic_params_rot_.rot_weight);
  dh_ros::getParam(ctrlPrefix + "/angular_velocity_weight", dynamic_params_rot_.angvel_weight);
  dh_ros::getParam(ctrlPrefix + "/thrust_force_weight", dynamic_params_rot_.thrust_weight);
  dh_ros::getParam(
    ctrlPrefix + "/thrust_force_rate_weight", dynamic_params_rot_.thrust_rate_weight);
}

void VelocityControllerRos::registerPublishers()
{
  rotor_speeds_pub_ =
    nh_.advertise<tobas_msgs::RotorSpeeds>("/" + drone_name_ + "/command/motor_speed", 1, false);
}

void VelocityControllerRos::registerSubscribers()
{
  string drone_prefix = "/" + drone_name_;

  base_state_sub_ =
    nh_.subscribe(drone_prefix + "/base_state", 1, &VelocityControllerRos::baseStateCb, this);
  if (is_transformable_)
  {
    joint_state_sub_ =
      nh_.subscribe(drone_prefix + "/joint_states", 1, &VelocityControllerRos::jointStateCb, this);
  }
  cmd_sub_ = nh_.subscribe(
    drone_prefix + "/command/velocity_yaw", 1, &VelocityControllerRos::commandCb, this);
}

void VelocityControllerRos::createTimers()
{
  check_topics_timer_ = nh_.createTimer(
    ros::Duration(checkTopicsTimerPeriod), &VelocityControllerRos::checkTopicsTimerCb, this);
}

bool VelocityControllerRos::isReady()
{
  if (!bs_received_)
  {
    return false;
  }

  if (is_transformable_ && !js_received_)
  {
    return false;
  }

  return true;
}

void VelocityControllerRos::initialize(const tobas_msgs::PoseVel& bs)
{
  t_last_ = ros::Time::now();
  tar_rpy_.z() = bs.pose.orientation.yaw;
  u_opt_ = VectorXd::Zero(num_rotors_);
}

void VelocityControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_rot_.pred_horizon = cfg.prediction_horizon;
  dynamic_params_rot_.pred_steps = cfg.prediction_steps;
  dynamic_params_rot_.rot_decay = cfg.rotation_decay;
  dynamic_params_rot_.angvel_decay = cfg.angular_velocity_decay;
  dynamic_params_rot_.rot_weight = cfg.rotation_weight;
  dynamic_params_rot_.angvel_weight = cfg.angular_velocity_weight;
  dynamic_params_rot_.thrust_weight = cfg.thrust_force_weight;
  dynamic_params_rot_.thrust_rate_weight = cfg.thrust_force_rate_weight;
}

void VelocityControllerRos::runOnce(const tobas_msgs::PoseVel& bs)
{
  // 時刻を更新
  ros::Time now = ros::Time::now();
  double dt = (now - t_last_).toSec();
  t_last_ = now;

  // 現在の状態を更新
  tf::vectorMsgToEigen(bs.twist.linear, cur_vel_W_);
  tf::eulerMsgToEigen(bs.pose.orientation, cur_rpy_);
  tf::vectorMsgToEigen(bs.twist.angular, cur_angvel_B_);  // 角速度だけはローカル座標系！

  // 速度制御器
  vel_controller_->update(cur_vel_W_, tar_vel_W_, tar_acc_W_);

  // 非線形変換
  acc_controller_->update(tar_acc_W_, bs.pose.orientation.yaw, U_, tar_rpy_.x(), tar_rpy_.y());
  if (U_ < 0. || acc_controller_->maxU() < U_)
  {
    const double& max_U = acc_controller_->maxU();
    dh_ros::rosWarnThrottle(
      warnPeriod, "U_out = " + to_string(U_) + " is out of range [0, " + to_string(max_U) + "].");
    U_ = dh_std::clamp(U_, 0., max_U);
  }

  // 姿勢制御器
  rot_controller_->update(cur_rpy_, cur_angvel_B_, q_, U_, tar_rpy_, u_opt_);

  // 各モータの回転速度を計算
  ctrlInputToRotorSpeeds(u_opt_, rotor_speeds_);

  // モータ速度を発行
  rotor_speeds_pub_.publish(rotor_speeds_);
}

void VelocityControllerRos::ctrlInputToRotorSpeeds(
  const Eigen::VectorXd& u,
  tobas_msgs::RotorSpeeds& speeds)
{
  ROS_ASSERT(u.rows() == num_rotors_);

  for (int i = 0; i < num_rotors_; ++i)
  {
    if (u(i) < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: u = " + to_string(u(i)));
      // TODO: 防御モードに移行
    }
    speeds.speeds[i] = sqrt(max(u(i), 0.) / rotor_configs_[i].motor_constant);
  }
}

void VelocityControllerRos::baseStateCb(const StateMsg& bs)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize(bs.pose_vel);
      is_initialized_ = true;
      dh_ros::rosInfo("Velocity controller is ready.");
    }
    return;
  }

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  runOnce(bs.pose_vel);
}

void VelocityControllerRos::jointStateCb(const sensor_msgs::JointState& js)
{
  if (js.name.size() != js.position.size())
  {
    dh_ros::rosError("The size of joint name and position is different.");
    return;
  }

  for (const auto& jnt_name : required_joints_)
  {
    try
    {
      const auto msg_idx = dh_std::findIndex(js.name, jnt_name);  // msg内でのインデックス
      const auto& jnt_pos = js.position[msg_idx];
      const auto& kdl_idx = jnt_name_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_(kdl_idx) = jnt_pos;
    }
    catch (const exception& e)
    {
      dh_ros::rosError(e.what());
      return;
    }
  }

  js_received_ = true;
}

void VelocityControllerRos::commandCb(const CmdMsg& cmd)
{
  switch (cmd.frame_id.frame_id)
  {
    case tobas_msgs::FrameId::GLOBAL:
    {
      tf::vectorMsgToEigen(cmd.velocity, tar_vel_W_);
      tar_rpy_.z() = cmd.yaw;
      break;
    }
    case tobas_msgs::FrameId::LOCAL:
    {
      const auto& cmd_vel = cmd.velocity;
      rotateVector(
        0., 0., cur_rpy_.z(), cmd_vel.x, cmd_vel.y, cmd_vel.z, tar_vel_W_.x(), tar_vel_W_.y(),
        tar_vel_W_.z());
      tar_rpy_.z() = cmd.yaw;
      break;
    }
    default:
    {
      dh_ros::rosError("Invalid FrameId: " + to_string(cmd.frame_id.frame_id));
      return;
    }
  }

  cmd_received_ = true;
}

void VelocityControllerRos::checkTopicsTimerCb(const ros::TimerEvent& event)
{
  if (!bs_received_)
  {
    dh_ros::rosWarn("Base state is not received yet.");
  }

  if (is_transformable_ && !js_received_)
  {
    dh_ros::rosWarn("Joint states are not received yet.");
  }
}

void VelocityControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  updateDynamicParams(cfg);
  rot_controller_->reconfigure(dynamic_params_rot_);
}
}  // namespace tobas_multirotor_controller
