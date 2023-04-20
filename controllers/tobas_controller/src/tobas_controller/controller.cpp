#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <tobas_tools/operators.hpp>
#include <tobas_tools/utils.hpp>

#include "../../include/tobas_controller/controller.hpp"

#define INFO_PERIOD 1.
#define INIT_ELEVATION 1.

using namespace std;
using namespace KDL;

Controller::Controller()
  : kdl_model_(tree_),
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    required_joints_(dh_ros::getParam<vector<string>>("/required_joint_names")),
    transformable_(required_joints_.size() > 0),
    rotor_configs_(getRotorConfigs()),
    is_first_run_(true),
    bs_received_(false),
    js_received_(false),
    cmd_received_(false),
    acc_controller_(tree_),
    rot_controller_(tree_)
{
  // URDFを取得
  const string drone_name = dh_ros::getParam<string>("/drone_name");
  const string description = dh_ros::getParam<string>("/robot_description");

  // Treeを取得
  const bool ok = kdl_parser::treeFromString(description, tree_);
  ROS_ASSERT(ok);
  kdl_model_.updateInternalDataStructures();
  acc_controller_.updateInternalDataStructures();
  rot_controller_.updateInternalDataStructures();

  q_.resize(kdl_model_.getNrOfJoints());
  feedback_.thrust_forces.resize(num_rotors_);
  rotor_speeds_.speeds.resize(num_rotors_);

  // PubSub
  rotor_speeds_pub_ =
    nh_.advertise<tobas_msgs::RotorSpeeds>("/" + drone_name + "/command/motor_speed", 1, false);
  feedback_pub_ =
    nh_.advertise<tobas_msgs::ControllerFeedback>("/tobas_controller/feedback", 1, false);
  bs_sub_ = nh_.subscribe("/" + drone_name + "/base_state", 1, &Controller::bsCb, this);
  if (transformable_)
  {
    js_sub_ = nh_.subscribe("/" + drone_name + "/joint_states", 1, &Controller::jsCb, this);
  }
  cmd_sub_ = nh_.subscribe("/tobas_controller/command", 1, &Controller::commandCb, this);

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::runOnce()
{
  // 初回動作時は時刻を更新するのみ
  if (is_first_run_)
  {
    t_last_ = ros::Time::now();
    is_first_run_ = false;
    return;
  }

  // 時刻を更新
  ros::Time now = ros::Time::now();
  double dt = (now - t_last_).toSec();
  t_last_ = now;

  // 目標状態を更新
  if (cmd_received_)
  {
    updateDesiredState(dt);
  }

  auto& pos_des = feedback_.desired_position;
  auto& acc_des = feedback_.desired_acceleration;
  auto& rpy_des = feedback_.desired_orientation;
  auto& U = feedback_.thrust_force_sum;
  auto& u = feedback_.thrust_forces;

  // 位置とヨー角の目標値はコマンドどおり
  pos_des = pos_des_;
  rpy_des.yaw = yaw_des_;

  // TODO: 非ゼロの速度目標値を与える
  geometry_msgs::Vector3 vel_des;

  // 位置制御器，非線形変換，姿勢制御機の順に実行
  pos_controller_.update(bs_.pose.position, pos_des, bs_.twist.linear, vel_des, acc_des);
  acc_controller_.update(acc_des, bs_.pose.orientation.yaw, U, rpy_des.roll, rpy_des.pitch);
  rot_controller_.update(bs_, q_, U, rpy_des.roll, rpy_des.pitch, rpy_des.yaw, u);

  // 各モータの回転速度を計算
  ctrlInputToRotorSpeeds(u, rotor_speeds_);

  // モータ速度とフィードバックを発行
  rotor_speeds_pub_.publish(rotor_speeds_);
  feedback_pub_.publish(feedback_);
}

void Controller::updateDesiredState(double dt)
{
  ROS_ASSERT(dt >= 0.);

  switch (cmd_.mode)
  {
    case CmdMsg::GLOBAL_POSITION:
    {
      pos_des_ = cmd_.target_position;
      yaw_des_ = cmd_.target_yaw_angle;
      break;
    }
    case CmdMsg::GLOBAL_VELOCITY:
    {
      pos_des_ = pos_des_ + cmd_.target_velocity * dt;
      yaw_des_ = yaw_des_ + cmd_.target_yaw_rate * dt;
      break;
    }
    case CmdMsg::LOCAL_VELOCITY:
    {
      const auto& rpy = bs_.pose.orientation;
      const auto& V_B = cmd_.target_velocity;
      geometry_msgs::Vector3 V_W;
      // rotateVector(rpy.roll, rpy.pitch, rpy.yaw, V_B.x, V_B.y, V_B.z, V_W.x, V_W.y, V_W.z);
      rotateVector(0., 0., rpy.yaw, V_B.x, V_B.y, V_B.z, V_W.x, V_W.y, V_W.z);
      pos_des_ = pos_des_ + V_W * dt;
      yaw_des_ = yaw_des_ + cmd_.target_yaw_rate * dt;
      break;
    }
    default:
    {
      dh_ros::rosErrorThrottle(INFO_PERIOD, "Invalid command mode: " + to_string(cmd_.mode));
      return;
    }
  }
}

void Controller::ctrlInputToRotorSpeeds(const vector<double>& u, tobas_msgs::RotorSpeeds& speeds)
{
  ROS_ASSERT(u.size() == num_rotors_);

  for (int i = 0; i < num_rotors_; ++i)
  {
    if (u[i] < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: u = " + to_string(u[i]));
      // TODO: 防御モードに移行
    }
    speeds.speeds[i] = sqrt(max(u[i], 0.) / rotor_configs_[i].motor_constant);
  }
}

void Controller::bsCb(const StateMsg& bs)
{
  // 最初は暴れるのを防ぐために現在の状態を目標状態にする
  if (!bs_received_)
  {
    pos_des_ = bs.pose_vel.pose.position;
    pos_des_.z += INIT_ELEVATION;  // 地面との衝突を避けるためにZ座標だけは少し上げておく
    yaw_des_ = bs.pose_vel.pose.orientation.yaw;
    bs_received_ = true;
  }

  bs_ = bs.pose_vel;

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  if (!transformable_ || js_received_)
  {
    runOnce();
  }
}

void Controller::jsCb(const sensor_msgs::JointState& js)
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
      const auto& kdl_idx = kdl_model_.jointIndex(jnt_name);  // Tree内でのインデックス
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

void Controller::commandCb(const CmdMsg& cmd)
{
  cmd_ = cmd;
  cmd_received_ = true;
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  pos_controller_.reconfigure(cfg.natural_frequency, cfg.damping_ratio);
  rot_controller_.reconfigure(
    cfg.prediction_horizon, cfg.prediction_steps, cfg.rotation_decay, cfg.angular_velocity_decay,
    cfg.rotation_weight, cfg.angular_velocity_weight, cfg.thrust_force_weight,
    cfg.thrust_force_rate_weight);
}
