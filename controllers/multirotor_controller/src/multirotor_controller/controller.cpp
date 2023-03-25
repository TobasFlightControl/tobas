#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/iostream.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_eigen_tools/core.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/stopwatch.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/util.hpp>

#include "../../include/multirotor_controller/controller.hpp"

using namespace std;
using namespace KDL;

Controller::Controller(ros::NodeHandle& nh)
  : kdl_model_(tree_),
    num_rotors_(dh_ros::getParam<int>("/num_rotors")),
    required_joints_(dh_ros::getParam<vector<string>>("/required_joint_names")),
    transformable_(required_joints_.size() > 0),
    rotor_props_(getRotorProperties()),
    js_subscribed_(false),
    cmd_subscribed_(false),
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

  const string ns = ros::this_node::getNamespace();

  // PubSub
  rotor_speeds_pub_ =
    nh.advertise<multirotor_msgs::RotorSpeeds>("/" + drone_name + "/command/motor_speed", 1, false);
  feedback_pub_ =
    nh.advertise<multirotor_msgs::ControllerFeedback>("/multirotor_controller/feedback", 1, false);
  bs_sub_ = nh.subscribe("/" + drone_name + "/base_state", 1, &Controller::bsCb, this);
  if (transformable_)
  {
    js_sub_ = nh.subscribe("/" + drone_name + "/joint_states", 1, &Controller::jsCb, this);
  }
  cmd_sub_ = nh.subscribe("/multirotor_controller/command", 1, &Controller::commandCb, this);

  // Dynamic Reconfigure
  ConfigServer::CallbackType f = boost::bind(&Controller::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void Controller::runOnce()
{
  auto& pos_des = feedback_.desired_position;
  auto& acc_des = feedback_.desired_acceleration;
  auto& rpy_des = feedback_.desired_orientation;
  auto& U = feedback_.thrust_force_sum;
  auto& u = feedback_.thrust_forces;

  // 位置とヨー角の目標値はコマンドどおり
  pos_des = cmd_.target_position;
  rpy_des.yaw = cmd_.target_yaw_angle;

  // TODO: 非ゼロの速度目標値を与える
  geometry_msgs::Vector3 vel_des;

  // cout << bs_.pose.position << endl;
  // cout << pos_des << endl;
  // cout << endl;

  // stopwatch_.start();
  pos_controller_.update(bs_.pose.position, pos_des, bs_.twist.linear, vel_des, acc_des);
  acc_controller_.update(acc_des, rpy_des.yaw, U, rpy_des.roll, rpy_des.pitch);
  rot_controller_.update(bs_, q_, U, rpy_des.roll, rpy_des.pitch, rpy_des.yaw, u);
  // stopwatch_.stop();

  rotorVelsFromCtrlInput(u, rotor_speeds_);

  rotor_speeds_pub_.publish(rotor_speeds_);
  feedback_pub_.publish(feedback_);
}

void Controller::rotorVelsFromCtrlInput(
  const vector<double>& u,
  multirotor_msgs::RotorSpeeds& rotor_speeds)
{
  ROS_ASSERT(u.size() == num_rotors_);

  for (int i = 0; i < num_rotors_; ++i)
  {
    if (u[i] < -1.)
    {
      dh_ros::rosFatal("Negative thrust force: u = " + to_string(u[i]));
      // TODO: 防御モードに移行
    }
    rotor_speeds.speeds[i] = sqrt(max(u[i], 0.) / rotor_props_[i].motor_constant);
  }
}

bool Controller::allMsgReceived()
{
  if (transformable_)
  {
    return js_subscribed_ && cmd_subscribed_;
  }
  else
  {
    return cmd_subscribed_;
  }
}

void Controller::bsCb(const multirotor_msgs::PoseVelStamped& msg)
{
  bs_ = msg.pose_vel;

  // トピックが揃っていたら，状態を観測するたびに一回だけ制御器を回す．
  if (allMsgReceived())
  {
    runOnce();
  }
}

void Controller::jsCb(const sensor_msgs::JointState& msg)
{
  if (msg.name.size() != msg.position.size())
  {
    dh_ros::rosError("The size of joint name and position is different.");
    js_subscribed_ = false;
    return;
  }

  for (const auto& jnt_name : required_joints_)
  {
    try
    {
      const auto msg_idx = dh_std::findIndex(msg.name, jnt_name);  // msg内でのインデックス
      const auto& jnt_pos = msg.position[msg_idx];
      const auto& kdl_idx = kdl_model_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_(kdl_idx) = jnt_pos;
    }
    catch (const exception& e)
    {
      dh_ros::rosError(e.what());
      js_subscribed_ = false;
      return;
    }
  }

  js_subscribed_ = true;
}

void Controller::commandCb(const multirotor_msgs::Command& msg)
{
  cmd_ = msg;
  cmd_subscribed_ = true;
}

void Controller::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  pos_controller_.reconfigure(cfg.natural_frequency, cfg.damping_ratio);
  rot_controller_.reconfigure(
    cfg.prediction_horizon, cfg.prediction_steps, cfg.rotation_decay, cfg.angular_velocity_decay,
    cfg.rotation_weight, cfg.angular_velocity_weight, cfg.thrust_force_weight,
    cfg.thrust_force_rate_weight);
}
