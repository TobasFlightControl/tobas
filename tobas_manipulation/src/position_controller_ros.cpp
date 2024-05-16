#include <tobas_std_tools/zip.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_manipulation/position_controller_ros.hpp"
#include "../include/tobas_manipulation/common.hpp"

using namespace std;
using namespace tobas_kdl;

namespace tobas_manipulation
{
PositionControllerRos::PositionControllerRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  drone_.loadFromParam(nh_);

  // 位置指令タイプの関節のホームポジションを取得
  for (const auto& [jnt_name, jnt_cfg] : drone_.jointConfigMap())
  {
    if (jnt_cfg.cmd_type != tobas::JointConfig::POSITION)
      continue;
    home_js_.name.push_back(jnt_name);
    home_js_.position.push_back(jnt_cfg.home_pos);
    home_js_.velocity.push_back(0.);
    home_js_.effort.push_back(0.);
  }

  // ホームポジションを初期目標状態に設定
  if (home_js_.name.size() > 0)
    tar_js_ = boost::make_shared<sensor_msgs::JointState>(home_js_);

  positions_pub_ = nh_.advertise<tobas_msgs::JointCommandArray>(tobas::kJointPositionsCmdTopic, 1);

  cur_js_sub_ =
    nh_.subscribe(tobas::kJointStatesTopic, 1, &self::currentJointStateCb, this, tcpNoDelay());
  tar_js_sub_ =
    nh_.subscribe(tobas::kPosCtrlJSTopic, 1, &self::targetJointStateCb, this, tcpNoDelay());
  tar_ls_sub_ =
    nh_.subscribe(tobas::kPosCtrlLSTopic, 1, &self::targetLinkStateCb, this, tcpNoDelay());
}

int PositionControllerRos::jointSpaceControl(tobas_msgs::JointCommandArray& positions_msg)
{
  // 位置コマンドをそのまま流すだけ
  for (const auto& [name, pos] : tobas_std::zip(tar_js_->name, tar_js_->position))
    positions_msg.commands.emplace_back(name, pos);

  return 0;
}

int PositionControllerRos::taskSpaceControl(tobas_msgs::JointCommandArray&)
{
  TOBAS_ERROR("Task space control of joint position controller is not implemented.");  // TODO

  return 0;
}

void PositionControllerRos::currentJointStateCb(const sensor_msgs::JointStateConstPtr&)
{
  if (tar_js_ == nullptr && tar_ls_ == nullptr)
    return;

  const auto time_after_last_cmd = (ros::Time::now() - t_last_cmd_).toSec();
  if (is_commanded_ && time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    tar_js_ = boost::make_shared<sensor_msgs::JointState>(home_js_);
    tar_ls_ = nullptr;
    is_commanded_ = false;
    TOBAS_WARN(
      "The target joint states are automatically reset because ", tobas::kAutoResetTimeThreshold,
      " seconds have elapsed since the last command.");
  }

  // Create joint velocities command
  const auto positions_msg = boost::make_shared<tobas_msgs::JointCommandArray>();

  // Joint space control or Task space control
  if (tar_js_ != nullptr)
  {
    if (jointSpaceControl(*positions_msg) < 0)
      return;
  }
  else if (tar_ls_ != nullptr)
  {
    if (taskSpaceControl(*positions_msg) < 0)
      return;
  }
  else
  {
    TOBAS_ERROR("Both target joint state and target link state are NULL.");
    return;
  }

  // Publish joint velocities command
  positions_pub_.publish(positions_msg);
}

void PositionControllerRos::targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js)
{
  tar_js_ = tar_js;
  tar_ls_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}

void PositionControllerRos::targetLinkStateCb(const tobas_msgs::LinkStateArrayConstPtr& tar_ls)
{
  tar_ls_ = tar_ls;
  tar_js_ = nullptr;

  t_last_cmd_ = ros::Time::now();
  is_commanded_ = true;
}
}  // namespace tobas_manipulation
