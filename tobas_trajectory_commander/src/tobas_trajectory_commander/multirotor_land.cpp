#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_trajectory_commander/multirotor_land.hpp"

using namespace std;

namespace tobas_trajectory_commander
{
// 外部リンケージをもつポインタ型はODR違反が起きる可能性があるため，ソースでconstexpr変数の再定義を行う必要がある．
// 再定義ではstaticを省略する．
constexpr char MultirotorLandServer::kActionName[];

MultirotorLandServer::MultirotorLandServer()
  : super(),
    is_action_running_(false),
    as_(nh_, kActionName, boost::bind(&MultirotorLandServer::executeCb, this, _1), false)
{
  getRosParams();

  cmd_.frame_id.frame_id = tobas_msgs::FrameId::GLOBAL;
  cmd_.level.level = tobas_msgs::CommandLevel::NORMAL;  // TODO: rosparamで切り替える
  cmd_.vel.z(-kVerticalSpeed);  // TODO: その場で着陸するだけでなく，場所も考慮して軌道を作る

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void MultirotorLandServer::getRosParams()
{
}

void MultirotorLandServer::registerPublishers()
{
  cmd_pub_ = nh_.advertise<tobas_msgs::VelocityYaw>("command/velocity_yaw", 1);
}

void MultirotorLandServer::registerSubscribers()
{
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorLandServer::baseStateCb, this);
}

void MultirotorLandServer::reset()
{
  is_history_filled_ = false;
  alt_history_.clear();
}

void MultirotorLandServer::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!is_action_running_)
  {
    return;
  }

  // 現在の時刻と高度を履歴に追加
  const auto& cur_time = bs.header.stamp;
  const auto& altitude = bs.pose.pos.z();
  alt_history_.emplace_back(cur_time, altitude);

  // 古い履歴を削除
  // 要素にアクセスできるように最低1つは残しておく
  while (alt_history_.size() > 2 && (cur_time - alt_history_.front().first).toSec() > kTimeWindow)
  {
    alt_history_.pop_front();
    if (!is_history_filled_)
    {
      is_history_filled_ = true;
    }
  }
}

void MultirotorLandServer::executeCb(const GoalType&)
{
  reset();
  is_action_running_ = true;

  ros::Rate rate(kUpdateRate);
  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // コマンドを発行
    cmd_pub_.publish(cmd_);

    if (is_history_filled_)
    {
      // 一定時間幅の高度が一定の範囲内なら終了
      const auto alt_range = abs(alt_history_.front().second - alt_history_.back().second);
      if (alt_range < kStableAltitudeRange)
      {
        rosInfo(
          "Altitude has remained stable for "
          << kTimeWindow << " seconds. Probably the drone has landed successfully.");
        is_action_running_ = false;
        result_.error_code = ResultType::NO_ERROR;
        as_.setSucceeded(result_);
        return;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  is_action_running_ = false;
}
}  // namespace tobas_trajectory_commander
