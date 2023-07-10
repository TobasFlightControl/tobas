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

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void MultirotorLandServer::getRosParams()
{
}

void MultirotorLandServer::registerPublishers()
{
  cmd_pub_ = nh_.advertise<tobas_msgs::PositionYaw>("command/position_yaw", 1);
}

void MultirotorLandServer::registerSubscribers()
{
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorLandServer::baseStateCb, this);
}

void MultirotorLandServer::reset()
{
  bs_received_ = false;
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

  // 最新の状態を更新
  bs_ = bs;

  if (!bs_received_)
  {
    bs_received_ = true;
  }
}

void MultirotorLandServer::executeCb(const GoalType&)
{
  reset();
  is_action_running_ = true;

  // 現在の状態を取得
  ros::spinOnce();
  ros::Duration(0.1).sleep();  // ベース状態が更新されるよう少し待機
  if (!bs_received_)
  {
    is_action_running_ = false;
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to get base state.");
    return;
  }

  // 速度指令だと水平位置が制御できないため，位置指令にする
  // 現在の位置を初期目標位置に設定
  cmd_.level.level = tobas_msgs::CommandLevel::EMERGENCY;  // TODO: goalで指定する
  cmd_.pos = bs_.pose.pos;
  cmd_.yaw = bs_.pose.euler.yaw;

  const auto start_alt = bs_.pose.pos.z();
  ros::Time start_time = ros::Time::now();
  ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_, "Preempt requested.");
      return;
    }

    // コマンドを更新
    const auto t = (ros::Time::now() - start_time).toSec();
    cmd_.pos.z(start_alt - kVerticalSpeed * t);

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
