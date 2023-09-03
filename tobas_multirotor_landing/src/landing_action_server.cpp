#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

using namespace std;

namespace tobas_multirotor_landing
{
MultirotorLandServer::MultirotorLandServer(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name),
    is_action_running_(false),
    as_(nh_, tobas::kLandingAction, boost::bind(&MultirotorLandServer::executeCb, this, _1), false)
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
  event_sub_ = nh_.subscribe("event", 1, &MultirotorLandServer::eventCb, this);
  pt_sub_ = nh_.subscribe("pose_twist", 1, &MultirotorLandServer::poseTwistCb, this);
}

void MultirotorLandServer::reset()
{
  pt_received_ = false;
  is_history_filled_ = false;
  alt_history_.clear();
}

void MultirotorLandServer::eventCb(const tobas_msgs::EventConstPtr& event)
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

void MultirotorLandServer::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!is_action_running_)
  {
    return;
  }

  // 現在の時刻と高度を履歴に追加
  const auto& cur_time = pt->header.stamp;
  const auto& altitude = pt->pose.pos.z();
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
  pt_ = pt;

  if (!pt_received_)
  {
    pt_received_ = true;
  }
}

void MultirotorLandServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  reset();
  is_action_running_ = true;

  // 現在の状態を取得
  ros::spinOnce();
  ros::Duration(0.1).sleep();  // ベース状態が更新されるよう少し待機
  if (!pt_received_)
  {
    is_action_running_ = false;
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Failed to get pose & twist.");
    return;
  }

  // 速度指令だと水平位置が制御できないため，位置指令にする
  // 現在の位置を初期目標位置に設定
  cmd_.level = goal->level;
  cmd_.pos = pt_->pose.pos;
  cmd_.yaw = pt_->pose.euler.yaw;

  const auto start_alt = pt_->pose.pos.z();
  const auto start_time = ros::Time::now();
  ros::Rate rate(kUpdateRate);

  while (nh_.ok())
  {
    if (as_.isPreemptRequested())
    {
      is_action_running_ = false;
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // コマンドを更新
    const auto t = (ros::Time::now() - start_time).toSec();
    cmd_.pos.z(start_alt - kVerticalSpeed * t);

    // コマンドを発行
    const auto cmd_ptr = boost::make_shared<tobas_msgs::PositionYaw>(cmd_);
    cmd_pub_.publish(cmd_ptr);

    if (is_history_filled_)
    {
      // 一定時間幅の高度が一定の範囲内なら終了
      const auto alt_range = abs(alt_history_.front().second - alt_history_.back().second);
      if (alt_range < kStableAltitudeRange)
      {
        rosInfo(
          name_, "Altitude has remained stable for "
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
}  // namespace tobas_multirotor_landing
