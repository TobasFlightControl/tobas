#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_common_actions
{
WaitForStillnessServer::WaitForStillnessServer(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh),
    is_action_running_(false),
    as_(nh_, kActionName, boost::bind(&WaitForStillnessServer::executeCb, this, _1), false)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();

  as_.start();
}

void WaitForStillnessServer::getRosParams()
{
}

void WaitForStillnessServer::registerPublishers()
{
}

void WaitForStillnessServer::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &WaitForStillnessServer::eventCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &WaitForStillnessServer::baseStateCb, this);
}

void WaitForStillnessServer::reset()
{
  is_history_filled_ = false;
  bs_history_.clear();
  t_last_valid_velocity_ = ros::Time::now();
}

bool WaitForStillnessServer::isValidGoal(const GoalType& goal)
{
  if (goal->time_window <= ros::Duration(0.))
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'time_window' must be positive.");
    return false;
  }

  if (goal->horizontal_position_variance_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'horizontal_position_variance_threshold' must be positive.");
    return false;
  }

  if (goal->vertical_position_variance_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'vertical_position_variance_threshold' must be positive.");
    return false;
  }

  if (goal->heading_variance_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'heading_variance_threshold' must be positive.");
    return false;
  }

  if (goal->attitude_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'attitude_threshold' must be positive.");
    return false;
  }

  if (goal->velocity_threshold <= 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'velocity_threshold' must be positive.");
    return false;
  }

  return true;
}

bool WaitForStillnessServer::isConditionsMet()
{
  if (!is_history_filled_)
  {
    return false;
  }

  // FIXME: 本当は最初と最後の差ではなく，範囲つまり最大値と最小値の差で評価すべき
  const auto& bs_front = bs_history_.front();
  const auto& bs_back = bs_history_.back();

  bool res = true;

  const auto dp = bs_back.pose.pos - bs_front.pose.pos;
  const auto hor_pos_var_norm = sqrt(sqr(dp.x()) + sqr(dp.y()));
  if (hor_pos_var_norm > goal_->horizontal_position_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, "The variance of horizontal position in "
                     << goal_->time_window << " seconds is over threshold: " << hor_pos_var_norm
                     << " > " << goal_->horizontal_position_variance_threshold);
    res = false;
  }
  if (abs(dp.z()) > goal_->vertical_position_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, "The variance of altitude in "
                     << goal_->time_window << " seconds is over threshold: " << abs(dp.z()) << " > "
                     << goal_->vertical_position_variance_threshold);
    res = false;
  }

  const auto yaw_diff = bs_back.pose.euler.yaw - bs_front.pose.euler.yaw;
  if (abs(yaw_diff) > goal_->heading_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, "The variance of yaw angle in "
                     << goal_->time_window << " seconds is over threshold: " << abs(yaw_diff)
                     << " > " << goal_->heading_variance_threshold);
    res = false;
  }

  if (bs_back.header.stamp - t_last_valid_attitude_ < goal_->time_window)
  {
    res = false;
  }

  if (bs_back.header.stamp - t_last_valid_velocity_ < goal_->time_window)
  {
    res = false;
  }

  return res;
}

void WaitForStillnessServer::fillResult()
{
  result_.base_state = bs_history_.back();
}

void WaitForStillnessServer::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void WaitForStillnessServer::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!is_action_running_)
  {
    return;
  }

  // 現在の時刻と高度を履歴に追加
  bs_history_.push_back(bs);

  // 古い履歴を削除
  while (bs.header.stamp - bs_history_.front().header.stamp > goal_->time_window)
  {
    bs_history_.pop_front();
    if (!is_history_filled_)
    {
      is_history_filled_ = true;
    }
  }

  // 最後に姿勢角が閾値を下回った時刻を更新
  const auto& roll = bs.pose.euler.roll;
  const auto& pitch = bs.pose.euler.pitch;
  if (abs(roll) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod,
      "Roll angle is over threshold: |" << roll << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = bs.header.stamp;
  }
  if (abs(pitch) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod,
      "Pitch angle is over threshold: |" << pitch << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = bs.header.stamp;
  }

  // 最後に速度が閾値を下回った時刻を更新
  if (bs.twist.vel.Norm() > goal_->velocity_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, "The norm of velocity is over threshold: " << bs.twist.vel.Norm() << " > "
                                                              << goal_->velocity_threshold);
    t_last_valid_velocity_ = bs.header.stamp;
  }
}

void WaitForStillnessServer::executeCb(const GoalType& goal)
{
  rosInfo("Action is called.");

  if (!isValidGoal(goal))
  {
    return;
  }

  reset();
  is_action_running_ = true;
  goal_ = goal;

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

    if (isConditionsMet())
    {
      rosInfo("All conditions are met.");
      is_action_running_ = false;
      fillResult();
      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_common_actions
