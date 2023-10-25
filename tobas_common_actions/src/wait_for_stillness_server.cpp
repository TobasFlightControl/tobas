#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_common_actions
{
WaitForStillnessServer::WaitForStillnessServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name),
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
  event_sub_ =
    nh_.subscribe(tobas::kEventTopic, 1, &WaitForStillnessServer::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(
    tobas::kPoseTwistTopic, 1, &WaitForStillnessServer::poseTwistCb, this, tcpNoDelay());
}

void WaitForStillnessServer::reset()
{
  is_history_filled_ = false;
  pt_history_.clear();
  t_last_valid_velocity_ = ros::Time::now();
}

bool WaitForStillnessServer::isGoalValid(const GoalType& goal)
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
  const auto& pt_front = pt_history_.front();
  const auto& pt_back = pt_history_.back();

  bool res = true;

  const auto dp = pt_back.pose.pos - pt_front.pose.pos;
  const auto hor_pos_var_norm = sqrt(sqr(dp.x()) + sqr(dp.y()));
  if (hor_pos_var_norm > goal_->horizontal_position_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The variance of horizontal position in "
        << goal_->time_window << " seconds is over threshold: " << hor_pos_var_norm << " > "
        << goal_->horizontal_position_variance_threshold);
    res = false;
  }
  if (abs(dp.z()) > goal_->vertical_position_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The variance of altitude in " << goal_->time_window
                                     << " seconds is over threshold: " << abs(dp.z()) << " > "
                                     << goal_->vertical_position_variance_threshold);
    res = false;
  }

  const auto yaw_diff = pt_back.pose.euler.yaw - pt_front.pose.euler.yaw;
  if (abs(yaw_diff) > goal_->heading_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The variance of yaw angle in " << goal_->time_window
                                      << " seconds is over threshold: " << abs(yaw_diff) << " > "
                                      << goal_->heading_variance_threshold);
    res = false;
  }

  if (pt_back.header.stamp - t_last_valid_attitude_ < goal_->time_window)
  {
    res = false;
  }

  if (pt_back.header.stamp - t_last_valid_velocity_ < goal_->time_window)
  {
    res = false;
  }

  return res;
}

void WaitForStillnessServer::fillResult()
{
  result_.pose_twist = pt_history_.back();
}

void WaitForStillnessServer::eventCb(const tobas_msgs::EventConstPtr& event)
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

void WaitForStillnessServer::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  if (!is_action_running_)
  {
    return;
  }

  // 現在の時刻と高度を履歴に追加
  pt_history_.push_back(*pt);

  // 古い履歴を削除
  while (pt->header.stamp - pt_history_.front().header.stamp > goal_->time_window)
  {
    pt_history_.pop_front();
    if (!is_history_filled_)
    {
      is_history_filled_ = true;
    }
  }

  // 最後に姿勢角が閾値を下回った時刻を更新
  const auto& roll = pt->pose.euler.roll;
  const auto& pitch = pt->pose.euler.pitch;
  if (abs(roll) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Roll angle is over threshold: |" << roll << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = pt->header.stamp;
  }
  if (abs(pitch) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Pitch angle is over threshold: |" << pitch << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = pt->header.stamp;
  }

  // 最後に速度が閾値を下回った時刻を更新
  if (pt->twist.vel.norm() > goal_->velocity_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The norm of velocity is over threshold: " << pt->twist.vel.norm() << " > "
                                                 << goal_->velocity_threshold);
    t_last_valid_velocity_ = pt->header.stamp;
  }
}

void WaitForStillnessServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  if (!isGoalValid(goal))
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
      rosInfo(name_, "All conditions are met.");
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
