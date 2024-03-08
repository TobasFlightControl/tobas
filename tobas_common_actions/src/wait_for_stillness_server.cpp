#include <tobas_std_tools/math.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_common_actions/wait_for_stillness_server.hpp"
#include "../include/tobas_common_actions/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_common_actions
{
WaitForStillnessServer::WaitForStillnessServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    is_action_running_(false),
    as_(nh_, kActionName, boost::bind(&self::executeCb, this, _1), false)
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
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
}

void WaitForStillnessServer::reset()
{
  is_history_filled_ = false;
  odom_history_.clear();
  t_last_valid_velocity_ = ros::Time::now();
}

bool WaitForStillnessServer::isGoalValid(const GoalType& goal)
{
  if (goal.time_window <= ros::Duration(0))
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'time_window' must be positive.");
    return false;
  }

  if (goal.horizontal_position_variance_threshold <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'horizontal_position_variance_threshold' must be positive.");
    return false;
  }

  if (goal.vertical_position_variance_threshold <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'vertical_position_variance_threshold' must be positive.");
    return false;
  }

  if (goal.heading_variance_threshold <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'heading_variance_threshold' must be positive.");
    return false;
  }

  if (goal.attitude_threshold <= 0)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "'attitude_threshold' must be positive.");
    return false;
  }

  if (goal.velocity_threshold <= 0)
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
  const auto& odom_front = odom_history_.front();
  const auto& odom_back = odom_history_.back();

  bool res = true;

  const auto dp = odom_back.frame.p - odom_front.frame.p;
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

  const KDL::Euler euler_front(odom_front.frame.M);
  const KDL::Euler euler_back(odom_back.frame.M);
  const auto yaw_diff = euler_back.yaw - euler_front.yaw;
  if (abs(yaw_diff) > goal_->heading_variance_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The variance of yaw angle in " << goal_->time_window
                                      << " seconds is over threshold: " << abs(yaw_diff) << " > "
                                      << goal_->heading_variance_threshold);
    res = false;
  }

  if (odom_back.header.stamp - t_last_valid_attitude_ < goal_->time_window)
  {
    res = false;
  }

  if (odom_back.header.stamp - t_last_valid_velocity_ < goal_->time_window)
  {
    res = false;
  }

  return res;
}

void WaitForStillnessServer::fillResult()
{
  result_.odom = odom_history_.back();
}

void WaitForStillnessServer::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (!is_action_running_)
  {
    return;
  }

  // 現在の時刻と高度を履歴に追加
  odom_history_.push_back(*odom);

  // 古い履歴を削除
  while (odom->header.stamp - odom_history_.front().header.stamp > goal_->time_window)
  {
    odom_history_.pop_front();
    if (!is_history_filled_)
    {
      is_history_filled_ = true;
    }
  }

  // 最後に姿勢角が閾値を下回った時刻を更新
  const KDL::Euler euler(odom->frame.M);
  if (abs(euler.roll) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Roll angle is over threshold: |" << euler.roll << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = odom->header.stamp;
  }
  if (abs(euler.pitch) > goal_->attitude_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Pitch angle is over threshold: |" << euler.pitch << "| > " << goal_->attitude_threshold);
    t_last_valid_attitude_ = odom->header.stamp;
  }

  // 最後に速度が閾値を下回った時刻を更新
  if (odom->twist.vel.norm() > goal_->velocity_threshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "The norm of velocity is over threshold: " << odom->twist.vel.norm() << " > "
                                                 << goal_->velocity_threshold);
    t_last_valid_velocity_ = odom->header.stamp;
  }
}

void WaitForStillnessServer::executeCb(const GoalType::ConstPtr& goal)
{
  rosInfo(name_, "Action is called.");

  if (!isGoalValid(*goal))
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
