#include <tobas_eigen_tools/spline.hpp>
#include <tobas_ros_tools/rate.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_trajectory_commander/position_yaw.hpp"
#include "../include/tobas_trajectory_commander/common.hpp"

#define COMMAND_DIMENSION 4  // x, y, z, yaw

using namespace std;
using namespace KDL;
using namespace Eigen;

using SplineType = Spline<double, Dynamic>;

namespace tobas_trajectory_commander
{
constexpr char FollowPositionYawTrajectoryServer::kActionName[];

FollowPositionYawTrajectoryServer::FollowPositionYawTrajectoryServer(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), as_(nh_, kActionName, boost::bind(&self::executeCb, this, _1), false)
{
  cmd_pub_ = nh_.advertise<CommandType>(tobas::kPositionYawCmdTopic, 1);
  as_.start();
}

bool FollowPositionYawTrajectoryServer::isGoalValid(const GoalType& goal)
{
  const auto& waypoints = goal.waypoints;

  // 次数は1以上3以下
  if (goal.degree < 1 || 3 < goal.degree)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    TOBAS_ERROR("Spline degree is ", goal.degree, ". It must be in range of [1, 3].");
    as_.setAborted(result_);
    return false;
  }

  // 点が2つ以上含まれているか
  if (waypoints.size() < 2)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    TOBAS_ERROR("Waypoints must include more than 1 points.");
    as_.setAborted(result_);
    return false;
  }

  // 点の数が多すぎるとダメ
  if (waypoints.size() > kMaxNrOfTrajPoint)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    TOBAS_ERROR("Too many number of trajectory points.");
    as_.setAborted(result_);
    return false;
  }

  // 最初のtime_from_startは0でなければならない
  if (waypoints[0].time_from_start.toSec() != 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    TOBAS_ERROR("The duration of the first trajectory point must be 0.");
    as_.setAborted(result_);
    return false;
  }

  // time_from_startは単純増加でなければならない
  for (size_t i = 0; i < waypoints.size() - 1; ++i)
  {
    if (waypoints[i].time_from_start >= waypoints[i + 1].time_from_start)
    {
      result_.error_code = ResultType::INVALID_GOAL;
      TOBAS_ERROR("The durations must be strictly increasing.");
      as_.setAborted(result_);
      return false;
    }
  }

  return true;
}

void FollowPositionYawTrajectoryServer::executeCb(const GoalType::ConstPtr& goal)
{
  if (!isGoalValid(*goal))
    return;

  // Prepare time and position vectors for spline fitting
  const auto& waypoints = goal->waypoints;
  VectorXd times(waypoints.size());
  vector<VectorXd> positions(COMMAND_DIMENSION, VectorXd::Zero(waypoints.size()));
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    times(i) = waypoints[i].time_from_start.toSec();
    positions[0](i) = waypoints[i].pos.x();
    positions[1](i) = waypoints[i].pos.y();
    positions[2](i) = waypoints[i].pos.z();
    positions[3](i) = waypoints[i].yaw;
  }

  // Compute spline fit
  vector<eigen_tools::SplineFunction> splines;
  for (size_t i = 0; i < COMMAND_DIMENSION; ++i)
    splines.emplace_back(times, positions[i], goal->degree);

  // Execute trajectory
  for (double t = times[0]; t <= times[times.size() - 1]; t += kControlnterval)
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      TOBAS_WARN("Preempt requested.");
      as_.setPreempted(result_);
      return;
    }

    // Create command message
    const auto cmd = boost::make_shared<CommandType>();
    cmd->level = goal->level;
    cmd->pos.x(splines[0](t));
    cmd->pos.y(splines[1](t));
    cmd->pos.z(splines[2](t));
    cmd->yaw = splines[3](t);

    // Publish command message
    cmd_pub_.publish(cmd);

    // Sleep for control rate
    ros::Duration(kControlnterval).sleep();
  }

  result_.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result_);
}
}  // namespace tobas_trajectory_commander
