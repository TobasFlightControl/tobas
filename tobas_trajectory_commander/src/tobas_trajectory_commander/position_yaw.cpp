#include <dh_eigen_tools/spline.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_trajectory_commander/position_yaw.hpp"
#include "../../include/tobas_trajectory_commander/common.hpp"

#define COMMAND_DIMENSION 4  // x, y, z, yaw

using namespace std;
using namespace KDL;
using namespace Eigen;

using SplineType = Spline<double, Dynamic>;

namespace tobas_trajectory_commander
{
constexpr char FollowPositionYawTrajectoryServer::kActionName[];

FollowPositionYawTrajectoryServer::FollowPositionYawTrajectoryServer()
  : super(),
    as_(
      nh_,
      kActionName,
      boost::bind(&FollowPositionYawTrajectoryServer::executeCb, this, _1),
      false)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void FollowPositionYawTrajectoryServer::getRosParams()
{
}

void FollowPositionYawTrajectoryServer::registerPublishers()
{
  cmd_pub_ = nh_.advertise<CommandType>("command/position_yaw", 1);
}

void FollowPositionYawTrajectoryServer::registerSubscribers()
{
}

bool FollowPositionYawTrajectoryServer::isValidGoal(const GoalType& goal)
{
  const auto& waypoints = goal->waypoints;

  // 次数は1以上3以下
  if (goal->degree < 1 || 3 < goal->degree)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Spline degree must be in range of [1, 3].");
    return false;
  }

  // 点が2つ以上含まれているか
  if (waypoints.size() < 2)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Waypoints must include more than 1 points.");
    return false;
  }

  // 点の数が多すぎるとダメ
  if (waypoints.size() > kMaxNrOfTrajPoint)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "Too many number of trajectory points.");
    return false;
  }

  // 最初のtime_from_startは0でなければならない
  if (waypoints[0].time_from_start.toSec() != 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    as_.setAborted(result_, "The duration of the first trajectory point must be 0.");
    return false;
  }

  // time_from_startは単純増加でなければならない
  for (uint32_t i = 0; i < waypoints.size() - 1; ++i)
  {
    if (waypoints[i].time_from_start >= waypoints[i + 1].time_from_start)
    {
      result_.error_code = ResultType::INVALID_GOAL;
      as_.setAborted(result_, "The durations must be strictly increasing.");
      return false;
    }
  }

  return true;
}

void FollowPositionYawTrajectoryServer::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void FollowPositionYawTrajectoryServer::executeCb(const GoalType& goal)
{
  if (!isValidGoal(goal))
  {
    return;
  }

  CommandType cmd;
  cmd.level = goal->level;

  const auto& waypoints = goal->waypoints;

  // Prepare time and position vectors for spline fitting
  VectorXd times(waypoints.size());
  vector<VectorXd> positions(COMMAND_DIMENSION, VectorXd::Zero(waypoints.size()));
  for (uint32_t i = 0; i < waypoints.size(); ++i)
  {
    times(i) = waypoints[i].time_from_start.toSec();
    positions[0](i) = waypoints[i].pos.x();
    positions[1](i) = waypoints[i].pos.y();
    positions[2](i) = waypoints[i].pos.z();
    positions[3](i) = waypoints[i].yaw;
  }

  // Compute spline fit
  vector<eigen_tools::SplineFunction> splines;
  for (uint32_t i = 0; i < COMMAND_DIMENSION; ++i)
  {
    splines.emplace_back(times, positions[i], goal->degree);
  }

  // Execute trajectory
  for (double t = times[0]; t <= times[times.size() - 1]; t += kControlnterval)
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    cmd.pos.x(splines[0](t));
    cmd.pos.y(splines[1](t));
    cmd.pos.z(splines[2](t));
    cmd.yaw = splines[3](t);

    cmd_pub_.publish(cmd);

    ros::Duration(kControlnterval).sleep();  // Sleep for control rate
  }

  as_.setSucceeded();
}
}  // namespace tobas_trajectory_commander
