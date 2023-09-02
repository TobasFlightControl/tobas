#include <dh_eigen_tools/spline.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>

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
  ros::NodeHandle nh,
  ros::NodeHandle pnh)
  : super(nh, pnh),
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
  event_sub_ = nh_.subscribe("event", 1, &FollowPositionYawTrajectoryServer::eventCb, this);
}

bool FollowPositionYawTrajectoryServer::isValidGoal(const GoalType& goal)
{
  const auto& waypoints = goal->waypoints;

  // 次数は1以上3以下
  if (goal->degree < 1 || 3 < goal->degree)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    rosError("Spline degree is " << goal->degree << ". It must be in range of [1, 3].");
    as_.setAborted(result_);
    return false;
  }

  // 点が2つ以上含まれているか
  if (waypoints.size() < 2)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    rosError("Waypoints must include more than 1 points.");
    as_.setAborted(result_);
    return false;
  }

  // 点の数が多すぎるとダメ
  if (waypoints.size() > kMaxNrOfTrajPoint)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    rosError("Too many number of trajectory points.");
    as_.setAborted(result_);
    return false;
  }

  // 最初のtime_from_startは0でなければならない
  if (waypoints[0].time_from_start.toSec() != 0.)
  {
    result_.error_code = ResultType::INVALID_GOAL;
    rosError("The duration of the first trajectory point must be 0.");
    as_.setAborted(result_);
    return false;
  }

  // time_from_startは単純増加でなければならない
  for (uint32_t i = 0; i < waypoints.size() - 1; ++i)
  {
    if (waypoints[i].time_from_start >= waypoints[i + 1].time_from_start)
    {
      result_.error_code = ResultType::INVALID_GOAL;
      rosError("The durations must be strictly increasing.");
      as_.setAborted(result_);
      return false;
    }
  }

  return true;
}

void FollowPositionYawTrajectoryServer::eventCb(const tobas_msgs::EventConstPtr& event)
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

void FollowPositionYawTrajectoryServer::executeCb(const GoalType& goal)
{
  if (!isValidGoal(goal))
  {
    return;
  }

  // Prepare time and position vectors for spline fitting
  const auto& waypoints = goal->waypoints;
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
      rosWarn("Preempt requested.");
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
