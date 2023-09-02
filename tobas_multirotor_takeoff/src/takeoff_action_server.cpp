#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

#define WAIT_FOR_STILLNESS "wait_for_stillness"

using namespace std;

namespace tobas_multirotor_takeoff
{
MultirotorTakeoffServer::MultirotorTakeoffServer(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh),
    as_(
      nh_,
      tobas::kTakeoffAction,
      boost::bind(&MultirotorTakeoffServer::executeCb, this, _1),
      false),
    wait_for_stillness_(WAIT_FOR_STILLNESS)
{
  getRosParams();

  wait_for_stillness_goal_.time_window = ros::Duration(kTimeWindow);
  wait_for_stillness_goal_.horizontal_position_variance_threshold = kHorPosVarThr;
  wait_for_stillness_goal_.vertical_position_variance_threshold = kVerPosVarThr;
  wait_for_stillness_goal_.heading_variance_threshold = kHeadingThreshold;
  wait_for_stillness_goal_.attitude_threshold = kAttitudeThreshold;
  wait_for_stillness_goal_.velocity_threshold = kVelThreshold;

  registerPublishers();
  registerSubscribers();

  as_.start();
}

void MultirotorTakeoffServer::getRosParams()
{
}

void MultirotorTakeoffServer::registerPublishers()
{
  pos_yaw_pub_ = nh_.advertise<tobas_msgs::PositionYaw>("command/position_yaw", 1);
}

void MultirotorTakeoffServer::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MultirotorTakeoffServer::eventCb, this);
}

void MultirotorTakeoffServer::fillResult()
{
  result_.last_command = pos_yaw_;
}

void MultirotorTakeoffServer::eventCb(const tobas_msgs::Event& event)
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

void MultirotorTakeoffServer::executeCb(const GoalType& goal)
{
  rosInfo("Action is called.");

  // 静止チェッカーを用意
  rosInfo("Waiting for '" << WAIT_FOR_STILLNESS << "' action server.");
  if (!wait_for_stillness_.waitForServer(ros::Duration(kWaitForExternalActionServer)))
  {
    rosInfo("Failed to connect to '" << WAIT_FOR_STILLNESS << "' action server.");
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_);
    return;
  }

  // 静止チェック
  rosInfo("Checking stillness.");
  wait_for_stillness_.sendGoalAndWait(wait_for_stillness_goal_);
  const auto wait_for_stillness_result = wait_for_stillness_.getResult();
  if (wait_for_stillness_result->error_code != tobas_msgs::WaitForStillnessResult::NO_ERROR)
  {
    rosInfo("'" << WAIT_FOR_STILLNESS << "' action failed.");
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_);
    return;
  }

  // 初期状態を取得
  rosInfo("Stillness is confirmed");
  const auto& init_bs = wait_for_stillness_result->base_state;

  // 位置制御コマンド
  // x, y, yawは初期値を維持する
  pos_yaw_.level = goal->level;
  pos_yaw_.pos.x(init_bs.pose.pos.x());
  pos_yaw_.pos.y(init_bs.pose.pos.y());
  pos_yaw_.yaw = init_bs.pose.euler.yaw;

  // 初期状態
  const auto start_alt = init_bs.pose.pos.z();
  const auto start_time = ros::Time::now();

  // 目標高度に到達するまで徐々に推力を上げていく
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // 時間とともに目標高度を上げていく
    const double t = (ros::Time::now() - start_time).toSec();
    const auto elevation = kInitElevation + kElevationSpeed * t;
    pos_yaw_.pos.z(start_alt + elevation);

    // コマンドを発行
    pos_yaw_pub_.publish(pos_yaw_);

    // 目標高度を指令したら終了
    if (elevation > kTargetElevation)
    {
      rosInfo("Target altitude is commanded.");
      fillResult();
      result_.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result_);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_takeoff
