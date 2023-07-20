#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_multirotor_takeoff
{
constexpr char MultirotorTakeoffServer::kActionName[];

MultirotorTakeoffServer::MultirotorTakeoffServer()
  : super(),
    bs_received_(false),
    as_(nh_, kActionName, boost::bind(&MultirotorTakeoffServer::executeCb, this, _1), false)
{
  getRosParams();

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
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorTakeoffServer::baseStateCb, this);
}

void MultirotorTakeoffServer::eventCb(const tobas_msgs::Event& event)
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

void MultirotorTakeoffServer::baseStateCb(const tobas_msgs::BaseState& bs)
{
  bs_ = bs;

  if (!bs_received_)
    bs_received_ = true;
}

void MultirotorTakeoffServer::executeCb(const GoalType& goal)
{
  rosInfo("Action is called.");

  if (!bs_received_)
  {
    result_.error_code = ResultType::NOT_READY;
    as_.setAborted(result_, "Base state is not received yet.");
    return;
  }

  // 初期状態
  const auto start_alt = bs_.pose.pos.z();
  const auto start_time = ros::Time::now();

  // 位置制御コマンド
  tobas_msgs::PositionYaw pos_yaw;
  pos_yaw.level = goal->level;
  pos_yaw.pos.z(kTargetAltitude);

  // 目標高度に到達するまで徐々に推力を上げていく
  ros::Rate rate(kUpdateRate);
  while (ros::ok() && bs_.pose.pos.z() - start_alt < kTargetAltitude)
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // 定常誤差に対応するため，一定時間を過ぎたら時間と共に少しずつ指令高度を上げる
    const double t = max((ros::Time::now() - start_time).toSec() - kElevationTimeThreshold, 0.);
    pos_yaw.pos.z(start_alt + kTargetAltitude + kVerticalSpeed * t);

    // Z以外は現在の値を指令し，垂直方向以外の力を書けないようにする．
    pos_yaw.pos.x(bs_.pose.pos.x());
    pos_yaw.pos.y(bs_.pose.pos.y());
    pos_yaw.yaw = bs_.pose.euler.yaw;

    // コマンドを発行
    pos_yaw_pub_.publish(pos_yaw);

    ros::spinOnce();
    rate.sleep();
  }

  // アクション成功
  rosInfo("Drone has reached the target altitude successfully");
  result_.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result_);
}
}  // namespace tobas_multirotor_takeoff
