#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_multirotor_takeoff
{
// 外部リンケージをもつポインタ型はODR違反が起きる可能性があるため，ソースでconstexpr変数の再定義を行う必要がある．
// 再定義ではstaticを省略する．
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
  rpy_thrust_pub_ = nh_.advertise<tobas_msgs::RollPitchYawThrust>("command/rpy_thrust", 1);
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

  // 姿勢制御コマンド
  tobas_msgs::RollPitchYawThrust rpy_thrust;
  rpy_thrust.level = goal->level;

  // 目標高度に到達するまで徐々に推力を上げていく
  ros::Rate rate(kUpdateRate);
  while (ros::ok())
  {
    if (as_.isPreemptRequested())
    {
      result_.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result_);
      return;
    }

    // コマンドを更新
    const auto t = (ros::Time::now() - start_time).toSec();
    rpy_thrust.thrust = kThrustRate * t;
    rpy_thrust_pub_.publish(rpy_thrust);

    // 目標高度に到達すればループを抜ける
    const auto cur_alt = bs_.pose.pos.z();
    if (cur_alt - start_alt > kTakeoffAltitudeThreshold)
    {
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  // 目標位置でホバリング
  tobas_msgs::PositionYaw pos_yaw;
  pos_yaw.level = goal->level;
  pos_yaw.pos.z(kHoverAltitude);
  pos_yaw_pub_.publish(pos_yaw);

  // アクション成功
  result_.error_code = ResultType::NO_ERROR;
  as_.setSucceeded(result_);
}
}  // namespace tobas_multirotor_takeoff
