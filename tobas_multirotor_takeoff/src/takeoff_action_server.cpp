#include <dh_ros_tools/console_message.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/VelocityYaw.h>

#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

using namespace std;

namespace tobas_multirotor_takeoff
{
MultirotorTakeoffServer::MultirotorTakeoffServer(
  ros::NodeHandle nh,
  ros::NodeHandle pnh,
  string name)
  : super(nh, pnh, name),
    as_(
      nh_,
      tobas::kTakeoffAction,
      boost::bind(&MultirotorTakeoffServer::executeCb, this, _1),
      false)
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
  cmd_pub_ = nh_.advertise<tobas_msgs::VelocityYaw>(tobas::kVelocityYawCmdTopic, 1);
}

void MultirotorTakeoffServer::registerSubscribers()
{
  event_sub_ =
    nh_.subscribe(tobas::kEventTopic, 1, &MultirotorTakeoffServer::eventCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(
    tobas::kPoseTwistTopic, 1, &MultirotorTakeoffServer::poseTwistCb, this, tcpNoDelay());
}

void MultirotorTakeoffServer::eventCb(const tobas_msgs::EventConstPtr& event)
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

void MultirotorTakeoffServer::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  pt_ = pt;
}

void MultirotorTakeoffServer::executeCb(const GoalType& goal)
{
  rosInfo(name_, "Action is called.");

  ResultType result;

  if (pt_ == nullptr)
  {
    result.error_code = ResultType::NOT_READY;
    as_.setAborted(result, "Pose & Twist is not received yet.");
    return;
  }

  // 離陸コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::VelocityYaw>();
  cmd->level = goal->level;
  cmd->frame_id.data = tobas_msgs::FrameId::GLOBAL;
  cmd->vel.z(kElevationSpeed);
  cmd->yaw = pt_->pose.euler.yaw;  // yawはアクションが呼ばれたときの値を維持する

  // 離陸コマンドを発行
  cmd_pub_.publish(cmd);

  // 初期状態
  const auto start_alt = pt_->pose.pos.z();

  // 高度チェック
  ros::Rate rate(kUpdateRate);
  while (nh_.ok())
  {
    if (as_.isPreemptRequested())
    {
      result.error_code = ResultType::PREEMPTED;
      as_.setPreempted(result);
      return;
    }

    // 目標高度に到達したら停止して終了
    if (pt_->pose.pos.z() - start_alt > kTargetElevation)
    {
      rosInfo(name_, "Target altitude is reached.");

      cmd->vel.z(0.);
      cmd_pub_.publish(cmd);

      result.error_code = ResultType::NO_ERROR;
      as_.setSucceeded(result);
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace tobas_multirotor_takeoff
