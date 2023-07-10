#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_state_checker/multirotor_state_checker.hpp"

using namespace std;

namespace tobas_state_checker
{
constexpr char MultirotorStateChecker::kLandActionName[];

MultirotorStateChecker::MultirotorStateChecker()
  : super(), bs_received_(false), cmd_received_(false), ac_(kLandActionName)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  // Wait for landing action server to start
  if (!ac_.waitForServer(ros::Duration(kWaitForActionServer)))
  {
    rosError(
      "'" << kLandActionName << "' action server failed to start within " << kWaitForActionServer
          << " seconds. Please check the server status.");
    // TODO: モータを無理矢理止める処理
  }
}

void MultirotorStateChecker::run()
{
  while (ros::ok())
  {
    // GCSとの通信が切れるなどして一定時間コマンドを受け取っていない場合は着陸指令を出す
    if (cmd_received_ && (ros::Time::now() - t_last_cmd_).toSec() > kCommandTimeoutThreshold)
    {
      cmd_received_ = false;

      rosInfo(
        "Issuing a landing command as no commands have been received for "
        << kCommandTimeoutThreshold << " seconds.");
      ac_.sendGoal(tobas_trajectory_commander::LandGoal());
      ac_.waitForResult();

      const auto result = ac_.getResult();
      const auto state = ac_.getState();
      if (result->error_code == tobas_trajectory_commander::LandResult::NO_ERROR)
      {
        rosInfo(state.getText());
        rosInfo("Landing action finished successfully.");
      }
      else
      {
        rosError(state.getText());
        rosFatal("Landing action failed.");
      }

      // 全てのシステムを停止する
      rosInfo("Shutting down the system.");
      tobas_msgs::Event event;
      event.data = tobas_msgs::Event::SHUTDOWN;
      event_pub_.publish(event);
    }

    ros::spinOnce();
    ros::Duration(kSleepTime).sleep();
  }
}

void MultirotorStateChecker::getRosParams()
{
}

void MultirotorStateChecker::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
}

void MultirotorStateChecker::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MultirotorStateChecker::eventCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorStateChecker::baseStateCb, this);
  cmd_sub_ = nh_.subscribe("command/velocity_yaw", 1, &MultirotorStateChecker::commandCb, this);
}

void MultirotorStateChecker::eventCb(const tobas_msgs::Event& event)
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

void MultirotorStateChecker::baseStateCb(const tobas_msgs::BaseState&)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }
}

void MultirotorStateChecker::commandCb(const tobas_msgs::VelocityYaw& cmd)
{
  // 緊急コマンドはスキップ
  if (cmd.level.data == tobas_msgs::CommandLevel::EMERGENCY)
  {
    return;
  }

  // 最新のコマンドを受け取った時刻を更新
  t_last_cmd_ = ros::Time::now();

  if (!cmd_received_)
  {
    cmd_received_ = true;
  }
}
}  // namespace tobas_state_checker
