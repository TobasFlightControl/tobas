#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_state_checker/multirotor_state_checker.hpp"
#include "../../include/tobas_state_checker/common.hpp"

using namespace std;

namespace tobas_state_checker
{
constexpr char MultirotorStateChecker::kLandActionName[];

MultirotorStateChecker::MultirotorStateChecker()
  : super(),
    battery_received_(false),
    bs_received_(false),
    cmd_received_(false),
    ac_(kLandActionName)
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
    requestShutdown();
  }
}

void MultirotorStateChecker::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    const auto cur_time = ros::Time::now();

    // バッテリー電圧の監視
    if (battery_received_ && battery_.voltage < warn_voltage_)
    {
      rosWarnThrottle(
        kWarnPeriod, "Battery voltage " << battery_.voltage << "V is less than the warning line "
                                        << warn_voltage_ << "V. It is time to stop flying.");
    }
    if (battery_received_ && battery_.voltage < fatal_voltage_)
    {
      rosFatal(
        "Battery voltage " << battery_.voltage << "V is less than the fatal line " << fatal_voltage_
                           << "V. Issuing a landing command.");
      requestLanding();
    }

    // 状態推定の共分散が閾値を超えた場合は着陸指令を出す
    const auto& pos_cov = bs_.position_covariance;
    const auto& rot_cov = bs_.orientation_covariance;
    if (dh_std::max(pos_cov[0], pos_cov[4], pos_cov[8]) > dh_std::sqr(kPositionStddevThreshold))
    {
      rosFatal("Position covariance value exceeds the threshold. Issuing a landing command.");
      requestLanding();
    }
    if (max(rot_cov[0], rot_cov[4]) > dh_std::sqr(kAttitudeStddevThreshold))
    {
      rosFatal("Attitude covariance value exceeds the threshold. Issuing a landing command.");
      requestLanding();
    }
    if (rot_cov[8] > dh_std::sqr(kHeadingStddevThreshold))
    {
      rosFatal("Heading covariance value exceeds the threshold. Issuing a landing command.");
      requestLanding();
    }

    // ベースの状態が一定時間得られていない場合は落とす
    if (bs_received_ && (cur_time - t_last_bs_).toSec() > kBaseStateTimeout)
    {
      rosFatal(
        "The base state is not received for " << kBaseStateTimeout
                                              << " seconds. Shutting down the system.");
      requestShutdown();
    }

    // GCSとの通信が切れるなどして一定時間コマンドを受け取っていない場合は着陸指令を出す
    if (cmd_received_ && (cur_time - t_last_cmd_).toSec() > kCommandTimeout)
    {
      cmd_received_ = false;

      rosWarn(
        "Issuing a landing command as no commands have been received for " << kCommandTimeout
                                                                           << " seconds.");
      requestLanding();
    }

    // 姿勢角が閾値を超えていたら落とす
    const auto& euler = bs_.pose.euler;
    if (abs(euler.roll) > kAttitudeThreshold || abs(euler.pitch) > kAttitudeThreshold)
    {
      rosFatal("The attitude angle exceeds the threshold. Shutting down the system.");
      requestShutdown();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MultirotorStateChecker::getRosParams()
{
  dh_ros::getParam("~warn_battery_voltage", warn_voltage_, dh_ros::POSITIVE);
  dh_ros::getParam("~fatal_battery_voltage", fatal_voltage_, dh_ros::POSITIVE);

  if (warn_voltage_ <= fatal_voltage_)
  {
    rosthrow("warn_battery_voltage must be greater than fatal_battery_voltage.");
  }
}

void MultirotorStateChecker::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
}

void MultirotorStateChecker::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MultirotorStateChecker::eventCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MultirotorStateChecker::batteryCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorStateChecker::baseStateCb, this);
  cmd_sub_ = nh_.subscribe("command/velocity_yaw", 1, &MultirotorStateChecker::commandCb, this);
}

void MultirotorStateChecker::requestLanding()
{
  tobas_msgs::LandGoal goal;
  goal.level.data = tobas_msgs::CommandLevel::EMERGENCY;
  ac_.sendGoal(goal);
  ac_.waitForResult();

  const auto result = ac_.getResult();
  const auto state = ac_.getState();
  if (result->error_code == tobas_msgs::LandResult::NO_ERROR)
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
  requestShutdown();
}

void MultirotorStateChecker::requestShutdown()
{
  event_.data = tobas_msgs::Event::SHUTDOWN;
  event_pub_.publish(event_);
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

void MultirotorStateChecker::batteryCb(const tobas_msgs::Battery& battery)
{
  battery_ = battery;

  if (!battery_received_)
  {
    battery_received_ = true;
  }
}

void MultirotorStateChecker::baseStateCb(const tobas_msgs::BaseState& bs)
{
  bs_ = bs;
  t_last_bs_ = ros::Time::now();

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
