#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_state_checker/state_checker.hpp"

using namespace std;

namespace tobas_state_checker
{
StateChecker::StateChecker(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name), ac_(tobas::kLandingAction)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  // Wait for landing action server to start
  if (!ac_.waitForServer(ros::Duration(kWaitForActionServer)))
  {
    rosError(
      name_, "'" << tobas::kLandingAction << "' action server failed to start within "
                 << kWaitForActionServer << " seconds. Please check the server status.");
    requestShutdown();
  }
}

void StateChecker::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (nh_.ok())
  {
    const auto cur_time = ros::Time::now();

    // ベースの状態が一定時間得られていない場合は落とす
    if (bs_received_ && (cur_time - t_last_bs_).toSec() > kBaseStateTimeout)
    {
      rosFatal(
        name_, "The base state is not received for " << kBaseStateTimeout
                                                     << " seconds. Shutting down the system.");
      requestShutdown();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void StateChecker::getRosParams()
{
  dh_ros::getParam(pnh_, "battery_voltage_threshold", voltage_threshold_, dh_ros::POSITIVE);
}

void StateChecker::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
}

void StateChecker::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &StateChecker::eventCb, this);
  cpu_sub_ = nh_.subscribe("cpu", 1, &StateChecker::cpuCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &StateChecker::batteryCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &StateChecker::baseStateCb, this);
}

void StateChecker::requestLanding()
{
  tobas_msgs::LandGoal goal;
  goal.level.data = tobas_msgs::CommandLevel::DEFENSIVE;
  ac_.sendGoal(goal);
  ac_.waitForResult();

  const auto result = ac_.getResult();
  const auto state = ac_.getState();
  if (result->error_code == tobas_msgs::LandResult::NO_ERROR)
  {
    rosInfo(name_, state.getText());
    rosInfo(name_, "Landing action finished successfully.");
  }
  else
  {
    rosError(name_, state.getText());
    rosFatal(name_, "Landing action failed.");
  }

  // 全てのシステムを停止する
  rosInfo(name_, "Shutting down the system.");
  requestShutdown();
}

void StateChecker::eventCb(const tobas_msgs::EventConstPtr& event)
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

void StateChecker::cpuCb(const tobas_msgs::CpuConstPtr& cpu)
{
  // 温度の警告ライン
  if (cpu->temperature > kWarnCpuTemperature)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "CPU temperature is too high: " << cpu->temperature << "℃. It is time to stop flying.");
  }

  // 温度の危険ライン
  if (cpu->temperature > kFatalCpuTemperture)
  {
    rosFatal(
      name_,
      "CPU temperature is too high: " << cpu->temperature << "℃. Issuing a landing command.");
    requestLanding();
  }
}

void StateChecker::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  if (battery->voltage > voltage_threshold_)
  {
    t_last_valid_voltage_ = battery->header.stamp;
    return;
  }

  // バッテリー電圧が閾値を下回ってからの経過時間をチェック
  const auto invalid_time = (battery->header.stamp - t_last_valid_voltage_).toSec();
  if (invalid_time > kBatteryVoltageFatalTime)
  {
    rosFatal(
      name_, "Battery voltage is lower than threshold for "
               << kBatteryVoltageFatalTime << " seconds. Issuing a landing command.");
    requestLanding();
  }
  else if (invalid_time > kBatteryVoltageWarnTime)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Battery voltage is too low: " << battery->voltage << "V. It is time to stop flying.");
    rosWarnOnce(
      name_, "If the battery voltage remains too low for "
               << kBatteryVoltageFatalTime << " seconds, a landing command wil be issued.");
  }
}

void StateChecker::baseStateCb(const tobas_msgs::BaseStateConstPtr& bs)
{
  t_last_bs_ = ros::Time::now();
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  // 状態推定の共分散が閾値を超えた場合は着陸指令を出す
  const auto& pos_cov = bs->position_covariance;
  const auto& rot_cov = bs->orientation_covariance;
  if (max(pos_cov[0], pos_cov[4]) > dh_std::sqr(kHorizontalPositionStddevThreshold))
  {
    rosFatal(
      name_, "Horizontal Position covariance exceeds the threshold. Issuing a landing command.");
    requestLanding();
  }
  if (pos_cov[8] > dh_std::sqr(kVerticalPositionStddevThreshold))
  {
    rosFatal(name_, "Vertical Position variance exceeds the threshold. Issuing a landing command.");
    requestLanding();
  }
  if (max(rot_cov[0], rot_cov[4]) > dh_std::sqr(kAttitudeStddevThreshold))
  {
    rosFatal(name_, "Attitude covariance value exceeds the threshold. Issuing a landing command.");
    requestLanding();
  }
  if (rot_cov[8] > dh_std::sqr(kHeadingStddevThreshold))
  {
    rosFatal(name_, "Heading covariance value exceeds the threshold. Issuing a landing command.");
    requestLanding();
  }

  // 姿勢角が閾値を超えていたら落とす
  const auto& euler = bs->pose.euler;
  if (abs(euler.roll) > kAttitudeThreshold || abs(euler.pitch) > kAttitudeThreshold)
  {
    rosFatal(name_, "The attitude angle exceeds the threshold. Shutting down the system.");
    requestShutdown();
  }
}
}  // namespace tobas_state_checker
