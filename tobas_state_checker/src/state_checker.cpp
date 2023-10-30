#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_state_checker/state_checker.hpp"

using namespace std;

namespace tobas_state_checker
{
StateChecker::StateChecker(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), landing_client_(tobas::kLandingAction)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();
}

void StateChecker::getRosParams()
{
  dh_ros::getParam(pnh_, "battery_voltage_threshold", voltage_threshold_, dh_ros::POSITIVE);
}

void StateChecker::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>(tobas::kEventTopic, 1);
}

void StateChecker::registerSubscribers()
{
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &StateChecker::eventCb, this, tcpNoDelay());
  cpu_sub_ = nh_.subscribe(tobas::kCpuTopic, 1, &StateChecker::cpuCb, this, tcpNoDelay());
  battery_sub_ =
    nh_.subscribe(tobas::kBatteryTopic, 1, &StateChecker::batteryCb, this, tcpNoDelay());
  pt_sub_ =
    nh_.subscribe(tobas::kPoseTwistTopic, 1, &StateChecker::poseTwistCb, this, tcpNoDelay());
}

void StateChecker::requestShutdown()
{
  const auto event = boost::make_shared<tobas_msgs::Event>();
  event->data = tobas_msgs::Event::STOP;
  event_pub_.publish(event);
}

void StateChecker::requestLanding()
{
  if (!landing_client_.waitForServer(ros::Duration(kWaitForActionServerTimeout)))
  {
    rosError(
      name_, "'" << tobas::kLandingAction << "' action server failed to start within "
                 << kWaitForActionServerTimeout << " seconds. Please check the server status.");
    return;
  }

  tobas_msgs::LandGoal goal;
  goal.level.data = tobas_msgs::CommandLevel::DEFENSIVE;
  landing_client_.sendGoal(goal);
  landing_client_.waitForResult();

  const auto result = landing_client_.getResult();
  const auto state = landing_client_.getState();
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
}

void StateChecker::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
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

void StateChecker::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  t_last_pt_ = ros::Time::now();
  if (!pt_received_)
  {
    pt_received_ = true;
  }

  // 姿勢角が閾値を超えていたら落とす
  const auto& euler = pt->pose.euler;
  if (abs(euler.roll) > kAttitudeThreshold || abs(euler.pitch) > kAttitudeThreshold)
  {
    rosFatal(name_, "The attitude angle exceeds the threshold. Shutting down the system.");
    requestShutdown();
  }
}
}  // namespace tobas_state_checker
