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
  super::registerSubscribers();

  cpu_sub_ = nh_.subscribe(tobas::kCpuTopic, 1, &self::cpuCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
  pt_sub_ = nh_.subscribe(tobas::kPoseTwistTopic, 1, &self::poseTwistCb, this, tcpNoDelay());
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

void StateChecker::publishEvent(const uint8_t& event)
{
  const auto event_msg = boost::make_shared<tobas_msgs::Event>();
  event_msg->data = event;
  event_pub_.publish(event_msg);
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
  if (cpu->temperature > kCpuTempertureThreshold)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "CPU temperature is too high: " << cpu->temperature << " [C]. It is time to stop flying.");
  }
}

void StateChecker::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  if (battery->voltage < voltage_threshold_)
  {
    rosWarnThrottle(
      kWarnPeriod, name_,
      "Battery voltage is too low: " << battery->voltage << " [V]. It is time to stop flying.");
  }
}

void StateChecker::poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt)
{
  // 離陸チェック
  if (!is_flying_ && pt->pose.pos.z() > kTakeoffAltitudeThreshold)
  {
    rosInfo(name_, "Takeoff detected.");
    is_flying_ = true;
    publishEvent(tobas_msgs::Event::TAKEOFF_DETECTED);
  }

  // 姿勢角が閾値を超えていたら落とす
  const auto& euler = pt->pose.euler;
  if (abs(euler.roll) > kAttitudeThreshold || abs(euler.pitch) > kAttitudeThreshold)
  {
    rosFatal(name_, "The attitude angle exceeds the threshold. Shutting down the system.");
    publishEvent(tobas_msgs::Event::STOP);
  }
}
}  // namespace tobas_state_checker
