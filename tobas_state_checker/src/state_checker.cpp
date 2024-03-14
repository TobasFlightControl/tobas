#include <std_srvs/SetBool.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/rate.hpp>
#include <tobas_ros_tools/rosparam.hpp>

#include "../include/tobas_state_checker/state_checker.hpp"

using namespace std;

namespace tobas_state_checker
{
StateChecker::StateChecker(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name), landing_ac_(tobas::kLandingAction)
{
  getRosParams();

  registerPublishers();
  registerSubscribers();

  set_arm_sc_ = nh_.serviceClient<std_srvs::SetBool>(tobas::kSetArmSrv);
}

void StateChecker::getRosParams()
{
  tobas_ros::getParam(pnh_, "battery_voltage_threshold", voltage_threshold_, tobas_ros::POSITIVE);
}

void StateChecker::registerPublishers()
{
  event_pub_ = nh_.advertise<tobas_msgs::Event>(tobas::kEventTopic, 1);
}

void StateChecker::registerSubscribers()
{
  cpu_sub_ = nh_.subscribe(tobas::kCpuTopic, 1, &self::cpuCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
}

void StateChecker::publishSystemCriticalEvent()
{
  const auto event = boost::make_shared<tobas_msgs::Event>();
  event->data = tobas_msgs::Event::SYSTEM_CRITICAL;
  event_pub_.publish(event);
}

void StateChecker::requestLanding()
{
  if (!landing_ac_.waitForServer(ros::Duration(kWaitForActionServerTimeout)))
  {
    rosError(
      name_, "'" << tobas::kLandingAction << "' action server failed to start within "
                 << kWaitForActionServerTimeout << " seconds. Please check the server status.");
    return;
  }

  tobas_msgs::LandGoal goal;
  goal.level.data = tobas_msgs::CommandLevel::DEFENSIVE;
  landing_ac_.sendGoal(goal);
  landing_ac_.waitForResult();

  const auto result = landing_ac_.getResult();
  const auto state = landing_ac_.getState();
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

void StateChecker::requestDisarmingRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    rosError(name_, "Failed to connect to '" << tobas::kSetArmSrv << "' service server.");
    return;
  }

  std_srvs::SetBool set_arm_msg;
  set_arm_msg.request.data = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    rosError(name_, "Failed to disarm rotors.");
    return;
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

void StateChecker::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  // TODO: tobas_msgs::EventでGood Stateに戻せるようにする
  if (!is_armed_)
    return;

  // 姿勢角が閾値を超えていたら全モータを非常停止
  // TODO: ここでパラシュートを開く
  odom->frame.M.getRPY(roll_, pitch_, yaw_);
  if (max(abs(roll_), abs(pitch_)) > kAttitudeThreshold)
  {
    rosFatal(name_, "The attitude angle exceeds the threshold. Stopping motors.");
    publishSystemCriticalEvent();
    requestDisarmingRotors();
    is_armed_ = false;
  }
}
}  // namespace tobas_state_checker
