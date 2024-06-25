#include <tobas_math/core.hpp>
#include <tobas_msgs/SetArm.h>

#include "../include/tobas_state_checker/state_checker.hpp"

using namespace std;

namespace tobas_state_checker
{
StateChecker::StateChecker(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), landing_ac_(tobas::kLandAction)
{
  drone_.loadFromParam(nh_);

  event_pub_ = nh_.advertise<tobas_msgs::Event>(tobas::kEventTopic, 1);

  arming_sub_ = nh_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this, tcpNoDelay());
  cpu_sub_ = nh_.subscribe(tobas::kCpuTopic, 1, &self::cpuCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  euler_sub_ = nh_.subscribe(tobas::kEulerTopic, 1, &self::eulerCb, this, tcpNoDelay());

  set_arm_sc_ = nh_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);
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
    TOBAS_ERROR(
      "'", tobas::kLandAction, "' action server failed to start within ", kWaitForActionServerTimeout,
      " seconds. Please check the server status.");
    return;
  }

  tobas_msgs::LandGoal goal;
  goal.level.data = tobas_msgs::CommandLevel::DEFENSIVE;
  landing_ac_.sendGoal(goal);
  landing_ac_.waitForResult();

  const auto result = landing_ac_.getResult();
  const auto state = landing_ac_.getState();
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    TOBAS_INFO(state.getText());
    TOBAS_INFO("Landing action finished successfully.");
  }
  else
  {
    TOBAS_ERROR(state.getText());
    TOBAS_FATAL("Landing action failed.");
  }
}

void StateChecker::requestDisarmingRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kSetArmSrv, "' service server.");
    return;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    TOBAS_ERROR("Failed to disarm rotors.");
    return;
  }
}

void StateChecker::armingCb(const std_msgs::BoolConstPtr& arming)
{
  arming_ = arming;
}

void StateChecker::cpuCb(const tobas_msgs::CpuConstPtr& cpu)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  if (cpu->temperature > kCpuTempertureThreshold)
  {
    TOBAS_WARN_THROTTLE(
      kWarnPeriod, "CPU temperature is too high: ", cpu->temperature, " [C]. It is time to stop flying.");
  }
}

void StateChecker::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  if (battery->voltage < drone_.batteryConfig().sag_voltage)
  {
    TOBAS_WARN_THROTTLE(
      kWarnPeriod, "Battery voltage is too low: ", battery->voltage, " [V]. It is time to stop flying.");
  }
}

void StateChecker::eulerCb(const tobas_kdl_msgs::EulerStampedConstPtr& euler)
{
  if (arming_ == nullptr || !arming_->data)
    return;

  // 姿勢角が閾値を超えていたら全モータを非常停止
  // TODO: ここでパラシュートを開く
  if (max(abs(euler->euler.roll), abs(euler->euler.pitch)) > kAttitudeThreshold)
  {
    TOBAS_FATAL("The attitude angle exceeds the threshold. Stopping motors.");
    publishSystemCriticalEvent();
    requestDisarmingRotors();
  }
}
}  // namespace tobas_state_checker
