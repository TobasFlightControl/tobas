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
    requestShutdown();
  }
}

void MultirotorStateChecker::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    const auto cur_time = ros::Time::now();

    // ベースの状態が一定時間得られていない場合は落とす
    if (bs_received_ && (cur_time - t_last_bs_).toSec() > kBaseStateTimeout)
    {
      rosFatal(
        "The base state is not received for " << kBaseStateTimeout
                                              << " seconds. Shutting down the system.");
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
  cpu_sub_ = nh_.subscribe("cpu", 1, &MultirotorStateChecker::cpuCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MultirotorStateChecker::batteryCb, this);
  bs_sub_ = nh_.subscribe("base_state", 1, &MultirotorStateChecker::baseStateCb, this);
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

void MultirotorStateChecker::cpuCb(const tobas_msgs::Cpu& cpu)
{
  // 温度の警告ライン
  if (cpu.temperature > kWarnCpuTemperature)
  {
    rosWarnThrottle(
      kWarnPeriod,
      "CPU temperature is too high: " << cpu.temperature << "℃. It is time to stop flying.");
  }

  // 温度の危険ライン
  if (cpu.temperature > kFatalCpuTemperture)
  {
    rosFatal("CPU temperature is too high: " << cpu.temperature << "℃. Issuing a landing command.");
    requestLanding();
  }
}

void MultirotorStateChecker::batteryCb(const tobas_msgs::Battery& battery)
{
  // 電圧の警告ライン
  if (battery.voltage < warn_voltage_)
  {
    rosWarnThrottle(
      kWarnPeriod,
      "Battery voltage is too low: " << battery.voltage << "V. It is time to stop flying.");
  }

  // 電圧の危険ライン
  if (battery.voltage < fatal_voltage_)
  {
    rosFatal("Battery voltage is too low: " << battery.voltage << "V. Issuing a landing command.");
    requestLanding();
  }
}

void MultirotorStateChecker::baseStateCb(const tobas_msgs::BaseState& bs)
{
  t_last_bs_ = ros::Time::now();
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  // 状態推定の共分散が閾値を超えた場合は着陸指令を出す
  const auto& pos_cov = bs.position_covariance;
  const auto& rot_cov = bs.orientation_covariance;
  if (max(pos_cov[0], pos_cov[4]) > dh_std::sqr(kHorizontalPositionStddevThreshold))
  {
    rosFatal("Horizontal Position covariance exceeds the threshold. Issuing a landing command.");
    requestLanding();
  }
  if (pos_cov[8] > dh_std::sqr(kVerticalPositionStddevThreshold))
  {
    rosFatal("Vertical Position variance exceeds the threshold. Issuing a landing command.");
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

  // 姿勢角が閾値を超えていたら落とす
  const auto& euler = bs.pose.euler;
  if (abs(euler.roll) > kAttitudeThreshold || abs(euler.pitch) > kAttitudeThreshold)
  {
    rosFatal("The attitude angle exceeds the threshold. Shutting down the system.");
    requestShutdown();
  }
}
}  // namespace tobas_state_checker
