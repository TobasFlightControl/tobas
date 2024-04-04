#include <tobas_ros_tools/console_message.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_preprocess/battery_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
BatteryLpf::BatteryLpf(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
}

void BatteryLpf::getRosParams()
{
}

void BatteryLpf::registerPublishers()
{
  battery_lpf_pub_ = nh_.advertise<tobas_msgs::Battery>(tobas::kBatteryLpfTopic, 1);
}

void BatteryLpf::registerSubscribers()
{
  battery_raw_sub_ =
    nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryRawCb, this, tcpNoDelay());
}

void BatteryLpf::batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw)
{
  if (!lpf_.isInitialized())
  {
    rosInfo(name_, "First raw battery message is received.");
    lpf_.initialize(kLpfTimeConst, battery_raw->voltage);
    t_last_ = ros::Time::now();
    return;
  }

  const auto cur_time = ros::Time::now();
  const auto ts = (cur_time - t_last_).toSec();
  t_last_ = cur_time;

  lpf_.update(battery_raw->voltage, ts);

  const auto battery = boost::make_shared<tobas_msgs::Battery>(*battery_raw);
  battery->voltage = lpf_.getState();
  battery_lpf_pub_.publish(battery);
}
}  // namespace tobas_preprocess
