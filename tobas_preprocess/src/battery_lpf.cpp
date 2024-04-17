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
  if (!voltage_lpf_.isInitialized() || !current_lpf_.isInitialized())
  {
    TOBAS_INFO("First raw battery message is received.");
    voltage_lpf_.initialize(kLpfTimeConst, battery_raw->voltage);
    current_lpf_.initialize(kLpfTimeConst, battery_raw->current);
    t_last_ = battery_raw->header.stamp;
    return;
  }

  const auto ts = (battery_raw->header.stamp - t_last_).toSec();
  t_last_ = battery_raw->header.stamp;

  voltage_lpf_.update(battery_raw->voltage, ts);
  current_lpf_.update(battery_raw->current, ts);

  const auto battery = boost::make_shared<tobas_msgs::Battery>(*battery_raw);
  battery->voltage = voltage_lpf_.getState();
  battery->current = current_lpf_.getState();
  battery_lpf_pub_.publish(battery);
}
}  // namespace tobas_preprocess
