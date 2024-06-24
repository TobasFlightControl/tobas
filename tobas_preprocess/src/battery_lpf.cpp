#include <tobas_tools/constants.hpp>

#include "../include/tobas_preprocess/battery_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
BatteryLpf::BatteryLpf(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  battery_lpf_pub_ = nh_.advertise<tobas_msgs::Battery>(tobas::kBatteryLpfTopic, 1);
  battery_raw_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryRawCb, this, tcpNoDelay());
}

void BatteryLpf::batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw battery message is received.");
    voltage_lpf_.initializeFromTimeConst(kLpfTimeConst, battery_raw->voltage);
    current_lpf_.initializeFromTimeConst(kLpfTimeConst, battery_raw->current);
    last_msg_ = battery_raw;
    return;
  }

  const auto dt = (battery_raw->header.stamp - last_msg_->header.stamp).toSec();
  last_msg_ = battery_raw;

  voltage_lpf_.update(battery_raw->voltage, dt);
  current_lpf_.update(battery_raw->current, dt);

  const auto battery_filtered = boost::make_shared<tobas_msgs::Battery>(*battery_raw);
  battery_filtered->voltage = voltage_lpf_.getOutput();
  battery_filtered->current = current_lpf_.getOutput();
  battery_lpf_pub_.publish(battery_filtered);
}
}  // namespace tobas_preprocess
