#include <tobas_constants/constants.hpp>

#include "../include/tobas_preprocess/battery_lpf.hpp"

using namespace std;

namespace tobas_preprocess
{
BatteryLpf::BatteryLpf(, const string& name) : super(node, pnh, name)
{
  battery_lpf_pub_ = node_.advertise<tobas_msgs::Battery>(tobas::kBatteryLpfTopic, 1);
  battery_raw_sub_ = node_.subscribe(tobas::kBatteryTopic, 1, &self::batteryRawCb, this, tcpNoDelay());
}

void BatteryLpf::batteryRawCb(const tobas_msgs::BatteryConstPtr& battery_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw battery message is received.");
    voltage_lpf_.initialize(kLpfCutoff, battery_raw->voltage);
    current_lpf_.initialize(kLpfCutoff, battery_raw->current);
    last_msg_ = battery_raw;
    return;
  }

  const auto dt = (battery_raw->header.stamp - last_msg_->header.stamp).seconds();
  last_msg_ = battery_raw;

  voltage_lpf_.update(battery_raw->voltage, dt);
  current_lpf_.update(battery_raw->current, dt);

  const auto battery_filtered = make_unique<tobas_msgs::Battery>(*battery_raw);
  battery_filtered->voltage = voltage_lpf_.getOutput();
  battery_filtered->current = current_lpf_.getOutput();
  battery_lpf_pub_.publish(battery_filtered);
}
}  // namespace tobas_preprocess
