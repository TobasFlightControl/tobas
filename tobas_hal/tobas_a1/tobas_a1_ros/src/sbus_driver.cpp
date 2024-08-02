#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Sbus.h>

#include "../include/tobas_a1_ros/sbus_driver.hpp"
#include "../include/tobas_a1_ros/common.hpp"

using namespace std;

namespace a1
{
SBUSDriver::SBUSDriver(, const string& name) : super(node, pnh, name)
{
  if (!sbus_.initialize())
    TOBAS_EXIT("Failed to initialize S.BUS driver.");

  sbus_pub_ = node_.advertise<tobas_hal_msgs::Sbus>(hal::kSbusTopic, 1);

  // S.BUSドライバはブロッキングモードだから，メインタイマーを最大レートで回してもCPU消費は低い．
  main_timer_ = node_.createTimer(rclcpp::Duration(0), &self::mainTimerCb, this);
}

void SBUSDriver::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Read S.BUS
  if (!sbus_.update())
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to read S.BUS.");
    return;
  }

  // Create message
  const auto sbus_msg = make_unique<tobas_hal_msgs::Sbus>();
  sbus_msg->header.stamp = event.current_real;
  for (size_t ch = 0; ch < sbus_msg->data.size(); ++ch)
    sbus_msg->data[ch] = sbus_.getPeriod(ch);

  // Publish message
  sbus_pub_.publish(sbus_msg);
}
}  // namespace a1
