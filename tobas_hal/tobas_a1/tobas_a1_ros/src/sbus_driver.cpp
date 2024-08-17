#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Sbus.h>

#include "../include/tobas_a1_ros/sbus_driver.hpp"
#include "../include/tobas_a1_ros/common.hpp"

using namespace std;

namespace a1
{
SBUSDriver::SBUSDriver(const rclcpp::NodeOptions& options) : super(name, options)
{
  if (!sbus_.initialize())
    TOBAS_EXIT("Failed to initialize S.BUS driver.");

  sbus_pub_ = createPublisher<tobas_hal_msgs::Sbus>(hal::kSbusTopic);

  // S.BUSドライバはブロッキングモードだから，メインタイマーを最大レートで回してもCPU消費は低い．
  main_timer_ = node_.createTimer(rclcpp::Duration(0), &self::mainTimerCb, this);
}

void SBUSDriver::mainTimerCb()
{
  // Read S.BUS
  if (!sbus_.update())
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to read S.BUS.");
    return;
  }

  // Create message
  const auto sbus_msg =std::make_unique<tobas_hal_msgs::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  for (size_t ch = 0; ch < sbus_msg->data.size(); ++ch)
    sbus_msg->data[ch] = sbus_.getPeriod(ch);

  // Publish message
  sbus_pub_->publish(sbus_msg);
}
}  // namespace a1
