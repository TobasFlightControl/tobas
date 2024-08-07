#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/MagneticField.h>

#include "../include/tobas_a1_ros/mag_driver.hpp"

using namespace std;

namespace a1
{
MagDriver::MagDriver(const rclcpp::NodeOptions& options) : super(node, pnh, name)
{
  if (!mag_.initialize())
    TOBAS_EXIT("Failed to initialize Magnetometer.");

  mag_pub_ = createPublisher<tobas_hal_msgs::MagneticField>(hal::kMagTopic);
  main_timer_ = node_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void MagDriver::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Create messages
  const auto msg =std::make_unique<tobas_hal_msgs::MagneticField>();

  // Fill headers
  msg->header.stamp = event.current_real;

  // Read sensor
  if (!mag_.readMag(msg->magnetic_field.x(), msg->magnetic_field.y(), msg->magnetic_field.z()))
  {
    TOBAS_FATAL("Failed to read magnetometer.");
    return;
  }

  // TODO: 軸や符号の変換が必要かも

  // Publish message
  mag_pub_->publish(msg);
}
}  // namespace a1
