#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Imu.h>

#include "../include/tobas_a1_ros/imu_driver.hpp"

using namespace std;

namespace a1
{
IMUDriver::IMUDriver(const rclcpp::NodeOptions& options) : super(name, options)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

  imu_pub_ = createPublisher<tobas_hal_msgs::Imu>(hal::kImuTopic);
  main_timer_ = createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void IMUDriver::mainTimerCb()
{
  // Create messages
  const auto msg =std::make_unique<tobas_hal_msgs::Imu>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read IMU
  if (!imu_.readAcc(msg->accel.x(), msg->accel.y(), msg->accel.z()))
  {
    TOBAS_FATAL("Failed to read accelerometer.");
    return;
  }
  if (!imu_.readGyro(msg->gyro.x(), msg->gyro.y(), msg->gyro.z()))
  {
    TOBAS_FATAL("Failed to read gyroscope.");
    return;
  }

  // TODO: 軸や符号の変換が必要かも

  // Publish message
  imu_pub_->publish(msg);
}
}  // namespace a1
