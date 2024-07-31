#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/MagneticField.h>

#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

using namespace std;

namespace tobas_navio_ros
{
MagnetometerHandler::MagnetometerHandler(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const string& name)
  : super(node, pnh, name)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

  mag_pub_ = nh_.advertise<tobas_hal_msgs::MagneticField>(hal::kMagTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void MagnetometerHandler::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Update IMU
  imu_.updateMagnetometer();

  // Read IMU
  imu_.readMagnetometer(&mag_.x(), &mag_.y(), &mag_.z());

  // Create messages
  const auto mag_msg = boost::make_shared<tobas_hal_msgs::MagneticField>();

  // Fill headers
  mag_msg->header.stamp = event.current_real;

  // Fill data (Convert to NWU coordinate system)
  mag_msg->magnetic_field.x(mag_.x());
  mag_msg->magnetic_field.y(-mag_.y());
  mag_msg->magnetic_field.z(-mag_.z());

  // Publish messages
  mag_pub_.publish(mag_msg);
}
}  // namespace tobas_navio_ros
