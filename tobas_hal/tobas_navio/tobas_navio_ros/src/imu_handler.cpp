#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Imu.h>

#include "../include/tobas_navio_ros/imu_handler.hpp"

using namespace std;

namespace tobas_navio_ros
{
ImuHandler::ImuHandler(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const string& name) : super(node, pnh, name)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

  imu_pub_ = nh_.advertise<tobas_hal_msgs::Imu>(hal::kImuTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void ImuHandler::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Update IMU
  imu_.updateAccelerometer();
  imu_.updateGyroscope();

  // Read IMU
  imu_.readAccelerometer(&acc_.x(), &acc_.y(), &acc_.z());
  imu_.readGyroscope(&gyro_.x(), &gyro_.y(), &gyro_.z());

  // Create messages
  const auto imu_msg = boost::make_shared<tobas_hal_msgs::Imu>();

  // Fill headers
  imu_msg->header.stamp = event.current_real;

  // Fill data (Convert to NWU coordinate system)
  imu_msg->accel.x(acc_.y());
  imu_msg->accel.y(-acc_.x());
  imu_msg->accel.z(acc_.z());

  imu_msg->gyro.x(gyro_.y());
  imu_msg->gyro.y(-gyro_.x());
  imu_msg->gyro.z(gyro_.z());

  // Publish message
  imu_pub_.publish(imu_msg);
}
}  // namespace tobas_navio_ros
