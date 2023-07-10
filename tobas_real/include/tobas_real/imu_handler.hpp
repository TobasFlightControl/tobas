#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Common/InertialSensor.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class ImuHandler : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using ImuPtr = std::unique_ptr<InertialSensor>;

public:
  explicit ImuHandler();

private:
  ImuPtr imu_;
  ImuMsg imu_msg_;
  MagMsg mag_msg_;
  float ax_, ay_, az_;
  float wx_, wy_, wz_;
  float mx_, my_, mz_;

  // Publisher
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  // Timer
  dh_ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setupImu();
  void setCovarianceMatrices();

  void eventCb(const tobas_msgs::Event& event) override;
  void mainLoopTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
