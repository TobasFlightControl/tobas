#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <dh_ros_tools/node.hpp>

#include <Common/InertialSensor.h>

class ImuHandler : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

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

  // rosparam
  double gravity_;

  // Publisher
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  // Timer
  ros::Timer main_loop_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void setupImu();
  void setCovarianceMatrices();

  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void mainLoopTimerCb(const ros::TimerEvent&);
};
