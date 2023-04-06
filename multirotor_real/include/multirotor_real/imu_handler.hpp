#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <Common/InertialSensor.h>

class ImuHandler
{
  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using ImuPtr = std::unique_ptr<InertialSensor>;

public:
  ImuHandler();

private:
  ros::NodeHandle nh_;

  // rosparam
  const double gravity_;

  ImuPtr imu_;
  ImuMsg imu_msg_;
  MagMsg mag_msg_;
  float ax_, ay_, az_;
  float wx_, wy_, wz_;
  float mx_, my_, mz_;

  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  ros::Timer timer_;

  void setupImu();
  void setCovarianceMatrices();
  void advertise();
  void timerCb(const ros::TimerEvent&);
};
