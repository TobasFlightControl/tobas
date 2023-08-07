#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Common/MPU9250.h>

#include <tobas_tools/node.hpp>

namespace tobas_real
{
class ImuHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;  // [Hz]

  // MPU9250
  // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
  static constexpr double kAccNoiseDensity = 300.;  // ug/sqrt(hz)  // TODO: 実際は遥かに大きい
  static constexpr double kGyroNoiseDensity = 0.01;  // deg/s/sqrt(hz)
  static constexpr double kMagNoiseStd = 0.;  // TODO: データシートに無かったため計測する

  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using ImuPtr = std::unique_ptr<InertialSensor>;

public:
  explicit ImuHandler();

  void run();

private:
  MPU9250 imu_;
  ImuMsg imu_msg_;
  MagMsg mag_msg_;
  Eigen::Vector3f acc_;
  Eigen::Vector3f gyro_;
  Eigen::Vector3f mag_;

  // 固定値
  Eigen::Vector3f acc_offset_;

  // Publisher
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void setupImu();
  void setCovarianceMatrices();
  void getAccelOffset();

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
