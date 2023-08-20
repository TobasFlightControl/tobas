#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <tobas_tools/node.hpp>

#include "./ellipse_transformer.hpp"
#include "./common.hpp"

namespace tobas_real
{
class ImuHandler : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;           // [Hz]
  static constexpr double kMeasureGyroBiasRate = 200.;  // [Hz]
  static constexpr uint32_t kMeasureGyroBiasCount = 1000;
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  using super = tobas::BaseNode;

  using ImuMsg = sensor_msgs::Imu;
  using MagMsg = sensor_msgs::MagneticField;
  using ImuPtr = std::unique_ptr<InertialSensor>;

public:
  explicit ImuHandler();

  void run();

private:
  ImuDevice imu_;

  ImuMsg imu_msg_;
  MagMsg mag_msg_;
  Eigen::Vector3f acc_;
  Eigen::Vector3f gyro_;
  Eigen::Vector3f mag_;
  EllipseTransformer mag_trans_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double mag_noise_density_;   // [/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;

  // 固定値
  Eigen::Vector3f gyro_bias_;

  // Publisher
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();
  void setCovarianceMatrices();
  void setupImu();
  void setGyroBias();

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_real
