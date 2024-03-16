#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_tools/node.hpp>

#include "./ellipse_transformer.hpp"
#include "./common.hpp"

namespace tobas_navio_ros
{
class ImuHandler : public tobas::BaseNode
{
  // Constants
  static constexpr size_t kSamplingRate = 400;  // [Hz]
  static constexpr size_t kMeasureGyroBiasCount = 1000;
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  // Defaults (例外を出さないためにデフォルト値は基本用意しておく)
  static constexpr double kDefaultAccNoiseDensity = 0.05;    // [m/s^2/sqrt(Hz)]
  static constexpr double kDefaultGyroNoiseDensity = 0.005;  // [rad/s/sqrt(Hz)]
  static constexpr double kDefaultMagNoiseDensity = 0.05;    // [/sqrt(Hz)]

  using self = ImuHandler;
  using super = tobas::BaseNode;

public:
  explicit ImuHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;

  double acc_var_, gyro_var_, mag_var_;
  Eigen::Vector3f acc_, gyro_, mag_;

  // ジャイロバイアス関連
  size_t loop_cnt_ = 0;
  Eigen::Vector3f gyro_sum_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double mag_noise_density_;   // [/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;   // [m/s^2]
  EllipseTransformer mag_trans_;

  // Publisher
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;

  // Timer
  ros::Timer main_timer_;
  ros::Timer measure_gyro_bias_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void readConfig();

  void mainTimerCb(const ros::TimerEvent& event);
  void measureGyroBiasTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_navio_ros
