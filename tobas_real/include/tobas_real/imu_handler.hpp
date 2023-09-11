#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/timer.h>

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
  static constexpr double kStaticGyroThreshold = 0.1;  // [rad/s]

  using super = tobas::BaseNode;

public:
  explicit ImuHandler(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  ImuDevice imu_;

  EllipseTransformer mag_trans_;
  Eigen::Vector3f acc_;
  Eigen::Vector3f gyro_;
  Eigen::Vector3f mag_;

  // ジャイロバイアス関連
  uint32_t gyro_cnt_ = 0;
  Eigen::Vector3f gyro_sum_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double mag_noise_density_;   // [/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;

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
  void setupImu();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void mainTimerCb(const ros::TimerEvent& event);
  void measureGyroBiasTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_real
