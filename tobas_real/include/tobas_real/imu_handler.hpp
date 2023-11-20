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
  // Constants
  static constexpr size_t kMeasureGyroBiasCount = 1000;
  static constexpr size_t kMeasureGyroBiasRate = 400;  // [Hz]
  static constexpr double kStaticGyroThreshold = 0.2;  // [rad/s]

  // Default Parameters
  static constexpr size_t kDefaultUpdateRate = 400;  // [Hz]

  using self = ImuHandler;
  using super = tobas::BaseNode;

public:
  explicit ImuHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;

  Eigen::Vector3f acc_;
  Eigen::Vector3f gyro_;
  Eigen::Vector3f mag_;
  EllipseTransformer mag_trans_;

  // ジャイロバイアス関連
  size_t loop_cnt_ = 0;
  Eigen::Vector3f gyro_sum_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double mag_noise_density_;   // [/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;

  // rosparams
  size_t update_rate_;

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
