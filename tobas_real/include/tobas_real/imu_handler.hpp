#pragma once

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/timer.h>

#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_tools/node.hpp>

#include "./ellipse_transformer.hpp"
#include "./common.hpp"

namespace tobas_real
{
class ImuHandler : public tobas::BaseNode
{
  // Constants
  static constexpr uint32_t kMeasureGyroBiasCount = 1000;
  static constexpr double kStaticGyroThreshold = 0.2;  // [rad/s]

  // Default parameters
  static constexpr uint32_t kDefaultOverSampling = 4;   // 8だと間に合わない
  static constexpr double kDefaultLpfCutoffFreq = 50.;  // [Hz]

  using super = tobas::BaseNode;

public:
  explicit ImuHandler(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  ImuDevice imu_;

  Eigen::Vector3f acc_;
  Eigen::Vector3f gyro_;
  Eigen::Vector3f mag_;
  EllipseTransformer mag_trans_;
  dh_std::FirstOrderFilter<Eigen::Vector3f> acc_lpf_;
  dh_std::FirstOrderFilter<Eigen::Vector3f> gyro_lpf_;
  dh_std::FirstOrderFilter<Eigen::Vector3f> mag_lpf_;
  uint32_t loop_cnt_ = 0;

  // ジャイロバイアス関連
  Eigen::Vector3f gyro_sum_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double mag_noise_density_;   // [/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;

  // rosparams
  uint32_t over_sampling_;
  double lpf_cutoff_freq_;

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
