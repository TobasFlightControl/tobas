#pragma once

#include <Eigen/Core>
#include <std_srvs/Trigger.h>

#include "./common.hpp"
#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class ImuHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 800;  // [Hz]
  static constexpr size_t kMeasureGyroBiasCount = 1000;
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  // Defaults (例外を出さないためにデフォルト値は基本用意しておく)
  static constexpr double kDefaultAccNoiseDensity = 0.05;    // [m/s^2/sqrt(Hz)]
  static constexpr double kDefaultGyroNoiseDensity = 0.005;  // [rad/s/sqrt(Hz)]

  using self = ImuHandler;
  using super = BaseSensorNode;

public:
  explicit ImuHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;

  double acc_var_, gyro_var_;
  Eigen::Vector3f acc_, gyro_;

  // ジャイロバイアス関連
  size_t loop_cnt_ = 0;
  Eigen::Vector3f gyro_sum_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_c_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias_;

  // Config
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  Eigen::Vector3f acc_bias_;   // [m/s^2]

  ros::Publisher imu_pub_;
  ros::ServiceServer reload_config_srv_;
  ros::Timer measure_gyro_bias_timer_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
  void measureGyroBiasTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_navio_ros
