#pragma once

#include <Eigen/Core>
#include <std_srvs/Trigger.h>

#include <tobas_property_tools/property_client.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>

#include "./common.hpp"
#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class ImuHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 400;         // [Hz]
  static constexpr double kHpfCutoff = 30.;            // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kNoiseStatTimeWindow = 500;  // [ms]
  static constexpr size_t kWindowSize = kSamplingRate * kNoiseStatTimeWindow / 1000;

  static constexpr int kMeasureGyroBiasCount = 1000;   // [-]
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  using self = ImuHandler;
  using super = BaseSensorNode;

public:
  explicit ImuHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;
  ptree::PropertyClient property_client_;

  Eigen::Vector3f acc_, gyro_;
  Eigen::Vector3f gyro_bias_;
  std::array<dsp::NoiseVarianceFilter, 3> acc_noise_, gyro_noise_;

  // Config
  Eigen::Vector3f acc_bias_;  // [m/s^2]

  ros::Publisher imu_pub_;
  ros::ServiceServer reload_config_srv_;
  ros::Timer initialize_timer_;

  bool reloadConfig();
  void measureGyroBias();
  void initializeNoiseFilters();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
  void initializeTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_navio_ros
