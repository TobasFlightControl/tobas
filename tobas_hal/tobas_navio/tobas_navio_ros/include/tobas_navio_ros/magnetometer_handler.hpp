#pragma once

#include <Eigen/Core>
#include <std_srvs/Trigger.h>

#include <tobas_math/ellipse_transformer.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>

#include "./common.hpp"
#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class MagnetometerHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 80;  // [Hz] LSM9DS1の地磁気のサンプリングレートの最大値
  static constexpr double kHpfCutoff = 10.;    // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kNoiseStatTimeWindow = 1000;  // [ms]
  static constexpr size_t kWindowSize = kSamplingRate * kNoiseStatTimeWindow / 1000;

  using self = MagnetometerHandler;
  using super = BaseSensorNode;

public:
  explicit MagnetometerHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;
  ptree::PropertyClient property_client_;

  Eigen::Vector3f mag_;
  std::array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  // Config
  math::EllipseTransformer mag_trans_;

  ros::Publisher mag_pub_;
  ros::ServiceServer reload_config_srv_;

  bool reloadConfig();
  void initializeNoiseFilter();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
