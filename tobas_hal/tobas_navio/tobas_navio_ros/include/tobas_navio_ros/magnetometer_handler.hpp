#pragma once

#include <Eigen/Core>
#include <std_srvs/Trigger.h>

#include "./common.hpp"
#include "./base_sensor_node.hpp"
#include "./ellipse_transformer.hpp"

namespace tobas_navio_ros
{
class MagnetometerHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 80;  // [Hz] LSM9DS1の地磁気のサンプリングレートの最大値

  // Defaults
  static constexpr double kDefaultMagNoiseDensity = 0.05;  // [/sqrt(Hz)]

  using self = MagnetometerHandler;
  using super = BaseSensorNode;

public:
  explicit MagnetometerHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ImuDevice imu_;

  double mag_var_;
  Eigen::Vector3f mag_;

  // Config
  double mag_noise_density_;  // [/sqrt(Hz)]
  EllipseTransformer mag_trans_;

  ros::Publisher mag_pub_;
  ros::ServiceServer reload_config_srv_;
  ros::Timer measure_gyro_bias_timer_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
  void measureGyroBiasTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_navio_ros
