#pragma once

#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_navio_core/ms5611.hpp>

#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class BarometerHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 50;           // [Hz]
  static constexpr double kHpfCutoff = 10.;             // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kNoiseStatTimeWindow = 1000;  // [ms]

  using self = BarometerHandler;
  using super = BaseSensorNode;

public:
  explicit BarometerHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::MS5611 barometer_;
  dsp::NoiseVarianceFilter pressure_noise_;

  ros::Publisher bar_pub_;

  void initializeNoiseFilter();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros
