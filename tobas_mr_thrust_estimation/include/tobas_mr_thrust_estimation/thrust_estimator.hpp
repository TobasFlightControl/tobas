#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dh_linear_control/kalman_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_mr_common/dynamics.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_thrust_estimation/ThrustEstimationConfig.h>

namespace tobas_mr_thrust_estimation
{
class ThrustEstimator : public tobas::BaseNode
{
  static constexpr double kInitFactorStddev = 0.;   // [-]
  static constexpr double kAltitudeThreshold = 2.;  // [m] 推定を開始する対地高度

  using self = ThrustEstimator;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_thrust_estimation::ThrustEstimationConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ThrustEstimator(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;
  tobas_mr_common::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  ctrl::IdentityKalmanFilter kf_;
  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;

  // PubSub
  ros::Publisher factor_pub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber rotor_speeds_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_mr_thrust_estimation
