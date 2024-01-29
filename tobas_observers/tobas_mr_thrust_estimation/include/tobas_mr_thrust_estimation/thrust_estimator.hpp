#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <tobas_linear_control/kalman_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_mr_common/dynamics.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_thrust_estimation/ThrustEstimationConfig.h>

namespace tobas_mr_thrust_estimation
{
class ThrustEstimator : public tobas::BaseNode
{
  static constexpr double kInitFactorStddev = 0.;  // [-]
  static constexpr double kMinFactor = 0.9;  // 接地時に下振れしないよう最小値に制限をかける

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
  ros::Subscriber odom_sub_;
  ros::Subscriber rotor_speeds_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_mr_thrust_estimation
