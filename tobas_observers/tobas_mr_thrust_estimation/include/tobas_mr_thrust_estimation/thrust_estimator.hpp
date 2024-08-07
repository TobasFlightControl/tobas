#pragma once

#include <rclcpp/rclcpp.hpp>
#include <dynamic_reconfigure/server.h>

#include <tobas_linear_control/kalman_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_drone_tools/dynamics.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_mr_thrust_estimation/ThrustEstimationConfig.h>

namespace tobas_mr_thrust_estimation
{
class ThrustEstimator : public tobas::BaseNode
{
  static constexpr double kInitFactorStddev = 0.;  // [-]
  static constexpr double kMinFactor = 0.9;        // 接地時に下振れしないよう最小値に制限をかける

  using self = ThrustEstimator;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_thrust_estimation::ThrustEstimationConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ThrustEstimator(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;
  tobas::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  ctrl::IdentityKalmanFilter kf_;
  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;

  // PubSub
  rclcpp::Publisher factor_pub_;
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber rotor_speeds_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_mr_thrust_estimation
