#pragma once

#include <rclcpp/rclcpp.hpp>


#include <tobas_linear_control/kalman_filter.hpp>

#include <tobas_node/node.hpp>
#include <tobas_drone_tools/dynamics.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>

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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;
  tobas::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  ctrl::IdentityKalmanFilter kf_;
  tobas_msgs::msg::RotorSpeeds::ConstSharedPtr rotor_speeds_;

  // PubSub
  PublisherPtr<> factor_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<> rotor_speeds_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& rotor_speeds);


};
}  // namespace tobas_mr_thrust_estimation
