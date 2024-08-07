#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_linear_control/kalman_filter.hpp>

#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_wind_model/dryden.hpp>
#include <tobas_drone_tools/dynamics.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>

namespace tobas_mr_wind_estimation
{
class WindEstimator : public tobas::BaseNode
{
  static constexpr size_t kStateSize = 2;
  static constexpr double kInitWindStddev = 10.;  // [m/s]

  using self = WindEstimator;
  using super = tobas::BaseNode;

public:
  explicit WindEstimator(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;
  tobas::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  rclcpp::Time t_last_loop_;
  ctrl::IdentityKalmanFilter kf_;
  tobas::DrydenComponents dryden_;

  tobas_msgs::msg::RotorSpeeds::ConstSharedPtr rotor_speeds_;

  // PubSub
  PublisherPtr<> wind_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<> rotor_speeds_sub_;

  Eigen::Matrix3d velCoef(const kdl::Rotation& R_W_B);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::msg::RotorSpeeds::ConstSharedPtr& rotor_speeds);
};
}  // namespace tobas_mr_wind_estimation
