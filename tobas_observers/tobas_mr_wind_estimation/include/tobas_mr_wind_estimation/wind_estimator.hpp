#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_linear_control/kalman_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/dryden_wind_model.hpp>
#include <tobas_mr_common/dynamics.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/RotorSpeeds.h>

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
  tobas_mr_common::MultirotorDynamicsComponents dynamics_;

  bool is_initialized_ = false;
  rclcpp::Time t_last_loop_;
  ctrl::IdentityKalmanFilter kf_;
  tobas::DrydenComponents dryden_;

  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;

  // PubSub
  rclcpp::Publisher wind_pub_;
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber rotor_speeds_sub_;

  Eigen::Matrix3d velCoef(const kdl::Rotation& R_W_B);

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
};
}  // namespace tobas_mr_wind_estimation
