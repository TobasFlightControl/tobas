#pragma once

#include <ros/ros.h>

#include <dh_linear_control/kalman_filter.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/dryden_wind_model.hpp>
#include <tobas_mr_common/dynamics.hpp>
#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RotorSpeeds.h>

namespace tobas_mr_wind_estimation
{
class WindEstimator : public tobas::BaseNode
{
  static constexpr uint32_t kStateSize = 2;
  static constexpr double kInitWindStddev = 10.;  // [m/s]

  using self = WindEstimator;
  using super = tobas::BaseNode;

public:
  explicit WindEstimator(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  void updateInternalDataStructures();

private:
  tobas::Drone drone_;
  tobas_mr_common::MultirotorDynamicsComponents dynamics_;

  bool is_flying_ = false;
  bool is_initialized_ = false;
  ros::Time t_last_loop_;
  ctrl::IdentityKalmanFilter kf_;
  tobas::DrydenComponents dryden_;

  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;

  // PubSub
  ros::Publisher wind_pub_;
  ros::Subscriber pt_sub_;
  ros::Subscriber rotor_speeds_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  Eigen::Matrix3d velCoef(const KDL::Euler& R_W_B);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void poseTwistCb(const tobas_msgs::PoseTwistConstPtr& pt);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
};
}  // namespace tobas_mr_wind_estimation
